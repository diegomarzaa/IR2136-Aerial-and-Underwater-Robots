import glob
import os
import re
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

_INDEX_RE = re.compile(r"(\d+)(?!.*\d)")


def _natural_key(path: str) -> Tuple[int, str]:
    name = os.path.basename(path)
    match = _INDEX_RE.search(name)
    if match:
        return int(match.group(1)), name
    return 10**9, name


def _mat_3x3_or_none(mat: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if mat is None:
        return None
    arr = np.asarray(mat, dtype=np.float64)
    if arr.size != 9:
        return None
    return arr.reshape(3, 3)


def _mat_3x4_or_none(mat: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if mat is None:
        return None
    arr = np.asarray(mat, dtype=np.float64)
    if arr.size != 12:
        return None
    return arr.reshape(3, 4)


def _vector3_or_none(mat: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if mat is None:
        return None
    arr = np.asarray(mat, dtype=np.float64).reshape(-1)
    if arr.size < 3:
        return None
    return arr[:3].reshape(3, 1)


class StereoImageFolderPublisher(Node):
    def __init__(self) -> None:
        super().__init__('stereo_image_folder_publisher')

        self.bridge = CvBridge()

        self.declare_parameter('image_folder', '')
        self.declare_parameter('left_pattern', 'image_left_*.png')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('loop', True)

        self.declare_parameter('publish_raw', True)
        self.declare_parameter('publish_rectified', True)
        self.declare_parameter('publish_rgb_rectified', True)
        self.declare_parameter('publish_camera_info', True)
        self.declare_parameter('rect_alpha', 0.0)

        self.declare_parameter('qos_reliability', 'reliable')

        self.declare_parameter('topic_prefix', '/zed/zed_node')
        self.declare_parameter('left_frame_id', 'zed_left_camera_optical_frame')
        self.declare_parameter('right_frame_id', 'zed_right_camera_optical_frame')

        self.declare_parameter('calibration_file', '')
        self.declare_parameter('ros_params_override_path', '')
        self.declare_parameter('general.optional_opencv_calibration_file', '')

        self.image_folder = os.path.abspath(os.path.expanduser(str(self.get_parameter('image_folder').value).strip()))
        self.left_pattern = str(self.get_parameter('left_pattern').value).strip()
        self.loop = bool(self.get_parameter('loop').value)

        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.publish_rectified = bool(self.get_parameter('publish_rectified').value)
        self.publish_rgb_rectified = bool(self.get_parameter('publish_rgb_rectified').value)
        self.publish_camera_info = bool(self.get_parameter('publish_camera_info').value)
        self.rect_alpha = float(self.get_parameter('rect_alpha').value)
        if self.rect_alpha < -1.0 or self.rect_alpha > 1.0:
            self.get_logger().warn(
                f'rect_alpha={self.rect_alpha} out of range [-1, 1]. Clamping.'
            )
            self.rect_alpha = max(-1.0, min(1.0, self.rect_alpha))

        if not self.publish_raw and not self.publish_rectified:
            raise RuntimeError('At least one of publish_raw/publish_rectified must be true.')
        if self.publish_rgb_rectified and not self.publish_rectified:
            self.get_logger().warn('publish_rgb_rectified=true but publish_rectified=false. Disabling RGB rectified output.')
            self.publish_rgb_rectified = False

        reliability_raw = str(self.get_parameter('qos_reliability').value).strip().lower()
        if reliability_raw not in ('reliable', 'best_effort'):
            self.get_logger().warn(
                f"Unknown qos_reliability '{reliability_raw}'. Using 'reliable'."
            )
            reliability_raw = 'reliable'
        self.qos_reliability = reliability_raw

        self.left_frame_id = str(self.get_parameter('left_frame_id').value).strip()
        self.right_frame_id = str(self.get_parameter('right_frame_id').value).strip()

        self.topic_prefix = str(self.get_parameter('topic_prefix').value).strip()
        if not self.topic_prefix:
            self.topic_prefix = '/zed/zed_node'
        if not self.topic_prefix.startswith('/'):
            self.topic_prefix = '/' + self.topic_prefix
        self.topic_prefix = self.topic_prefix.rstrip('/')

        self.calibration_file = str(self.get_parameter('calibration_file').value).strip()
        self.override_param_calibration_file = str(
            self.get_parameter('general.optional_opencv_calibration_file').value
        ).strip()
        self.override_yaml_path = str(self.get_parameter('ros_params_override_path').value).strip()

        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise RuntimeError('publish_rate_hz must be > 0.0')
        self.period_s = 1.0 / rate

        if not self.image_folder or not os.path.isdir(self.image_folder):
            raise RuntimeError(f'image_folder is not a valid directory: {self.image_folder}')

        self.image_pairs = self._discover_pairs(self.image_folder, self.left_pattern)
        self.frames = self._load_frames(self.image_pairs)
        if not self.frames:
            raise RuntimeError('No valid stereo frame pairs could be loaded.')

        left_h, left_w = self.frames[0][0].shape[:2]

        self.raw_left_info_template = self._default_camera_info(left_w, left_h)
        self.raw_right_info_template = self._default_camera_info(left_w, left_h)
        self.rect_left_info_template = self._default_camera_info(left_w, left_h)
        self.rect_right_info_template = self._default_camera_info(left_w, left_h)

        self.left_rect_maps: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.right_rect_maps: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.rectification_ready = False
        self._warned_no_rectification = False

        calib_path = self._resolve_calibration_path(
            self.calibration_file,
            self.override_param_calibration_file,
            self.override_yaml_path,
        )
        if calib_path:
            self._load_calibration_and_prepare_rectification(calib_path, left_w, left_h)
        else:
            self.get_logger().warn(
                'No calibration file found. Rectified topics will publish raw images. '
                'Pass calibration_file or ros_params_override_path.'
            )

        self.left_raw_image_topic = f'{self.topic_prefix}/left_raw/image_raw_color'
        self.right_raw_image_topic = f'{self.topic_prefix}/right_raw/image_raw_color'
        self.left_raw_info_topic = f'{self.topic_prefix}/left_raw/camera_info'
        self.right_raw_info_topic = f'{self.topic_prefix}/right_raw/camera_info'

        self.left_rect_image_topic = f'{self.topic_prefix}/left/image_rect_color'
        self.right_rect_image_topic = f'{self.topic_prefix}/right/image_rect_color'
        self.rgb_rect_image_topic = f'{self.topic_prefix}/rgb/image_rect_color'

        self.left_rect_info_topic = f'{self.topic_prefix}/left/camera_info'
        self.right_rect_info_topic = f'{self.topic_prefix}/right/camera_info'
        self.rgb_rect_info_topic = f'{self.topic_prefix}/rgb/camera_info'

        self.pub_qos = self._build_qos_profile(self.qos_reliability)

        self.pub_left_raw_image = None
        self.pub_right_raw_image = None
        self.pub_left_raw_info = None
        self.pub_right_raw_info = None

        self.pub_left_rect_image = None
        self.pub_right_rect_image = None
        self.pub_rgb_rect_image = None

        self.pub_left_rect_info = None
        self.pub_right_rect_info = None
        self.pub_rgb_rect_info = None

        if self.publish_raw:
            self.pub_left_raw_image = self.create_publisher(Image, self.left_raw_image_topic, self.pub_qos)
            self.pub_right_raw_image = self.create_publisher(Image, self.right_raw_image_topic, self.pub_qos)
            if self.publish_camera_info:
                self.pub_left_raw_info = self.create_publisher(CameraInfo, self.left_raw_info_topic, self.pub_qos)
                self.pub_right_raw_info = self.create_publisher(CameraInfo, self.right_raw_info_topic, self.pub_qos)

        if self.publish_rectified:
            self.pub_left_rect_image = self.create_publisher(Image, self.left_rect_image_topic, self.pub_qos)
            self.pub_right_rect_image = self.create_publisher(Image, self.right_rect_image_topic, self.pub_qos)
            if self.publish_rgb_rectified:
                self.pub_rgb_rect_image = self.create_publisher(Image, self.rgb_rect_image_topic, self.pub_qos)

            if self.publish_camera_info:
                self.pub_left_rect_info = self.create_publisher(CameraInfo, self.left_rect_info_topic, self.pub_qos)
                self.pub_right_rect_info = self.create_publisher(CameraInfo, self.right_rect_info_topic, self.pub_qos)
                if self.publish_rgb_rectified:
                    self.pub_rgb_rect_info = self.create_publisher(CameraInfo, self.rgb_rect_info_topic, self.pub_qos)

        self.index = 0
        self.timer = self.create_timer(self.period_s, self._publish_next)

        self.get_logger().info(f'Loaded {len(self.frames)} stereo pairs from: {self.image_folder}')
        self.get_logger().info(f'Publishing @ {rate:.2f} Hz. loop={self.loop}')
        self.get_logger().info(
            f'Outputs: raw={self.publish_raw}, rectified={self.publish_rectified}, '
            f'rgb_rectified={self.publish_rgb_rectified}, camera_info={self.publish_camera_info}'
        )
        self.get_logger().info(f'QoS reliability: {self.qos_reliability}')
        self.get_logger().info(f'Rectification alpha: {self.rect_alpha}')

        if self.publish_raw:
            self.get_logger().info(f'Left raw image topic:  {self.left_raw_image_topic}')
            self.get_logger().info(f'Right raw image topic: {self.right_raw_image_topic}')

        if self.publish_rectified:
            self.get_logger().info(f'Left rect image topic:  {self.left_rect_image_topic}')
            self.get_logger().info(f'Right rect image topic: {self.right_rect_image_topic}')
            if self.publish_rgb_rectified:
                self.get_logger().info(f'RGB rect image topic:  {self.rgb_rect_image_topic}')

        if self.publish_camera_info and self.publish_raw:
            self.get_logger().info(f'Left raw info topic:   {self.left_raw_info_topic}')
            self.get_logger().info(f'Right raw info topic:  {self.right_raw_info_topic}')

        if self.publish_camera_info and self.publish_rectified:
            self.get_logger().info(f'Left rect info topic:  {self.left_rect_info_topic}')
            self.get_logger().info(f'Right rect info topic: {self.right_rect_info_topic}')
            if self.publish_rgb_rectified:
                self.get_logger().info(f'RGB rect info topic:   {self.rgb_rect_info_topic}')

    def _build_qos_profile(self, reliability: str) -> QoSProfile:
        policy = (
            QoSReliabilityPolicy.RELIABLE
            if reliability == 'reliable'
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=policy,
        )

    def _discover_pairs(self, folder: str, left_pattern: str) -> List[Tuple[str, str]]:
        left_glob = os.path.join(folder, left_pattern)
        left_files = sorted(glob.glob(left_glob), key=_natural_key)
        if not left_files:
            raise RuntimeError(f'No left images found with pattern: {left_glob}')

        pairs: List[Tuple[str, str]] = []
        for left_path in left_files:
            right_path = self._infer_right_path(left_path)
            if right_path and os.path.isfile(right_path):
                pairs.append((left_path, right_path))
            else:
                self.get_logger().warn(f'Skipping {left_path}: right image not found')

        if not pairs:
            raise RuntimeError('No complete left/right pairs were found.')

        return pairs

    def _infer_right_path(self, left_path: str) -> str:
        base = os.path.basename(left_path)
        candidates = []

        if '_left_' in base:
            candidates.append(base.replace('_left_', '_right_'))
        if 'left' in base:
            candidates.append(base.replace('left', 'right', 1))

        for candidate in candidates:
            path = os.path.join(self.image_folder, candidate)
            if os.path.isfile(path):
                return path

        return ''

    def _load_frames(self, pairs: List[Tuple[str, str]]) -> List[Tuple[np.ndarray, np.ndarray, str]]:
        frames: List[Tuple[np.ndarray, np.ndarray, str]] = []

        for left_path, right_path in pairs:
            left_img = cv2.imread(left_path, cv2.IMREAD_COLOR)
            right_img = cv2.imread(right_path, cv2.IMREAD_COLOR)

            if left_img is None or right_img is None:
                self.get_logger().warn(f'Skipping unreadable pair: {left_path} | {right_path}')
                continue

            if left_img.shape != right_img.shape:
                self.get_logger().warn(
                    f'Skipping size-mismatched pair: {left_path} {left_img.shape} vs {right_path} {right_img.shape}'
                )
                continue

            frames.append((left_img, right_img, os.path.basename(left_path)))

        return frames

    def _resolve_calibration_path(
        self,
        explicit_calibration_path: str,
        override_calibration_path: str,
        override_yaml_path: str,
    ) -> str:
        for candidate in (explicit_calibration_path, override_calibration_path):
            resolved = self._resolve_one_path(candidate, override_yaml_path)
            if resolved:
                return resolved
        return ''

    def _resolve_one_path(self, path_value: str, override_yaml_path: str) -> str:
        if not path_value:
            return ''

        expanded = os.path.abspath(os.path.expanduser(os.path.expandvars(path_value)))
        if os.path.isfile(expanded):
            return expanded

        if override_yaml_path:
            override_path = os.path.abspath(os.path.expanduser(os.path.expandvars(override_yaml_path)))
            override_dir = os.path.dirname(override_path)
            basename = os.path.basename(path_value)
            if basename:
                candidate = os.path.join(override_dir, basename)
                if os.path.isfile(candidate):
                    return os.path.abspath(candidate)

        return ''

    def _load_calibration_and_prepare_rectification(self, calib_path: str, img_w: int, img_h: int) -> None:
        fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise RuntimeError(f'Cannot open calibration file: {calib_path}')

        def mat(name: str) -> Optional[np.ndarray]:
            node = fs.getNode(name)
            if node.empty():
                return None
            value = node.mat()
            if value is None:
                return None
            return np.asarray(value, dtype=np.float64)

        k_left = _mat_3x3_or_none(mat('K_LEFT'))
        d_left = mat('D_LEFT')
        k_right = _mat_3x3_or_none(mat('K_RIGHT'))
        d_right = mat('D_RIGHT')

        r_left = _mat_3x3_or_none(mat('R_LEFT'))
        p_left = _mat_3x4_or_none(mat('P_LEFT'))
        r_right = _mat_3x3_or_none(mat('R_RIGHT'))
        p_right = _mat_3x4_or_none(mat('P_RIGHT'))

        stereo_r_raw = mat('R')
        stereo_t = _vector3_or_none(mat('T'))

        width, height = self._read_size(fs)
        fs.release()

        if k_left is None or k_right is None:
            raise RuntimeError(f'Missing K_LEFT/K_RIGHT in calibration file: {calib_path}')

        if d_left is None:
            d_left = np.zeros((1, 5), dtype=np.float64)
        if d_right is None:
            d_right = np.zeros((1, 5), dtype=np.float64)

        if width is not None and height is not None and (width != img_w or height != img_h):
            self.get_logger().warn(
                f'Calibration size ({width}x{height}) != image size ({img_w}x{img_h}). '
                'Using image size for rectification maps.'
            )

        raw_r_identity = np.eye(3, dtype=np.float64)
        raw_p_left = np.hstack((k_left, np.zeros((3, 1), dtype=np.float64)))
        raw_p_right = np.hstack((k_right, np.zeros((3, 1), dtype=np.float64)))

        self._fill_camera_info(self.raw_left_info_template, img_w, img_h, k_left, d_left, raw_r_identity, raw_p_left)
        self._fill_camera_info(self.raw_right_info_template, img_w, img_h, k_right, d_right, raw_r_identity, raw_p_right)

        rect_r1, rect_r2, rect_p1, rect_p2 = self._compute_rectification_matrices(
            img_w,
            img_h,
            k_left,
            d_left,
            k_right,
            d_right,
            r_left,
            p_left,
            r_right,
            p_right,
            stereo_r_raw,
            stereo_t,
            self.rect_alpha,
        )

        rect_k_left = rect_p1[:3, :3]
        rect_k_right = rect_p2[:3, :3]

        zero_dist = np.zeros((1, 5), dtype=np.float64)
        self._fill_camera_info(
            self.rect_left_info_template,
            img_w,
            img_h,
            rect_k_left,
            zero_dist,
            rect_r1,
            rect_p1,
        )
        self._fill_camera_info(
            self.rect_right_info_template,
            img_w,
            img_h,
            rect_k_right,
            zero_dist,
            rect_r2,
            rect_p2,
        )

        self.left_rect_maps = cv2.initUndistortRectifyMap(
            k_left,
            d_left,
            rect_r1,
            rect_k_left,
            (img_w, img_h),
            cv2.CV_32FC1,
        )
        self.right_rect_maps = cv2.initUndistortRectifyMap(
            k_right,
            d_right,
            rect_r2,
            rect_k_right,
            (img_w, img_h),
            cv2.CV_32FC1,
        )
        self.rectification_ready = True

        self.get_logger().info(f'Loaded calibration from: {calib_path}')

    def _compute_rectification_matrices(
        self,
        img_w: int,
        img_h: int,
        k_left: np.ndarray,
        d_left: np.ndarray,
        k_right: np.ndarray,
        d_right: np.ndarray,
        r_left: Optional[np.ndarray],
        p_left: Optional[np.ndarray],
        r_right: Optional[np.ndarray],
        p_right: Optional[np.ndarray],
        stereo_r_raw: Optional[np.ndarray],
        stereo_t: Optional[np.ndarray],
        rect_alpha: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if (
            r_left is not None
            and p_left is not None
            and r_right is not None
            and p_right is not None
        ):
            return r_left, r_right, p_left, p_right

        stereo_r = None
        if stereo_r_raw is not None:
            if stereo_r_raw.size == 9:
                stereo_r = stereo_r_raw.reshape(3, 3)
            elif stereo_r_raw.size >= 3:
                stereo_r, _ = cv2.Rodrigues(stereo_r_raw.reshape(-1)[:3].reshape(3, 1))

        if stereo_r is not None and stereo_t is not None:
            try:
                rect_r1, rect_r2, rect_p1, rect_p2, _, _, _ = cv2.stereoRectify(
                    k_left,
                    d_left,
                    k_right,
                    d_right,
                    (img_w, img_h),
                    stereo_r,
                    stereo_t,
                    flags=cv2.CALIB_ZERO_DISPARITY,
                    alpha=rect_alpha,
                )
                return rect_r1, rect_r2, rect_p1, rect_p2
            except cv2.error as err:
                self.get_logger().warn(
                    f'stereoRectify failed ({err}). Falling back to monocular undistortion.'
                )

        self.get_logger().warn(
            'No full stereo extrinsics available (R/T or R_*/P_*). '
            'Rectified topics will use monocular undistortion only.'
        )
        identity = np.eye(3, dtype=np.float64)
        mono_p_left = np.hstack((k_left, np.zeros((3, 1), dtype=np.float64)))
        mono_p_right = np.hstack((k_right, np.zeros((3, 1), dtype=np.float64)))
        return identity, identity, mono_p_left, mono_p_right

    def _read_size(self, fs: cv2.FileStorage) -> Tuple[Optional[int], Optional[int]]:
        size = fs.getNode('Size')
        if not size.empty():
            if hasattr(size, 'isSeq') and size.isSeq() and size.size() >= 2:
                try:
                    return int(size.at(0).real()), int(size.at(1).real())
                except cv2.error:
                    pass

            if hasattr(size, 'isString') and size.isString():
                size_text = size.string()
                parsed_numbers = re.findall(r'-?\d+(?:\.\d+)?', size_text)
                if len(parsed_numbers) >= 2:
                    return int(float(parsed_numbers[0])), int(float(parsed_numbers[1]))

            if hasattr(size, 'isMap') and size.isMap():
                try:
                    size_mat = size.mat()
                except cv2.error:
                    size_mat = None
                if size_mat is not None:
                    arr = np.asarray(size_mat, dtype=np.float64).reshape(-1)
                    if arr.size >= 2:
                        return int(arr[0]), int(arr[1])

        width_node = fs.getNode('WIDTH')
        height_node = fs.getNode('HEIGHT')
        if not width_node.empty() and not height_node.empty():
            try:
                return int(width_node.real()), int(height_node.real())
            except cv2.error:
                pass

        return None, None

    def _default_camera_info(self, width: int, height: int) -> CameraInfo:
        info = CameraInfo()
        info.width = int(width)
        info.height = int(height)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        info.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        info.p = [
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info

    def _fill_camera_info(
        self,
        info: CameraInfo,
        width: int,
        height: int,
        k: np.ndarray,
        d: Optional[np.ndarray],
        r: np.ndarray,
        p: np.ndarray,
    ) -> None:
        info.width = int(width)
        info.height = int(height)

        info.k = [float(v) for v in np.asarray(k, dtype=np.float64).reshape(-1)]
        info.r = [float(v) for v in np.asarray(r, dtype=np.float64).reshape(-1)]
        info.p = [float(v) for v in np.asarray(p, dtype=np.float64).reshape(-1)]

        if d is None:
            info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info.distortion_model = 'plumb_bob'
        else:
            distortion = [float(v) for v in np.asarray(d, dtype=np.float64).reshape(-1)]
            info.d = distortion
            info.distortion_model = 'rational_polynomial' if len(distortion) >= 8 else 'plumb_bob'

    def _clone_camera_info(self, template: CameraInfo, stamp, frame_id: str) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id

        info.width = template.width
        info.height = template.height
        info.distortion_model = template.distortion_model
        info.d = list(template.d)
        info.k = list(template.k)
        info.r = list(template.r)
        info.p = list(template.p)
        info.binning_x = template.binning_x
        info.binning_y = template.binning_y
        info.roi = template.roi

        return info

    def _to_msg(self, image: np.ndarray, stamp, frame_id: str) -> Image:
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        return msg

    def _rectify_pair(self, left_img: np.ndarray, right_img: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if not self.rectification_ready or self.left_rect_maps is None or self.right_rect_maps is None:
            if not self._warned_no_rectification and self.publish_rectified:
                self.get_logger().warn('Publishing rectified topics without rectification maps (raw passthrough).')
                self._warned_no_rectification = True
            return left_img, right_img

        left_map1, left_map2 = self.left_rect_maps
        right_map1, right_map2 = self.right_rect_maps

        left_rect = cv2.remap(
            left_img,
            left_map1,
            left_map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        right_rect = cv2.remap(
            right_img,
            right_map1,
            right_map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        return left_rect, right_rect

    def _publish_next(self) -> None:
        left_img, right_img, frame_name = self.frames[self.index]

        stamp = self.get_clock().now().to_msg()

        rect_left_img = None
        rect_right_img = None
        if self.publish_rectified:
            rect_left_img, rect_right_img = self._rectify_pair(left_img, right_img)

        if self.publish_raw and self.pub_left_raw_image and self.pub_right_raw_image:
            self.pub_left_raw_image.publish(self._to_msg(left_img, stamp, self.left_frame_id))
            self.pub_right_raw_image.publish(self._to_msg(right_img, stamp, self.right_frame_id))

            if self.publish_camera_info and self.pub_left_raw_info and self.pub_right_raw_info:
                self.pub_left_raw_info.publish(self._clone_camera_info(self.raw_left_info_template, stamp, self.left_frame_id))
                self.pub_right_raw_info.publish(
                    self._clone_camera_info(self.raw_right_info_template, stamp, self.right_frame_id)
                )

        if self.publish_rectified and self.pub_left_rect_image and self.pub_right_rect_image:
            assert rect_left_img is not None and rect_right_img is not None

            self.pub_left_rect_image.publish(self._to_msg(rect_left_img, stamp, self.left_frame_id))
            self.pub_right_rect_image.publish(self._to_msg(rect_right_img, stamp, self.right_frame_id))

            if self.publish_rgb_rectified and self.pub_rgb_rect_image:
                self.pub_rgb_rect_image.publish(self._to_msg(rect_left_img, stamp, self.left_frame_id))

            if self.publish_camera_info and self.pub_left_rect_info and self.pub_right_rect_info:
                self.pub_left_rect_info.publish(
                    self._clone_camera_info(self.rect_left_info_template, stamp, self.left_frame_id)
                )
                self.pub_right_rect_info.publish(
                    self._clone_camera_info(self.rect_right_info_template, stamp, self.right_frame_id)
                )
                if self.publish_rgb_rectified and self.pub_rgb_rect_info:
                    self.pub_rgb_rect_info.publish(
                        self._clone_camera_info(self.rect_left_info_template, stamp, self.left_frame_id)
                    )

        self.index += 1
        if self.index >= len(self.frames):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info('Published all pairs once. Timer stopped.')
                self.timer.cancel()

        if self.index == 0:
            self.get_logger().debug(f'Looped image sequence after frame {frame_name}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoImageFolderPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
