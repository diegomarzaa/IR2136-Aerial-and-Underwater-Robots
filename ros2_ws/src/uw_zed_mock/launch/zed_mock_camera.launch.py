import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_IMAGE_FOLDER = '/home/diego/Documents/02-Universidad/Cirtesu/ZedCalibration/data/captures/underwater_images'


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 't', 'yes', 'y', 'on')


def _safe_rate(value: str) -> float:
    try:
        parsed = float(value)
        return parsed if parsed > 0.0 else 5.0
    except ValueError:
        return 5.0


def _safe_alpha(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError:
        return 0.0
    return max(-1.0, min(1.0, parsed))


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context).strip() or 'zed'
    camera_model = LaunchConfiguration('camera_model').perform(context).strip()
    namespace = LaunchConfiguration('namespace').perform(context).strip()
    node_name = LaunchConfiguration('node_name').perform(context).strip() or 'zed_node'

    ros_params_override_path = LaunchConfiguration('ros_params_override_path').perform(context).strip()
    image_folder = LaunchConfiguration('image_folder').perform(context).strip()
    left_pattern = LaunchConfiguration('left_pattern').perform(context).strip()
    publish_rate_hz = _safe_rate(LaunchConfiguration('publish_rate_hz').perform(context))
    loop = _as_bool(LaunchConfiguration('loop').perform(context))
    publish_raw = _as_bool(LaunchConfiguration('publish_raw').perform(context))
    publish_rectified = _as_bool(LaunchConfiguration('publish_rectified').perform(context))
    publish_rgb_rectified = _as_bool(LaunchConfiguration('publish_rgb_rectified').perform(context))
    publish_camera_info = _as_bool(LaunchConfiguration('publish_camera_info').perform(context))
    qos_reliability = LaunchConfiguration('qos_reliability').perform(context).strip().lower()
    rect_alpha = _safe_alpha(LaunchConfiguration('rect_alpha').perform(context))

    left_frame_id = LaunchConfiguration('left_frame_id').perform(context).strip()
    right_frame_id = LaunchConfiguration('right_frame_id').perform(context).strip()
    calibration_file = LaunchConfiguration('calibration_file').perform(context).strip()

    namespace_value = namespace.lstrip('/') if namespace else camera_name
    topic_prefix = f'/{namespace_value}/{node_name}'

    node_parameters = [
        {
            'image_folder': image_folder,
            'left_pattern': left_pattern,
            'publish_rate_hz': publish_rate_hz,
            'loop': loop,
            'publish_raw': publish_raw,
            'publish_rectified': publish_rectified,
            'publish_rgb_rectified': publish_rgb_rectified,
            'publish_camera_info': publish_camera_info,
            'qos_reliability': qos_reliability,
            'rect_alpha': rect_alpha,
            'topic_prefix': topic_prefix,
            'left_frame_id': left_frame_id,
            'right_frame_id': right_frame_id,
            'calibration_file': calibration_file,
            'ros_params_override_path': ros_params_override_path,
        }
    ]

    launch_entities = [
        LogInfo(msg=f'[uw_zed_mock] namespace={namespace_value} node_name={node_name} topic_prefix={topic_prefix}'),
        LogInfo(msg=f'[uw_zed_mock] image_folder={image_folder} rate_hz={publish_rate_hz:.2f} loop={loop}'),
        LogInfo(msg=f"[uw_zed_mock] camera_model={camera_model or '(ignored)'}"),
        LogInfo(
            msg=(
                f'[uw_zed_mock] publish_raw={publish_raw} publish_rectified={publish_rectified} '
                f'publish_rgb_rectified={publish_rgb_rectified} qos_reliability={qos_reliability} '
                f'rect_alpha={rect_alpha}'
            )
        ),
    ]

    if ros_params_override_path:
        expanded_override = os.path.abspath(os.path.expanduser(os.path.expandvars(ros_params_override_path)))
        if os.path.isfile(expanded_override):
            node_parameters.insert(0, expanded_override)
            launch_entities.append(LogInfo(msg=f'[uw_zed_mock] Using override file: {expanded_override}'))
        else:
            launch_entities.append(
                LogInfo(
                    msg=f'[uw_zed_mock] Override file not found: {expanded_override}. '
                        'Continuing without loading that parameter file.'
                )
            )

    launch_entities.append(
        Node(
            package='uw_zed_mock',
            executable='stereo_image_folder_publisher',
            name=node_name,
            namespace=namespace_value,
            output='screen',
            parameters=node_parameters,
        )
    )

    return launch_entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='Camera name used as default namespace when namespace is empty.',
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zedxm',
            description='Compatibility argument with zed_camera.launch.py. Ignored by this mock publisher.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Node namespace. If empty, camera_name is used.',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='zed_node',
            description='Node name. Topic prefix becomes /<namespace>/<node_name>.',
        ),
        DeclareLaunchArgument(
            'ros_params_override_path',
            default_value='',
            description='Path to a ROS params YAML override file (supports /** wildcard).',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description='Optional explicit calibration path. If empty, uses override param general.optional_opencv_calibration_file.',
        ),
        DeclareLaunchArgument(
            'image_folder',
            default_value=DEFAULT_IMAGE_FOLDER,
            description='Folder with stereo pairs named like image_left_N.png and image_right_N.png.',
        ),
        DeclareLaunchArgument(
            'left_pattern',
            default_value='image_left_*.png',
            description='Glob for left images inside image_folder.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='5.0',
            description='Publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Loop image sequence forever.',
        ),
        DeclareLaunchArgument(
            'publish_raw',
            default_value='true',
            description='Publish raw stereo topics.',
        ),
        DeclareLaunchArgument(
            'publish_rectified',
            default_value='true',
            description='Publish rectified stereo topics from calibration.',
        ),
        DeclareLaunchArgument(
            'publish_rgb_rectified',
            default_value='true',
            description='Publish /rgb/image_rect_color as alias of left rectified image.',
        ),
        DeclareLaunchArgument(
            'publish_camera_info',
            default_value='true',
            description='Publish left/right camera_info topics.',
        ),
        DeclareLaunchArgument(
            'qos_reliability',
            default_value='reliable',
            description='QoS reliability: reliable or best_effort.',
        ),
        DeclareLaunchArgument(
            'rect_alpha',
            default_value='0.0',
            description='OpenCV stereoRectify alpha in [-1,1]. 0=crop/zoom, 1=keep FOV with black borders.',
        ),
        DeclareLaunchArgument(
            'left_frame_id',
            default_value='zed_left_camera_optical_frame',
            description='frame_id for left image/camera_info.',
        ),
        DeclareLaunchArgument(
            'right_frame_id',
            default_value='zed_right_camera_optical_frame',
            description='frame_id for right image/camera_info.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
