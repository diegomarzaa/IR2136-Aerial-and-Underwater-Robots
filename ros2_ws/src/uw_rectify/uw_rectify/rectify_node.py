import yaml
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def _as_mat(x):
    a = np.array(x, dtype=np.float64)
    if a.size == 9:
        return a.reshape(3, 3)
    if a.size == 12:
        return a.reshape(3, 4)
    return a


def load_zed_ocv_yaml(path: str):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open calib file: {path}")

    def mat(name):
        n = fs.getNode(name)
        if n.empty():
            raise KeyError(name)
        return n.mat()

    K_L = mat("K_LEFT")
    D_L = mat("D_LEFT")
    K_R = mat("K_RIGHT")
    D_R = mat("D_RIGHT")

    # Si existen estos, mejor (no siempre están)
    R_L = fs.getNode("R_LEFT").mat() if not fs.getNode("R_LEFT").empty() else np.eye(3, dtype=np.float64)
    P_L = fs.getNode("P_LEFT").mat() if not fs.getNode("P_LEFT").empty() else None
    R_R = fs.getNode("R_RIGHT").mat() if not fs.getNode("R_RIGHT").empty() else np.eye(3, dtype=np.float64)
    P_R = fs.getNode("P_RIGHT").mat() if not fs.getNode("P_RIGHT").empty() else None

    W = int(fs.getNode("WIDTH").real()) if not fs.getNode("WIDTH").empty() else None
    H = int(fs.getNode("HEIGHT").real()) if not fs.getNode("HEIGHT").empty() else None

    fs.release()
    return W, H, K_L, D_L, R_L, P_L, K_R, D_R, R_R, P_R


class RectifyNode(Node):
    def __init__(self):
        super().__init__("uw_rectify")

        self.bridge = CvBridge()

        self.declare_parameter("calib_yml", "")
        self.declare_parameter("left_in", "/zed/zed_node/left_raw/image_raw_color")
        self.declare_parameter("right_in", "")
        self.declare_parameter("left_out", "/uw/left_rect/image")
        self.declare_parameter("right_out", "/uw/right_rect/image")
        self.declare_parameter("force_size", "")  # e.g. "1104x621" si quieres forzar

        yml = self.get_parameter("calib_yml").get_parameter_value().string_value
        if not yml:
            raise RuntimeError("Pasa calib_yml:=/ruta/a/zed_underwater_calibration.yml")

        self.left_in = self.get_parameter("left_in").get_parameter_value().string_value
        self.right_in = self.get_parameter("right_in").get_parameter_value().string_value
        self.left_out = self.get_parameter("left_out").get_parameter_value().string_value
        self.right_out = self.get_parameter("right_out").get_parameter_value().string_value

        self.force_size = self.get_parameter("force_size").get_parameter_value().string_value.strip()

        # carga calib
        W, H, K_l, D_l, R_l, P_l, K_r, D_r, R_r, P_r = load_zed_ocv_yaml(yml)

        # el YAML debe ser consistente
        if (W, H) != (W, H):
            self.get_logger().warn(f"YAML width/height left!=right: {(W,H)} vs {(W,H)}")

        self.yaml_size = (W, H)

        self.K_l, self.D_l, self.R_l, self.P_l = K_l, D_l, R_l, P_l
        self.K_r, self.D_r, self.R_r, self.P_r = K_r, D_r, R_r, P_r

        self.pub_l = self.create_publisher(Image, self.left_out, 10)
        self.pub_r = self.create_publisher(Image, self.right_out, 10) if self.right_in else None

        self.map_l = None
        self.map_r = None
        self.last_in_size = None

        self.sub_l = self.create_subscription(Image, self.left_in, self.cb_left, qos_profile_sensor_data)
        if self.right_in:
            self.sub_r = self.create_subscription(Image, self.right_in, self.cb_right, qos_profile_sensor_data)
        else:
            self.sub_r = None

        self.get_logger().info(f"Loaded calib {yml}. YAML size={self.yaml_size}.")
        self.get_logger().info(f"Sub L: {self.left_in} -> Pub L: {self.left_out}")
        if self.right_in:
            self.get_logger().info(f"Sub R: {self.right_in} -> Pub R: {self.right_out}")

    def _desired_size(self, in_w, in_h):
        if self.force_size:
            try:
                w, h = self.force_size.lower().split("x")
                return (int(w), int(h))
            except Exception:
                self.get_logger().warn("force_size mal formado; usa '1104x621'. Ignoro.")
        return (in_w, in_h)

    def _ensure_maps(self, in_w, in_h):
        if self.last_in_size == (in_w, in_h) and self.map_l is not None and (self.map_r is not None or not self.right_in):
            return

        out_w, out_h = self._desired_size(in_w, in_h)
        self.get_logger().info(f"Building maps for input {(in_w,in_h)} -> output {(out_w,out_h)}")

        # Nota: si tu YAML fue calibrado a 960x600 y tú metes 1104x621,
        # estás probando “a lo bruto”. Para diagnóstico sirve, para calib final no.

        # New camera matrix:
        # Si P existe (3x4), usamos su parte 3x3; si no, usamos K
        newK_l = self.P_l[:3, :3] if (self.P_l is not None and self.P_l.shape == (3, 4)) else self.K_l
        newK_r = self.P_r[:3, :3] if (self.P_r is not None and self.P_r.shape == (3, 4)) else self.K_r

        self.map_l = cv2.initUndistortRectifyMap(
            self.K_l, self.D_l, self.R_l, newK_l, (out_w, out_h), cv2.CV_32FC1
        )
        if self.right_in:
            self.map_r = cv2.initUndistortRectifyMap(
                self.K_r, self.D_r, self.R_r, newK_r, (out_w, out_h), cv2.CV_32FC1
            )
        self.last_in_size = (in_w, in_h)

    def _rectify_and_publish(self, msg: Image, is_left: bool):
        # intenta respetar encoding; zed suele publicar bgra8 en RGB color
        cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        if cv.ndim == 3 and cv.shape[2] == 4:
            # BGRA -> BGR para OpenCV
            cv = cv2.cvtColor(cv, cv2.COLOR_BGRA2BGR)

        in_h, in_w = cv.shape[:2]
        self._ensure_maps(in_w, in_h)

        maps = self.map_l if is_left else self.map_r
        if maps is None:
            return

        map1, map2 = maps
        out = cv2.remap(cv, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        out_msg = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
        out_msg.header = msg.header  # mantiene stamp/frame_id
        if is_left:
            self.pub_l.publish(out_msg)
        else:
            self.pub_r.publish(out_msg)

    def cb_left(self, msg: Image):
        self._rectify_and_publish(msg, is_left=True)

    def cb_right(self, msg: Image):
        self._rectify_and_publish(msg, is_left=False)


def main():
    rclpy.init()
    node = RectifyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()