import base64
import socket
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from smartfactory_interfaces.msg import YoloPose
from ultralytics import YOLO

POSE_TOPIC = "/jetcobot/storage/camera/yolo_pose"


def load_intrinsics(calib_path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not calib_path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calib_path}")

    calib = np.load(str(calib_path))
    camera_matrix = calib["camera_matrix"].astype(np.float64)
    dist_coeffs = calib["dist_coeffs"].astype(np.float64)

    if dist_coeffs.ndim == 1:
        dist_coeffs = dist_coeffs.reshape(-1, 1)
    elif dist_coeffs.ndim == 2 and dist_coeffs.shape[0] == 1:
        dist_coeffs = dist_coeffs.reshape(-1, 1)

    return camera_matrix, dist_coeffs


def order_points_tl_tr_br_bl(pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=np.float64)
    if pts.shape != (4, 2):
        raise ValueError(f"Expected (4,2), got {pts.shape}")

    rect = np.zeros((4, 2), dtype=np.float64)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def extract_detection_candidates(
    result_obj: Any, conf_thres: float
) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    names = getattr(result_obj, "names", {})

    if getattr(result_obj, "obb", None) is not None and len(result_obj.obb) > 0:
        obb_xyxyxyxy = result_obj.obb.xyxyxyxy.cpu().numpy()
        confs = (
            result_obj.obb.conf.cpu().numpy()
            if getattr(result_obj.obb, "conf", None) is not None
            else np.ones((len(obb_xyxyxyxy),), dtype=np.float64)
        )
        clses = (
            result_obj.obb.cls.cpu().numpy().astype(int)
            if getattr(result_obj.obb, "cls", None) is not None
            else np.full((len(obb_xyxyxyxy),), -1, dtype=int)
        )

        for det_idx, (pts, conf, cls_id) in enumerate(
            zip(obb_xyxyxyxy, confs, clses)
        ):
            if float(conf) < conf_thres:
                continue
            ordered = order_points_tl_tr_br_bl(pts.reshape(4, 2))
            label = (
                names.get(int(cls_id), str(int(cls_id)))
                if isinstance(names, dict) and cls_id >= 0
                else "unknown"
            )
            candidates.append(
                {
                    "det_idx": int(det_idx),
                    "mode": "obb",
                    "conf": float(conf),
                    "cls_id": int(cls_id),
                    "label": label,
                    "img_points": ordered,
                }
            )
        return candidates

    if getattr(result_obj, "boxes", None) is not None and len(result_obj.boxes) > 0:
        xyxy = result_obj.boxes.xyxy.cpu().numpy()
        confs = (
            result_obj.boxes.conf.cpu().numpy()
            if getattr(result_obj.boxes, "conf", None) is not None
            else np.ones((len(xyxy),), dtype=np.float64)
        )
        clses = (
            result_obj.boxes.cls.cpu().numpy().astype(int)
            if getattr(result_obj.boxes, "cls", None) is not None
            else np.full((len(xyxy),), -1, dtype=int)
        )

        for det_idx, (bbox, conf, cls_id) in enumerate(zip(xyxy, confs, clses)):
            if float(conf) < conf_thres:
                continue
            x1, y1, x2, y2 = bbox.astype(np.float64)
            pts = np.array(
                [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float64
            )
            ordered = order_points_tl_tr_br_bl(pts)
            label = (
                names.get(int(cls_id), str(int(cls_id)))
                if isinstance(names, dict) and cls_id >= 0
                else "unknown"
            )
            candidates.append(
                {
                    "det_idx": int(det_idx),
                    "mode": "bbox",
                    "conf": float(conf),
                    "cls_id": int(cls_id),
                    "label": label,
                    "img_points": ordered,
                }
            )

    return candidates


def estimate_poses(
    detections: list[dict[str, Any]],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    object_width_m: float,
    object_height_m: float,
) -> list[dict[str, Any]]:
    object_points = np.array(
        [
            [-object_width_m / 2, -object_height_m / 2, 0.0],
            [object_width_m / 2, -object_height_m / 2, 0.0],
            [object_width_m / 2, object_height_m / 2, 0.0],
            [-object_width_m / 2, object_height_m / 2, 0.0],
        ],
        dtype=np.float64,
    )

    poses: list[dict[str, Any]] = []
    for det in detections:
        image_points = det["img_points"].astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            objectPoints=object_points,
            imagePoints=image_points,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            continue

        tx, ty, tz = tvec.reshape(3)
        poses.append(
            {
                **det,
                "rvec": rvec,
                "tvec": tvec,
                "xyz": (float(tx), float(ty), float(tz)),
            }
        )
    return poses


def rotation_matrix_to_quaternion(
    rot: np.ndarray,
) -> tuple[float, float, float, float]:
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rot[2, 1] - rot[1, 2]) * s
        qy = (rot[0, 2] - rot[2, 0]) * s
        qz = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s

    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm > 0:
        quat /= norm
    return (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))


def rvec_to_quaternion(rvec: np.ndarray) -> tuple[float, float, float, float]:
    rot, _ = cv2.Rodrigues(rvec)
    return rotation_matrix_to_quaternion(rot)


class YoloUdpPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_udp_pose_node")

        self.declare_parameter("host_ip", "192.168.0.52")
        self.declare_parameter("port", 9999)
        self.declare_parameter("buff_size", 65536)
        self.declare_parameter("start_message", "Hello")
        self.declare_parameter(
            "model_path", "/home/addinedu/yolo_ws/src/inference/bolts-obb.pt"
        )
        self.declare_parameter(
            "calib_path",
            "/home/addinedu/yolo_ws/src/inference/"
            "camera_calib_upgraded_2026-01-12_16-17-24.npz",
        )
        self.declare_parameter("model_task", "obb")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("object_width_m", 0.06)
        self.declare_parameter("object_height_m", 0.04)
        self.declare_parameter("topic_name", POSE_TOPIC)
        self.declare_parameter("timer_period_s", 0.01)

        self.host_ip = str(self.get_parameter("host_ip").value)
        self.port = int(self.get_parameter("port").value)
        self.buff_size = int(self.get_parameter("buff_size").value)
        self.start_message = str(self.get_parameter("start_message").value).encode()
        self.model_path = Path(str(self.get_parameter("model_path").value))
        self.calib_path = Path(str(self.get_parameter("calib_path").value))
        self.model_task = str(self.get_parameter("model_task").value)
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.object_width_m = float(self.get_parameter("object_width_m").value)
        self.object_height_m = float(self.get_parameter("object_height_m").value)
        self.topic_name = str(self.get_parameter("topic_name").value)
        self.timer_period_s = float(self.get_parameter("timer_period_s").value)

        if not self.model_path.exists():
            raise FileNotFoundError(f"Model file not found: {self.model_path}")

        self.camera_matrix, self.dist_coeffs = load_intrinsics(self.calib_path)
        self.model = YOLO(str(self.model_path))

        self.pose_pub = self.create_publisher(YoloPose, self.topic_name, 10)

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_RCVBUF, self.buff_size
        )
        self.client_socket.settimeout(0.001)
        self.client_socket.sendto(self.start_message, (self.host_ip, self.port))

        self.frame_count = 0
        self.timer = self.create_timer(self.timer_period_s, self._on_timer)

        self.get_logger().info(
            f"YOLO UDP Pose Node started -> {self.host_ip}:{self.port}, "
            f"topic: {self.topic_name}"
        )

    def _on_timer(self) -> None:
        try:
            packet, _ = self.client_socket.recvfrom(self.buff_size)
        except socket.timeout:
            return
        except OSError as exc:
            self.get_logger().error(f"UDP recv error: {exc}")
            return

        try:
            data = base64.b64decode(packet, " /")
        except Exception:
            return

        npdata = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)
        if frame is None:
            return

        result = self.model.predict(
            source=frame,
            task=self.model_task,
            conf=self.conf_thres,
            verbose=False,
        )[0]

        detections = extract_detection_candidates(result, self.conf_thres)
        poses = estimate_poses(
            detections,
            self.camera_matrix,
            self.dist_coeffs,
            self.object_width_m,
            self.object_height_m,
        )

        for pose in poses:
            msg = YoloPose()
            tx, ty, tz = pose["xyz"]
            qx, qy, qz, qw = rvec_to_quaternion(pose["rvec"])

            msg.part_class = str(pose["label"])
            msg.instance_id = int(pose["det_idx"])

            msg.pose.position.x = tx
            msg.pose.position.y = ty
            msg.pose.position.z = tz
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            self.pose_pub.publish(msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"frame={self.frame_count}, detections={len(detections)}, "
                f"pose_published={len(poses)}"
            )

    def destroy_node(self) -> bool:
        try:
            self.client_socket.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloUdpPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
