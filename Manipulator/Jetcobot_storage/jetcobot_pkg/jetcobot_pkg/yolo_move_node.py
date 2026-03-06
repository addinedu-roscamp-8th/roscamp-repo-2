#!/usr/bin/env python3
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from smartfactory_interfaces.msg import YoloPose
from smartfactory_interfaces.srv import CoordsAngles

from jetcobot_pkg.utils.camera_utils import (
    T_to_pose,
    mycobot_coords_to_T_b2g,
    quat_xyzw_to_rvec,
    rvec_tvec_to_T,
)
from jetcobot_pkg.utils.cobot_utils import (
    pose_mm_to_xyz_quat,
    quat_normalize,
    quat_to_rotmat,
    rotmat_to_euler_intrinsic_ZYX_deg,
)
from jetcobot_pkg.utils.jetcobot_action_client import PickClient


T_G2C = np.array([
    [0.7071, 0.7071, 0.0, -0.03394],
    [-0.7071, 0.7071, 0.0, -0.03394],
    [0.0, 0.0, 1.0, 0.02700],
    [0.0, 0.0, 0.0, 1.0],
], dtype=np.float64)
T_G2C_MM = T_G2C.copy()
T_G2C_MM[:3, 3] *= 1000.0

USE_FIXED_T_B2G = False
HOME_COORDS = [-61.2, -178.8, 254.8, 177.26, -1.74, 135.9] 
T_B2G_FIXED_MM = mycobot_coords_to_T_b2g(HOME_COORDS)

FILTER_BUF_N = 30
MIN_FILTER_SAMPLES_DEFAULT = max(3, FILTER_BUF_N // 2)

def average_quaternions_xyzw(q_list: list[np.ndarray]) -> np.ndarray | None:
    if len(q_list) == 0:
        return None
    ref = q_list[0]
    qs = []
    for q in q_list:
        qs.append(-q if np.dot(q, ref) < 0 else q)
    q_mean = np.mean(np.stack(qs, axis=0), axis=0)
    return q_mean / (np.linalg.norm(q_mean) + 1e-12)


def filter_pose_from_buffers(tbuf: list[np.ndarray], qbuf: list[np.ndarray]):
    t_filt = np.median(np.stack(tbuf, axis=0), axis=0)
    q_filt = average_quaternions_xyzw(qbuf)
    return t_filt, q_filt


class YoloMoveNode(Node):
    def __init__(self):
        super().__init__("yolo_move_node")

        self.declare_parameter("topic_yolo_pose", "/jetcobot/storage/camera/yolo_pose")
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("pick_z_offset_mm", 0.0)
        self.declare_parameter("safe_pick", True)
        self.declare_parameter("lost_timeout_sec", 1.0)
        self.declare_parameter("topic_pick_trigger", "/jetcobot/storage/yolo_pick_trigger")
        self.declare_parameter("min_filter_samples", MIN_FILTER_SAMPLES_DEFAULT)
        self.declare_parameter("wait_log_interval_sec", 1.0)

        self.topic_yolo_pose = str(self.get_parameter("topic_yolo_pose").value)
        self.tick_hz = max(1.0, float(self.get_parameter("tick_hz").value))
        self.pick_z_offset_mm = float(self.get_parameter("pick_z_offset_mm").value)
        self.safe_pick = bool(self.get_parameter("safe_pick").value)
        self.lost_timeout_sec = max(0.1, float(self.get_parameter("lost_timeout_sec").value))
        self.topic_pick_trigger = str(self.get_parameter("topic_pick_trigger").value)
        self.min_filter_samples = max(1, int(self.get_parameter("min_filter_samples").value))
        self.wait_log_interval_sec = max(0.1, float(self.get_parameter("wait_log_interval_sec").value))

        self.state = "IDLE"
        self.parts: dict[int, dict] = {}
        self.pick_requested = False
        self.pick_coords = None
        self.target_instance_id = None
        self._last_wait_log_time = 0.0

        self.angles_coords = None
        self.cli_ang_coord = self.create_client(CoordsAngles, "get_coords_angles")
        self.req = CoordsAngles.Request()

        self.sub_yolo = self.create_subscription(YoloPose, self.topic_yolo_pose, self.cb_yolo_pose, 10)
        self.sub_pick_trigger = self.create_subscription(Bool, self.topic_pick_trigger, self.cb_pick_trigger, 10)

        self.pick_cli = PickClient(self, action_name="/pick")

        self.timer = self.create_timer(1.0 / self.tick_hz, self.tick)

        self.get_logger().info("✅ YoloMoveNode started")
        self.get_logger().info(f"- subscribe topic: {self.topic_yolo_pose}")
        self.get_logger().info(f"- pick trigger topic: {self.topic_pick_trigger} (Bool)")
        self.get_logger().info(f"- min_filter_samples: {self.min_filter_samples}")

    def future_callback(self, future):
        try:
            response = future.result()
            self.angles_coords = response.coords_angles
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def get_T_b2c_mm(self) -> np.ndarray:
        if USE_FIXED_T_B2G:
            T_b2g_mm = T_B2G_FIXED_MM
        else:
            self.req.type = 1
            future = self.cli_ang_coord.call_async(self.req)
            future.add_done_callback(self.future_callback)

            if self.angles_coords is None:
                T_b2g_mm = T_B2G_FIXED_MM
            else:
                T_b2g_mm = mycobot_coords_to_T_b2g(self.angles_coords)

        return T_b2g_mm @ T_G2C_MM

    def _ensure_entry(self, instance_id: int):
        if instance_id in self.parts:
            return
        self.parts[instance_id] = {
            "part_class": "",
            "instance_id": int(instance_id),
            "pose_base": None,
            "tbuf_cam": [],
            "qbuf_cam": [],
            "last_seen": time.time(),
        }

    def _reset_lost(self, d: dict):
        d["pose_base"] = None
        d["tbuf_cam"].clear()
        d["qbuf_cam"].clear()

    def cb_pick_trigger(self, msg: Bool):
        if not bool(msg.data):
            return
        self.pick_requested = True
        self.get_logger().info("Bool trigger received -> pick request queued")

    def cb_yolo_pose(self, msg: YoloPose):
        now_time = time.time()
        instance_id = int(msg.instance_id)
        self._ensure_entry(instance_id)
        d = self.parts[instance_id]

        d["part_class"] = str(msg.part_class)
        d["last_seen"] = now_time

        tvec_raw = np.array([
            float(msg.pose.position.x) * 1000.0,
            float(msg.pose.position.y) * 1000.0,
            float(msg.pose.position.z) * 1000.0,
        ], dtype=np.float64)
        q_raw = np.array([
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        ], dtype=np.float64)
        if np.linalg.norm(q_raw) < 1e-12:
            return

        d["tbuf_cam"].append(tvec_raw.copy())
        d["qbuf_cam"].append(q_raw / (np.linalg.norm(q_raw) + 1e-12))
        if len(d["tbuf_cam"]) > FILTER_BUF_N:
            d["tbuf_cam"].pop(0)
            d["qbuf_cam"].pop(0)

        if len(d["tbuf_cam"]) < self.min_filter_samples:
            return

        t_filt_mm, q_filt = filter_pose_from_buffers(d["tbuf_cam"], d["qbuf_cam"])
        if q_filt is None:
            return
        rvec_filt = quat_xyzw_to_rvec(q_filt)

        T_b2c_mm = self.get_T_b2c_mm()
        T_c2t_filt = rvec_tvec_to_T(rvec_filt, t_filt_mm)
        T_b2t_filt = T_b2c_mm @ T_c2t_filt

        d["pose_base"] = T_to_pose(T_b2t_filt)

    def _cleanup_lost_parts(self):
        now = time.time()
        for d in self.parts.values():
            if now - float(d["last_seen"]) > self.lost_timeout_sec:
                self._reset_lost(d)

    def _normalize_vec(self, v: np.ndarray):
        n = float(np.linalg.norm(v))
        if n < 1e-12:
            return None
        return v / n

    def _fix_rotmat_z_up_keep_xy(self, R_target: np.ndarray) -> np.ndarray:
        z_base = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        x_t = R_target[:, 0].astype(np.float64)
        y_t = R_target[:, 1].astype(np.float64)

        x_proj = x_t - np.dot(x_t, z_base) * z_base
        x_new = self._normalize_vec(x_proj)

        if x_new is None:
            y_proj = y_t - np.dot(y_t, z_base) * z_base
            y_new = self._normalize_vec(y_proj)
            if y_new is None:
                x_new = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                x_new = self._normalize_vec(np.cross(y_new, z_base))
                if x_new is None:
                    x_new = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        y_new = self._normalize_vec(np.cross(z_base, x_new))
        if y_new is None:
            y_new = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            x_new = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        R_new = np.column_stack((x_new, y_new, z_base))

        # Final adjustment: rotate target frame -90 deg around base Z axis.
        rz90 = np.array([
            [0.0, 1.0, 0.0],
            [-1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0],
        ], dtype=np.float64)
        return rz90 @ R_new

    def _pose_to_pick_coords_mm(self, pose_mm):
        xyz, q = pose_mm_to_xyz_quat(pose_mm)
        q = quat_normalize(q)
        R_target = quat_to_rotmat(q)
        R_pick = self._fix_rotmat_z_up_keep_xy(R_target)
        xyz_pick = np.array([
            float(xyz[0]),
            float(xyz[1]),
            float(xyz[2] + self.pick_z_offset_mm),
        ], dtype=np.float64)
        rx, ry, rz = rotmat_to_euler_intrinsic_ZYX_deg(R_pick)
        coords6 = [
            float(xyz_pick[0]),
            float(xyz_pick[1]),
            float(xyz_pick[2]),
            float(rx),
            float(ry),
            float(rz),
        ]
        return coords6, R_pick

    def _select_target_for_pick(self):
        candidates = [
            d for d in self.parts.values()
            if d["pose_base"] is not None
        ]
        if not candidates:
            return None
        candidates.sort(key=lambda x: int(x["instance_id"]))
        return candidates[0]

    def _pick_candidate_stats(self):
        total = len(self.parts)
        filtered = 0
        min_buffer_needed = 0
        for d in self.parts.values():
            if d["pose_base"] is not None:
                filtered += 1
            if len(d["tbuf_cam"]) < self.min_filter_samples:
                min_buffer_needed += 1
        return filtered, total, min_buffer_needed

    def _is_action_done(self, done):
        if done is None:
            return False
        success, message = done
        if success:
            self.get_logger().info(f"[TASK DONE] success={success} msg={message}")
            return True
        self.get_logger().error(f"[TASK FAIL] success={success} msg={message}")
        self.state = "IDLE"
        self.pick_requested = False
        return False

    def tick(self):
        self._cleanup_lost_parts()

        if self.state == "IDLE":
            if not self.pick_requested:
                return

            target = self._select_target_for_pick()
            if target is None:
                now = time.time()
                if now - self._last_wait_log_time >= self.wait_log_interval_sec:
                    filtered, total, need_buffer = self._pick_candidate_stats()
                    self.get_logger().warn(
                        "No filtered object to pick yet (waiting for pose_base). "
                        f"filtered={filtered} total={total} need_buffer={need_buffer}"
                    )
                    self._last_wait_log_time = now
                return

            self.target_instance_id = int(target["instance_id"])
            self.pick_coords, pick_rotmat = self._pose_to_pick_coords_mm(target["pose_base"])

            self.get_logger().info(
                f"[PICK COORDS BASE] instance={self.target_instance_id} class={target['part_class']} "
                f"coords_mm_deg={self.pick_coords} "
                f"R=\n{np.array2string(pick_rotmat, precision=4, suppress_small=True)}"
            )

            if not self.pick_cli.send_goal(self.pick_coords, self.safe_pick):
                self.get_logger().error("Pick send_goal failed")
                return

            self.state = "EXEC_PICK"
            self.get_logger().info(
                f"[PICK START] instance={self.target_instance_id} class={target['part_class']}"
            )
            return

        if self.state == "EXEC_PICK":
            if not self._is_action_done(self.pick_cli.action_done()):
                return

            self.get_logger().info(
                f"[TASK COMPLETE] instance={self.target_instance_id} pick={self.pick_coords}"
            )
            self.state = "IDLE"
            self.pick_requested = False
            self.pick_coords = None
            self.target_instance_id = None

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
