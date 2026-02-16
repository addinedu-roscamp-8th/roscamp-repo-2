#!/usr/bin/env python3
import time
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from jetcobot_interfaces.msg import Part, PartArray

from jetcobot_pkg.utils.camera_utils import (
    load_intrinsics,                 # ✅ no-arg
    solve_marker_pose_from_corners,   # ✅ rvec,tvec from 2D corners
    rvec_tvec_to_T,                  # ✅ 4x4
    T_to_pose,                       # ✅ geometry_msgs/Pose
)

# =========================================================
# ✅ 전역 설정
# =========================================================
CAM_DEVICE = "/dev/jetcocam0"
MARKER_LENGTH_M = 0.020

TOPIC_PARTS = "/parts"
FPS = 15.0

SHOW_IMSHOW = True
WINDOW_NAME = "camera_parts_filtered_onoff_with_Tg2c_spikeoff"
IMSHOW_W = 1600
IMSHOW_H = 900

# =========================================================
# ✅ Hand-Eye: gripper -> camera (여기에 넣기!)
# =========================================================
T_G2C = np.array([
    [ 0.7071,  0.7071,  0.0, -0.03394],
    [-0.7071,  0.7071,  0.0, -0.03394],
    [ 0.0,     0.0,     1.0,  0.02700],
    [ 0.0,     0.0,     0.0,  1.0    ],
], dtype=np.float64)

# =========================================================
# ✅ base -> gripper (지금은 임시 고정)
# - 로봇이 움직이는 환경이면 이 값은 로봇에서 계속 받아야 함
# - 현재는 테스트/단독 실행용
# =========================================================
T_B2G_FIXED = np.eye(4, dtype=np.float64)
USE_FIXED_T_B2G = True

# =========================================================
# ✅ ArUco detecting Parameters (전역)
# =========================================================
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

params = aruco.DetectorParameters()
params.adaptiveThreshConstant = 7
params.minMarkerPerimeterRate = 0.02
params.maxMarkerPerimeterRate = 4.0
params.polygonalApproxAccuracyRate = 0.03
params.minCornerDistanceRate = 0.05
params.minMarkerDistanceRate = 0.02
params.minOtsuStdDev = 5.0
params.perspectiveRemoveIgnoredMarginPerCell = 0.13

# =========================================================
# ✅ Filtering buffer
# =========================================================
FILTER_BUF_N = 9
MIN_FILTER_SAMPLES = max(3, FILTER_BUF_N // 2)

# =========================================================
# ✅ READY 판단 (모두 filtered dp 기반)
# =========================================================
# ✅ ON: filtered dp가 충분히 작고 안정시간 유지하면 ON
READY_ON_STABLE_DP_THRESH_M = 0.003     # 3mm 이하이면 안정
READY_ON_STABLE_TIME_SEC = 0.7          # 이 시간 이상 안정 -> ready ON
READY_ON_MIN_DETECTED_FRAMES = 8        # 감지 프레임 최소

# ✅ OFF: filtered dp가 일정 이상이면 move_counter 증가 → 연속 프레임이면 OFF
READY_OFF_MOVE_DP_THRESH_M = 0.001      # 7mm 이상이면 움직임 후보 (ON보다 크게!)
READY_OFF_DEBOUNCE_FRAMES = 5           # 3프레임 연속이면 OFF
READY_OFF_MOVE_TIME_SEC = None          # 예: 0.3 (원치 않으면 None)

# ✅ OFF FAST SPIKE: filtered dp가 매우 크면 즉시 OFF (debounce 무시)
READY_OFF_FAST_SPIKE_DP_THRESH_M = 0.012   # ✅ 12mm 이상이면 1프레임이라도 즉시 OFF

# =========================================================
# ✅ Quaternion 평균 유틸
# =========================================================
def rvec_to_quat_xyzw(rvec: np.ndarray) -> np.ndarray:
    Rm, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    qw = np.sqrt(max(0.0, 1.0 + Rm[0, 0] + Rm[1, 1] + Rm[2, 2])) / 2.0
    qx = (Rm[2, 1] - Rm[1, 2]) / (4.0 * qw + 1e-12)
    qy = (Rm[0, 2] - Rm[2, 0]) / (4.0 * qw + 1e-12)
    qz = (Rm[1, 0] - Rm[0, 1]) / (4.0 * qw + 1e-12)
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    return q / (np.linalg.norm(q) + 1e-12)


def quat_xyzw_to_rvec(q: np.ndarray) -> np.ndarray:
    q = q / (np.linalg.norm(q) + 1e-12)
    x, y, z, w = q.tolist()
    Rm = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(Rm)
    return rvec.reshape(3)


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


# =========================================================
# ✅ Overlay
# =========================================================
def draw_overlay(frame, corners_2d, mid: int, ready: bool,
                 rvec_show, tvec_show, K, dist,
                 dp_filt_mm=None, stable_t=None, move_cnt=None,
                 update_enabled=True):
    color_box = (0, 255, 0) if ready else (0, 0, 255)
    pts = corners_2d.astype(int)
    cv2.polylines(frame, [pts], True, color_box, 2)

    x, y = int(pts[0][0]), int(pts[0][1])
    state = "ON" if update_enabled else "FROZEN"
    cv2.putText(frame, f"ID:{mid} READY:{int(ready)}  {state}",
                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_box, 2)

    if dp_filt_mm is not None:
        cv2.putText(frame, f"dp_filt={dp_filt_mm:.2f}mm",
                    (x, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    if stable_t is not None:
        cv2.putText(frame, f"stable={stable_t:.2f}s",
                    (x, y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    if move_cnt is not None:
        cv2.putText(frame, f"move_cnt={move_cnt}",
                    (x, y + 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    if rvec_show is not None and tvec_show is not None:
        cv2.drawFrameAxes(frame, K, dist, rvec_show.reshape(3, 1), tvec_show.reshape(3, 1), 0.03)


# =========================================================
# Node
# =========================================================
class CameraPartsNode(Node):
    def __init__(self):
        super().__init__("camera_parts_filtered_onoff_with_Tg2c_spikeoff")

        self.K, self.dist = load_intrinsics()

        self.cap = cv2.VideoCapture(CAM_DEVICE)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {CAM_DEVICE}")

        self.detector = aruco.ArucoDetector(aruco_dict, params)

        self.pub_parts = self.create_publisher(PartArray, TOPIC_PARTS, 10)

        # ✅ Part DB
        self.parts: dict[int, dict] = {}

        # ✅ update만 멈추고 publish는 유지
        self.update_enabled = True

        period = 1.0 / max(1.0, FPS)
        self.timer = self.create_timer(period, self.tick)

        if SHOW_IMSHOW:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, IMSHOW_W, IMSHOW_H)

        self.get_logger().info("✅ Camera node started (T_g2c + filtered ON/OFF + spike OFF)")
        self.get_logger().info("Keys: [f]=freeze toggle  [q]=quit")

    def get_T_b2c(self) -> np.ndarray:
        if USE_FIXED_T_B2G:
            T_b2g = T_B2G_FIXED
        else:
            # TODO: 로봇 노드에서 최신 base->gripper 갱신해서 사용
            T_b2g = T_B2G_FIXED
        return T_b2g @ T_G2C

    def _ensure_entry(self, mid: int, stamp):
        if mid in self.parts:
            return
        if not self.update_enabled:
            return

        self.parts[mid] = {
            "pose": None,
            "ready": False,
            "conf": 1.0,
            "last_seen": stamp,

            "det_count": 0,
            "tbuf_cam": [],
            "qbuf_cam": [],
            "prev_filt_base_t": None,

            "stable_start_time": None,
            "stable_elapsed": 0.0,

            "move_counter": 0,
            "move_start_time": None,
        }

    def _reset_lost(self, d: dict):
        d["pose"] = None
        d["ready"] = False
        d["det_count"] = 0
        d["tbuf_cam"].clear()
        d["qbuf_cam"].clear()

        d["prev_filt_base_t"] = None
        d["stable_start_time"] = None
        d["stable_elapsed"] = 0.0

        d["move_counter"] = 0
        d["move_start_time"] = None

    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = self.detector.detectMarkers(gray)

        now_stamp = self.get_clock().now().to_msg()
        detected_ids = set()

        # ✅ base->camera 업데이트 (T_g2c 반영)
        T_b2c = self.get_T_b2c()

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)

            for i, mid in enumerate(ids_flat):
                mid = int(mid)
                detected_ids.add(mid)

                corners_2d = np.asarray(corners_list[i], dtype=np.float64).reshape(4, 2)

                # raw rvec/tvec (cam->target)
                rvec_raw, tvec_raw = solve_marker_pose_from_corners(
                    corners_2d, MARKER_LENGTH_M, self.K, self.dist
                )
                if rvec_raw is None:
                    continue

                self._ensure_entry(mid, now_stamp)

                if mid not in self.parts or not self.update_enabled:
                    if SHOW_IMSHOW:
                        draw_overlay(frame, corners_2d, mid, False,
                                     rvec_raw, tvec_raw, self.K, self.dist,
                                     update_enabled=self.update_enabled)
                    continue

                d = self.parts[mid]

                # -----------------------------------------------------
                # ✅ Buffer update
                # -----------------------------------------------------
                d["tbuf_cam"].append(tvec_raw.copy())
                d["qbuf_cam"].append(rvec_to_quat_xyzw(rvec_raw))
                if len(d["tbuf_cam"]) > FILTER_BUF_N:
                    d["tbuf_cam"].pop(0)
                    d["qbuf_cam"].pop(0)

                if len(d["tbuf_cam"]) < MIN_FILTER_SAMPLES:
                    if SHOW_IMSHOW:
                        draw_overlay(frame, corners_2d, mid, d["ready"],
                                     rvec_raw, tvec_raw, self.K, self.dist,
                                     update_enabled=True)
                    continue

                # -----------------------------------------------------
                # ✅ Filtered pose (cam->target)
                # -----------------------------------------------------
                t_filt, q_filt = filter_pose_from_buffers(d["tbuf_cam"], d["qbuf_cam"])
                if q_filt is None:
                    continue
                rvec_filt = quat_xyzw_to_rvec(q_filt)

                # base->target (filtered)
                T_c2t_filt = rvec_tvec_to_T(rvec_filt, t_filt)
                T_b2t_filt = T_b2c @ T_c2t_filt
                filt_base_t = T_b2t_filt[:3, 3].copy()

                # dp_filt (base 기준)
                dp_filt = 0.0
                if d["prev_filt_base_t"] is not None:
                    dp_filt = float(np.linalg.norm(filt_base_t - d["prev_filt_base_t"]))
                d["prev_filt_base_t"] = filt_base_t.copy()

                now_time = time.time()

                # =====================================================
                # ✅ READY OFF (filtered dp)
                #   (1) FAST SPIKE -> 즉시 OFF
                #   (2) debounce -> 연속 조건 만족시 OFF
                # =====================================================
                fast_spike = dp_filt >= READY_OFF_FAST_SPIKE_DP_THRESH_M

                if d["ready"] and fast_spike:
                    d["ready"] = False
                    d["stable_start_time"] = None
                    d["stable_elapsed"] = 0.0
                    d["move_counter"] = 0
                    d["move_start_time"] = None

                moving_candidate = dp_filt >= READY_OFF_MOVE_DP_THRESH_M

                if moving_candidate:
                    d["move_counter"] += 1
                    if d["move_start_time"] is None:
                        d["move_start_time"] = now_time
                else:
                    d["move_counter"] = max(0, d["move_counter"] - 1)
                    d["move_start_time"] = None

                off_by_frames = d["move_counter"] >= READY_OFF_DEBOUNCE_FRAMES

                off_by_time = False
                if READY_OFF_MOVE_TIME_SEC is not None and d["move_start_time"] is not None:
                    off_by_time = (now_time - d["move_start_time"]) >= READY_OFF_MOVE_TIME_SEC

                if d["ready"] and (off_by_frames or off_by_time):
                    d["ready"] = False
                    d["stable_start_time"] = None
                    d["stable_elapsed"] = 0.0
                    d["move_counter"] = 0
                    d["move_start_time"] = None

                # =====================================================
                # ✅ READY ON (filtered dp + stable time)
                # =====================================================
                d["det_count"] += 1
                stable_ok = dp_filt <= READY_ON_STABLE_DP_THRESH_M

                if stable_ok:
                    if d["stable_start_time"] is None:
                        d["stable_start_time"] = now_time
                    d["stable_elapsed"] = now_time - d["stable_start_time"]

                    if (not d["ready"]
                        and d["det_count"] >= READY_ON_MIN_DETECTED_FRAMES
                        and d["stable_elapsed"] >= READY_ON_STABLE_TIME_SEC):
                        d["ready"] = True
                else:
                    d["stable_start_time"] = None
                    d["stable_elapsed"] = 0.0

                # =====================================================
                # ✅ publish pose는 항상 filtered base->target
                # =====================================================
                d["pose"] = T_to_pose(T_b2t_filt)
                d["last_seen"] = now_stamp
                d["conf"] = 1.0

                if SHOW_IMSHOW:
                    draw_overlay(frame, corners_2d, mid, d["ready"],
                                 rvec_filt, t_filt, self.K, self.dist,
                                 dp_filt_mm=dp_filt * 1000.0,
                                 stable_t=d["stable_elapsed"],
                                 move_cnt=d["move_counter"],
                                 update_enabled=True)

        # -----------------------------------------------------
        # ✅ marker lost 처리 (update ON일 때만)
        # -----------------------------------------------------
        if self.update_enabled:
            for mid, d in self.parts.items():
                if mid not in detected_ids:
                    self._reset_lost(d)

        # -----------------------------------------------------
        # ✅ publish는 항상 ON
        # -----------------------------------------------------
        msg = PartArray()
        msg.header = Header()
        msg.header.stamp = now_stamp
        msg.header.frame_id = "base"

        for mid, d in self.parts.items():
            if d["pose"] is None:
                continue
            p = Part()
            p.id = int(mid)
            p.pose = d["pose"]
            p.ready_to_pick = bool(d["ready"])
            p.confidence = float(d["conf"])
            p.last_seen = d["last_seen"]
            msg.parts.append(p)

        self.pub_parts.publish(msg)

        # -----------------------------------------------------
        # ✅ imshow / key handling
        # -----------------------------------------------------
        if SHOW_IMSHOW:
            state_txt = "UPDATE:ON" if self.update_enabled else "UPDATE:OFF(FROZEN)"
            cv2.putText(frame, state_txt, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 0) if self.update_enabled else (0, 0, 255), 2)

            cv2.putText(frame, "Keys: [f]=freeze toggle  [q]=quit",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (255, 255, 255), 2)

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rclpy.shutdown()
                return
            if key == ord("f"):
                self.update_enabled = not self.update_enabled
                self.get_logger().warn(f"update_enabled -> {self.update_enabled}")

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        if SHOW_IMSHOW:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraPartsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
