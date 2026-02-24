#!/usr/bin/env python3
import time
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from smartfactory_interfaces.msg import Part, PartArray

from smartfactory_interfaces.srv import CoordsAngles

from jetcobot_pkg.utils.camera_utils import (
    load_intrinsics,                    # Camera Matrix, Dist Coeff 저장 npz 파일 불러오는 함수
    solve_marker_pose_from_corners,     # cv2 solvePNP 통해서 마커 위치 추정
    rvec_tvec_to_T,                     # rvec,tvec --> T(4x4)
    T_to_pose,                          # T --> geometry_msg Pose
    rvec_to_quat_xyzw,                  # rvec --> quaternion
    quat_xyzw_to_rvec,                  # quaternion --> rvec
    mycobot_coords_to_T_b2g             #Mycobot EE cords --> T(4x4)

)

# ================================
# ✅ 전역 설정
# ================================

# -------------------
# 카메라 전역 설정
# -------------------
CAM_DEVICE = "/dev/jetcocam0"   # Camera 장치 위치
TOPIC_PARTS = "/jetcobot/storage/camera/parts"          # 발행 토픽 이름
FPS = 15.0                      # 카메라 초당 프레임수
    
# -------------------
# MyCobot 전역 설정
# -------------------
T_G2C = np.array([              # Hand-eye Calibration 결과 matrix
    [ 0.7071,  0.7071,  0.0, -0.03394],
    [-0.7071,  0.7071,  0.0, -0.03394],
    [ 0.0,     0.0,     1.0,  0.02700],
    [ 0.0,     0.0,     0.0,  1.0    ],
], dtype=np.float64)

T_G2C_MM = T_G2C.copy()
T_G2C_MM[:3, 3] *= 1000.0       # T_G2C 단위 변환 mm

USE_FIXED_T_B2G = False          # cobot으로 부터 실시간 T_B2G coords 받을때 False

HOME_COORDS = [-64.2, 23.2, 235.1, -150.48, 27.49, 142.74]
T_B2G_FIXED_MM = mycobot_coords_to_T_b2g(HOME_COORDS) # T_B2G

# -------------------
# Aruco 전역 설정
# -------------------
MARKER_LENGTH_MM = 20.0         # Aruco 마커 한변 길이(mm)
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

# -------------------
# 부품 상태 판단 파라미터
# -------------------
FILTER_BUF_N = 30                                # 필터링 버퍼 수
MIN_FILTER_SAMPLES = max(3, FILTER_BUF_N // 2)  # 최소 필터 샘플 수

READY_ON_STABLE_DP_THRESH_MM = 3.0              # ready on 최대 dp 값 임계 수치
READY_ON_STABLE_TIME_SEC = 0.7                  # ready on 최소 임계 수치 유지 시간
READY_ON_MIN_DETECTED_FRAMES = 0                # ready on 최소 탐지 프레임 수

READY_OFF_MOVE_DP_THRESH_MM = 1.0               # ready off 최소 dp 값 임계 수치
READY_OFF_DEBOUNCE_FRAMES = 7                   # ready off 최소 임계 수치 유지 프레임 수
READY_OFF_MOVE_TIME_SEC = None                  # ready off 최소 임계 수치 유지 시간(현재 꺼둠)

READY_OFF_FAST_SPIKE_DP_THRESH_MM = 12.0        # ready off spike 판단 최소 수치

# -------------------
# 좌표 필터링 유틸 함수
# -------------------
def average_quaternions_xyzw(q_list: list[np.ndarray]) -> np.ndarray | None: # 샘플 quaternion 평균값 계산 함수
    if len(q_list) == 0:
        return None
    ref = q_list[0]
    qs = []
    for q in q_list:
        qs.append(-q if np.dot(q, ref) < 0 else q)
    q_mean = np.mean(np.stack(qs, axis=0), axis=0)
    return q_mean / (np.linalg.norm(q_mean) + 1e-12)

def filter_pose_from_buffers(tbuf: list[np.ndarray], qbuf: list[np.ndarray]): # ⭐ 위치, 쿼터니언 회전 필터링 함수
    t_filt = np.median(np.stack(tbuf, axis=0), axis=0)
    q_filt = average_quaternions_xyzw(qbuf)
    return t_filt, q_filt


# ================================
# ✅ Camera Node
# ================================
class CameraPartsNode(Node):
    def __init__(self):
        super().__init__("camera_node") 

        # =================
        # 📷 Camera 설정
        # =================  
        self.K, self.dist = load_intrinsics()
        self.cap = cv2.VideoCapture(CAM_DEVICE)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {CAM_DEVICE}")
        self.detector = aruco.ArucoDetector(aruco_dict, params)

        # =================
        # ✖️ Class 변수 
        # =================
        self.parts: dict[int, dict] = {}         #  Part DB
        self.angles_coords = None

        # =================
        # 📡 ROS 통신 
        # =================  
        self.pub_parts = self.create_publisher(PartArray, TOPIC_PARTS, 10)
        period = 1.0 / max(1.0, FPS)
        self.timer = self.create_timer(period, self.tick)

        self.cli_ang_coord = self.create_client(CoordsAngles, 'get_coords_angles')
        self.req = CoordsAngles.Request()

        self.get_logger().info("✅ CameraPartsNode started")
        self.get_logger().info(f"- device: {CAM_DEVICE}")
        self.get_logger().info(f"- topic: {TOPIC_PARTS}")

    # =================
    # 🖨️ Node 함수
    # =================

    def future_callback(self, future):
        try:
            response = future.result()
            self.angles_coords = response.coords_angles
            
        except Exception as e:
            self.get_logger().error('Service call failed %s' % e)


    def get_T_b2c_mm(self) -> np.ndarray: # base2camera 4x4 matrix 반환 함수

        if USE_FIXED_T_B2G:
            T_b2g_mm = T_B2G_FIXED_MM
        else:
            self.req.type = 1 # 1: coords
            self.future = self.cli_ang_coord.call_async(self.req)
            self.future.add_done_callback(self.future_callback)

            if self.angles_coords is None:
                T_b2g_mm = T_B2G_FIXED_MM
            else:
                T_b2g_mm = mycobot_coords_to_T_b2g(self.angles_coords)

        return T_b2g_mm @ T_G2C_MM

    def _ensure_entry(self, mid: int, stamp): # 내부 db 특정 id 초기화(공간할당) 함수
        if mid in self.parts:
            return

        self.parts[mid] = {
            "id": None,
            "pose": None,
            "ready": False,

            "det_count": 0,
            "tbuf_cam": [],
            "qbuf_cam": [],
            "prev_filt_base_t_mm": None,

            "stable_start_time": None,
            "stable_elapsed": 0.0,

            "move_counter": 0,
            "move_start_time": None,
        }

    def _reset_lost(self, d: dict): # 내부 db 특정 id 초기화(공간할당) 함수
        d["id"] = None
        d["pose"] = None
        d["ready"] = False

        d["det_count"] = 0
        d["tbuf_cam"].clear()
        d["qbuf_cam"].clear()

        d["prev_filt_base_t_mm"] = None
        d["stable_start_time"] = None
        d["stable_elapsed"] = 0.0

        d["move_counter"] = 0
        d["move_start_time"] = None

    def tick(self): # 타이머 콜백 함수 (프레임 읽기 --> 마커 위치 읽기 --> 부품 정보 dictionary 기록 --> 토픽으로 발행)
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = self.detector.detectMarkers(gray)

        now_stamp = self.get_clock().now().to_msg()
        detected_ids = set()

        # ✅ base->camera(mm)
        T_b2c_mm = self.get_T_b2c_mm()

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)

            # 같은 id가 여러개면 frame 내에서 instance 부여하기 위한 카운터
            instance_counter = {}  

            for i, mid in enumerate(ids_flat):
                mid = int(mid)

                # frame 내 동일 id instance index 부여 (1,2,3...)
                inst = instance_counter.get(mid, 0) + 1
                instance_counter[mid] = inst

                # publish에 사용할 "encoded id" (예: id=1 -> 1001,1002,1003)
                mid_encoded = mid * 1000 + inst
                detected_ids.add(mid_encoded)

                corners_2d = np.asarray(corners_list[i], dtype=np.float64).reshape(4, 2)

                # ✅ cam->target raw pose (여기서 tvec 단위 = marker_length 단위)
                rvec_raw, tvec_raw = solve_marker_pose_from_corners(
                    corners_2d, MARKER_LENGTH_MM, self.K, self.dist
                )
                if rvec_raw is None:
                    continue

                self._ensure_entry(mid_encoded, now_stamp)
                d = self.parts[mid_encoded]
                d["id"] = mid_encoded

                # -----------------------------------------------------
                # ✅ Buffer update (cam frame)  (tvec 단위 = mm)
                # -----------------------------------------------------
                d["tbuf_cam"].append(tvec_raw.copy())
                d["qbuf_cam"].append(rvec_to_quat_xyzw(rvec_raw))
                if len(d["tbuf_cam"]) > FILTER_BUF_N:
                    d["tbuf_cam"].pop(0)
                    d["qbuf_cam"].pop(0)

                if len(d["tbuf_cam"]) < MIN_FILTER_SAMPLES:
                    continue

                # -----------------------------------------------------
                # ✅ Filtered pose (cam->target) (mm)
                # -----------------------------------------------------
                t_filt_mm, q_filt = filter_pose_from_buffers(d["tbuf_cam"], d["qbuf_cam"])
                if q_filt is None:
                    continue
                rvec_filt = quat_xyzw_to_rvec(q_filt)

                # base->target (filtered) in mm
                T_c2t_filt = rvec_tvec_to_T(rvec_filt, t_filt_mm)
                T_b2t_filt = T_b2c_mm @ T_c2t_filt
                filt_base_t_mm = T_b2t_filt[:3, 3].copy()

                # dp_filt (mm)
                dp_filt_mm = 0.0
                if d["prev_filt_base_t_mm"] is not None:
                    dp_filt_mm = float(np.linalg.norm(filt_base_t_mm - d["prev_filt_base_t_mm"]))
                d["prev_filt_base_t_mm"] = filt_base_t_mm.copy()

                now_time = time.time()

                # =====================================================
                # ⭐⭐ READY OFF/ON 판단 로직 ⭐⭐
                # =====================================================

                # --- OFF FAST SPIKE ---
                fast_spike = dp_filt_mm >= READY_OFF_FAST_SPIKE_DP_THRESH_MM
                if d["ready"] and fast_spike:
                    d["ready"] = False
                    d["stable_start_time"] = None
                    d["stable_elapsed"] = 0.0
                    d["move_counter"] = 0
                    d["move_start_time"] = None

                # --- OFF debounce ---
                moving_candidate = dp_filt_mm >= READY_OFF_MOVE_DP_THRESH_MM

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

                # --- ON stable time ---
                d["det_count"] += 1
                stable_ok = dp_filt_mm <= READY_ON_STABLE_DP_THRESH_MM

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

                # --- pose, 이외 다른 정보 기록 ---
                d["pose"] = T_to_pose(T_b2t_filt)   # Pose.position = mm 로 들어감


        # -----------------------------------------------------
        # ✅ marker lost 처리
        # -----------------------------------------------------
        for mid, d in self.parts.items():
            if mid not in detected_ids:
                self._reset_lost(d)

        # -----------------------------------------------------
        # ✅ publish는 항상 ON
        # -----------------------------------------------------
        msg = PartArray()
        msg.header = Header()
        msg.header.stamp = now_stamp
        msg.header.frame_id = "base_mm"

        for mid, d in self.parts.items():
            if d["pose"] is None:
                continue
            p = Part()
            p.id = int(d["id"])
            p.pose_mm = d["pose"]
            p.ready_to_pick = bool(d["ready"])
            p.stable_time_sec = float(d["stable_elapsed"])

            msg.parts.append(p)

        self.pub_parts.publish(msg)

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
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
