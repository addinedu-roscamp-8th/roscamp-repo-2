## vision_node.py

import os
import time
import numpy as np
import cv2
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory

# from jetcobot_interfaces.action import ObserveMarker
# from jetcobot_interfaces.srv import GetCurrentCoords
# from jetcobot_interfaces.msg import SectionResult

from smartfactory_interfaces.action import ObserveMarker
from smartfactory_interfaces.srv import GetCurrentCoords
from smartfactory_interfaces.msg import SectionResult


# ==================================================
# Utility (동일)
# ==================================================
def coords_to_T_base_ee(coords_mm_deg):
    x, y, z, rx, ry, rz = coords_mm_deg
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = np.array([x, y, z], dtype=float) * 1e-3  # mm -> m
    return T

def T_to_coords_mm_deg(T):
    t_mm = T[:3, 3] * 1e3
    rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return [
        float(t_mm[0]), float(t_mm[1]), float(t_mm[2]),
        float(rpy[0]), float(rpy[1]), float(rpy[2])
    ]

def average_quaternions(quats: np.ndarray) -> np.ndarray:
    q0 = quats[0]
    fixed = []
    for q in quats:
        fixed.append(q if np.dot(q0, q) >= 0 else -q)
    q_mean = np.mean(np.vstack(fixed), axis=0)
    return q_mean / np.linalg.norm(q_mean)

def refine_corners_subpix(gray, corners, do_subpix=True):
    if not do_subpix:
        return
    for c in corners:
        cv2.cornerSubPix(
            gray, c,
            winSize=(5, 5),
            zeroZone=(-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

def solve_marker_pose_pnp(corner_4x2, marker_length, camera_matrix, dist_coeffs):
    L = marker_length / 2.0
    objp = np.array([[-L, L, 0.0], [L, L, 0.0], [L, -L, 0.0], [-L, -L, 0.0]], dtype=np.float32)
    imgp = np.asarray(corner_4x2, dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    return ok, rvec.reshape(3) if ok else None, tvec.reshape(3) if ok else None

# ==================================================
# Vision Node
# ==================================================
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ✅ 병렬 처리를 위한 ReentrantCallbackGroup 추가
        self.callback_group = ReentrantCallbackGroup()

        # ---------------- Parameters ----------------
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('marker_length', 0.02)
        self.declare_parameter('do_subpix', True)

        self.declare_parameter('offset_x', -0.012)
        self.declare_parameter('offset_y', -0.007)
        self.declare_parameter('offset_z', -0.023)
        self.declare_parameter('sag_coeff', 0.09)

        self.declare_parameter('dist_threshold', 0.22)   # 22cm 기준
        self.declare_parameter('extra_offset_x', 0.003)  # 추가 X 보정
        self.declare_parameter('extra_offset_z', 0.003)  # 추가 Z 보정

        # ---------------- Camera ----------------
        cam_idx = int(self.get_parameter('camera_index').value)
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera open failed (index={cam_idx})")

        # ---------------- Calibration ----------------
        pkg_share = get_package_share_directory('mycobot_system')
        calib_path = os.path.join(pkg_share, 'config', 'calib_data.npz')
        data = np.load(calib_path)
        self.camera_matrix = data["mtx"]
        self.dist_coeffs = data["dist"]

        # ---------------- ArUco ----------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.aruco_params.adaptiveThreshConstant = 9
        self.aruco_params.minMarkerPerimeterRate = 0.02
        self.aruco_params.maxMarkerPerimeterRate = 2.5
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.minMarkerDistanceRate = 0.02
        self.aruco_params.minOtsuStdDev = 5.0
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13

        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.aruco_params
        )

        # ---------------- Hand–eye ----------------
        self.T_ee_cam = np.array([
            [ 0.7071,  0.7071, 0.0, -0.03394],
            [-0.7071,  0.7071, 0.0, -0.03394],
            [ 0.0,     0.0,    1.0,  0.027  ],
            [ 0.0,     0.0,    0.0,  1.0    ]
        ], dtype=float)

        # ---------------- DB update topic ----------------
        self.db_pub = self.create_publisher(
            SectionResult,
            # '/jetcobot/assembly/db_update',
            '/jetcobot/assembly/db_update',
            10
        )


        # ---------------- Service Client (Callback Group 적용) ----------------
        self.coords_cli = self.create_client(
            GetCurrentCoords, 
            '/manipulator/get_current_coords',
            callback_group=self.callback_group
        )

        # ---------------- Action Server (Callback Group 적용) ----------------
        self._as = ActionServer(
            self,
            ObserveMarker,
            '/vision/observe_marker',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.callback_group
        )

        self.get_logger().info("👀 Vision Node Initialized with Multi-threading")

    # ---------------- Section Classification ----------------
    def classify_section(self, x_m: float, y_m: float):
        # section_centers = {
        #     # ("A", 1): (0.23781751694392866,  0.005020475948445496),
        #     ("A", 1): (0.15790524245835882,  -0.0017191770956796783),
        #     ("A", 2): (0.2828007676862617,   0.0065217138851096056),
        #     ("A", 3): (0.32454790312282095,  0.009331413706595664),

        #     ("B", 1): (0.15338815569746786, -0.05724197420221029),
        #     ("B", 2): (0.19567735731776645, -0.05395115480071706),
        #     ("B", 3): (0.23414924255243028, -0.05313336349773359),

        #     ("C", 1): (0.15314696106941145, -0.128611351270929),
        #     ("C", 2): (0.19358146932409173, -0.12353170613596874),
        #     ("C", 3): (0.3179742518793567,  -0.1257460945504737),
        # }

        section_centers = {
            # ("A", 1): (0.23781751694392866,  0.005020475948445496),
            ("A", 1): (0.15790524245835882,  -0.0017191770956796783),
            ("A", 2): (0.202106558426872,   0.0013644264664570338),
            ("A", 3): (0.24146487674280892,  0.009331413706595664),

            ("B", 1): (0.15338815569746786, -0.05724197420221029),
            ("B", 2): (0.19567735731776645, -0.05395115480071706),
            ("B", 3): (0.2378663434811138, -0.05313336349773359),

            ("C", 1): (0.15314696106941145, -0.128611351270929),
            ("C", 2): (0.19358146932409173, -0.12353170613596874),
            ("C", 3): (0.23997599650102297,  -0.1257460945504737),
        }


        min_dist = float("inf")
        best_section = None
        best_id = None

        for (section, idx), (cx, cy) in section_centers.items():
            dist = np.sqrt((x_m - cx)**2 + (y_m - cy)**2)

            if dist < min_dist:
                min_dist = dist
                best_section = section
                best_id = idx

        # # 5cm 이상이면 무효 처리 (선택사항)
        # if min_dist > 0.05:
        #     self.get_logger().warn(f"⚠ No valid section (dist={min_dist:.3f}m)")
        #     return None, None

        return best_section, best_id


    def goal_cb(self, goal):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    # ✅ async/await 방식으로 변경하여 Blocking 제거
    async def execute_cb(self, goal_handle):
        goal = goal_handle.request



        self.get_logger().info(
            f"🔥 Vision execute_cb START marker_id={goal.marker_id}"
        )




        feedback = ObserveMarker.Feedback()
        
        marker_id = int(goal.marker_id)
        n_samples = int(goal.n_samples)
        timeout_sec = float(goal.timeout_sec)
        outlier_thresh = float(goal.outlier_thresh)

        do_subpix = bool(self.get_parameter('do_subpix').value)
        marker_length = float(self.get_parameter('marker_length').value)

        feedback.stage = "OBSERVING"
        goal_handle.publish_feedback(feedback)

        # ---- collect samples ----
        t_start = self.get_clock().now()
        tvec_list = []
        quat_list = []

        # ROS 2에서는 time.time() 대신 self.get_clock().now() 권장
        while (self.get_clock().now() - t_start).nanoseconds / 1e9 < timeout_sec and len(tvec_list) < n_samples:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ObserveMarker.Result(ok=False)

            ret, frame = self.cap.read()
            if not ret: continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            if ids is None: continue

            refine_corners_subpix(gray, corners, do_subpix=do_subpix)
            ids_flat = ids.flatten().astype(int)
            if marker_id not in ids_flat: continue

            idx = int(np.where(ids_flat == marker_id)[0][0])
            ok, rvec, tvec = solve_marker_pose_pnp(corners[idx][0], marker_length, self.camera_matrix, self.dist_coeffs)
            
            if ok:
                tvec_list.append(tvec)
                quat_list.append(R.from_rotvec(rvec).as_quat())

        if len(tvec_list) < max(3, n_samples // 2):
            self.get_logger().error("❌ Not enough samples collected")
            goal_handle.abort()
            return ObserveMarker.Result(ok=False)

        # ---- Average Calculation ----
        tvec_arr = np.vstack(tvec_list)
        med = np.median(tvec_arr, axis=0)
        keep = np.linalg.norm(tvec_arr - med, axis=1) < outlier_thresh
        
        tvec_avg = np.mean(tvec_arr[keep], axis=0) if np.sum(keep) >= 3 else np.mean(tvec_arr, axis=0)
        q_avg = average_quaternions(np.vstack(quat_list)[keep] if np.sum(keep) >= 3 else np.vstack(quat_list))

        T_cam_target = np.eye(4)
        T_cam_target[:3, :3] = R.from_quat(q_avg).as_matrix()
        T_cam_target[:3, 3] = tvec_avg

        # ---- Get Current EE Pose (비동기 호출) ----
        feedback.stage = "GET_EE"
        goal_handle.publish_feedback(feedback)

        if not self.coords_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Manipulator service not found")
            goal_handle.abort()
            return ObserveMarker.Result(ok=False)

        # ✅ await를 사용하여 응답 대기 (노드는 멈추지 않음)
        future = self.coords_cli.call_async(GetCurrentCoords.Request())
        response = await future

        if response is None or not response.coords:
            self.get_logger().error("❌ Failed to get coords from manipulator")
            goal_handle.abort()
            return ObserveMarker.Result(ok=False)

        # ---- Final Calculation ----
        T_base_ee = coords_to_T_base_ee(response.coords)
        T_base_target = T_base_ee @ self.T_ee_cam @ T_cam_target

        # Tuning offsets
        T_base_target[0, 3] += float(self.get_parameter('offset_x').value) # -= 0.015
        T_base_target[1, 3] += float(self.get_parameter('offset_y').value) # -= 0.010
        T_base_target[2, 3] += float(self.get_parameter('offset_z').value) # -= 0.023
        print(T_base_target[0, 3])
        print(T_base_target[1, 3])

        T_base_target[2, 3] += float(self.get_parameter('sag_coeff').value) * T_base_target[0, 3]

        dist_limit = float(self.get_parameter('dist_threshold').value)
        if T_base_target[0, 3] > dist_limit:
            self.get_logger().info(f"📏 Threshold exceeded ({T_base_target[0, 3]:.3f}m). Applying extra offsets.")
            T_base_target[0, 3] += float(self.get_parameter('extra_offset_x').value) # += 0.003
            T_base_target[2, 3] += float(self.get_parameter('extra_offset_z').value) # += 0.005

        # ---- Section classification & DB publish ----
        x_m = float(T_base_target[0, 3])
        y_m = float(T_base_target[1, 3])

        section, idx = self.classify_section(x_m, y_m)

        if section is not None:
            msg = SectionResult()
            self.get_logger().info("🔥 DB_PUBLISH about to execute")



            msg.section = section
            msg.id = int(idx)
            msg.occupy = 0  # pick이므로 제거
            self.db_pub.publish(msg)

            self.get_logger().info(
                f"[DB_UPDATE] section={section}, id={idx}, occupy=0"
            )
        else:
            self.get_logger().warn(
                f"[DB_UPDATE] No matching section for x={x_m:.3f}, y={y_m:.3f}"
            )

        feedback.stage = "DONE"
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        return ObserveMarker.Result(ok=True, base_target_coords=T_to_coords_mm_deg(T_base_target))

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = VisionNode()
    # ✅ 멀티스레드 실행기 사용
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
