import math
import numpy as np
import cv2
from geometry_msgs.msg import Pose

# =========================
# Camera Setting Utils
# =========================

# ===Camera Calibration 호출 함수=== #
def load_intrinsics():
    # 📂 Copy & Paste the Path of your .Npz file
    npz_path = '/home/jetcobot/robot_ws/src/jetcobot_pkg/jetcobot_pkg/calib_data/camera_calib_upgraded_2026-01-12_16-17-24.npz'
    calib = np.load(npz_path)
    if "camera_matrix" in calib.files:
        K = calib["camera_matrix"].astype(np.float64)
        dist = calib["dist_coeffs"].astype(np.float64).reshape(-1, 1)
    else:
        K = calib["mtx"].astype(np.float64)
        dist = calib["dist"].astype(np.float64).reshape(-1, 1)
    return K, dist


# =========================
# Math Utils
# =========================

# ===rvec --> 쿼터니언 변환 함수(반환값: qx qy qz qw)=== #
def rvec_to_quat_xyzw(rvec: np.ndarray) -> np.ndarray:
    Rm, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    qw = np.sqrt(max(0.0, 1.0 + Rm[0, 0] + Rm[1, 1] + Rm[2, 2])) / 2.0
    qx = (Rm[2, 1] - Rm[1, 2]) / (4.0 * qw + 1e-12)
    qy = (Rm[0, 2] - Rm[2, 0]) / (4.0 * qw + 1e-12)
    qz = (Rm[1, 0] - Rm[0, 1]) / (4.0 * qw + 1e-12)
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    return q / (np.linalg.norm(q) + 1e-12)

# ===쿼터니언 --> rvec 변환 함수(반환값: rvec)=== #
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


# ===rvec 쿼터니언 변환 함수(반환값: ros geometry_msg Pose)=== #
def rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
    """OpenCV rvec/tvec (cam->obj) -> geometry_msgs/Pose"""
    Rm, _ = cv2.Rodrigues(rvec.reshape(3, 1))

    qw = np.sqrt(max(0.0, 1.0 + Rm[0, 0] + Rm[1, 1] + Rm[2, 2])) / 2.0
    qx = (Rm[2, 1] - Rm[1, 2]) / (4.0 * qw + 1e-12)
    qy = (Rm[0, 2] - Rm[2, 0]) / (4.0 * qw + 1e-12)
    qz = (Rm[1, 0] - Rm[0, 1]) / (4.0 * qw + 1e-12)

    pose = Pose()
    pose.position.x = float(tvec[0])
    pose.position.y = float(tvec[1])
    pose.position.z = float(tvec[2])
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


# ===rvec,tvec => Homogeneous Transformation 변환 함수(반환값 4x4 mtx)=== #
def rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """(cam->target) rvec/tvec -> 4x4"""
    R_ct, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_ct
    T[:3, 3] = tvec.reshape(3)
    return T


# ===Homogeneous Transformation => geometrymsg Pose 변환 함수(반환값: ros geometry_msg Pose)=== #
def T_to_pose(T: np.ndarray) -> Pose:
    """4x4 -> Pose"""
    Rm = T[:3, :3]
    t = T[:3, 3]

    qw = np.sqrt(max(0.0, 1.0 + Rm[0, 0] + Rm[1, 1] + Rm[2, 2])) / 2.0
    qx = (Rm[2, 1] - Rm[1, 2]) / (4.0 * qw + 1e-12)
    qy = (Rm[0, 2] - Rm[2, 0]) / (4.0 * qw + 1e-12)
    qz = (Rm[1, 0] - Rm[0, 1]) / (4.0 * qw + 1e-12)

    pose = Pose()
    pose.position.x = float(t[0])
    pose.position.y = float(t[1])
    pose.position.z = float(t[2])
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


def mycobot_coords_to_T_b2g(coords) -> np.ndarray:

    if coords is None or len(coords) < 6:
        raise ValueError("coords must be length >= 6: [x,y,z,rx,ry,rz]")

    x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = coords[:6]


    t = np.array([x_mm, y_mm, z_mm], dtype=np.float64)

    # angles (deg -> rad)
    rx = np.deg2rad(rx_deg)
    ry = np.deg2rad(ry_deg)
    rz = np.deg2rad(rz_deg)

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    # Rx, Ry, Rz
    Rx = np.array([
        [1, 0,  0],
        [0, cx, -sx],
        [0, sx,  cx],
    ], dtype=np.float64)

    Ry = np.array([
        [ cy, 0, sy],
        [  0, 1,  0],
        [-sy, 0, cy],
    ], dtype=np.float64)

    Rz = np.array([
        [cz, -sz, 0],
        [sz,  cz, 0],
        [ 0,   0, 1],
    ], dtype=np.float64)

    Rm = Rz @ Ry @ Rx  # ✅ intrinsic ZYX

    # 4x4 transform
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rm
    T[:3, 3] = t
    return T


# =========================
# Aruco Utils
# =========================

# === 인식된 마커 rvec, tvec 계산 함수=== #
def solve_marker_pose_from_corners(corners_2d: np.ndarray, marker_len: float, K, dist):
    """(4,2) corners -> (rvec,tvec) cam->marker"""
    half = marker_len / 2.0
    obj_pts = np.array(
        [
            [-half, +half, 0.0],
            [+half, +half, 0.0],
            [+half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )

    img_pts = corners_2d.astype(np.float64).reshape(4, 2)

    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, K, dist,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if not ok:
        return None, None
    return rvec.reshape(3), tvec.reshape(3)
