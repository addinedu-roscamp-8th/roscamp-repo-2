import cv2
import cv2.aruco as aruco
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import os

from pymycobot.mycobot280 import MyCobot280


# =========================
# 사용자 설정
# =========================
CAM_DEVICE = "/dev/video0"
MYCOBOT_PORT = "/dev/ttyUSB0"
MYCOBOT_BAUD = 1000000

TARGET_MARKER_ID = 3
MARKER_LENGTH_M = 0.020

# Intrinsic npz
BASE_DIR = os.getcwd()
INTRINSICS_NPZ = os.path.join(
    BASE_DIR, "hand_eye_calibration", "calibrationv.0.1", "cameracalib_npzdata",
    "camera_calib_upgraded_2026-01-12_16-17-24.npz"
)

# Hand-eye (gripper->camera)
T_g2c = np.array([[ 8.57986976e-01,  5.13481580e-01,  1.39648065e-02, -1.50842180e-02],
 [-5.13670857e-01,  8.57627880e-01,  2.48328530e-02, -3.46132180e-02],
 [ 7.74605256e-04, -2.84795786e-02,  9.99594074e-01, -6.09937520e-03],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]], dtype=np.float64)

#merged layer123



ROBOT_EULER_INTRINSIC_ZYX = True

# 표시 안정화(포즈 튐 줄이기) - 필요 없으면 False
USE_EMA_FILTER = True
EMA_ALPHA_POS = 0.15
EMA_ALPHA_ROT = 0.15


# =========================
# 유틸
# =========================
def load_intrinsics(npz_path: str):
    calib = np.load(npz_path)
    if "camera_matrix" in calib.files:
        K = calib["camera_matrix"].astype(np.float64)
        dist = calib["dist_coeffs"].astype(np.float64).reshape(-1, 1)
    else:
        K = calib["mtx"].astype(np.float64)
        dist = calib["dist"].astype(np.float64).reshape(-1, 1)
    return K, dist

def rvec_tvec_to_T(rvec, tvec):
    R_ct, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_ct
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T

def mycobot_coords_to_T_b2g(coords):
    x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = coords[:6]
    t = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0

    if ROBOT_EULER_INTRINSIC_ZYX:
        rot = R.from_euler('ZYX', [rz_deg, ry_deg, rx_deg], degrees=True)
    else:
        rot = R.from_euler('zyx', [rz_deg, ry_deg, rx_deg], degrees=True)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = t
    return T

def T_to_xyzrpy_deg(T):
    t = T[:3, 3]
    rot = R.from_matrix(T[:3, :3])
    z_deg, y_deg, x_deg = rot.as_euler('ZYX', degrees=True)
    rx_deg = float(x_deg)
    ry_deg = float(y_deg)
    rz_deg = float(z_deg)
    return float(t[0]), float(t[1]), float(t[2]), rx_deg, ry_deg, rz_deg

def ema_update(prev, new, alpha):
    if prev is None:
        return new.copy()
    return (1 - alpha) * prev + alpha * new

def ema_update_quat(prev_q, new_q, alpha):
    if prev_q is None:
        return new_q.copy()

    if np.dot(prev_q, new_q) < 0:
        new_q = -new_q

    q = (1 - alpha) * prev_q + alpha * new_q
    q /= (np.linalg.norm(q) + 1e-12)
    return q


# =========================
# 초기화
# =========================
K, dist = load_intrinsics(INTRINSICS_NPZ)

cap = cv2.VideoCapture(CAM_DEVICE)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open camera: {CAM_DEVICE}")

mc = MyCobot280(MYCOBOT_PORT, MYCOBOT_BAUD)

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

filtered_pos = None
filtered_quat = None

print("✅ Running... keys: a=servos ON, r=servos OFF, q=quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera frame not received.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

    disp = frame.copy()

    # 안내 텍스트
    info_lines = [
        "Base->Target Pose Monitor",
        "Keys: a=servos ON | r=servos OFF | q=quit",
    ]

    # -------------------------
    # 마커 포즈 계산/표시
    # -------------------------
    if ids is not None and len(ids) > 0:
        ids_flat = ids.flatten().astype(int)
        aruco.drawDetectedMarkers(disp, corners, ids)

        if TARGET_MARKER_ID in ids_flat:
            idx = int(np.where(ids_flat == TARGET_MARKER_ID)[0][0])

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                [corners[idx]], MARKER_LENGTH_M, K, dist
            )
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            # 카메라 기준 축 표시
            cv2.drawFrameAxes(disp, K, dist, rvec, tvec, MARKER_LENGTH_M * 0.5)

            T_c2t = rvec_tvec_to_T(rvec, tvec)

            coords = mc.get_coords()
            if coords and len(coords) >= 6:
                T_b2g = mycobot_coords_to_T_b2g(coords)
                T_b2t = T_b2g @ T_g2c @ T_c2t

                pos = T_b2t[:3, 3]
                quat = R.from_matrix(T_b2t[:3, :3]).as_quat()

                if USE_EMA_FILTER:
                    filtered_pos = ema_update(filtered_pos, pos, EMA_ALPHA_POS)
                    filtered_quat = ema_update_quat(filtered_quat, quat, EMA_ALPHA_ROT)
                    R_f = R.from_quat(filtered_quat).as_matrix()
                    T_show = np.eye(4, dtype=np.float64)
                    T_show[:3, :3] = R_f
                    T_show[:3, 3] = filtered_pos
                else:
                    T_show = T_b2t

                x, y, z, rx, ry, rz = T_to_xyzrpy_deg(T_show)

                info_lines.append(f"Marker ID: {TARGET_MARKER_ID}")
                info_lines.append("BASE->TARGET position (m)")
                info_lines.append(f"  x={x:+.4f}  y={y:+.4f}  z={z:+.4f}")
                info_lines.append("BASE->TARGET rotation (deg)  (rx,ry,rz = X,Y,Z)")
                info_lines.append(f"  rx={rx:+.1f}  ry={ry:+.1f}  rz={rz:+.1f}")
            else:
                info_lines.append("Robot coords not available (get_coords failed)")
        else:
            info_lines.append(f"Target marker ID {TARGET_MARKER_ID} not found.")
    else:
        info_lines.append("No markers detected.")

    # 텍스트 출력
    y0 = 30
    for i, line in enumerate(info_lines):
        y = y0 + i * 26
        cv2.putText(disp, line, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Base->Target Pose Monitor", disp)

    # -------------------------
    # 키 입력 처리
    # -------------------------
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    if key == ord('a'):
        print("[a] focus_all_servos() -> servos ON")
        mc.focus_all_servos()
        time.sleep(0.2)

    if key == ord('r'):
        print("[r] release_all_servos() -> servos OFF")
        mc.release_all_servos()
        time.sleep(0.2)


cap.release()
cv2.destroyAllWindows()
print("Done.")
