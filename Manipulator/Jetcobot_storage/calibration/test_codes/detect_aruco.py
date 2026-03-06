import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
import math


# =========================
# 설정
# =========================
CAM_DEVICE = "/dev/video0"

# 마커 한 변 길이(미터)
MARKER_LENGTH = 0.025  # 25mm

# 특정 ID만 보고 싶으면 숫자 지정 (예: 0), 아니면 None
TARGET_ID = 3  # 예: 0 또는 None

# 출력 주기(초) - 너무 빠르면 터미널이 난리남
PRINT_INTERVAL = 0.2

# 카메라 intrinsic 파일(.npz)
# keys는 camera_matrix/dist_coeffs 또는 mtx/dist 둘 다 지원
BASE_DIR = os.getcwd()
INTRINSICS_NPZ = os.path.join(
    BASE_DIR, "hand_eye_calibration", "calibrationv.0.1", "cameracalib_npzdata",
    "camera_calib_upgraded_2026-01-12_16-17-24.npz"
)

# =========================
# 로드: 카메라 파라미터
# =========================
calib = np.load(INTRINSICS_NPZ)
if "camera_matrix" in calib.files:
    K = calib["camera_matrix"].astype(np.float64)
    dist = calib["dist_coeffs"].astype(np.float64)
else:
    K = calib["mtx"].astype(np.float64)
    dist = calib["dist"].astype(np.float64)

# =========================
# ArUco detector params
# =========================
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

# =========================
# Camera open
# =========================
cap = cv2.VideoCapture(CAM_DEVICE)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open {CAM_DEVICE}")

print("\n[INFO]")
print(" - This prints ArUco pose as cam -> marker (cam frame).")
print(" - Expect tvec.z > 0 when marker is in front of camera.")
print("Keys: [q]=quit\n")

last_print = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera frame not received.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

    selected = None  # (idx, id, rvec, tvec, dist)

    if ids is not None and len(ids) > 0:
        ids_flat = ids.flatten().astype(int)

        # pose estimation for all detected markers
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, MARKER_LENGTH, K, dist
        )

        # marker selection:
        if TARGET_ID is not None and TARGET_ID in ids_flat:
            idx = int(np.where(ids_flat == TARGET_ID)[0][0])
            t = tvecs[idx][0]
            d = float(np.linalg.norm(t))
            selected = (idx, int(ids_flat[idx]), rvecs[idx][0], t, d)
        else:
            # pick nearest marker (min distance)
            dists = [float(np.linalg.norm(tvecs[i][0])) for i in range(len(ids_flat))]
            idx = int(np.argmin(dists))
            t = tvecs[idx][0]
            d = float(dists[idx])
            selected = (idx, int(ids_flat[idx]), rvecs[idx][0], t, d)

        # draw all
        aruco.drawDetectedMarkers(frame, corners, ids)

        # draw axis for selected marker
        idx, mid, rvec, tvec, dist_m = selected
        cv2.drawFrameAxes(frame, K, dist, rvec.reshape(1, 3), tvec.reshape(1, 3), 0.02)

        # overlay text
        cv2.putText(frame, f"ID:{mid}  t=[{tvec[0]:.3f},{tvec[1]:.3f},{tvec[2]:.3f}] m  dist={dist_m:.3f} m",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        # periodic print
        now = time.time()
        if now - last_print > PRINT_INTERVAL:
            # rvec magnitude -> rotation angle (rad -> deg)
            angle_rad = float(np.linalg.norm(rvec))
            angle_deg = angle_rad * 180.0 / math.pi

            print(
                f"[AruCo] ID={mid} | "
                f"tvec(cam->marker) [m]=({tvec[0]:+.4f}, {tvec[1]:+.4f}, {tvec[2]:+.4f}) | "
                f"dist={dist_m:.4f} m | "
                f"rvec [rad]=({rvec[0]:+.4f}, {rvec[1]:+.4f}, {rvec[2]:+.4f}) | "
                f"rot_angle≈{angle_deg:.2f} deg"
            )
            last_print = now

    else:
        cv2.putText(frame, "No marker detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Aruco Pose (cam -> marker)", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("✅ 종료")
