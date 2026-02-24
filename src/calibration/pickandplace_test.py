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
CAM_DEVICE = "/dev/video0"          # 카메라 디바이스
MYCOBOT_PORT = "/dev/ttyUSB0"
MYCOBOT_BAUD = 1000000

TARGET_MARKER_ID = 3
MARKER_LENGTH_M = 0.025             # ArUco 실제 한 변 길이(m). 반드시 정확히!
Z_OFFSET = 0.10                     # 마커 위로 접근 오프셋(m) (충돌 방지용)
MOVE_SPEED = 30                     # myCobot 속도(1~100)

# 카메라 intrinsic npz (K, dist)
BASE_DIR = os.getcwd()
INTRINSICS_NPZ = os.path.join(
    BASE_DIR, "hand_eye_calibration", "calibrationv.0.1", "cameracalib_npzdata",
    "camera_calib_upgraded_2026-01-12_16-17-24.npz"
)

# Hand-eye 결과 (그리퍼-카메라 변환)
# 아래 4x4 행렬은 "T_g2c (gripper -> camera)" 를 넣으세요.
# (만약 T_c2g만 있으면 inverse해서 T_g2c로 바꿔 넣으면 됩니다)
T_g2c = np.array([[ 8.57986976e-01,  5.13481580e-01,  1.39648065e-02, -1.50842180e-02],
 [-5.13670857e-01,  8.57627880e-01,  2.48328530e-02, -3.46132180e-02],
 [ 7.74605256e-04, -2.84795786e-02,  9.99594074e-01, -6.09937520e-03],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
, dtype=np.float64)

# 로봇이 내주는 end-effector Euler가 "tool frame 기준 intrinsic ZYX" 라고 했으므로
# 아래 변환에서 'ZYX'(intrinsic)을 사용합니다.
ROBOT_EULER_INTRINSIC_ZYX = True

# =========================
# 유틸 함수들
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
    """OpenCV rvec,tvec (cam->target) -> 4x4"""
    R_ct, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3,1))
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R_ct
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T

def mycobot_coords_to_T_b2g(coords):
    """
    myCobot get_coords() = [x,y,z,rx,ry,rz] (mm, deg)
    반환: T_b2g (base->gripper) 4x4
    """
    x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = coords[:6]
    t = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0

    # myCobot convention: yaw(z)=rz, pitch(y)=ry, roll(x)=rx (deg)
    if ROBOT_EULER_INTRINSIC_ZYX:
        rot = R.from_euler('ZYX', [rz_deg, ry_deg, rx_deg], degrees=True)  # intrinsic
    else:
        rot = R.from_euler('zyx', [rz_deg, ry_deg, rx_deg], degrees=True)  # extrinsic

    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = rot.as_matrix()
    T[:3, 3] = t
    return T

def T_to_xyzrpy_deg(T):
    """
    4x4 -> (x,y,z, rx,ry,rz) for myCobot send_coords
    여기서는 현재 EE 회전을 그대로 쓰는 방식이 더 안전하므로
    실사용에서는 "현재 coords의 rx,ry,rz 유지" 전략 추천.
    """
    t = T[:3, 3]
    rot = R.from_matrix(T[:3,:3])
    # myCobot이 내주는 rx,ry,rz가 ZYX intrinsic 기준이었다고 했으니 동일하게 역변환
    rz_deg, ry_deg, rx_deg = rot.as_euler('ZYX', degrees=True)  # returns [z,y,x] in that order
    return float(t[0]), float(t[1]), float(t[2]), float(rx_deg), float(ry_deg), float(rz_deg)

# =========================
# myCobot servo utilities
# =========================
def servo_release(mc: MyCobot280):
    """토크 OFF: 손으로 로봇을 움직일 수 있게"""
    try:
        mc.release_all_servos()
        return True
    except Exception:
        pass
    ok = True
    for sid in range(1, 7):
        try:
            mc.release_servo(sid)
        except Exception:
            ok = False
    return ok


def servo_activate(mc: MyCobot280):
    """토크 ON: 현재 자세 유지"""
    try:
        mc.power_on()
        return True
    except Exception:
        pass
    try:
        mc.focus_all_servos()
        return True
    except Exception:
        pass
    ok = True
    for sid in range(1, 7):
        try:
            mc.focus_servo(sid)
        except Exception:
            ok = False
    return ok


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

print("Ready. Keys in OpenCV window:")
print("  [m]  : 마커 찾고 EE를 마커 위(Z_OFFSET)로 이동")
print("  [r]  : servo release (torque OFF)")
print("  [a]  : servo activate (torque ON)")
print("  [q]  : 종료")



# =========================
# 메인 루프
# =========================
while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera frame not received.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

    # 표시용
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.putText(frame, "Press 'm' to move EE above marker | 'q' to quit",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    cv2.imshow("Aruco + Move Demo", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    if key == ord('r'):
        ok = servo_release(mc)
        print("[r] Servo release:", "OK" if ok else "FAILED")

    if key == ord('a'):
        ok = servo_activate(mc)
        print("[a] Servo activate:", "OK" if ok else "FAILED")

    if key == ord('q'):
        break

    if key == ord('m'):

        servo_activate(mc)

        if ids is None:
            print("[m] No markers detected.")
            continue
        
        ids_flat = ids.flatten().astype(int)
        if TARGET_MARKER_ID not in ids_flat:
            print(f"[m] Target ID {TARGET_MARKER_ID} not found. detected={ids_flat.tolist()}")
            continue

        idx = int(np.where(ids_flat == TARGET_MARKER_ID)[0][0])

        # 1) cam->target (OpenCV estimatePoseSingleMarkers는 일반적으로 cam->marker를 반환)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            [corners[idx]], MARKER_LENGTH_M, K, dist
        )
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        T_c2t = rvec_tvec_to_T(rvec, tvec)

        # 2) base->gripper
        coords = mc.get_coords()
        if not coords or len(coords) < 6:
            print("[m] mc.get_coords() failed.")
            continue
        T_b2g = mycobot_coords_to_T_b2g(coords)

        # 3) base->target = base->gripper * gripper->camera * camera->target
        T_b2t = T_b2g @ T_g2c @ T_c2t

        # 4) 목표 EE 위치: 마커 위치(base) 위로 Z_OFFSET만큼 (base z축 방향으로 올림)
        target_pos_b = T_b2t[:3, 3].copy()
        target_pos_b[2] += Z_OFFSET

        # 5) EE 회전은 일단 "현재 로봇 자세 유지" (가장 안전)
        #    myCobot send_coords는 [x,y,z,rx,ry,rz] (mm, deg) 형태로 움직일 수 있음
        #    현재 rx,ry,rz를 유지하고 xyz만 바꿔서 이동
        cur_coords = coords[:]  # mm, deg
        cur_rx, cur_ry, cur_rz = cur_coords[3], cur_coords[4], cur_coords[5]

        x_mm = float(target_pos_b[0] * 1000.0)
        y_mm = float(target_pos_b[1] * 1000.0)
        z_mm = float(target_pos_b[2] * 1000.0)



        print("\n[m] Marker pose (cam->target): tvec(m) =", tvec)
        print("[m] Target position in base (m) =", T_b2t[:3, 3])
        print("[m] Marker rotation in base (m) =", T_b2t[:3, :3])
        print("[m] Command EE to (mm) =", [x_mm, y_mm, z_mm], " keep rpy(deg) =", [cur_rx, cur_ry, cur_rz])

        # 6) 이동 명령 (간단 데모)
        # send_coords(coords, speed, mode)
        # mode는 펌웨어/버전에 따라 다를 수 있어 기본 0 사용. (문제 있으면 1/2로 바꿔 테스트)
        mc.send_coords([x_mm, y_mm, z_mm, cur_rx, cur_ry, cur_rz], MOVE_SPEED, 0)

        # 약간 대기 (로봇이 움직이는 동안 너무 자주 명령하면 흔들릴 수 있음)
        time.sleep(1.0)


# =========================
# 종료 처리
# =========================
cap.release()
cv2.destroyAllWindows()
print("Done.")
