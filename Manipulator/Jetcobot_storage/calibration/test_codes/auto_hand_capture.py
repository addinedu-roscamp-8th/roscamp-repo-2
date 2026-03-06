import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
import datetime
from scipy.spatial.transform import Rotation as R
from pymycobot.mycobot280 import MyCobot280

# =============================
# 사용자 설정
# =============================
CAM_DEVICE = "/dev/video2"
MYCOBOT_PORT = "/dev/ttyUSB1"
MYCOBOT_BAUD = 1000000

TARGET_MARKER_ID = 3
marker_length = 0.025  # meters (실측 권장)

# camera intrinsics 파일
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
INTRINSICS_NPZ = os.path.join(
    BASE_DIR, "camera_calibration_capture", "session_2026-01-12_16-17-24",
    "camera_calib_upgraded_2026-01-12_16-17-24.npz"
)

# 저장 폴더
TARGET_DIR = os.path.join(BASE_DIR, "hand_eye_calibration")
os.makedirs(TARGET_DIR, exist_ok=True)

# ----- 자동 이동 파라미터 -----
MOVE_SPEED = 25             # 1~100
ANGLE_TOL_DEG = 2.0         # 도착 판단 tolerance
ARRIVE_TIMEOUT_SEC = 20.0   # 이동 타임아웃
SETTLE_SEC = 1.0            # 정착 대기 (진동/동기화 개선)

# ----- 평균/검증 파라미터 -----
ARUCO_AVG_FRAMES = 7        # ArUco 평균 프레임 수
ARUCO_FRAME_INTERVAL = 0.05 # 프레임 샘플 간격
ROBOT_AVG_SAMPLES = 5       # myCobot coords 평균 샘플 수
ROBOT_SAMPLE_INTERVAL = 0.05

MIN_MARKER_AREA_PX = 1500   # 너무 작은 마커(불안정) 자동 skipo
MAX_RVEC_JUMP_DEG = 25.0    # 직전 샘플 대비 회전 튐 제한
MAX_TVEC_JUMP_M = 0.08      # 직전 샘플 대비 이동 튐 제한 (8cm)

# =============================
# 유틸
# =============================
def load_intrinsics(npz_path):
    calib = np.load(npz_path)
    if "camera_matrix" in calib.files:
        K = calib["camera_matrix"].astype(np.float64)
        dist = calib["dist_coeffs"].astype(np.float64)
    else:
        K = calib["mtx"].astype(np.float64)
        dist = calib["dist"].astype(np.float64)
    return K, dist

def servo_activate(mc: MyCobot280):
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

def move_wait(mc: MyCobot280, target_angles, speed, tol_deg=2.0, timeout_s=20.0):
    mc.send_angles(target_angles, speed)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        cur = mc.get_angles()
        if cur and len(cur) >= 6:
            err = [abs(cur[i] - target_angles[i]) for i in range(6)]
            if max(err) <= tol_deg:
                return True
        time.sleep(0.1)
    return False

def marker_area_px(corner4):
    # corner4: (1,4,2) or (4,2)
    pts = np.asarray(corner4, dtype=np.float32).reshape(-1, 2)
    return float(cv2.contourArea(pts))

def rotvec_mean(rotvecs):
    # rotvecs: list of (3,)
    Rs = [R.from_rotvec(v).as_matrix() for v in rotvecs]
    Rm = Rs[0].copy()
    for _ in range(30):
        ws = []
        for Ri in Rs:
            w = R.from_matrix(Rm.T @ Ri).as_rotvec()
            ws.append(w)
        w_mean = np.mean(np.stack(ws, axis=0), axis=0)
        if np.linalg.norm(w_mean) < 1e-10:
            break
        Rm = Rm @ R.from_rotvec(w_mean).as_matrix()
    return R.from_matrix(Rm).as_rotvec()

def quat_from_myCobot_tool_intrinsic_zyx(rx_deg, ry_deg, rz_deg):
    # myCobot: rx,ry,rz(deg). tool frame 기준 intrinsic ZYX 라고 가정
    rot = R.from_euler('ZYX', [rz_deg, ry_deg, rx_deg], degrees=True)
    return rot.as_quat()  # [x,y,z,w]

def average_robot_coords(mc: MyCobot280, n=5, dt=0.05):
    coords_list = []
    for _ in range(n):
        c = mc.get_coords()
        if c and len(c) >= 6:
            coords_list.append(c[:6])
        time.sleep(dt)
    if len(coords_list) == 0:
        return None

    coords = np.asarray(coords_list, dtype=np.float64)  # (n,6) [mm,deg]
    xyz_m = np.mean(coords[:, 0:3], axis=0) / 1000.0

    # Euler 평균은 그냥 평균내면 위험 -> rotvec 평균
    rotvecs = []
    for rx, ry, rz in coords[:, 3:6]:
        q = quat_from_myCobot_tool_intrinsic_zyx(rx, ry, rz)
        rotvecs.append(R.from_quat(q).as_rotvec())
    rv_mean = rotvec_mean(rotvecs)
    q_mean = R.from_rotvec(rv_mean).as_quat()  # [x,y,z,w]
    return xyz_m, q_mean

def sample_aruco_pose(cap, aruco_dict, params, K, dist, marker_len, target_id,
                      n_frames=7, dt=0.05, min_area_px=1500):
    t_list = []
    rv_list = []
    area_list = []

    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            time.sleep(dt)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is None:
            time.sleep(dt)
            continue

        ids_flat = ids.flatten().astype(int)
        if target_id not in ids_flat:
            time.sleep(dt)
            continue

        idx = int(np.where(ids_flat == target_id)[0][0])
        area = marker_area_px(corners[idx])
        if area < min_area_px:
            time.sleep(dt)
            continue

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers([corners[idx]], marker_len, K, dist)
        rvec = rvecs[0][0].astype(np.float64)
        tvec = tvecs[0][0].astype(np.float64)

        t_list.append(tvec)
        rv_list.append(rvec)
        area_list.append(area)

        time.sleep(dt)

    if len(t_list) < max(3, n_frames // 2):
        return None  # 실패

    t_mean = np.mean(np.stack(t_list, axis=0), axis=0)
    rv_mean = rotvec_mean(rv_list)  # Rodrigues(rotvec) 평균
    area_med = float(np.median(area_list))
    return t_mean, rv_mean, area_med

def rot_angle_deg(Ra, Rb):
    return np.degrees(R.from_matrix(Ra.T @ Rb).magnitude())

# =============================
# 자동 포즈 리스트(예시)
# =============================
POSES_DEG = [
    # [j1,j2,j3,j4,j5,j6]  (deg)
    [0,   0,   0,   0,  0,  0],
    [10, -10,  20,  0,  0,  0],
    [-10, -15,  25,  0,  0,  0],
    [20, -20,  30,  10, 0,  0],
    [-20, -20, 30, -10, 0,  0],
    [0,  -30,  35,  0,  10, 0],
    [0,  -30,  35,  0, -10, 0],
    [30, -10,  25,  0,  0,  10],
    [-30,-10,  25,  0,  0, -10],
    [15, -25,  40,  15, 0,  0],
    [-15,-25,  40, -15,0,  0],
    [0,  -15,  20,  0,  0,  20],
    [0,  -15,  20,  0,  0, -20],
]

# =============================
# main
# =============================
def main():
    cap = cv2.VideoCapture(CAM_DEVICE)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {CAM_DEVICE}")

    K, dist = load_intrinsics(INTRINSICS_NPZ)

    mc = MyCobot280(MYCOBOT_PORT, MYCOBOT_BAUD)
    servo_activate(mc)

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

    now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_path = os.path.join(TARGET_DIR, f"{now}_auto_hand_eye_calibration.npz")

    tvecs_list = []   # cam->target
    rvecs_list = []   # cam->target (Rodrigues)
    gpose_list = []   # base->gripper (coords 기반)
    quality_list = [] # [count, area_px, n_aruco_used]

    last_t = None
    last_R = None

    print("\n=== Auto Hand-Eye Capture ===")
    print(" - Move -> arrive check -> settle -> avg ArUco + avg robot coords -> save")
    print(f" - marker_id={TARGET_MARKER_ID}, marker_len={marker_length}m")
    print(f" - poses={len(POSES_DEG)}\n")

    for idx_pose, ang in enumerate(POSES_DEG, start=1):
        print(f"[{idx_pose}/{len(POSES_DEG)}] Moving to angles: {ang}")
        ok = move_wait(mc, ang, MOVE_SPEED, tol_deg=ANGLE_TOL_DEG, timeout_s=ARRIVE_TIMEOUT_SEC)
        if not ok:
            print("  -> ARRIVE TIMEOUT, skip pose\n")
            continue

        time.sleep(SETTLE_SEC)

        # 1) ArUco 평균
        ar = sample_aruco_pose(
            cap, aruco_dict, params, K, dist,
            marker_length, TARGET_MARKER_ID,
            n_frames=ARUCO_AVG_FRAMES,
            dt=ARUCO_FRAME_INTERVAL,
            min_area_px=MIN_MARKER_AREA_PX,
        )
        if ar is None:
            print("  -> ArUco unstable/not found, skip pose\n")
            continue

        t_ct, rv_ct, area_med = ar
        R_ct = R.from_rotvec(rv_ct).as_matrix()

        # 2) Robot 평균
        rob = average_robot_coords(mc, n=ROBOT_AVG_SAMPLES, dt=ROBOT_SAMPLE_INTERVAL)
        if rob is None:
            print("  -> robot coords read fail, skip pose\n")
            continue
        t_bg, q_bg = rob
        R_bg = R.from_quat(q_bg).as_matrix()

        # 3) jump reject (나쁜 샘플 자동 제거)
        if last_t is not None:
            t_jump = np.linalg.norm(t_ct - last_t)
            r_jump = rot_angle_deg(last_R, R_ct)
            if t_jump > MAX_TVEC_JUMP_M or r_jump > MAX_RVEC_JUMP_DEG:
                print(f"  -> Reject (jump) t_jump={t_jump:.3f}m r_jump={r_jump:.1f}deg\n")
                continue

        count = len(gpose_list) + 1
        tvecs_list.append([count, TARGET_MARKER_ID, t_ct[0], t_ct[1], t_ct[2]])
        rvecs_list.append([count, rv_ct[0], rv_ct[1], rv_ct[2]])
        gpose_list.append([t_bg[0], t_bg[1], t_bg[2], q_bg[0], q_bg[1], q_bg[2], q_bg[3]])
        quality_list.append([count, area_med, ARUCO_AVG_FRAMES])

        last_t = t_ct.copy()
        last_R = R_ct.copy()

        print(f"  -> Saved #{count} | area_med={area_med:.0f}px | t_ct={t_ct} m\n")

        # 미리보기 화면(선택)
        ret, frame = cap.read()
        if ret:
            cv2.putText(frame, f"Saved {count}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
            cv2.imshow("Preview", frame)
            cv2.waitKey(1)

    # save
    if len(gpose_list) < 12:
        print(f"\n[WARN] too few samples: {len(gpose_list)} (>= 15~30 recommended)")
    np.savez(
        out_path,
        Target_tvecs=np.array(tvecs_list, dtype=np.float64),
        Target_rvecs=np.array(rvecs_list, dtype=np.float64),
        gripper_pose=np.array(gpose_list, dtype=np.float64),
        quality=np.array(quality_list, dtype=np.float64),

        marker_length=float(marker_length),
        target_marker_id=int(TARGET_MARKER_ID),

        robot_pose_definition="base_to_gripper(get_coords avg)",
        robot_euler_convention="intrinsic_ZYX_tool_frame_deg(rotvec-mean)",
        aruco_pose_definition="cam_to_target(avg frames)",
        capture_settle_sec=float(SETTLE_SEC),
        aruco_avg_frames=int(ARUCO_AVG_FRAMES),
        robot_avg_samples=int(ROBOT_AVG_SAMPLES),
    )
    print(f"\n✅ Saved: {out_path}")

    cap.release()
    cv2.destroyAllWindows()
    print("Done.")

if __name__ == "__main__":
    main()
