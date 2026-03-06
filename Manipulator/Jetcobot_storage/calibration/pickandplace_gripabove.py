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
MARKER_LENGTH_M = 0.025          # ArUco 실제 한 변 길이(m). 반드시 정확히!
GRIPPER_OFFSET = 0.1             # End Effector 와 Gripper의 오프셋 ~> 측정 상 대략 10cm
Z_OFFSET = 0.05                     # 마커 위로 접근 오프셋(m) (충돌 방지용)
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


# T_g2c = np.array([[ 0.85597414,  0.51699369,  0.00507874, -0.02438041],
#  [-0.51688963,  0.85549986,  0.03074243, -0.02790223],
#  [ 0.01154878, -0.02893988,  0.99951444, +0.01722133],
#  [ 0.        ,  0.        ,  0.        ,  1.        ]]
# , dtype=np.float64)


T_g2c = np.array([
    [ 0.7071, 0.7071,  0, -0.03394],
    [-0.7071, 0.7071,  0, -0.03394],
    [      0,      0,  1.0,   0.027],
    [      0,      0,  0,      1.0]
], dtype=np.float64)



#layer1
# [[ 0.85597414,  0.51699369,  0.00507874, -0.02438041],
#  [-0.51688963,  0.85549986,  0.03074243, -0.02790223],
#  [ 0.01154878, -0.02893988,  0.99951444, -0.01722133],
#  [ 0.        ,  0.        ,  0.        ,  1.        ]]

#merged layer123
# [[ 8.57986976e-01,  5.13481580e-01,  1.39648065e-02, -1.50842180e-02],
#  [-5.13670857e-01,  8.57627880e-01,  2.48328530e-02, -3.46132180e-02],
#  [ 7.74605256e-04, -2.84795786e-02,  9.99594074e-01, -6.09937520e-03],
#  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]



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

import numpy as np
from scipy.spatial.transform import Rotation as R

def T_base2target_to_mycobot_rxyz_deg(
    T_base2target: np.ndarray,
    R_tool_from_target: np.ndarray | None = None,
    euler_seq: str = "ZYX",
):
    """
    Args
    - T_base2target: (4,4) base->target transform
    - R_tool_from_target: (3,3) optional fixed rotation offset (target->tool).
        If your target frame axes differ from desired tool axes, set this.
        (Example: rotate tool 180deg about X to point "down".)
    - euler_seq: intrinsic rotation order for myCobot convention. Usually "ZYX".

    Returns
    - rx_deg, ry_deg, rz_deg: myCobot send_coords rotation fields (deg)
      (Interpretation: tool-frame intrinsic rotations consistent with euler_seq)
    """

    T = np.asarray(T_base2target, dtype=np.float64)
    assert T.shape == (4, 4), "T must be 4x4"

    R_b_t = T[:3, :3]

    # 원하는 tool 자세가 target 자세와 동일하면 offset=None
    # target->tool 오프셋이 있으면 base->tool = base->target * target->tool
    if R_tool_from_target is not None:
        R_t_tool = np.asarray(R_tool_from_target, dtype=np.float64)
        assert R_t_tool.shape == (3, 3), "R_tool_from_target must be 3x3"
        R_b_tool = R_b_t @ R_t_tool
    else:
        R_b_tool = R_b_t

    # myCobot: tool(frame) 기준 intrinsic ZYX 라면 SciPy는 'ZYX' (대문자) 사용
    # 결과는 [Z, Y, X] 순서로 나오므로 -> rz, ry, rx로 매핑
    z_deg, y_deg, x_deg = R.from_matrix(R_b_tool).as_euler(euler_seq, degrees=True)
    print("Sending gripper to Calculated euler angles(ZYX) \n"
          "Z: ", z_deg,
          "Y: ", y_deg,
          "X: ", x_deg
          )
    

    # send_coords는 [x,y,z, rx,ry,rz] 이므로
    rx_deg = float(x_deg)
    ry_deg = float(y_deg)
    rz_deg = float(z_deg)
    return rx_deg, ry_deg, rz_deg

def R_x_deg(deg: float) -> np.ndarray:
    return R.from_euler("X", deg, degrees=True).as_matrix()

# 예: target프레임 대비 tool을 X축으로 180도 뒤집기
R_tool_from_target = R_x_deg(180.0)


def T_to_mycobot_coords_mm_deg(T):
    """base->(tool) 4x4 -> [x_mm,y_mm,z_mm, rx_deg,ry_deg,rz_deg]"""
    t = T[:3, 3]
    rot = R.from_matrix(T[:3,:3])
    z_deg, y_deg, x_deg = rot.as_euler('ZYX', degrees=True)  # [Z,Y,X]
    x_mm = float(t[0] * 1000.0)
    y_mm = float(t[1] * 1000.0)
    z_mm = float(t[2] * 1000.0)
    rx = float(x_deg)
    ry = float(y_deg)
    rz = float(z_deg)
    return [x_mm-10, y_mm+2, z_mm, rx, ry, rz]

def make_T(Rm: np.ndarray, t: np.ndarray):
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = Rm
    T[:3, 3] = t.reshape(3)
    return T

def pose_to_quat(rvec):
    """Rodrigues rvec -> quaternion (x,y,z,w)"""
    Rm, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    q = R.from_matrix(Rm).as_quat()  # (x,y,z,w)
    return q

def quat_to_rvec(q):
    """quaternion (x,y,z,w) -> Rodrigues rvec"""
    Rm = R.from_quat(q).as_matrix()
    rvec, _ = cv2.Rodrigues(Rm)
    return rvec.reshape(3)

def average_quaternions(quats):
    """
    Markley quaternion average (robust mean)
    quats: list of np.array shape(4,) (x,y,z,w)
    """
    Q = np.array(quats, dtype=np.float64)
    # sign consistency: flip quats to be close to first quat
    q0 = Q[0]
    for i in range(len(Q)):
        if np.dot(Q[i], q0) < 0:
            Q[i] = -Q[i]

    # Markley method: eigenvector of sum(q q^T)
    A = np.zeros((4, 4), dtype=np.float64)
    for q in Q:
        A += np.outer(q, q)
    A /= len(Q)

    eigvals, eigvecs = np.linalg.eigh(A)
    q_avg = eigvecs[:, np.argmax(eigvals)]
    # normalize
    q_avg /= np.linalg.norm(q_avg) + 1e-12
    return q_avg

def robust_pose_from_samples(tvec_list, rvec_list, method="auto"):
    """
    tvec_list: list of (3,)
    rvec_list: list of (3,)
    method:
      - "median": translation median + rotation quat mean
      - "mean":   translation mean + rotation quat mean
      - "auto":   튐이 있으면 median, 아니면 mean (translation만)
    return: tvec_final(3,), rvec_final(3,)
    """
    T = np.array(tvec_list, dtype=np.float64)  # (N,3)

    # translation: median vs mean 판단(자동)
    t_mean = np.mean(T, axis=0)
    t_median = np.median(T, axis=0)

    # 튐 정도 체크: 각 샘플이 median에서 얼마나 벗어나는지
    dev = np.linalg.norm(T - t_median[None, :], axis=1)
    med_dev = np.median(dev)
    max_dev = np.max(dev)

    if method == "auto":
        # 경험적으로: max_dev가 median_dev의 3배 이상이면 outlier 많다고 보고 median 택
        use_median = (med_dev > 1e-6) and (max_dev > 3.0 * med_dev)
    elif method == "median":
        use_median = True
    else:
        use_median = False

    t_final = t_median if use_median else t_mean

    # rotation: quaternion 평균 (회전은 무조건 quat mean이 안전)
    quats = [pose_to_quat(rv) for rv in rvec_list]
    q_avg = average_quaternions(quats)
    r_final = quat_to_rvec(q_avg)

    return t_final, r_final, use_median, float(med_dev), float(max_dev)

def collect_marker_pose_samples(
    cap, aruco_dict, params, K, dist,
    target_id: int,
    marker_length_m: float,
    n_samples: int = 20,
    max_trials: int = 200,
    inter_delay_sec: float = 0.02,
):
    """
    카메라에서 target_id 마커를 n_samples개 모을 때까지 반복 캡처.
    return: (tvec_list, rvec_list)  or (None, None)
    """
    tvec_list = []
    rvec_list = []

    trial = 0
    while len(tvec_list) < n_samples and trial < max_trials:
        trial += 1
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is None:
            time.sleep(inter_delay_sec)
            continue

        ids_flat = ids.flatten().astype(int)
        if target_id not in ids_flat:
            time.sleep(inter_delay_sec)
            continue

        idx = int(np.where(ids_flat == target_id)[0][0])

        # pose 추정
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            [corners[idx]], marker_length_m, K, dist
        )
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        # ✅ flip 방지 힌트(선택)
        # tvec[2]는 보통 카메라 앞쪽(+z)이므로 음수면 이상치로 버림
        if tvec[2] < 0:
            time.sleep(inter_delay_sec)
            continue

        tvec_list.append(tvec.copy())
        rvec_list.append(rvec.copy())

        time.sleep(inter_delay_sec)

    if len(tvec_list) < n_samples:
        return None, None

    return tvec_list, rvec_list



# =========================
# ✅ EE(gripper) -> TOOL 변환 (실측 기반)
# 1) +z 방향 0.1 m
# 2) z축 기준 -45 deg 회전
# =========================

t_g2tool = np.array([0.0, 0.0, 0.1], dtype=np.float64)
R_g2tool = R.from_euler('Z', -45.0, degrees=True).as_matrix()
T_g2tool = make_T(R_g2tool, t_g2tool)


# =========================
# ✅ target -> tool desired (집기 자세 정의)
# - tool z축이 target z축의 -방향을 보도록: 180deg X flip 같은 것 사용 가능
# - 여기서는 “마커 평면으로 내려찍기” 기본값: target 기준 X축 180도 회전
# =========================
R_t2tool_align = R.from_euler('X', 180.0, degrees=True).as_matrix()

def T_t2tool_desired_with_offset(z_offset: float):
    # target 좌표계에서 +z 방향으로 z_offset 만큼 띄운 위치
    t = np.array([0.0, 0.0, z_offset], dtype=np.float64)
    return make_T(R_t2tool_align, t)

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

    cv2.putText(frame, "Press 'm' to move EE above marker | 'a' to activate servos | 'r' to release servos | 'g' to activate gripper | 'o' to open gripper | 'q' to quit " ,
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    cv2.imshow("Aruco + Move Demo", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'): #quit, shut down
        break

    if key == ord('h'):
        mc.send_angles([-85.86, 33.22, -67.85, -42.8, 4.21, 47.02], 30)
        time.sleep(1.0)


    if key == ord('c'):
        print(mc.get_angles())

    if key == ord('g'): #grip
        mc.set_gripper_value(20, 50)
        time.sleep(1)
        
    if key == ord('o'): #open
        mc.set_gripper_value(100, 50)
        time.sleep(1)

    if key == ord('a'): #servo 활성화
        mc.focus_all_servos()
        
    if key == ord('r'): #servo 비활성화
        mc.release_all_servos()

    if key == ord('d'): #down motion
        curr_x, curr_y, curr_z, curr_rx, curr_ry, curr_rz= mc.get_coords()
        mc.send_coords([curr_x, curr_y, curr_z-Z_OFFSET*1000,curr_rx,curr_ry,curr_rz], 20, 1)
        print("[m] moving downward -z 10cm in base frame")
        time.sleep(1)
    
    if key == ord('u'): #up motion
        curr_x, curr_y, curr_z, curr_rx, curr_ry, curr_rz= mc.get_coords()
        mc.send_coords([curr_x, curr_y, curr_z+Z_OFFSET*1000,curr_rx,curr_ry,curr_rz], 20, 1)
        print("[m] moving upward +z 10cm in base frame")
        time.sleep(1)
       
    if key == ord('m'): #move --> 마커 좌표 + z offset로 이동 && 마커 회전 성분 xyz allign            
        if ids is None:
            print("[m] No markers detected.")
            continue

        # ✅ 20번 샘플 모으기
        print("[m] Collecting 50 pose samples... hold camera steady.")
        samples = collect_marker_pose_samples(
            cap=cap,
            aruco_dict=aruco_dict,
            params=params,
            K=K,
            dist=dist,
            target_id=TARGET_MARKER_ID,
            marker_length_m=MARKER_LENGTH_M,
            n_samples=50,
            max_trials=250,
            inter_delay_sec=0.01,
        )
        if samples[0] is None:
            print("[m] Failed to collect enough samples (marker unstable / not visible).")
            continue

        tvec_list, rvec_list = samples

        # ✅ robust pose 계산 (translation은 auto(평균/중앙값 자동선택), rotation은 quat mean)
        tvec_final, rvec_final, used_median, med_dev, max_dev = robust_pose_from_samples(
            tvec_list, rvec_list, method="auto"
        )

        print(f"[m] Pose fusion done. translation={'MEDIAN' if used_median else 'MEAN'} "
              f"(med_dev={med_dev:.4f} m, max_dev={max_dev:.4f} m)")
        print("[m] fused tvec(m) =", tvec_final)

        # 1) cam->target
        T_c2t = rvec_tvec_to_T(rvec_final, tvec_final)

        # 2) base->gripper
        coords = mc.get_coords()
        if not coords or len(coords) < 6:
            print("[m] mc.get_coords() failed.")
            continue
        T_b2g = mycobot_coords_to_T_b2g(coords)

        # 3) base->target
        T_b2t = T_b2g @ T_g2c @ T_c2t

        # ✅ target->tool(desired): 마커 위 Z_OFFSET + 다운 정렬
        T_t2tool_des = T_t2tool_desired_with_offset(Z_OFFSET)

        # ✅ base->tool(desired)
        T_b2tool_des = T_b2t @ T_t2tool_des

        T_tool2g = np.linalg.inv(T_g2tool)
        T_b2g_cmd = T_b2tool_des @ T_tool2g

        cmd = T_to_mycobot_coords_mm_deg(T_b2g_cmd)

        print("\n[m] Sending EE pose that results in desired TOOL pose (robust 20 samples):")
        print("    cmd [x,y,z,rx,ry,rz] =", cmd)
        mc.send_coords(cmd, MOVE_SPEED, 0)
        time.sleep(1.0)






        #     print("[m] No markers detected.")
        #     continue

        # ids_flat = ids.flatten().astype(int)
        # if TARGET_MARKER_ID not in ids_flat:
        #     print(f"[m] Target ID {TARGET_MARKER_ID} not found. detected={ids_flat.tolist()}")
        #     continue

        # idx = int(np.where(ids_flat == TARGET_MARKER_ID)[0][0])

        # # 1) cam->target (OpenCV estimatePoseSingleMarkers는 일반적으로 cam->marker를 반환)
        # rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        #     [corners[idx]], MARKER_LENGTH_M, K, dist
        # )
        # rvec = rvecs[0][0]
        # tvec = tvecs[0][0]
        # T_c2t = rvec_tvec_to_T(rvec, tvec)

        # # 2) base->gripper
        # coords = mc.get_coords()
        # if not coords or len(coords) < 6:
        #     print("[m] mc.get_coords() failed.")
        #     continue
        # T_b2g = mycobot_coords_to_T_b2g(coords)

        # # 3) base->target = base->gripper * gripper->camera * camera->target
        # T_b2t = T_b2g @ T_g2c @ T_c2t


        # # ✅ target->tool(desired): 마커 위 Z_OFFSET + 다운 정렬
        # T_t2tool_des = T_t2tool_desired_with_offset(Z_OFFSET)

        # # ✅ base->tool(desired)
        # T_b2tool_des = T_b2t @ T_t2tool_des

        # T_tool2g = np.linalg.inv(T_g2tool)
        # T_b2g_cmd = T_b2tool_des @ T_tool2g

        # cmd = T_to_mycobot_coords_mm_deg(T_b2g_cmd)

        # print("\n[m] Sending EE pose that results in desired TOOL pose:")
        # print("    cmd [x,y,z,rx,ry,rz] =", cmd)
        # mc.send_coords(cmd, MOVE_SPEED, 0)
        # time.sleep(1.0)


#         # 4) 목표 EE 위치: 마커 위치(base) 위로 Z_OFFSET만큼 (base z축 방향으로 올림)
#         target_pos_b = T_b2t[:3, 3].copy()
#         target_pos_b[2] += Z_OFFSET

#         x_mm = float(target_pos_b[0] * 1000.0)
#         y_mm = float(target_pos_b[1] * 1000.0)
#         z_mm = float(target_pos_b[2] * 1000.0)

#         rx, ry, rz = T_base2target_to_mycobot_rxyz_deg(
#         T_b2t,
#         R_tool_from_target,  # 필요 없으면 None
#         euler_seq="ZYX"
# )


#         print("\n[m] Marker pose (cam->target): tvec(m) =", tvec)
#         print("[m] Target position in base (m) =", T_b2t[:3, 3])
#         print("[m] Command EE to (mm) =", [x_mm, y_mm, z_mm], " keep rpy(deg) =", [rx, ry, rz])

#         # 6) 이동 명령 (간단 데모)
#         # send_coords(coords, speed, mode)
#         # mode는 펌웨어/버전에 따라 다를 수 있어 기본 0 사용. (문제 있으면 1/2로 바꿔 테스트)
#         mc.send_coords([x_mm, y_mm, z_mm , rx, ry, rz], MOVE_SPEED, 0)
#         time.sleep(2.0)
#         curr_ang = mc.get_angles()
#         mc.send_angle(6, curr_ang[5]+45, 30)
#         time.sleep(1.0)

        
        


# =========================
# 종료 처리
# =========================
cap.release()
cv2.destroyAllWindows()
print("Done.")
