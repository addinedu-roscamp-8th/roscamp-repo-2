import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
import datetime
from scipy.spatial.transform import Rotation as R

from pymycobot.mycobot280 import MyCobot280

# -----------------------------
# 옵션
# -----------------------------
CAM_DEVICE = "/dev/video0"
MYCOBOT_PORT = "/dev/ttyUSB0"
MYCOBOT_BAUD = 1000000

# ✅ Space 한번에 몇 프레임을 모을지
BURST_FRAMES = 40          # 예: 15~40
BURST_DT_SEC = 0.05        # 프레임 간 대기 (카메라 FPS에 맞춰 0~0.05)
MIN_VALID_FRAMES = 20      # 이보다 적으면 샘플 저장 안 함
TOP_AREA_RATIO = 0.6       # marker area 상위 60%만 사용

TARGET_MARKER_ID = 3
SAVE_ONLY_WHEN_SERVO_ON = True

RETURN_TO_INITIAL_ON_EXIT = True
initial_angles = [0, 0, 0, 0, 0, 0]
RETURN_SPEED = 30

marker_length = 0.08  # meters

BASE_DIR = os.getcwd()
INTRINSICS_NPZ = os.path.join(
    BASE_DIR, "hand_eye_calibration", "calibrationv.0.1", "cameracalib_npzdata",
    "camera_calib_upgraded_2026-01-12_16-17-24.npz"
)

TARGET_DIR = os.path.join(BASE_DIR, "hand_eye_calibration")
os.makedirs(TARGET_DIR, exist_ok=True)

# -----------------------------
# myCobot servo utilities
# -----------------------------
def servo_release(mc: MyCobot280):
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

def move_to_initial_angles(mc: MyCobot280, angles_deg, speed=30, wait=True, timeout_s=15.0):
    try:
        mc.send_angles(angles_deg, speed)
    except Exception as e:
        print("[move_to_initial_angles] send_angles error:", e)
        return False
    if not wait:
        return True

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            cur = mc.get_angles()
            if cur and len(cur) >= 6:
                err = [abs(cur[i] - angles_deg[i]) for i in range(6)]
                if max(err) < 2.0:
                    return True
        except Exception:
            pass
        time.sleep(0.2)

    print("[move_to_initial_angles] timeout (may not have reached initial pose)")
    return False

# -----------------------------
# ✅ 로봇 Euler(ZYX) -> quaternion(x,y,z,w)
#    - myCobot: rx,ry,rz(deg)
#    - "tool frame 기준 intrinsic ZYX" 이라고 했으므로 'ZYX' 사용
# -----------------------------
def mycobot_coords_to_pose(coords):
    """
    coords: [x,y,z,rx,ry,rz] (mm, deg)
    returns:
      t (m) shape(3,), quat_xyzw shape(4,)  where quat=[x,y,z,w]
    """
    x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = coords[:6]
    t = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0

    # ✅ intrinsic ZYX (body/tool frame rotations): use 'ZYX'
    # myCobot convention: Z=rz (yaw), Y=ry (pitch), X=rx (roll) in degrees
    rot = R.from_euler('ZYX', [rz_deg, ry_deg, rx_deg], degrees=True)

    qx, qy, qz, qw = rot.as_quat()  # [x,y,z,w]
    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    return t, quat

def rvec_to_quat_xyzw(rvec):
    Rm, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64).reshape(3,1))
    quat = R.from_matrix(Rm).as_quat()  # [x,y,z,w]
    return quat.astype(np.float64)

def quat_xyzw_to_rvec(quat_xyzw):
    Rm = R.from_quat(quat_xyzw).as_matrix()
    rvec, _ = cv2.Rodrigues(Rm.astype(np.float64))
    return rvec.reshape(3).astype(np.float64)

def polygon_area_px(corners_4x2):
    pts = corners_4x2.reshape(-1, 2).astype(np.float32)
    return float(cv2.contourArea(pts))

def average_quaternions_xyzw(quats):
    """
    quats: (N,4) [x,y,z,w], sign 정렬 후 평균 -> 정규화
    """
    q = quats.copy()
    ref = q[0]
    for i in range(len(q)):
        if np.dot(q[i], ref) < 0:
            q[i] *= -1.0
    q_mean = np.mean(q, axis=0)
    q_mean /= np.linalg.norm(q_mean) + 1e-12
    return q_mean

def capture_burst_sample(cap, mc, aruco_dict, params, camera_matrix, dist_coeffs,
                         target_id, marker_length,
                         burst_frames=25, burst_dt=0.03,
                         top_area_ratio=0.6, min_valid=10):
    """
    return:
      t_med (3,), r_rep (3,), t_bg(3,), q_bg(4,)  or None if failed
    """
    t_list, r_list, area_list = [], [], []
    # 로봇 pose는 '버스트 중간'에 한 번만 읽는 게 보통 더 안정적 (측정 순간 기준)
    robot_pose = None

    for k in range(burst_frames):
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is None:
            time.sleep(burst_dt)
            continue

        ids_flat = ids.flatten().astype(int)
        if target_id not in ids_flat:
            time.sleep(burst_dt)
            continue

        idx = int(np.where(ids_flat == target_id)[0][0])
        c = corners[idx]
        area = polygon_area_px(c)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers([c], marker_length, camera_matrix, dist_coeffs)
        rvec = rvecs[0][0].astype(np.float64)
        tvec = tvecs[0][0].astype(np.float64)

        t_list.append(tvec)
        r_list.append(rvec)
        area_list.append(area)

        # 버스트 중간에서 로봇 pose를 1회 읽기
        if robot_pose is None and k >= burst_frames // 2:
            coords = mc.get_coords()
            if coords and len(coords) >= 6:
                t_bg, q_bg = mycobot_coords_to_pose(coords)
                robot_pose = (t_bg, q_bg)

        time.sleep(burst_dt)

    if robot_pose is None:
        coords = mc.get_coords()
        if coords and len(coords) >= 6:
            t_bg, q_bg = mycobot_coords_to_pose(coords)
            robot_pose = (t_bg, q_bg)
        else:
            return None

    if len(t_list) < min_valid:
        return None

    t_arr = np.array(t_list, dtype=np.float64)       # (M,3)
    r_arr = np.array(r_list, dtype=np.float64)       # (M,3)
    a_arr = np.array(area_list, dtype=np.float64)    # (M,)

    # ✅ marker area 큰 것 위주로 top-area만 사용
    if len(a_arr) >= 3:
        thr = np.quantile(a_arr, 1.0 - top_area_ratio)
        keep = a_arr >= thr
        t_arr = t_arr[keep]
        r_arr = r_arr[keep]

    if len(t_arr) < min_valid:
        return None

    # ✅ translation 대표값: median
    t_med = np.median(t_arr, axis=0)

    # ✅ rotation 대표값: rvec->quat -> sign 정렬 -> 평균 -> rvec 복원
    quats = np.array([rvec_to_quat_xyzw(rv) for rv in r_arr], dtype=np.float64)
    q_mean = average_quaternions_xyzw(quats)
    r_rep = quat_xyzw_to_rvec(q_mean)

    t_bg, q_bg = robot_pose
    return t_med, r_rep, t_bg, q_bg


# -----------------------------
# main
# -----------------------------
def main():
    cap = cv2.VideoCapture(CAM_DEVICE)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {CAM_DEVICE}")

    calib = np.load(INTRINSICS_NPZ)
    camera_matrix = calib["camera_matrix"].astype(np.float64)
    dist_coeffs = calib["dist_coeffs"].astype(np.float64)

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

    # 저장: (검증된 정의대로) cam->target, base->gripper 그대로 저장
    tvecs_list = []    # [count, id, tx, ty, tz]  (cam->target, m)
    rvecs_list = []    # [count, rx, ry, rz]      (cam->target, Rodrigues)
    gripper_pose = []  # [x,y,z,qx,qy,qz,qw]      (base->gripper, m + quat xyzw)

    count = 1
    servo_enabled = True

    now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_path = os.path.join(TARGET_DIR, f"{now}_hand_eye_calibration.npz")

    print("\nKeys: [r]=release(torque off), [a]=activate(torque on), [space]=save, [esc]=save&exit")
    print(f"TARGET_MARKER_ID={TARGET_MARKER_ID}, marker_length={marker_length}m\n")

    try:
        while True:
            ret, frame = cap.read()
            key = cv2.waitKey(1) & 0xFF
            if not ret:
                print("Camera frame not received.")
                break

            if key == ord('r'):
                ok = servo_release(mc)
                servo_enabled = False if ok else servo_enabled
                print("Servo release:", "OK" if ok else "FAILED")

            elif key == ord('a'):
                ok = servo_activate(mc)
                servo_enabled = True if ok else servo_enabled
                print("Servo activate:", "OK" if ok else "FAILED")

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

            if ids is not None:
                ids_flat = ids.flatten().astype(int)

                if TARGET_MARKER_ID in ids_flat:
                    idx = int(np.where(ids_flat == TARGET_MARKER_ID)[0][0])

                    # cam->target
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        [corners[idx]], marker_length, camera_matrix, dist_coeffs
                    )

                    aruco.drawDetectedMarkers(frame, [corners[idx]], ids[idx:idx+1])
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.02)

                    if key == 32:
                        sample = capture_burst_sample(
                            cap, mc, aruco_dict, params, camera_matrix, dist_coeffs,
                            TARGET_MARKER_ID, marker_length,
                            burst_frames=BURST_FRAMES,
                            burst_dt=BURST_DT_SEC,
                            top_area_ratio=TOP_AREA_RATIO,
                            min_valid=MIN_VALID_FRAMES
                        )

                        if sample is None:
                            print("[burst] not enough valid frames -> skip")
                            continue

                        t_med, r_rep, t_bg, q_bg = sample

                        # 저장: cam->target 대표값
                        tvecs_list.append([count, TARGET_MARKER_ID, t_med[0], t_med[1], t_med[2]])
                        rvecs_list.append([count, r_rep[0], r_rep[1], r_rep[2]])

                        # 저장: base->gripper (동기화)
                        gripper_pose.append([t_bg[0], t_bg[1], t_bg[2], q_bg[0], q_bg[1], q_bg[2], q_bg[3]])

                        print(f"[{count}] burst-saved (M~{BURST_FRAMES} -> rep)")
                        print("t_med:", t_med, " r_rep:", r_rep)
                        


                  
                        



            cv2.putText(frame, "r:release  a:activate  space:save  esc:exit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(frame, f"Servo:{'ON' if servo_enabled else 'OFF'}  Samples:{len(gripper_pose)}  ID:{TARGET_MARKER_ID}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            cv2.imshow("Aruco Detection (Raw)", frame)

            if key == 27:
                if len(gripper_pose) < 10:
                    print("Too few samples. Collect >= 15~30 samples with diverse rotations.")
                    continue

                
                print(f"\nSaved: {out_path}")
                break

    finally:
        try:
            if RETURN_TO_INITIAL_ON_EXIT:
                np.savez(
                    out_path,
                    Target_tvecs=np.array(tvecs_list, dtype=np.float64),
                    Target_rvecs=np.array(rvecs_list, dtype=np.float64),
                    gripper_pose=np.array(gripper_pose, dtype=np.float64),
                    marker_length=float(marker_length),
                    target_marker_id=int(TARGET_MARKER_ID),
                    robot_euler_convention="intrinsic_ZYX_tool_frame_deg",
                    aruco_pose_definition="cam_to_target",
                    robot_pose_definition="base_to_gripper",
                )
                print("\n[Exit] Activating servos and returning to initial pose...")
                servo_activate(mc)
                move_to_initial_angles(mc, initial_angles, speed=RETURN_SPEED, wait=True, timeout_s=20.0)
        except Exception as e:
            print("[Exit] return-to-initial error:", e)

        cap.release()
        cv2.destroyAllWindows()
        print("✅ Done.")

if __name__ == "__main__":
    main()
