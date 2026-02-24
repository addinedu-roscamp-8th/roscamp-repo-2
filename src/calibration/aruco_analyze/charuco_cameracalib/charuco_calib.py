import os
import time
import datetime

import cv2
import numpy as np

# =========================
# 측정시 주의사항 (ChArUco)
# - 보드가 영상 "가장자리/중앙/상하좌우" 골고루 나오게
# - 보드가 다양한 거리/각도(roll/pitch/yaw)로 찍히도록
# - 자동노출/자동초점이 흔들리면 코너가 불안정 → 가능하면 OFF
# =========================


# =========================
# 설정 (여기만 수정)
# =========================
CAM_DEVICE = "/dev/video0"

# ChArUco 보드 설정
# (squares_x, squares_y) = 체커 사각형 개수
CHARUCO_SQUARES_X = 5
CHARUCO_SQUARES_Y = 7

# 실제 보드의 길이 단위(미터/센티미터/밀리미터 등 아무거나 OK, 일관성만 유지)
SQUARE_LENGTH_M = 0.039  # 3cm
MARKER_LENGTH_M = 0.023  # 2cm (square_length보다 작아야 함)

# ArUco dictionary (출력한 보드와 동일해야 함)
ARUCO_DICT_NAME = "DICT_6X6_250"
# 사용할 수 있는 옵션 예:
# "DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250", ...

# 목표 샘플 개수
TARGET_SAMPLES = 40

# 코너 검출 성공 시 자동 저장 후 대기(초)  ※ 손으로 카메라 이동 시간
WAIT_AFTER_SAVE_SEC = 3.0

# 연속 저장 방지(같은 자세로 여러 장 저장되는 것 방지)
COOLDOWN_SEC = 0.3

# 캘리브레이션 품질 개선: 나쁜 이미지 자동 제거 비율(0~0.5 권장)
DROP_WORST_RATIO = 0.2   # 상위 20% (오차 큰 이미지) 제거 후 재캘리브레이션

# 최소 유효 이미지(캘리브레이션 실행 가능 조건)
MIN_VALID_IMAGES = 12    # 보통 10~15 이상 추천

# calibrateCamera flags (필요시 조정)
# 예: 과적합 방지용
# flags = cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_SKEW
flags = 0

# 저장 폴더
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "camera_calibration_capture_charuco")
os.makedirs(OUT_DIR, exist_ok=True)


# =========================
# ArUco dict 문자열 -> OpenCV enum
# =========================
def get_aruco_dict(dict_name: str):
    if not hasattr(cv2.aruco, dict_name):
        raise ValueError(f"Unknown ArUco dictionary: {dict_name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))


# =========================
# Detector 생성 (OpenCV 버전 대응)
# =========================
def create_aruco_detector(aruco_dict):
    # OpenCV 4.7+ : ArucoDetector
    try:
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector, True
    except Exception:
        # 구버전 호환
        params = cv2.aruco.DetectorParameters_create()
        return (params, aruco_dict), False


# =========================
# ChArUco 감지 (마커 + charuco corner)
# =========================
def detect_charuco(gray, board, detector, use_new_api: bool):
    """
    return:
      ok (bool), ids, marker_corners, charuco_corners, charuco_ids
    """
    if use_new_api:
        marker_corners, ids, _rejected = detector.detectMarkers(gray)
    else:
        params, aruco_dict = detector
        marker_corners, ids, _rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    if ids is None or len(ids) == 0:
        return False, None, None, None, None

    # marker corners -> charuco corners
    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        markerCorners=marker_corners,
        markerIds=ids,
        image=gray,
        board=board
    )

    # retval: 보간된 코너 수
    if retval is None or retval < 4 or charuco_corners is None or charuco_ids is None:
        # 코너가 너무 적으면 calibration에 부적합
        return False, ids, marker_corners, None, None

    return True, ids, marker_corners, charuco_corners, charuco_ids


# =========================
# 캘리브레이션 + 프레임별 reprojection error 계산
# =========================
def calibrate_and_score_charuco(
    image_paths,
    board,
    detector,
    use_new_api: bool,
    flags=0
):
    all_charuco_corners = []
    all_charuco_ids = []
    used_paths = []
    img_size = None

    for p in image_paths:
        img = cv2.imread(p)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = (gray.shape[1], gray.shape[0])  # (w,h)

        ok, ids, marker_corners, charuco_corners, charuco_ids = detect_charuco(
            gray, board, detector, use_new_api
        )

        if not ok:
            continue

        all_charuco_corners.append(charuco_corners)
        all_charuco_ids.append(charuco_ids)
        used_paths.append(p)

    if len(used_paths) < MIN_VALID_IMAGES:
        raise RuntimeError(f"Valid images too few: {len(used_paths)} (need >= {MIN_VALID_IMAGES}).")

    # cv2.aruco.calibrateCameraCharuco:
    # returns rms, cameraMatrix, distCoeffs, rvecs, tvecs
    rms, K, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_charuco_corners,
        charucoIds=all_charuco_ids,
        board=board,
        imageSize=img_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=flags
    )

    # 프레임별 reprojection error(평균 px)
    per_img_err = []
    for (char_corners, char_ids, rv, tv, path) in zip(all_charuco_corners, all_charuco_ids, rvecs, tvecs, used_paths):
        # 해당 프레임에서 사용된 3D-2D 대응점 만들기
        obj_pts, img_pts = board.matchImagePoints(char_corners, char_ids)
        if obj_pts is None or img_pts is None or len(obj_pts) < 4:
            continue

        proj, _ = cv2.projectPoints(obj_pts, rv, tv, K, dist)
        proj = proj.reshape(-1, 2)
        obs = img_pts.reshape(-1, 2)

        err = np.linalg.norm(proj - obs, axis=1)
        mean_err = float(np.mean(err))
        per_img_err.append((mean_err, path))

    per_img_err.sort(reverse=True, key=lambda x: x[0])

    result = {
        "rms": float(rms),
        "K": K.astype(np.float64),
        "dist": dist.astype(np.float64),
        "used_paths": used_paths,
        "per_img_err": per_img_err,
        "img_size": img_size,
        "n_used": len(used_paths),
    }
    return result


# =========================
# 메인: 자동 캡처 → 1차 캘리브 → 나쁜 이미지 제거 → 재캘리브 → npz 저장
# =========================
def main():
    # 1) 보드 구성
    aruco_dict = get_aruco_dict(ARUCO_DICT_NAME)
    board = cv2.aruco.CharucoBoard(
        (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
        SQUARE_LENGTH_M,
        MARKER_LENGTH_M,
        aruco_dict
    )

    # 2) detector 준비
    detector, use_new_api = create_aruco_detector(aruco_dict)

    # 3) 카메라 열기
    cap = cv2.VideoCapture(CAM_DEVICE)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {CAM_DEVICE}")

    ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_dir = os.path.join(OUT_DIR, f"session_{ts}")
    os.makedirs(session_dir, exist_ok=True)

    print("\n=== Camera Calibration (ChArUco) Auto Capture - Upgraded ===")
    print(f"- device: {CAM_DEVICE}")
    print(f"- board squares: {CHARUCO_SQUARES_X} x {CHARUCO_SQUARES_Y}")
    print(f"- square length: {SQUARE_LENGTH_M} m")
    print(f"- marker length: {MARKER_LENGTH_M} m")
    print(f"- aruco dict: {ARUCO_DICT_NAME}")
    print(f"- target samples: {TARGET_SAMPLES}")
    print(f"- auto save on OK, then wait {WAIT_AFTER_SAVE_SEC}s")
    print(f"- auto drop worst ratio: {DROP_WORST_RATIO:.0%}")
    print("Keys: q/ESC=quit immediately (without calibration)\n")

    saved_paths = []
    last_save_time = 0.0

    try:
        while len(saved_paths) < TARGET_SAMPLES:
            ret, frame = cap.read()
            if not ret:
                print("Camera frame not received")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            ok, ids, marker_corners, charuco_corners, charuco_ids = detect_charuco(
                gray, board, detector, use_new_api
            )

            disp = frame.copy()

            # 마커 표시
            if ids is not None and marker_corners is not None:
                cv2.aruco.drawDetectedMarkers(disp, marker_corners, ids)

            # charuco 코너 표시
            if charuco_corners is not None and charuco_ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(disp, charuco_corners, charuco_ids, (0, 0, 255))

            status = "OK" if ok else "NOT OK"
            cv2.putText(disp, f"Samples: {len(saved_paths)}/{TARGET_SAMPLES}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.putText(disp, f"ChArUco: {status}",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 0) if ok else (0, 0, 255), 2)
            cv2.putText(disp, "Auto-save on OK. Move camera during wait. (q/ESC to quit)",
                        (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            cv2.imshow("Calibration Capture (ChArUco)", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("User aborted.")
                return

            now_t = time.time()

            # 저장 조건: ok + cooldown
            if ok and (now_t - last_save_time) > COOLDOWN_SEC:
                img_name = f"img_{len(saved_paths)+1:03d}_{datetime.datetime.now().strftime('%H-%M-%S')}.jpg"
                img_path = os.path.join(session_dir, img_name)
                cv2.imwrite(img_path, frame)  # 원본 저장
                saved_paths.append(img_path)
                last_save_time = now_t
                print(f"[SAVE] {img_name}  ({len(saved_paths)}/{TARGET_SAMPLES})")

                # 저장 후 대기 (이 동안 카메라 이동)
                t_end = time.time() + WAIT_AFTER_SAVE_SEC
                while time.time() < t_end:
                    ret2, frame2 = cap.read()
                    if not ret2:
                        break
                    remain = t_end - time.time()
                    disp2 = frame2.copy()
                    cv2.putText(disp2, f"Saved. Move camera now... {remain:0.1f}s",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
                    cv2.putText(disp2, f"Samples: {len(saved_paths)}/{TARGET_SAMPLES}",
                                (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                    cv2.imshow("Calibration Capture (ChArUco)", disp2)
                    k2 = cv2.waitKey(1) & 0xFF
                    if k2 == ord('q') or k2 == 27:
                        print("User aborted during wait.")
                        return

        cv2.destroyAllWindows()
        cap.release()

        # =========================
        # 1차 캘리브레이션 + 프레임별 오차 계산
        # =========================
        print("\n=== Calibrate (Pass 1 / ChArUco) ===")
        pass1 = calibrate_and_score_charuco(saved_paths, board, detector, use_new_api, flags=flags)
        print(f"Pass1 used images: {pass1['n_used']} / captured {len(saved_paths)}")
        print(f"Pass1 RMS: {pass1['rms']:.6f}")
        print("Pass1 K:\n", pass1["K"])
        print("Pass1 dist:\n", pass1["dist"])

        print("\nWorst images (mean reproj error px):")
        for e, p in pass1["per_img_err"][:min(10, len(pass1["per_img_err"]))]:
            print(f"{e:.3f} px  -  {os.path.basename(p)}")

        # =========================
        # 나쁜 이미지 제거 후 재캘리브레이션
        # =========================
        if DROP_WORST_RATIO > 0:
            n_drop = int(round(pass1["n_used"] * DROP_WORST_RATIO))
            n_drop = max(0, min(n_drop, pass1["n_used"] - MIN_VALID_IMAGES))  # 최소 개수 보장

            drop_set = set([p for _, p in pass1["per_img_err"][:n_drop]])
            kept_paths = [p for p in pass1["used_paths"] if p not in drop_set]

            print(f"\n=== Recalibrate (Pass 2 / ChArUco) dropping worst {n_drop} / {pass1['n_used']} ===")
            pass2 = calibrate_and_score_charuco(kept_paths, board, detector, use_new_api, flags=flags)
            print(f"Pass2 used images: {pass2['n_used']}")
            print(f"Pass2 RMS: {pass2['rms']:.6f}")
            print("Pass2 K:\n", pass2["K"])
            print("Pass2 dist:\n", pass2["dist"])
        else:
            pass2 = None
            kept_paths = pass1["used_paths"]
            drop_set = set()

        # =========================
        # 결과 저장 (npz)
        # =========================
        out_npz = os.path.join(session_dir, f"camera_calib_charuco_upgraded_{ts}.npz")

        per_err1 = np.array([[e, os.path.basename(p)] for e, p in pass1["per_img_err"]], dtype=object)

        if pass2 is not None:
            per_err2 = np.array([[e, os.path.basename(p)] for e, p in pass2["per_img_err"]], dtype=object)
            K_final = pass2["K"]
            dist_final = pass2["dist"]
            rms_final = pass2["rms"]
        else:
            per_err2 = np.empty((0, 2), dtype=object)
            K_final = pass1["K"]
            dist_final = pass1["dist"]
            rms_final = pass1["rms"]

        np.savez(
            out_npz,
            # 최종 결과
            camera_matrix=K_final.astype(np.float64),
            dist_coeffs=dist_final.astype(np.float64),
            reproj_rms=float(rms_final),

            # pass1 정보
            pass1_rms=float(pass1["rms"]),
            pass1_camera_matrix=pass1["K"].astype(np.float64),
            pass1_dist_coeffs=pass1["dist"].astype(np.float64),
            pass1_used_image_paths=np.array([os.path.abspath(p) for p in pass1["used_paths"]], dtype=str),
            pass1_per_image_error=per_err1,

            # pass2 정보(있으면)
            pass2_rms=float(pass2["rms"]) if pass2 is not None else np.nan,
            pass2_used_image_paths=np.array([os.path.abspath(p) for p in (pass2["used_paths"] if pass2 else [])], dtype=str),
            pass2_per_image_error=per_err2,

            # 드롭된 이미지 목록
            dropped_image_paths=np.array([os.path.abspath(p) for p in sorted(list(drop_set))], dtype=str),

            # 메타
            charuco_squares=np.array([CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y], dtype=np.int32),
            square_length_m=float(SQUARE_LENGTH_M),
            marker_length_m=float(MARKER_LENGTH_M),
            aruco_dict_name=str(ARUCO_DICT_NAME),
            capture_paths=np.array([os.path.abspath(p) for p in saved_paths], dtype=str),
            drop_worst_ratio=float(DROP_WORST_RATIO),
            flags=int(flags),
        )

        print("\nSaved:", out_npz)
        print("✅ Done.")

    finally:
        try:
            cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
