import os
import time
import datetime
import glob

import cv2
import numpy as np

# =========================
# 측정시 주의사항
# 카메라 측정은 다양한 각도에서 이루어져야한다.
#        ㄴ체커 보드가 중앙에 몰려 있는 이미지가 많으면 rms 감소
#        ㄴ체커보드가 기울어지게 + 이미지의 다양한 곳에 위치 하도록 여러번 측정
# =========================


# =========================
# 설정 (여기만 수정)
# =========================
CAM_DEVICE = "/dev/video2"

# 체커보드 "내부 코너" 수 (cols, rows)  ※ 칸 수 아님
CHESSBOARD_SIZE = (9, 6)

# 체커보드 한 칸 크기 (미터)
SQUARE_SIZE_M = 0.025  # 25mm

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

# findChessboardCornersSB 사용 여부 (OpenCV 4+ 권장)
USE_SB = True

# ROI crop 사용 여부 (보통 intrinsic은 crop 안 하는 게 더 안정적)
USE_ROI_CROP = False

# calibrateCamera flags (필요시 조정)
# 과적합 방지용 옵션 예:
#   flags = cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_SKEW
flags = 0

# 저장 폴더
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "camera_calibration_capture")
os.makedirs(OUT_DIR, exist_ok=True)


# =========================
# 체커보드 코너 검출 (SB 포함)
# =========================
def detect_chessboard(gray, chessboard_size, use_sb=True):
    cols, rows = chessboard_size

    if use_sb and hasattr(cv2, "findChessboardCornersSB"):
        # SB는 subpix가 내장되어 더 안정적인 경우가 많음
        sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=sb_flags)
        return found, corners

    # legacy 방식
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
    if not found:
        return False, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


# =========================
# 캘리브레이션 데이터 생성
# =========================
def make_objp(chessboard_size, square_size_m):
    cols, rows = chessboard_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_m
    return objp


# =========================
# 캘리브레이션 + 프레임별 reprojection error 계산
# =========================
def calibrate_and_score(image_paths, chessboard_size, square_size_m, use_sb=True, flags=0, use_roi_crop=False):
    objp = make_objp(chessboard_size, square_size_m)

    objpoints, imgpoints, used_paths = [], [], []
    img_size = None

    for p in image_paths:
        img = cv2.imread(p)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = (gray.shape[1], gray.shape[0])  # (w,h)

        found, corners = detect_chessboard(gray, chessboard_size, use_sb=use_sb)
        if not found or corners is None:
            continue

        objpoints.append(objp)
        imgpoints.append(corners.astype(np.float32))
        used_paths.append(p)

    if len(used_paths) < MIN_VALID_IMAGES:
        raise RuntimeError(f"Valid images too few: {len(used_paths)} (need >= {MIN_VALID_IMAGES}).")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None, flags=flags)

    # 프레임별 reprojection error(평균 px)
    per_img_err = []
    for (objp_i, imgp_i, rv, tv, path) in zip(objpoints, imgpoints, rvecs, tvecs, used_paths):
        proj, _ = cv2.projectPoints(objp_i, rv, tv, K, dist)
        proj = proj.reshape(-1, 2)
        obs = imgp_i.reshape(-1, 2)
        err = np.linalg.norm(proj - obs, axis=1)  # 코너별 오차
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
    cap = cv2.VideoCapture(CAM_DEVICE)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {CAM_DEVICE}")

    ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_dir = os.path.join(OUT_DIR, f"session_{ts}")
    os.makedirs(session_dir, exist_ok=True)

    print("\n=== Camera Calibration (Detached) Auto Capture - Upgraded ===")
    print(f"- chessboard inner corners: {CHESSBOARD_SIZE} (cols,rows)")
    print(f"- square size: {SQUARE_SIZE_M} m")
    print(f"- target samples: {TARGET_SAMPLES}")
    print(f"- auto save on FOUND, then wait {WAIT_AFTER_SAVE_SEC}s")
    print(f"- detection: {'findChessboardCornersSB' if USE_SB else 'findChessboardCorners'}")
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
            found, corners = detect_chessboard(gray, CHESSBOARD_SIZE, use_sb=USE_SB)

            disp = frame.copy()
            if found and corners is not None:
                cv2.drawChessboardCorners(disp, CHESSBOARD_SIZE, corners, True)

            status = "FOUND" if found else "NOT FOUND"
            cv2.putText(disp, f"Samples: {len(saved_paths)}/{TARGET_SAMPLES}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
            cv2.putText(disp, f"Chessboard: {status}",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0) if found else (0,0,255), 2)
            cv2.putText(disp, "Auto-save on FOUND. Move camera during wait. (q/ESC to quit)",
                        (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)

            cv2.imshow("Calibration Capture", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("User aborted.")
                return

            now_t = time.time()
            if found and (now_t - last_save_time) > COOLDOWN_SEC:
                img_name = f"img_{len(saved_paths)+1:03d}_{datetime.datetime.now().strftime('%H-%M-%S')}.jpg"
                img_path = os.path.join(session_dir, img_name)
                cv2.imwrite(img_path, frame)  # 원본(왜곡 포함) 저장
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
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,255), 2)
                    cv2.putText(disp2, f"Samples: {len(saved_paths)}/{TARGET_SAMPLES}",
                                (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
                    cv2.imshow("Calibration Capture", disp2)
                    k2 = cv2.waitKey(1) & 0xFF
                    if k2 == ord('q') or k2 == 27:
                        print("User aborted during wait.")
                        return

        cv2.destroyAllWindows()
        cap.release()

        # =========================
        # 1차 캘리브레이션 + 프레임별 오차 계산
        # =========================
        print("\n=== Calibrate (Pass 1) ===")
        pass1 = calibrate_and_score(saved_paths, CHESSBOARD_SIZE, SQUARE_SIZE_M, use_sb=USE_SB, flags=flags)
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

            print(f"\n=== Recalibrate (Pass 2) dropping worst {n_drop} / {pass1['n_used']} ===")
            pass2 = calibrate_and_score(kept_paths, CHESSBOARD_SIZE, SQUARE_SIZE_M, use_sb=USE_SB, flags=flags)
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
        # - image_paths는 dtype=str로 저장 (pickle 문제 방지)
        # - per-image error는 (err, filename) 형태로 저장
        # =========================
        out_npz = os.path.join(session_dir, f"camera_calib_upgraded_{ts}.npz")

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
            chessboard_size=np.array(CHESSBOARD_SIZE, dtype=np.int32),
            square_size_m=float(SQUARE_SIZE_M),
            capture_paths=np.array([os.path.abspath(p) for p in saved_paths], dtype=str),
            used_sb=int(USE_SB),
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
