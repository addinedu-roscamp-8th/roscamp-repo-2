import os
import time
import datetime

import cv2
import numpy as np
from pymycobot.mycobot280 import MyCobot280

# ======================
# 설정
# ======================
CAM_DEVICE = "/dev/video2"

# 체커보드 "내부 코너" 수 (cols, rows)
CHESSBOARD_SIZE = (9, 6)

# 캡처 저장 폴더
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "checkerboard_check")
os.makedirs(OUT_DIR, exist_ok=True)

# myCobot
MYCOBOT_PORT = "/dev/ttyUSB1"   # 필요시 수정
MYCOBOT_BAUD = 1000000

# 코너 정밀화 옵션
SUBPIX = True


# ======================
# myCobot servo control
# ======================
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


def main():
    # 로봇 연결
    mc = MyCobot280(MYCOBOT_PORT, MYCOBOT_BAUD)
    servo_enabled = True
    servo_activate(mc)

    # 카메라
    cap = cv2.VideoCapture(CAM_DEVICE)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {CAM_DEVICE}")

    print("\n=== Checkerboard Live Check ===")
    print(f"- chessboard inner corners: {CHESSBOARD_SIZE} (cols,rows)")
    print("Keys: r=release, a=activate, p=save image, q/ESC=quit\n")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera frame not received")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 코너 검출
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        if found and SUBPIX:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 시각화
        disp = frame.copy()
        if found:
            cv2.drawChessboardCorners(disp, CHESSBOARD_SIZE, corners, found)

        status = "FOUND" if found else "NOT FOUND"
        cv2.putText(disp, f"Checkerboard: {status}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.putText(disp, f"Servo: {'ON' if servo_enabled else 'OFF'}  (r=release, a=activate)",
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(disp, "p=save image   q/ESC=quit",
                    (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Checkerboard Live Check", disp)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            ok = servo_release(mc)
            servo_enabled = False if ok else servo_enabled
            print("Servo release:", "OK" if ok else "FAILED")

        elif key == ord('a'):
            ok = servo_activate(mc)
            servo_enabled = True if ok else servo_enabled
            print("Servo activate:", "OK" if ok else "FAILED")

        elif key == ord('p'):
            ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            name = f"check_{ts}_found_{int(found)}.jpg"
            path = os.path.join(OUT_DIR, name)
            cv2.imwrite(path, frame)  # 원본 저장
            print("Saved:", path)

        elif key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    print("✅ 종료 완료")


if __name__ == "__main__":
    main()
