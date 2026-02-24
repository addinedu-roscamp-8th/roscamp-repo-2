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


mc = MyCobot280(MYCOBOT_PORT, MYCOBOT_BAUD)
SAFE_COORDS = [150, 0, 200, 0, 0, 0]  # [mm, mm, mm, deg, deg, deg]

mc.send_angles([0,0,0,0,0,45],30)
time.sleep(1)



print("Send to zero")
mc.send_angles([0,0,0,0,0,0],30)
time.sleep(1)

for i in range(1,10):
    print(mc.get_coords())




# curr_x, curr_y, curr_z, curr_rx, curr_ry, curr_rz =mc.get_coords()

# print(curr_x, curr_y, curr_z, curr_rx, curr_ry, curr_rz)


# mc.send_coord(4, 90, 30, 0)
# time.sleep(1)
# print(curr_rx, curr_ry, curr_rz)

# print("Send to Coords")
# mc.send_coords([curr_x, curr_y, curr_z, 0, 90, 0],30,0)
# time.sleep(1)
# curr_x, curr_y, curr_z, curr_rx, curr_ry, curr_rz =mc.get_coords()
# print(curr_x, curr_y, curr_z, curr_rx, curr_ry, curr_rz)




print("Done.")
