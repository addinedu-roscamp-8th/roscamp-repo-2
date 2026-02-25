import os
import time
import threading

from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord



mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True
print("로봇이 연결되었습니다.")



# # 모터 비활성화
# print("전체 모터를 비활성화합니다.")
# mc.release_all_servos()



print("homepose로 이동합니다.")
homepose = [0, 90, -90, -50, 0, 45]
# homepose = [0, 0, -90, -50, 0, 45]

# homepose = [0, 80, -115, -30, 0, 45]
mc.send_angles(homepose, speed=50)



print("그리퍼를 엽니다.")
mc.set_gripper_value(100, 50)
time.sleep(3)