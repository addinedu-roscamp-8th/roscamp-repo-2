#!/usr/bin/env python3
import time
from pymycobot.mycobot280 import MyCobot280

PORT="/dev/ttyUSB0"; BAUD=1000000; SPEED=30

def wait_inpos(mc, tgt):
    # 너무 빡빡하면 CPU 100% 되니 아주 작은 sleep만
    while mc.is_in_position(tgt, 0) != 1:
        time.sleep(0.01)  # 5ms

mc = MyCobot280(PORT, BAUD)
mc.focus_all_servos()

poses = [[0,0,0,0,0,0],[30,0,0,0,0,0],[60,0,0,0,0,0]]

while True:
    for p in poses:
        # base만 움직이기 (HOME도 base=0으로 처리)
        mc.send_angle(1, p[0], SPEED)
        wait_inpos(mc, p)
