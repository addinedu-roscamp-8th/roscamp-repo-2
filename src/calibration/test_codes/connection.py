import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

mc = MyCobot280('/dev/ttyUSB0', 1000000)
mc.thread_lock = True

print("로봇이 연결되었습니다.")