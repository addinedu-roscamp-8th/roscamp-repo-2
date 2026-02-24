import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

mc = MyCobot280('/dev/ttyUSB0', 1000000)

angles = mc.get_angles()
print(angles)

mc.send_angles([0, 0, 0, 0, 0, 0], 30)
