import time
import math
import numpy as np

from kinematic import Kinematic
from robot import RobotArm

k = Kinematic()
robot_arm = RobotArm()

robot_arm.reference()
time.sleep(1)

robot_arm.home()
time.sleep(1)

q = k.backward([-150,-150,60], np.array([0,0,-1]))
robot_arm.pos(q)
time.sleep(1)

q = k.backward([-150,-150,60], np.array([-math.sqrt(2)/2,0,-math.sqrt(2)/2]))
robot_arm.pos(q)
time.sleep(1)

for i in range(0,101,5):
    x = -math.sqrt(2)/2 + i/100 * math.sqrt(2)/2
    y = 0 - i/100 * math.sqrt(2)/2
    tcp_n = np.array([x,y,-math.sqrt(2)/2])
    tcp_n /= np.linalg.norm(tcp_n)
    q = k.backward([-150,-150,60], tcp_n)
    robot_arm.pos(q)