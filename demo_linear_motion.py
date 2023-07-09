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

for i in range(90,-1,-10):
    q = k.backward([-150,-150,i], np.array([0,0,-1]))
    robot_arm.pos(q)