import time
import math
import numpy as np

from robotic_arm.kinematic import Kinematic
from robotic_arm.robot import RobotArm
from robotic_arm.transform import Transform

k = Kinematic()
robot_arm = RobotArm()

robot_arm.reference()
time.sleep(1)

robot_arm.home()
time.sleep(1)

for i in range(-60,-150,-10):
    tf = Transform().rotate_x(math.pi).translate([-150, i, 60])
    q = k.backward(tf)
    robot_arm.pos(q)