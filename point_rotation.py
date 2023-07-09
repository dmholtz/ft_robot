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

tf = Transform().rotate_x(3/4*math.pi).translate([-150, -150, 60])
q = k.backward(tf)
robot_arm.pos(q)
time.sleep(1)

for i in range(0,101,10):
    tf = Transform().rotate_x((3/4+1/4*i/100)*math.pi).rotate_y(-math.pi/4*i/100).translate([-150, -150, 60])
    q = k.backward(tf)
    robot_arm.pos(q)