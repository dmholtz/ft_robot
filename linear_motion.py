import time
import math
import numpy as np
import yaml

from robotic_arm.kinematic import Kinematic, KinematicConfig
from robotic_arm.robot import AxesConfig, RobotArm
from robotic_arm.transform import Transform

with open("robot_config.yaml") as f:
    robot_config = yaml.safe_load(f)

kinematic_config = KinematicConfig(**robot_config["kinematic"])
k = Kinematic(kinematic_config)

axes_config = AxesConfig(**robot_config["axes"])
robot_arm = RobotArm(axes_config)

robot_arm.reference()
time.sleep(1)

robot_arm.home()
time.sleep(1)

for i in range(-60,-150,-10):
    tf = Transform().rotate_x(math.pi).translate([-150, i, 60])
    q = k.backward(tf)
    robot_arm.pos(q)