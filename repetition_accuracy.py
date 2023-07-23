import time
import math
import numpy as np
import yaml

from robotic_arm.kinematic import Kinematic, KinematicConfig
from robotic_arm.robot import AxesConfig, RobotArm
from robotic_arm.transform import Transform

with open("robot_config.yaml") as f:
    robot_config = yaml.safe_load(f)

axes_config = AxesConfig(**robot_config["axes"])
kinematic_config = KinematicConfig(**robot_config["kinematic"])
kinematic = Kinematic(kinematic_config)

robot_arm = RobotArm(axes_config, kinematic)

robot_arm.reference()
time.sleep(1)

robot_arm.home()
time.sleep(1)

for i in range(10):
    robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate([-82.5, -202.5, 60]))
    time.sleep(0.5)
    robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate([-86, -191, 23]))
    time.sleep(1)
    robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate([-82.5, -202.5, 60]))
    robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate([-158, -191, 60]))
    time.sleep(0.5)
    robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate([-158, -191, 25]))
    time.sleep(1)
    robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate([-158, -191, 60]))

# final position
robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate([-158, -191, 25]))