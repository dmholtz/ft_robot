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

def continuous_path(from_tcp, to_tcp):
    robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate(from_tcp))
    time.sleep(0.5)
    delta = to_tcp-from_tcp
    delta = delta / np.linalg.norm(delta) * 8
    print(delta)
    current_tcp = from_tcp
    while np.linalg.norm(to_tcp-current_tcp) > np.linalg.norm(delta):
        print(current_tcp)
        robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate(current_tcp))
        current_tcp = current_tcp + delta
    robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate(to_tcp))

left = np.array([-158.0, -191.0, 80.0])
right = np.array([-10.0, -191.0, 80.0])

for i in range(1):
    continuous_path(left, right)
    time.sleep(1)
    continuous_path(right, left)
    time.sleep(1)

# final position
robot_arm.pos_cartesian(Transform().rotate_x(math.pi).translate([-158, -191, 25]))