import time
import math
import numpy as np
import yaml

from robotic_arm.constants import ENCODER_STEPS_PER_REVOLUTION, SERVO_PWM_PER_DEGREE
from robotic_arm.kinematic import Kinematic, KinematicConfig
from robotic_arm.transform import Transform

with open("robot_config.yaml") as f:
    robot_config = yaml.safe_load(f)

kinematic_config = KinematicConfig(**robot_config["kinematic"])
kinematic = Kinematic(kinematic_config)

# min_degree_noise is given by the resolution of the stepper motors and the ratio of gearing
min_steps_per_degree = np.array([
    ENCODER_STEPS_PER_REVOLUTION * 58/360 * 0.5,
    ENCODER_STEPS_PER_REVOLUTION * 40/360 * 1,
    ENCODER_STEPS_PER_REVOLUTION * 40/360 * 1,
    SERVO_PWM_PER_DEGREE,
    SERVO_PWM_PER_DEGREE,
    SERVO_PWM_PER_DEGREE
])

min_degree_noise = 1 / min_steps_per_degree
one_degree_noise = np.ones((6,))

test_tcps = [
    np.array([-150, -150, 10]),
    np.array([-150, -150, 100]),
]

for test_tcp in test_tcps:
    q = kinematic.backward(Transform().rotate_x(math.pi).translate(test_tcp))
    # add noise
    q += one_degree_noise
    tcp_prime = kinematic.forward(q).translation
    error = test_tcp - tcp_prime
    print("Error at tcp={test_tcp}: {error}".format(test_tcp=test_tcp, error=error))