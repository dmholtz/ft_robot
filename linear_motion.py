from lib.controller import *

import time
import math
import numpy as np
import yaml

from robotic_arm.kinematic import Kinematic, KinematicConfig
from robotic_arm.robot import AxesConfig, RobotArm
from robotic_arm.transform import Transform

with open("/opt/ft/workspaces/ft_robot/robot_config.yaml") as f:
    robot_config = yaml.safe_load(f)

axes_config = AxesConfig(**robot_config["axes"])
kinematic_config = KinematicConfig(**robot_config["kinematic"])
kinematic = Kinematic(kinematic_config)

robot_arm = RobotArm(axes_config, kinematic)

robot_arm.reference()
time.sleep(1)

robot_arm.home()
time.sleep(1)

listpos = {
    'home1': [ -100,  -10,  130],
    'home2': [  -80, -100,  130], #home for din, cs, nfc, nio
    'home3': [  100,   10,  100], #home for fts1, fts2, dout
    
    'din':   [  -80, -235,   80],
    'din2':  [  -80, -200,  120],
    'nfc':   [   25, -230,  100],
    'nfc2':  [   25, -200,  120],
    'cs':    [-31.5, -230,  100],
    'cs2':   [-31.5, -200,  120],
    'nio':   [   40, -160,   90],

    'fts1':  [ 120,   120,  110],
    'fts2':  [ 130,   174,  110],
    'fts3':  [ 140,   174,   90],
    'dout':  [ 80,   -200,  120]
}

def grip():
    TXT_M_O7_compressor.on()
    time.sleep(1)
    TXT_M_O8_magnetic_valve.on()
    time.sleep(1)

def drop():
    TXT_M_O7_compressor.off()
    TXT_M_O8_magnetic_valve.off()
    time.sleep(1)

def move_ptp(pos_name, wait_s=0):
    rad = np.radians(180)
    robot_arm.pos_cartesian(Transform().rotate_x(rad).translate(listpos[pos_name]))
    time.sleep(wait_s)


move_ptp('home1')
move_ptp('home2')
move_ptp('din', 1)
grip()
move_ptp('din2', 1)
move_ptp('home2')
move_ptp('nfc', 1)
move_ptp('nfc2')
move_ptp('home2')
move_ptp('cs', 1)
move_ptp('cs2')
move_ptp('home2')
move_ptp('nfc', 1)
move_ptp('nfc2')
move_ptp('home2')
move_ptp('nio', 1)
drop()
move_ptp('home1')

robot_arm.home()

move_ptp('home1')
move_ptp('home2')
move_ptp('home3')
move_ptp('fts1')
move_ptp('fts2')
move_ptp('fts3', 1)
grip()
move_ptp('fts2')
move_ptp('fts1')
move_ptp('home3')
move_ptp('dout', 1)
drop()
move_ptp('home2')

robot_arm.home()
