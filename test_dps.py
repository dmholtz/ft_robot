from lib.controller import *

import time
import math
import numpy as np
import yaml
import logging

from robotic_arm.kinematic import Kinematic, KinematicConfig
from robotic_arm.robot import AxesConfig, RobotArm
from robotic_arm.transform import Transform

logging.basicConfig(level=logging.DEBUG)

with open("/opt/ft/workspaces/ft_robot_ft_dps/robot_config.yaml") as f:
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
    'home3': [  100,   10,  100], #home for fts1, fts2, ft3, dout
    
    'din':   [-109, -176, 120],
    'din2':  [-109, -176,  64],
    'nfc':   [ -15, -190, 120],
    'nfc2':  [ -15, -190,  80],
    'cs':    [ -63, -190, 120],
    'cs2':   [ -63, -190,  80],
    'nio':   [  15, -125, 120],
    'nio2':  [  15, -125,  50],

    'fts1':  [ 130,  100, 120],
    'fts2':  [ 167,  139, 120],
    'fts3':  [ 167,  139,  71],
    
    'dout':  [ 65,  -185,  60],
    'dout2': [ 65,  -170, 120]
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
    robot_arm.pos_cartesian(Transform().rotate_x(np.radians(180)).translate(listpos[pos_name]))
    time.sleep(wait_s)

linear_z_steps = 50
def move_linear_z(pos_name, z, wait_s=0):
    if z>0:
        for i in range(listpos[pos_name][2], abs(z), linear_z_steps):
            tf = Transform().rotate_x(math.pi).translate([listpos[pos_name][0], listpos[pos_name][1], i])
            logging.debug("tf: {p})".format(p=tf.translation))
            robot_arm.pos_cartesian(tf)
    else:
        for i in range(listpos[pos_name][2], abs(z), -linear_z_steps):
            tf = Transform().rotate_x(math.pi).translate([listpos[pos_name][0], listpos[pos_name][1], i])
            logging.debug("tf: {p})".format(p=tf.translation))
            robot_arm.pos_cartesian(tf)
    tf = Transform().rotate_x(math.pi).translate([listpos[pos_name][0], listpos[pos_name][1], listpos[pos_name][2]+z])
    logging.debug("tf: {p})".format(p=tf.translation))
    robot_arm.pos_cartesian(tf)
    time.sleep(wait_s)


def din2nfc():
    move_ptp('home1')
    move_ptp('home2')

    move_linear_z('din', -60, 1)
    grip()
    move_linear_z('din2', 60)

    move_linear_z('cs', -40, 1)
    drop()
    time.sleep(1)
    grip()
    move_linear_z('cs2', 40)

    move_linear_z('nfc', -40, 1)
    drop()
    time.sleep(1)
    grip()
    move_linear_z('nfc2', 40)

def din2nio():
    din2nfc()
    move_ptp('nio2')
    move_ptp('nio', 1)
    drop()

    move_ptp('home1')

def din2fts():
    din2nfc()
    move_ptp('home2')
    move_ptp('home3')

    move_ptp('fts1')
    move_ptp('fts2')
    move_ptp('fts3', 1)
    drop()
    move_ptp('fts2')
    move_ptp('fts1')

    move_ptp('home3')

    move_ptp('home1')

def fts2dout():
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

    move_ptp('dout2')
    move_ptp('dout', 1)
    drop()
    move_ptp('dout2')

    move_ptp('home2')

    robot_arm.home()

#fts2dout()
din2fts()

fts2dout()
din2fts()

din2nio()
