from lib.controller import *

from typing import List

from robotic_arm.axis import MechanicalAxisConfig, ServoAxisConfig, RobotAxis, ServoAxis
from robotic_arm.constants import ENCODER_STEPS_PER_REVOLUTION, SERVO_PWM_PER_DEGREE
from robotic_arm.kinematic import Kinematic
from robotic_arm.transform import Transform

class AxesConfig:

    def __init__(self, *, axis1, axis2, axis3, axis4, axis5, axis6):
        
        self.axis1 = MechanicalAxisConfig(
            ENCODER_STEPS_PER_REVOLUTION,
            58/360*0.5,
            **axis1,
        )
        self.axis2 = MechanicalAxisConfig(
            ENCODER_STEPS_PER_REVOLUTION,
            40/360*1,
            **axis2,
        )
        self.axis3 = MechanicalAxisConfig(
            ENCODER_STEPS_PER_REVOLUTION,
            40/360*1,
            **axis3,
        )
        self.axis4 = ServoAxisConfig(
            SERVO_PWM_PER_DEGREE,
            **axis4,
        )
        self.axis5 = ServoAxisConfig(
            SERVO_PWM_PER_DEGREE,
            **axis5,
        )
        self.axis6 = ServoAxisConfig(
            SERVO_PWM_PER_DEGREE,
            **axis6,
        )

class RobotArm:

    def __init__(self, axes_config: AxesConfig, kinematic: Kinematic):
        self.config = axes_config
        self.kinematic = kinematic

        self.axis1 = RobotAxis(
            TXT_M_M1_encodermotor, 
            TXT_M_I1_mini_switch, 
            TXT_M_C1_motor_step_counter, 
            self.config.axis1,
        )
        self.axis2 = RobotAxis(
            TXT_M_M2_encodermotor, 
            TXT_M_I2_mini_switch,
            TXT_M_C2_motor_step_counter,
            self.config.axis2,
        )
        self.axis3 = RobotAxis(
            TXT_M_M3_encodermotor,
            TXT_M_I3_mini_switch,
            TXT_M_C3_motor_step_counter,
            self.config.axis3,
        )
        self.axis4 = ServoAxis(
            TXT_M_S1_servomotor,
            self.config.axis4,
        )
        self.axis5 = ServoAxis(
            TXT_M_S2_servomotor,
            self.config.axis5,
        )
        self.axis6 = ServoAxis(
            TXT_M_S3_servomotor,
            self.config.axis6,
        )

        self.axes = [self.axis1, self.axis2, self.axis3, self.axis4, self.axis5, self.axis6]

    def reference(self):
        for axis in self.axes[1:]:
            axis.blocking_home()
        self.axis1.blocking_home()

    def home(self):
        self.pos([0,90,180,180,225,0]) 

    def pos(self, phi: List[float]):
        assert len(phi) == 6

        for axis, phi_i in zip(self.axes, phi):
            axis.async_pos(phi_i)

        while not all(axis.poll_axis() for axis in self.axes):
            pass
        
    def pos_cartesian(self, transform: Transform):
        q = None
        try:
            q = self.kinematic.backward(transform)
        except:
            print("Error: TCP is not reachable.")
        if q:
            self.pos(q)