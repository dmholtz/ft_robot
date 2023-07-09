from lib.controller import *

from typing import List

from robotic_arm.axis import MechanicalAxisConfig, ServoAxisConfig, RobotAxis, ServoAxis
from robotic_arm.constants import ENCODER_STEPS_PER_REVOLUTION, SERVO_PWM_PER_DEGREE

class RobotArm:

    def __init__(self):
        self.axis1 = RobotAxis(
            TXT_M_M1_encodermotor, 
            TXT_M_I1_mini_switch, 
            TXT_M_C1_motor_step_counter, 
            MechanicalAxisConfig(ENCODER_STEPS_PER_REVOLUTION, 58/360*0.5,0),
        )
        self.axis2 = RobotAxis(
            TXT_M_M2_encodermotor, 
            TXT_M_I2_mini_switch,
            TXT_M_C2_motor_step_counter,
            MechanicalAxisConfig(ENCODER_STEPS_PER_REVOLUTION, 40/360*1,62),
        )
        self.axis3 = RobotAxis(
            TXT_M_M3_encodermotor,
            TXT_M_I3_mini_switch,
            TXT_M_C3_motor_step_counter,
            MechanicalAxisConfig(ENCODER_STEPS_PER_REVOLUTION, 40/360*1,90),
        )
        self.axis4 = ServoAxis(
            TXT_M_S1_servomotor,
            ServoAxisConfig(SERVO_PWM_PER_DEGREE, 87.5)
        )
        self.axis5 = ServoAxis(
            TXT_M_S2_servomotor,
            ServoAxisConfig(SERVO_PWM_PER_DEGREE, 90)
        )
        self.axis6 = ServoAxis(
            TXT_M_S3_servomotor,
            ServoAxisConfig(SERVO_PWM_PER_DEGREE, -92)
        )

        self.axes = [self.axis1, self.axis2, self.axis3, self.axis4, self.axis5, self.axis6]

    def reference(self):
        for axis in self.axes:
            axis.blocking_home()  

    def home(self):
        self.pos([0,90,180,180,225,0]) 

    def pos(self, phi: List[float]):
        assert len(phi) == 6

        for axis, phi_i in zip(self.axes, phi):
            axis.async_pos(phi_i)

        while not all(axis.poll_axis() for axis in self.axes):
            pass