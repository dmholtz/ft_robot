import time

from typing import List, Tuple
from fischertechnik.controller.Motor import Motor
from lib.controller import *

ENCODER_STEPS_PER_REVOLUTION = 63.3
SERVO_PWM_PER_DEGREE = 512/180
SERVO_HOME_PWM = 256

class MechanicalAxisConfig:

    def __init__(self, steps_per_revolution, revolution_per_degree, degree_offset):
        self.steps_per_revolution = steps_per_revolution # encoder steps per motor revolution
        self.revolution_per_degree = revolution_per_degree # motor revolutions per degree
        self.degree_offset = degree_offset # degree at reference position

    def delta_degree_to_steps(self, degree) -> int:
        return int(degree * self.revolution_per_degree * self.steps_per_revolution)

class ServoAxisConfig:

    def __init__(self, pwm_per_degree, degree_offset):
        self.pwm_per_degree = pwm_per_degree # PWM value per degree
        self.degree_offset = degree_offset # degree at reference position (PWM=0)

    def degree_to_pwm(self, degree) -> int:
        return int((degree-self.degree_offset)*self.pwm_per_degree)

    def pwm_to_degree(self, pwm) -> float:
        return pwm / self.pwm_per_degree + self.degree_offset


class RobotAxis:

    def __init__(self, motor, limit_switch, counter, mechanical_axis_config):
        self.motor = motor
        self.limit_switch = limit_switch
        self.counter = counter
        self.mechanical_axis_config = mechanical_axis_config

        self.pos = None
        self._target = None
        self.is_running = False

    def async_home(self):
        pass
        #def on_reference_point(event):
        #    print("stop motor")
        #    self.motor.stop_sync()
        #    print("stopped motor")
        #    #self.limit_switch.remove_change_listener("closed", lambda x: print(x))
        #    self.is_running = False  
        #    print("is not running")
        #
        #if self.limit_switch.is_open():   
        #    self.limit_switch.add_change_listener("closed", on_reference_point)
        #    self.is_running = True
        #   
        #    self.motor.set_speed(384, Motor.CCW)
        #    self.motor.start_sync()

    def _compute_motion(self, target: float) -> Tuple[int, int]:
        """Compute (steps, direction) pair to reach given target"""

        delta_degree = abs(target - self.pos)
        if target >= self.pos:
            direction = Motor.CW
        else:
            direction = Motor.CCW
        steps = self.mechanical_axis_config.delta_degree_to_steps(delta_degree)
        return steps, direction


    def blocking_home(self):
        while self.limit_switch.is_open():
            self.motor.set_speed(int(384), Motor.CCW)
            self.motor.start_sync()
        self.motor.stop_sync()
        self.pos = self.mechanical_axis_config.degree_offset

    def blocking_pos(self, degree):
        assert self.pos is not None

        steps, direction = self._compute_motion(degree) 
        self.motor.set_speed(512, direction)
        self.motor.set_distance(steps)
        while self.motor.is_running():
            pass
        
        self.pos = degree

    def async_pos(self, degree):
        assert self.pos is not None

        steps, direction = self._compute_motion(degree) 
        self.motor.set_speed(512, direction)
        self.motor.set_distance(steps)
        self._target = degree 

    def poll_axis(self) -> bool:
        assert self._target is not None
        if self.motor.is_running():
            return False
        else:
            self.pos = self._target
            return True 


class ServoAxis:

    def __init__(self, servo, servo_axis_config):
        self.servo = servo
        self.servo_axis_config = servo_axis_config

        self.pos = None

    def blocking_home(self):
        self.servo.set_position(SERVO_HOME_PWM)
        self.pos = self.servo_axis_config.pwm_to_degree(SERVO_HOME_PWM)

    def blocking_pos(self, degree):
        assert self.pos is not None

        pwm = self.servo_axis_config.degree_to_pwm(degree)
        self.servo.set_position(pwm)
        
        self.pos = degree

    def async_pos(self, degree):
        self.blocking_pos(degree)

    def poll_axis(self):
        return True

class Kinematic:

    def __init__(self):
        pass


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
            MechanicalAxisConfig(ENCODER_STEPS_PER_REVOLUTION, 40/360*1,0),
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
        self.pos([90,90,90,180,225,0]) 

    def pos(self, phi: List[float]):
        assert len(phi) == 6

        for axis, phi_i in zip(self.axes, phi):
            axis.async_pos(phi_i)

        while not all(axis.poll_axis() for axis in self.axes):
            pass  


robot_arm = RobotArm()
robot_arm.reference()
time.sleep(1)
robot_arm.home()
time.sleep(2)
robot_arm.pos([45, 120, 70, 135, 135, 20])
time.sleep(2)

for x,y,z in zip(range(95,265), range(95,265), range(-85,85)):
    for i in range(4):
        time.sleep(0.001)
        robot_arm.axis4.blocking_pos(x+i/4)
        robot_arm.axis5.blocking_pos(y+i/4)
        robot_arm.axis6.blocking_pos(z+i/4)



#robot_arm.axis1.async_home()
#while robot_arm.axis1.is_running:
#    pass
#robot_arm.axis1.limit_switch.remove_change_listener("closed", lambda x: print(x))