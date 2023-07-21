import time
from typing import Tuple

from fischertechnik.controller.Motor import Motor
from robotic_arm.constants import SERVO_HOME_PWM

class MechanicalAxisConfig:

    def __init__(self,
        steps_per_revolution,
        revolution_per_degree,
        *,
        degree_offset: float, # axis angle offset at reference position
        invert_direction: bool = False # by convention, CCW motor rotation increments axis angle theta
    ):
        self.steps_per_revolution = steps_per_revolution # encoder steps per motor revolution
        self.revolution_per_degree = revolution_per_degree # motor revolutions per degree
        self.degree_offset = degree_offset # degree at reference position
        self.invert_direction = invert_direction

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
        self.count = None
        self.count_last = None
        self.count_diff = None

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
        if target < self.pos:
            if self.mechanical_axis_config.invert_direction:
                direction = Motor.CCW
            else:
                direction = Motor.CW
        else:
            if self.mechanical_axis_config.invert_direction:
                direction = Motor.CW
            else:
                direction = Motor.CCW

        steps = self.mechanical_axis_config.delta_degree_to_steps(delta_degree)
        return steps, direction


    def blocking_home(self):
        #move ref fast
        while self.limit_switch.is_open():
            self.motor.set_speed(int(512), Motor.CCW)
            self.motor.start_sync()
        self.motor.stop_sync()
        #ref revert const distance
        self.motor.set_speed(int(512), Motor.CW)
        self.motor.set_distance(30) #const steps
        while self.motor.is_running():
            pass
        #ref slow
        while self.limit_switch.is_open():
            self.motor.set_speed(int(200), Motor.CCW)
            self.motor.start_sync()
        self.motor.stop_sync()
        self.pos = self.mechanical_axis_config.degree_offset
        time.sleep(0.01)
        self.count_last = self.counter.get_count()
        self.count_diff = 0
        #print("home pos: ", self.pos)

    def blocking_pos(self, degree):
        assert self.pos is not None

        steps, direction = self._compute_motion(degree) 
        self.motor.set_speed(512, direction)
        self.motor.set_distance(steps)
        while self.motor.is_running():
            pass
        
        self.pos = degree
        #print("blocking pos: ", steps, self.pos)

    def async_pos(self, degree):
        assert self.pos is not None

        steps, direction = self._compute_motion(degree) 
        self.motor.set_speed(512, direction)
        self.motor.set_distance(steps)
        self.count = steps
        self._target = degree 
        #print("async_pos ", self._target)

    def poll_axis(self) -> bool:
        assert self._target is not None
        if self.motor.is_running():
            return False
        else:
            self.pos = self._target
            #time.sleep(0.01)
            self.count_last = self.counter.get_count()
            self.count_diff = self.count_last - self.count
            #print("poll_axis (diff, steps, last) ", self.count_diff, self.count, self.count_last)
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