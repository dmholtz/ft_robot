import time
import math
import numpy as np

from typing import List, Tuple
from fischertechnik.controller.Motor import Motor
from lib.controller import *

from denavit_hartenberg import DenavitHartenbergMatrix

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

dh = DenavitHartenbergMatrix(d=10, a=0, alpha=math.pi/2)
print(dh.homogeneous_matrix(0))

mat = dh.homogeneous_matrix(0)
print(np.matmul(mat, np.array([1,0,1,1])))

class Kinematic:

    # lengths (unit: mm)
    l_1 = 92.5
    l_2 = 150
    l_4 = 140
    l_5 = 67.5
    l_6 = 33

    # vectors
    p_01 = np.array([0,0,l_1])

    def __init__(self):
        self.d_6 = self.l_5+self.l_6

        # Denavit-Hartenberg matrices
        self.dh_0_1 = DenavitHartenbergMatrix(d=self.l_1, a=0, alpha=math.pi/2)
        self.dh_1_2 = DenavitHartenbergMatrix(d=0, a=self.l_2, alpha=0)
        self.dh_2_3 = DenavitHartenbergMatrix(d=0, a=0, alpha=math.pi/2)

        mat = np.matmul(self.dh_0_1.homogeneous_matrix(0), self.dh_1_2.homogeneous_matrix(2/3*math.pi))
        mat = np.matmul(mat, self.dh_2_3.homogeneous_matrix(math.pi))
        #mat = self.dh_1_2.homogeneous_matrix(2/3*math.pi)
        #mat = self.dh_0_1.homogeneous_matrix(0)
        #print(np.matmul(mat, np.array([-75,129,0,1])))
        print(np.matmul(mat, np.array([0,0,0,1])))

    def backward(self, tcp, tcp_n) -> List[float]:
        
        p_04 = tcp - tcp_n*self.d_6

        # add pi to reach preferred orientation of second joint 
        q_1 = math.atan2(p_04[1], p_04[0]) + math.pi
        print("q_1:", q_1/2/math.pi*360)

        p_14 = p_04 - self.p_01
        l_14 = np.linalg.norm(p_14)

        q_3_prime = math.acos((self.l_2**2 + self.l_4**2 - l_14**2)/(2*self.l_2*self.l_4))
        q_3 = 3/2*math.pi - q_3_prime
        print("q_3:", q_3/2/math.pi*360)

        q_2_prime = math.asin(p_14[2]/l_14)
        print(q_2_prime/2/math.pi*360)
        q_2_prime_prime = math.acos((self.l_2**2 + l_14**2 - self.l_4**2)/(2*self.l_2*l_14))
        print(q_2_prime_prime/2/math.pi*360)
        q_2 = math.pi - q_2_prime - q_2_prime_prime
        print("q_2:", q_2/2/math.pi*360)

        #
        t_0_3 = np.matmul(np.matmul(self.dh_0_1.homogeneous_matrix(q_1), self.dh_1_2.homogeneous_matrix(q_2)), self.dh_2_3.homogeneous_matrix(q_3))
        z_3_0 = np.matmul(t_0_3, np.array([0,0,1,1])) - np.matmul(t_0_3, np.array([0,0,0,1]))
        print("z_3_0:", z_3_0)

        z_3_0_dot_tcp_n = np.dot(z_3_0[:3], tcp_n)
        if z_3_0_dot_tcp_n < 0:
            q_5_prime = math.acos(z_3_0_dot_tcp_n) - math.pi
        else:
            q_5_prime = math.acos(z_3_0_dot_tcp_n)
        q_5 = q_5_prime + math.pi
        print("q_5:", q_5/2/math.pi*360)

        # TODO distinguish singular and non-singular case
        if True:
            # non-singular case
            tcp_n_cross_z_3_0 = np.cross(tcp_n, z_3_0[:3])
            tcp_n_cross_z_3_0 /= np.linalg.norm(tcp_n_cross_z_3_0)
            print("cross:", tcp_n_cross_z_3_0)
            y_3_0 = (np.matmul(t_0_3, np.array([0,1,0,1])) - np.matmul(t_0_3, np.array([0,0,0,1])))[:3]
            print("y_3_0", y_3_0)
            y_3_0_dot_cross = np.dot(y_3_0, tcp_n_cross_z_3_0)
            print("y_3_0_dot_cross:", y_3_0_dot_cross)
            delta_q_4 = math.acos(y_3_0_dot_cross)
            print("delta_q_4:", delta_q_4/2/math.pi*360)

            # determine direction
            x_3_0 = (np.matmul(t_0_3, np.array([1,0,0,1])) - np.matmul(t_0_3, np.array([0,0,0,1])))[:3]
            if np.dot(x_3_0, tcp_n_cross_z_3_0) > 0:
                q_4 = 2 * math.pi - delta_q_4
            else:
                q_4 = delta_q_4
            print("q_4:", q_4)
        else:
            pass

        #q_4 = math.pi
        q_6 = 0

        q = [q_1, q_2, q_3, q_4, q_5, q_6] # radian
        radian_to_degree = lambda rad: rad / 2 / math.pi * 360
        return [deg for deg in map(radian_to_degree, q)]



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
        self.pos([90,90,180,180,225,0]) 

    def pos(self, phi: List[float]):
        assert len(phi) == 6

        for axis, phi_i in zip(self.axes, phi):
            axis.async_pos(phi_i)

        while not all(axis.poll_axis() for axis in self.axes):
            pass

k = Kinematic()

q = k.backward([-150,-150,60], np.array([-math.sqrt(2)/2,0,-math.sqrt(2)/2]))
#q = k.backward([-150,-150,60], np.array([0,-math.sqrt(2)/2,-math.sqrt(2)/2]))
#q = k.backward([-150,-150,60], np.array([0,0,-1]))
print(q)

#assert False


robot_arm = RobotArm()
robot_arm.reference()
time.sleep(1)
robot_arm.home()
time.sleep(1)
robot_arm.pos(q)
assert False
for i in range(90,-1,-10):
    q = k.backward([-150,-150,i], np.array([0,0,-1]))
    robot_arm.pos(q)
#robot_arm.pos([45, 120, 70, 135, 135, 20])
time.sleep(2)

assert False

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