import numpy as np
import math
import logging

from typing import List

from robotic_arm.denavit_hartenberg import DenavitHartenbergMatrix
from robotic_arm.transform import Transform

class KinematicConfig:

    def __init__(self, *, l_1: float, l_2: float, l_4: float, l_5: float, l_6: float):
        self.l_1 = l_1
        self.l_2 = l_2
        self.l_4 = l_4
        self.l_5 = l_5
        self.l_6 = l_6

class Kinematic:    

    def __init__(self, kinematic_config: KinematicConfig):
        self.config = kinematic_config

        # vectors
        self.p_01 = np.array([0,0,self.config.l_1])

        # distances
        self.d_6 = self.config.l_5+self.config.l_6

        # Denavit-Hartenberg matrices
        self.dh_0_1 = DenavitHartenbergMatrix(d=self.config.l_1, a=0, alpha=math.pi/2)
        self.dh_1_2 = DenavitHartenbergMatrix(d=0, a=self.config.l_2, alpha=0)
        self.dh_2_3 = DenavitHartenbergMatrix(d=0, a=0, alpha=math.pi/2)
        self.dh_3_4 = DenavitHartenbergMatrix(d=self.config.l_4, a=0, alpha=math.pi/2)
        self.dh_4_5 = DenavitHartenbergMatrix(d=0, a=0, alpha=math.pi/2)
        self.dh_5_6 = DenavitHartenbergMatrix(d=self.d_6, a=0, alpha=0)
        self.dh_matrices = [
            self.dh_0_1,
            self.dh_1_2,
            self.dh_2_3,
            self.dh_3_4,
            self.dh_4_5,
            self.dh_5_6,
        ]

    def forward(self, q_deg: List[float]) -> Transform:
        """Compute the forward kinematic given a list of six angles in degree.
        """

        assert len(q_deg) == 6, "len(q_deg) should be 6"

        deg_to_rad = lambda deg: deg/360*2*math.pi
        q_rad = [deg_to_rad(q_i) for q_i in q_deg]        

        homogeneous_matrix = np.eye(4)
        for dh_matrix, q_i in zip(self.dh_matrices, q_rad):
            homogeneous_matrix = np.matmul(homogeneous_matrix, dh_matrix.homogeneous_matrix(q_i))

        return Transform(homogeneous_matrix)


    def backward(self, transform: Transform) -> List[float]:

        tcp = transform.translation
        tcp_n = transform.z
        
        p_04 = tcp - tcp_n*self.d_6

        # add pi to reach preferred orientation of second joint 
        q_1 = math.atan2(p_04[1], p_04[0]) + math.pi
        logging.debug("q_1={q}".format(q=q_1/2/math.pi*360))

        p_14 = p_04 - self.p_01
        l_14 = np.linalg.norm(p_14)

        q_3_prime = math.acos((self.config.l_2**2 + self.config.l_4**2 - l_14**2)/(2*self.config.l_2*self.config.l_4))
        q_3 = 3/2*math.pi - q_3_prime
        logging.debug("q_3={q}".format(q=q_3/2/math.pi*360))

        q_2_prime = math.asin(p_14[2]/l_14)
        #print(q_2_prime/2/math.pi*360)
        q_2_prime_prime = math.acos((self.config.l_2**2 + l_14**2 - self.config.l_4**2)/(2*self.config.l_2*l_14))
        #print(q_2_prime_prime/2/math.pi*360)
        q_2 = math.pi - q_2_prime - q_2_prime_prime
        logging.debug("q_2={q}".format(q=q_2/2/math.pi*360))

        t_0_3 = np.matmul(np.matmul(self.dh_0_1.homogeneous_matrix(q_1), self.dh_1_2.homogeneous_matrix(q_2)), self.dh_2_3.homogeneous_matrix(q_3))
        z_3_0 = np.matmul(t_0_3, np.array([0,0,1,1])) - np.matmul(t_0_3, np.array([0,0,0,1]))

        # TODO distinguish singular and non-singular case
        if True:
            # non-singular case
            tcp_n_cross_z_3_0 = np.cross(tcp_n, z_3_0[:3])
            tcp_n_cross_z_3_0 /= np.linalg.norm(tcp_n_cross_z_3_0)
            y_3_0 = (np.matmul(t_0_3, np.array([0,1,0,1])) - np.matmul(t_0_3, np.array([0,0,0,1])))[:3]
            
            y_3_0_dot_cross = np.dot(y_3_0, tcp_n_cross_z_3_0)
            y_3_0_dot_cross = max(y_3_0_dot_cross, -1)
            y_3_0_dot_cross = min(y_3_0_dot_cross, 1)
            
            delta_q_4 = math.acos(y_3_0_dot_cross)

            # determine direction
            x_3_0 = (np.matmul(t_0_3, np.array([1,0,0,1])) - np.matmul(t_0_3, np.array([0,0,0,1])))[:3]
            if np.dot(x_3_0, tcp_n_cross_z_3_0) > 0:
                q_4 = 2 * math.pi - delta_q_4
            else:
                q_4 = delta_q_4
            logging.debug("q_4={q}".format(q=q_4/2/math.pi*360))
        else:
            pass

        t_0_4 = np.matmul(t_0_3, self.dh_3_4.homogeneous_matrix(q_4))
        # calculate x_4 in world coordinate system
        x_4_0 = np.matmul(t_0_4, np.array([1,0,0,1])) - np.matmul(t_0_4, np.array([0,0,0,1]))

        # calculate the relevant inner products
        x_4_0_dot_tcp_n = np.dot(x_4_0[:3], tcp_n)
        z_3_0_dot_tcp_n = np.dot(z_3_0[:3], tcp_n)

        # distinguish two cases to compute q_5
        if x_4_0_dot_tcp_n < 0:
            # tcp_n is "below" z_3
            q_5_prime = math.acos(z_3_0_dot_tcp_n)
        else:
            # tcp_n is "above" z_3
            q_5_prime = -math.acos(z_3_0_dot_tcp_n)
        q_5 = q_5_prime + math.pi
        logging.debug("q_5={q}".format(q=q_5/2/math.pi*360))

        q_6 = 0

        q = [q_1, q_2, q_3, q_4, q_5, q_6] # in radian
        radian_to_degree = lambda rad: rad / 2 / math.pi * 360
        return [deg for deg in map(radian_to_degree, q)]