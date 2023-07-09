import numpy as np
import math

from typing import List

from denavit_hartenberg import DenavitHartenbergMatrix

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
            y_3_0_dot_cross /= np.linalg.norm(y_3_0_dot_cross)
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