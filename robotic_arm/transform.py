import numpy as np
from math import cos, sin

class Transform:

    def __init__(self, matrix: np.ndarray = np.eye(4)):
        self.matrix = matrix

    def rotate_x(self, phi):
        rotation = np.array([
                [1, 0,          0,          0],
                [0, cos(phi),   -sin(phi),  0],
                [0, sin(phi),   cos(phi),   0],
                [0, 0,          0,          1],
            ])
        new_matrix = np.matmul(self.matrix, rotation)
        return Transform(new_matrix)

    def rotate_y(self, phi):
        rotation = np.array([
                [cos(phi),  0,  sin(phi),   0],
                [0,         1,  0,          0],
                [-sin(phi), 0,  cos(phi),   0],
                [0, 0,          0,          1],
            ])
        new_matrix = np.matmul(self.matrix, rotation)
        return Transform(new_matrix)

    def rotate_z(self, phi):
        rotation = np.array([
                [0, 0,          0,          0],
                [0, cos(phi),   -sin(phi),  0],
                [0, sin(phi),  cos(phi),    0],
                [0, 0,          0,          1],
            ])
        new_matrix = np.matmul(self.matrix, rotation)
        return Transform(new_matrix)

    def translate(self, translation):
        new_matrix = np.array(self.matrix)
        new_matrix[:3,3] = translation
        return Transform(new_matrix)

    @property
    def translation(self):
        return self.matrix[:3, 3]

    @property
    def x(self):
        return self.matrix[:3, 0]

    @property
    def y(self):
        return self.matrix[:3, 1]

    @property
    def z(self):
        return self.matrix[:3, 2]
        