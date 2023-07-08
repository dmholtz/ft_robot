class DenavitHartenbergMatrix:
    """Transformation matrix to convert coordinate system K_i into coordinate system
    K_j according to the Denavit-Hartenberg convention.
    """

    def __init__(self, d: float, a: float, alpha: float, phi=0: float):
        """Initialize the Denavit-Hartenberg matrix using the Denavit-Hartenberg
        parameters.

        Args:
            d: translation d of origin K_i in the direction of z_i such that the
                distance between K_i and K_j is minimal.
            a: translation in direction x_j such that K_i and K_j are as close as
                possible.
            alpha: angle of rotation around x_j that transforms z_i into z_j
            phi: angle of rotation around joint j, i.e. around z_i such that x_i is
                transformed into x_j
        """
        self.d = d
        self.a = a
        self.alpha = alpha
        self.phi = phi

    def homogeneous_matrix(self, phi: float) -> np.ndarray:
        """Returns the homogeneous matrix that transforms coordinates of K_j into
        coordinates of K_i.
        """

        return np.array([
            [math.cos(phi), -math.sin(phi)*math.cos(self.alpha), math.sin(phi)*math.sin(self.alpha),    self.a*math.cos(phi)],
            [math.sin(phi), math.cos(phi)*math.cos(self.alpha), -math.cos(phi)*math.sin(self.alpha),    self.a*math.sin(phi)],
            [0,             math.sin(self.alpha),               math.cos(self.alpha),                   self.d              ],
            [0,             0,                                  0,                                      1                   ],
        ])