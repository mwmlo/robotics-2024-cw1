import numpy as np


class Particle:
    def __init__(self, x, y, theta, weight):
        self.loc = np.array([x, y, theta])
        self.weight = weight

    def update_line(self, dist, e, f):
        self.loc[0] += (dist + e) * np.cos(self.loc[2])
        self.loc[1] += (dist + e) * np.sin(self.loc[2])
        self.loc[2] += f

    def update_rot(self, alpha, g):
        self.loc[2] += (alpha + g)

    def draw(self):
        return self.loc[0], self.loc[1], self.loc[2]
