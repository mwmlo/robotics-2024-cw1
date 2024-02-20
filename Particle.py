import numpy as np


class Particle:
    def __init__(self, x, y, theta, weight):
        self.x, self.y, self.theta = x, y, theta
        self.weight = weight

    def update_line(self, dist, e, f):
        self.x += (dist + e) * np.cos(self.theta)
        self.y += (dist + e) * np.sin(self.theta)
        self.theta += f

    def update_rot(self, alpha, g):
        self.theta += (alpha + g)

    def draw(self):
        return self.x, self.y, self.theta
    
    def __str__(self) -> str:
        return f"({self.x}, {self.y}, {self.theta}, {self.weight})"
