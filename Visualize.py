import random

import numpy as np

from Constants import *
from Particle import Particle
from Utils import deg_to_rad


class Visualize:
    def __init__(self, e_std, f_std, g_std):
        # standard deviations
        self.e_std = e_std
        self.f_std = f_std
        self.g_std = g_std
        # draw axes
        self.draw_axes()

    def particles_gen(self, n_particles, x, y, theta):
        # generate list of particles
        self.particles = [Particle(x, y, theta, 1 / n_particles) for _ in range(n_particles)]
        
    @staticmethod
    def __gx(x):
        return CENTER[0] + x * GRID_SCALE_FACTOR

    @staticmethod
    def __gy(y):
        return CENTER[1] - y * GRID_SCALE_FACTOR

    def draw_axes(self):
        x_axis = (-NEGATIVE_AXIS_LEN, 0, POSITIVE_AXIS_LEN, 0)
        y_axis = (0, -NEGATIVE_AXIS_LEN, 0, POSITIVE_AXIS_LEN)
        self.draw_line(x_axis)
        self.draw_line(y_axis)

    def draw_line(self, line_coords):
        (x0, y0, x1, y1) = line_coords
        # print("drawing:" + str((self.__gx(x0), self.__gy(y0), self.__gx(x1), self.__gy(y1))))
        print("drawLine:" + str((self.__gx(x0), self.__gy(y0), self.__gx(x1), self.__gy(y1))))

    def draw_particles(self):
        particle_coords = [p.draw() for p in self.particles]
        gparticle_coords = [(self.__gx(x), self.__gy(y), t) for (x, y, t) in particle_coords]
        # print("drawing:" + str(gparticle_coords))
        print("drawParticles:" + str(gparticle_coords))

    def estimate_location(self):
        locs = []
        weights = []
        for p in self.particles:
            if p.weight > 0:
                locs.append([p.x, p.y, p.theta])
                weights.append([p.weight])
        locs = np.array(locs)
        weights = np.array(weights)
        s = locs * weights
        #print("estimate location", np.sum(s, axis=0))
        return np.sum(s, axis=0)

    def turn(self, ang):
        rad = deg_to_rad(ang)
        for p in self.particles:
            g = random.gauss(0, self.g_std)
            p.update_rot(rad, g)
        if DRAW:
            self.draw_particles()

    def forward(self, dist):
        for p in self.particles:
            e = random.gauss(0, self.e_std)
            f = random.gauss(0, self.f_std)
            p.update_line(dist, e, f)

        if DRAW:
            self.draw_particles()
