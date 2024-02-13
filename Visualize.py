import random
import time

import numpy as np

from Constants import *
from Particle import Particle
from Utils import direction, ang_diff, deg_to_rad
from main import BP, start


class Visualize:
    def __init__(self, e_std, f_std, g_std, n_particles):
        # standard deviations
        self.e_std = e_std
        self.f_std = f_std
        self.g_std = g_std
        # generate list of particles
        self.particles = [Particle(0, 0, 0, 1 / n_particles) for p in range(n_particles)]
        # draw axes
        self.draw_axes()

    def gx(self, x):
        return CENTER[0] + x * GRID_SCALE_FACTOR

    def gy(self, y):
        return CENTER[1] - y * GRID_SCALE_FACTOR

    def draw_axes(self):
        x_axis = (-NEGATIVE_AXIS_LEN, 0, POSITIVE_AXIS_LEN, 0)
        y_axis = (0, -NEGATIVE_AXIS_LEN, 0, POSITIVE_AXIS_LEN)
        self.draw_line(x_axis)
        self.draw_line(y_axis)

    def draw_line(self, line_coords):
        (x0, y0, x1, y1) = line_coords
        print("drawLine:" + str((self.gx(x0), self.gy(y0), self.gx(x1), self.gy(y1))))

    def draw_particles(self):
        particle_coords = [p.draw() for p in self.particles]
        gparticle_coords = [(self.gx(x), self.gy(y), t) for (x, y, t) in particle_coords]
        print("drawParticles:" + str(gparticle_coords))

    def loc_estimate(self):
        tot_w = 0
        locs = np.zeros(3)
        for p in self.particles:
            tot_w += p.weight
            locs += p.weight * p.loc
        mean_loc = locs / tot_w
        return mean_loc

    def navigate(self, loc, x_targ, y_targ, draw=False):
        vec = np.array([x_targ, y_targ]) - loc[:2]
        distance = np.sqrt(pow(vec[0], 2) + pow(vec[1], 2))
        rad = direction(vec[0], vec[1])
        self.turn(ang_diff(loc[2], rad), draw)
        time.sleep(0.5)
        self.forward(distance, draw)

    def turn(self, ang, draw=True):
        angle = ang * TURN_PER_DEG
        BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)
        BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)

        R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)

        BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + angle)
        BP.set_motor_position(LEFT_WHEEL_PORT, L_POS - angle)

        start(L_POS - angle, R_POS + angle)
        rad = deg_to_rad(ang)
        for p in self.particles:
            g = random.gauss(0, self.g_std)
            p.update_rot(rad, g)
        if draw:
            self.draw_particles()

    def forward(self, dist, draw=True):
        distance = dist * FORWARD_PER_CM
        BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
        BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)

        R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)

        BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + distance)
        BP.set_motor_position(LEFT_WHEEL_PORT, L_POS + distance)

        start(L_POS + distance, R_POS + distance)
        for p in self.particles:
            e = random.gauss(0, self.e_std)
            f = random.gauss(0, self.f_std)
            p.update_line(dist, e, f)
        if draw:
            self.draw_particles()

    def draw_square(self, size=40):
        for i in range(4):
            for j in range(4):
                self.forward(size / 4)
                time.sleep(0.5)
            self.turn(90)
            time.sleep(0.5)

    def draw_star(self, size=30):
        direction = 1
        for i in range(5):
            self.forward(direction * size)
            time.sleep(0.5)
            self.turn(36)
            time.sleep(0.5)
            direction = -direction
