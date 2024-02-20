from __future__ import division
from __future__ import print_function

import time  # import the time library for the sleep function

# import brickpi3  # import the BrickPi3 drivers
import numpy as np

from Constants import *
from Utils import deg_to_rad, direction, ang_diff
from Visualize import Visualize
from likelihood import wall_distance, likelihood
from mcl_resample import normalize, resample
from Map import Map, myMap


class Robot:
    def __init__(self, visualizer: Visualize, terrain: Map):
        self.v = visualizer
        self.terrain = terrain

    @staticmethod
    def start(l_angle_target, r_angle_target, threshold=5, interval=0.5):
        while True:
            print("R:", BP.get_motor_status(RIGHT_WHEEL_PORT))
            print("L:", BP.get_motor_status(LEFT_WHEEL_PORT))

            r_angle = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
            l_angle = BP.get_motor_encoder(LEFT_WHEEL_PORT)

            if abs(r_angle - r_angle_target) <= threshold or abs(l_angle - l_angle_target) <= threshold:
                break

            time.sleep(interval)

    def navigate(self, loc, x_targ, y_targ):
        vec = np.array([x_targ, y_targ]) - loc[:2]
        distance = np.sqrt(pow(vec[0], 2) + pow(vec[1], 2))
        rad = direction(vec[0], vec[1])

        self.turn(ang_diff(loc[2], rad), draw)
        time.sleep(0.5)
        # move in 20cm steps
        while distance > 20:
            distance -= 20
            self.forward(20)
            time.sleep(0.5)
        self.forward(distance)

    def turn(self, ang):
        angle = ang * TURN_PER_DEG
        BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)
        BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)

        R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)

        BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + angle)
        BP.set_motor_position(LEFT_WHEEL_PORT, L_POS - angle)

        self.start(L_POS - angle, R_POS + angle)
        self.v.turn(ang)

    def forward(self, dist):
        distance = dist * FORWARD_PER_CM
        BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
        BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)

        R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)

        BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + distance)
        BP.set_motor_position(LEFT_WHEEL_PORT, L_POS + distance)

        self.start(L_POS + distance, R_POS + distance)
        self.v.forward(dist)

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

    def calculate_likelihood(self, x, y, theta, z):
        d_true = wall_distance(x, y, theta, self.terrain)
        return likelihood(z, d_true)

    def localize(self, waypoints: list[tuple]):
        (x,y) = waypoints[0]
        self.v.particles_gen(NUMBER_PARTICLES, x, y, 0)
        loc = np.array([x,y,0])
        for (x,y) in waypoints[1:]:
            print(f"Heading towards point: ({x}, {y})")
            self.navigate(loc, x, y, draw=True)
            d_measure = BP.get_sensor(SONAR_PORT)  # Sonar measure result
            for p in self.v.particles:
                p.weight *= self.calculate_likelihood(p.x, p.y, p.theta, d_measure)
            normalize(self.v)
            resample(self.v)
            loc = self.v.estimate_location()


if __name__ == "__main__":
    try:
        visualizer = Visualize(0.2, deg_to_rad(1), deg_to_rad(3))
        terrain = myMap(visualizer)
        robot = Robot(visualizer, terrain)
        # robot.draw_star(10)
        # robot.forward(40)
        # robot.turn(90)
        waypoints = [(84, 30), (180, 30), (180, 54), (138, 54),
                     (138, 168), (114, 168), (114, 84), (84, 84), (84, 30)]
        robot.localize(waypoints)

    except KeyboardInterrupt:  # program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()
