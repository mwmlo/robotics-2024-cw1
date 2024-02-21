#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for reading an NXT ultrasonic sensor connected to PORT_1 of the BrickPi3
# 
# Hardware: Connect an NXT ultrasonic sensor to BrickPi3 Port 1.
# 
# Results:  When you run this program, you should see the distance in CM.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import numpy as np
import time     # import the time library for the sleep function
from Constants import *
from Utils import deg_to_rad, direction, ang_diff, rad_to_deg
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
            r_angle = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
            l_angle = BP.get_motor_encoder(LEFT_WHEEL_PORT)

            if abs(r_angle - r_angle_target) <= threshold or abs(l_angle - l_angle_target) <= threshold:
                break

            time.sleep(interval)
            
    # returns true if moved
    def step(self, waypoint):
        vec = np.array([waypoint[0], waypoint[1]]) - self.loc[:2]
        distance = np.sqrt(pow(vec[0], 2) + pow(vec[1], 2))
        if distance <= WAYPOINT_SUCCESS_THRESHOLD:
            return False
        
        rad = direction(vec[0], vec[1])
        self.turn(ang_diff(self.loc[2], rad))
        time.sleep(0.5)
        # move in 20cm steps
        if distance > 20:
            self.forward(20)
        else:
            self.forward(distance)
        time.sleep(0.5)
        return True


    def navigate(self, x_targ, y_targ):
        while self.step([x_targ,y_targ]):
            self.recalc_sensor()
        print("At-waypoint location:", self.loc)
        
    def recalc_sensor(self):
        # Sonar measure result\
        while True:
            try:
                d_measure = BP.get_sensor(SONAR_PORT) + 6
                break
            except:
                print("Sonar error")
                time.sleep(0.5)
            
        print(f"Sonar dist {d_measure}")
        for p in self.v.particles:
            if not self.terrain.is_particle_in(p):
                p.weight *= 0.01
            p.weight *= self.calculate_likelihood(p.x, p.y, p.theta, d_measure)
        normalize(self.v)
        resample(self.v)
        self.loc = self.v.estimate_location()
        

    def turn(self, ang):
        print(f"I think I am at ({self.loc[0]}, {self.loc[1]}), orientation {rad_to_deg(self.loc[2])} degrees")
        print(f"I am going to turn {ang} degrees")
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
        self.loc = np.array([x,y,0])
        for (x,y) in waypoints[1:]:
            print(f"Heading towards point: ({x}, {y})")
            self.navigate(x, y)
            
if __name__ == "__main__":
    try:
        BP.set_sensor_type(SONAR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)
        visualizer = Visualize(0.05, deg_to_rad(0.5), deg_to_rad(1))
        terrain = myMap(visualizer)
        terrain.draw()
        robot = Robot(visualizer, terrain)
        waypoints = [(84, 30), (180, 30), (180, 54), (138, 54),
                     (138, 168), (114, 168), (114, 84), (84, 84), (84, 30)]
        #visualizer.particles_gen(NUMBER_PARTICLES, 0, 0, 0)
        #robot.turn(90)
        robot.localize(waypoints)
        #for _ in range(10):
        #    try:
        #        value = BP.get_sensor(SONAR_PORT)
        #        print(value)
        #    except brickpi3.SensorError as error:
        #        print(error)

        #    time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

    except KeyboardInterrupt:  # program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()