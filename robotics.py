from __future__ import division
from __future__ import print_function

import time  # import the time library for the sleep function
#import brickpi3  # import the BrickPi3 drivers
import numpy as np

from Constants import *
from Utils import deg_to_rad, rad_to_deg, direction, ang_diff
from Visualize import Visualize
from mcl_resample import normalize, resample
from Map import Map, myMap

den_lim = 0.001

def wall_distance(x, y, theta, terrain: Map):
    dmin = MAX_TRUE_D
    for (x1, y1, x2, y2) in terrain.walls:
        dy = y2 - y1
        dx = x2 - x1
        denom = dy * np.cos(theta) - dx * np.sin(theta)
        if denom > den_lim or denom < -den_lim:
            d = (dy * (x1 - x) - dx * (y1 - y)) / denom
            if d > 0 and d < dmin:
                # check if this wall is valid
                cross = np.array([x + d*np.cos(theta), y + d*np.sin(theta)])
                vec1 = np.array([x1, y1]) - cross
                vec2 = np.array([x2, y2]) - cross
                r = vec2 * vec1
                #print("r: ", r)
                if r[0] < 0 or r[1] < 0:
                    # replace the result if current wall is closer
                    dmin = d

    return dmin


def likelihood(d_measure, d_true):
    p = - pow(d_measure - d_true, 2) / (2 * pow(LIKELIHOOD_STD, 2))
    return np.exp(p) + LIKELIHOOD_K

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
        
    def recalc_sensor(self):
        # Sonar measure result\
        dts = []
        i = 0
        while i < len(self.v.particles):
            p = self.v.particles[i]
            d_true = wall_distance(p.x, p.y, p.theta, self.terrain)
            if d_true < MAX_TRUE_D:
                dts.append(d_true)
                i += 1
            else:
                self.v.particles.pop(i)
        dt_mean = np.mean(np.array(dts))
        print("dtm,",dt_mean)
        d_measure = 0
        rep = 0
        while rep < 4:
            try:
                d_measure = BP.get_sensor(SONAR_PORT) + 4.5
            except:
                print(">>Sonar error")
                time.sleep(0.5)
            #z_score = np.abs(d_measure-dt_mean)/dt_std
            if np.abs(dt_mean - d_measure) < SONAR_OUTSCORE:
                print("Sonar accepted")
                break
            print(">>Sonar outlier", rep)
            time.sleep(0.5)
            rep += 1
        if rep >= 3:
            print("outlier error")
        
        print(f"Sonar dist {d_measure}", f"Mean std true {dt_mean}")
        for i in range(self.v.n_particles):
            self.v.particles[i].weight *= likelihood(d_measure, dts[i])
        
        normalize(self.v)
        resample(self.v)
        self.loc = self.v.estimate_location()
        
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
            time.sleep(0.5)
            return True
        self.forward(distance)
        time.sleep(0.5)
        return True


    def navigate(self, x_targ, y_targ):
        while self.step([x_targ,y_targ]):
            self.recalc_sensor()
        print("At-waypoint estimate:", self.loc[:2], rad_to_deg(self.loc[2]))
        

    def turn(self, ang):
        print(f"I think I am at ({self.loc[0]}, {self.loc[1]}), orientation {rad_to_deg(self.loc[2])} degrees;", f"I am going to turn {ang} degrees")
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
        if d_true == MAX_TRUE_D:
            return 0
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
        visualizer = Visualize(2, deg_to_rad(2), deg_to_rad(3.9))
        terrain = myMap(visualizer)
        terrain.draw()
        robot = Robot(visualizer, terrain)
        waypoints = [(84, 30), (180, 30), (180, 54), (138, 54),
                    (138, 168), (114, 168), (114, 84), (84, 84), (84, 30)]
        #waypoints = [(160, 20), (190, 20)]
        #visualizer.particles_gen(NUMBER_PARTICLES, 0, 0, 0)
        #robot.turn(90)
        robot.localize(waypoints)

    except KeyboardInterrupt:  # program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()