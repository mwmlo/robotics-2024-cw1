from __future__ import print_function
from __future__ import division

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import math, random

BP = brickpi3.BrickPi3()

RIGHT_WHEEL_PORT = BP.PORT_A
LEFT_WHEEL_PORT = BP.PORT_D
POWER_LIMIT = 70
MAX_DPS = 360
TURN_DPS = 230
# Robot physical characteristics
TURN_PER_DEG = 272/90
FORWARD_PER_CM = 851/40

class Visualize:
    def __init__(self, x_std, y_std, theta_std, n_particles):
        self.x_std = x_std
        self.y_std = y_std
        self.theta_std = theta_std
        self.particles = [Particle(0,0,0, 1/n_particles) for p in range(n_particles)]
        x_axis = (-10, 0, 50, 0)
        y_axis = (0, -10, 0, 50)
        print("drawLine:" + str((x_axis)))
        print("drawLine:" + str((y_axis)))

    def draw_particles(self):
        print("drawParticles:"+str([p.draw() for p in self.particles]))

    def turn(self, ang):  
        self.draw_particles()
        angle = ang * TURN_PER_DEG 
        BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)
        BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)
        
        R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)
        
        BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + angle)
        BP.set_motor_position(LEFT_WHEEL_PORT, L_POS - angle)
        
        start(L_POS - angle, R_POS + angle)
        for p in self.particles:
            g = random.gauss(0, self.theta_std)
            p.update_line(ang, g)
    
    def forward(self, dist):
        self.draw_particles()
        distance = dist * FORWARD_PER_CM
        BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
        BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
        
        R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)

        BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + distance)
        BP.set_motor_position(LEFT_WHEEL_PORT, L_POS + distance)
        
        start(L_POS + distance, R_POS + distance)
        for p in self.particles:
            e = random.gauss(0, self.x_std)
            f = random.gauss(0, self.y_std)
            p.update_line(dist, e, f) 

    def draw_square(self, size=40):
        for i in range(4):
            for j in range(4):
                self.forward(size/4)
                time.sleep(0.5)
            self.turn(90)
            time.sleep(0.5)

class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

    def update_line(self, dist, e, f):
        self.x += (dist+e) * math.cos(self.theta)
        self.y += (dist+e) * math.sin(self.theta)
        self.theta += f

    def update_rot(self, alpha, g):
        self.theta += alpha+g

    def draw(self):
        return (self.x, self.y, self.theta)



def start(l_angle_target, r_angle_target, threshold=5, interval=0.5):
    while True:
        print("R:", BP.get_motor_status(RIGHT_WHEEL_PORT)) 
        print("L:", BP.get_motor_status(LEFT_WHEEL_PORT)) 
        
        r_angle = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        l_angle = BP.get_motor_encoder(LEFT_WHEEL_PORT)
        
        if (abs(r_angle - r_angle_target) <= threshold or abs(l_angle - l_angle_target) <= threshold):
            break

        time.sleep(interval)
    
def draw_star(size=30):
    direction = 1
    for i in range(5):
        forward(direction * size)
        turn(36)
        direction = -direction

try:
    v = Visualize(2,2,2, 100)
    v.draw_square()
    # forward(40)
    # turn(90)
        
except KeyboardInterrupt: # program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()
        
