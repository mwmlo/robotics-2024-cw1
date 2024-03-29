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
TURN_PER_DEG = 250/90
FORWARD_PER_CM = 1100/40
# Drawing constants
CENTER = (200, 600)
GRID_SCALE_FACTOR = 10
NEGATIVE_AXIS_LEN = 100
POSITIVE_AXIS_LEN = 500
NUMBER_OF_PARTICLES = 100


def deg_to_rad(deg):
    return deg/180*math.pi

def rad_to_deg(rad):
    return rad/math.pi*180

class Visualize:
    def __init__(self, e_std, f_std, g_std, n_particles):
        # standard deviations
        self.e_std = e_std
        self.f_std = f_std
        self.g_std = g_std
        # generate list of particles
        self.particles = [Particle(0,0,0, 1/n_particles) for p in range(n_particles)]
        # draw axes
        self.draw_axes()

    def gx(self, x):
        return CENTER[0] + x*GRID_SCALE_FACTOR
    
    def gy(self, y):
        return CENTER[1] - y*GRID_SCALE_FACTOR

    def draw_axes(self):
        x_axis = (-NEGATIVE_AXIS_LEN, 0, POSITIVE_AXIS_LEN, 0)
        y_axis = (0, -NEGATIVE_AXIS_LEN, 0, POSITIVE_AXIS_LEN)
        self.draw_line(x_axis)
        self.draw_line(y_axis)

    def draw_line(self, line_coords):
        (x0,y0,x1,y1) = line_coords
        print("drawLine:" + str((self.gx(x0), self.gy(y0), self.gx(x1), self.gy(y1))))

    def draw_particles(self):
        particle_coords = [p.draw() for p in self.particles]
        gparticle_coords = [(self.gx(x), self.gy(y), t) for (x,y,t) in particle_coords]
        print("drawParticles:"+str(gparticle_coords))

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
        rad = deg_to_rad(ang)
        for p in self.particles:
            g = random.gauss(0, self.g_std)
            p.update_rot(rad, g)
    
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
            e = random.gauss(0, self.e_std)
            f = random.gauss(0, self.f_std)
            p.update_line(dist, e, f) 

    def draw_square(self, size=40):
        for i in range(4):
            for j in range(4):
                self.forward(size/4)
                time.sleep(0.5)
            self.turn(90)
            time.sleep(0.5)
            
    def draw_star(self,size=30):
        direction = 1
        for i in range(5):
            self.forward(direction * size)
            time.sleep(0.5)
            self.turn(36)
            time.sleep(0.5)
            direction = -direction        
    

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


try:
    v = Visualize(0.2,deg_to_rad(1),deg_to_rad(3),NUMBER_OF_PARTICLES)
    v.forward(60)
    #v.draw_star(10)
    # forward(40)
    # turn(90)
        
except KeyboardInterrupt: # program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()
        