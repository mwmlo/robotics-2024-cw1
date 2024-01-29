from __future__ import print_function
from __future__ import division

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import math

BP = brickpi3.BrickPi3()

RIGHT_WHEEL_PORT = BP.PORT_A
LEFT_WHEEL_PORT = BP.PORT_D
POWER_LIMIT = 70
MAX_DPS = 200
FORWARD_DPS = 180
TURN_DPS = 90
# Robot physical characteristics
TURN_PER_DEG = 250/90
FORWARD_PER_CM = 850/40

def start(l_angle_target, r_angle_target, threshold=5, interval=0.5):
    while True:
        print("R:", BP.get_motor_status(RIGHT_WHEEL_PORT)) 
        print("L:", BP.get_motor_status(LEFT_WHEEL_PORT)) 
        
        r_angle = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        l_angle = BP.get_motor_encoder(LEFT_WHEEL_PORT)
        
        if (math.abs(r_angle - r_angle_target) <= threshold or math.abs(l_angle - l_angle_target) <= threshold):
            break

        time.sleep(interval)

def turn(angle):   
    angle *= TURN_PER_DEG 
    BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)
    BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)
    
    R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
    L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)
    
    BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + angle)
    BP.set_motor_position(LEFT_WHEEL_PORT, L_POS - angle)
    
    start(L_POS - angle, R_POS + angle)
    
    
def forward(distance):
    distance *= FORWARD_PER_CM
    BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
    BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
    
    R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
    L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)

    BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + distance)
    BP.set_motor_position(LEFT_WHEEL_PORT, L_POS + distance)
    
    start(L_POS + distance, R_POS + distance)
    
def draw_square(size=40):
    for i in range(4):
        forward(size)
        turn(90)
    
def draw_star(size=30):
    direction = 1
    for i in range(5):
        forward(direction * size)
        turn(36)
        direction = -direction

try:
    draw_square()
        
except KeyboardInterrupt: # program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()