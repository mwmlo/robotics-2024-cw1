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

TURN_ANGLE = 250
FORWARD_DISTANCE = 850

def start(l_angle_diff, r_angle_diff):
    
    initial_r = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
    initial_l = BP.get_motor_encoder(LEFT_WHEEL_PORT)
    
    while True:

        print("R:", BP.get_motor_status(RIGHT_WHEEL_PORT)) 
        print("L:", BP.get_motor_status(LEFT_WHEEL_PORT)) 
        
        r_angle = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        l_angle = BP.get_motor_encoder(LEFT_WHEEL_PORT)
        
        if (math.abs(r_angle - initial_r) >= r_angle_diff or math.abs(l_angle - initial_l) >= l_angle_diff):
            break;

        time.sleep(1)

def turn(angle = TURN_ANGLE):   
    BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)
    BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, TURN_DPS)
    
    R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
    L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)
    
    BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + angle)
    BP.set_motor_position(LEFT_WHEEL_PORT, L_POS - angle)
    
    start(L_POS - angle, R_POS + angle)
    
    
def forward(distance = FORWARD_DISTANCE):
    BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
    BP.set_motor_limits(LEFT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
    
    R_POS = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
    L_POS = BP.get_motor_encoder(LEFT_WHEEL_PORT)

    BP.set_motor_position(RIGHT_WHEEL_PORT, R_POS + distance)
    BP.set_motor_position(LEFT_WHEEL_PORT, L_POS + distance)
    
    start(L_POS + distance, R_POS + distance)
    
    
try:
    for i in range(4):
        forward()
        turn()
        
except KeyboardInterrupt: # program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()