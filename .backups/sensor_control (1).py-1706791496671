from __future__ import print_function
from __future__ import division

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import math

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
L_TOUCH_PORT = BP.PORT_1
R_TOUCH_PORT = BP.PORT_2
SONAR_PORT = BP.PORT_1
RIGHT_WHEEL_PORT = BP.PORT_A
LEFT_WHEEL_PORT = BP.PORT_D
SPEED = 360

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP.set_sensor_type(L_TOUCH_PORT, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(R_TOUCH_PORT, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)

def to_speed(vl_target, vr_target, threshold=5, interval=0.02):
    while True:
        vl = BP.get_motor_status(LEFT_WHEEL_PORT)[3]
        vr = BP.get_motor_status(RIGHT_WHEEL_PORT)[3]
        
        if (math.abs(vl - vl_target) <= threshold or math.abs(vr - vr_target) <= threshold):
            break

        time.sleep(interval)

def wall_follow(duration):
    to_speed(SPEED, SPEED)
    t = time.time()
    end = t + duration
    while t < duration:
        wall_speed_adjust(SPEED, 0.5)
        time.sleep(0.2)
        t += 0.05

def wall_speed_adjust(target, k):
    diff = BP.get_sensor(SONAR_PORT) - target
    vl = BP.get_motor_status(LEFT_WHEEL_PORT)[3]
    vr = BP.get_motor_status(RIGHT_WHEEL_PORT)[3]
    vc = (vl+vr)/2
    vl = vc - k*diff
    vr = vc + k*diff
    to_speed(vl, vr)

try:
    wall_follow(5)
        
except KeyboardInterrupt: # program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()

