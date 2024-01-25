from __future__ import print_function
from __future__ import division

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3()

RIGHT_WHEEL_PORT = BP.PORT_A
LEFT_WHEEL_PORT = BP.PORT_D
POWER_LIMIT = 70
MAX_DPS = 200
FORWARD_DPS = 180 # TODO: DETERMINE WHAT IS 40 CM
TURN_DPS = 90
# wheel radius = 2.5 CM

try:
    BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)
    BP.set_motor_limits(RIGHT_WHEEL_PORT, POWER_LIMIT, MAX_DPS)

    # Go forwards
    BP.set_motor_dps(RIGHT_WHEEL_PORT, FORWARD_DPS)
    BP.set_motor_dps(LEFT_WHEEL_PORT, FORWARD_DPS)

    time.sleep(1.5) # TODO: calibrate!

    # Turn 90 degrees
    BP.set_motor_dps(RIGHT_WHEEL_PORT, TURN_DPS)
    BP.set_motor_dps(LEFT_WHEEL_PORT, -TURN_DPS)

    time.sleep(0.02) # TODO: calibrate!



except KeyboardInterrupt: # program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()