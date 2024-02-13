from __future__ import division
from __future__ import print_function

import time  # import the time library for the sleep function

import brickpi3  # import the BrickPi3 drivers

from Constants import *
from Utils import deg_to_rad
from Visualize import Visualize

BP = brickpi3.BrickPi3()


def start(l_angle_target, r_angle_target, threshold=5, interval=0.5):
    while True:
        print("R:", BP.get_motor_status(RIGHT_WHEEL_PORT))
        print("L:", BP.get_motor_status(LEFT_WHEEL_PORT))

        r_angle = BP.get_motor_encoder(RIGHT_WHEEL_PORT)
        l_angle = BP.get_motor_encoder(LEFT_WHEEL_PORT)

        if abs(r_angle - r_angle_target) <= threshold or abs(l_angle - l_angle_target) <= threshold:
            break

        time.sleep(interval)


if __name__ == "__main__":
    try:
        v = Visualize(0.2, deg_to_rad(1), deg_to_rad(3), 100)
        v.draw_square()
        # v.draw_star(10)
        # forward(40)
        # turn(90)

    except KeyboardInterrupt:  # program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()
