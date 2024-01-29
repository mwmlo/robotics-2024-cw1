from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division 
      
import brickpi3         
import time     # import the time library for the sleep function

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

try:
    try:
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
    except IOError as error:
        print(error)
        
    BP.set_motor_limits(BP.PORT_A, 50)
    BP.set_motor_limits(BP.PORT_D, 50) 
    
    n = 0
    ang_a = 0
    ang_d = 0
    dt = 1
    while n < 10:
        sa = -72
        sd = 72
        BP.set_motor_dps(BP.PORT_A, sa)   
        BP.set_motor_dps(BP.PORT_D, sd)
        # optionally set a power limit
        ang_a += sa*dt
        ang_d += sd*dt
        print("Motor A Status: ", BP.get_motor_status(BP.PORT_A), ", angle a:", ang_a, "; Motor D Status: ", BP.get_motor_status(BP.PORT_D), ", angle d:", ang_d)
        
        time.sleep(dt)
        n += 1
        
    BP.reset_all()

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
