import brickpi3

BP = brickpi3.BrickPi3()

RIGHT_WHEEL_PORT = BP.PORT_A
LEFT_WHEEL_PORT = BP.PORT_D
SONAR_PORT = BP.PORT_1

BP.set_sensor_type(SONAR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)

POWER_LIMIT = 70
MAX_DPS = 360
TURN_DPS = 230
# Robot physical characteristics
TURN_PER_DEG = 272 / 90
FORWARD_PER_CM = 857 / 40
# Drawing constants
CENTER = (200, 600)
GRID_SCALE_FACTOR = 10
NEGATIVE_AXIS_LEN = 100
POSITIVE_AXIS_LEN = 500
# LIKELIHOOD PARAMETERS
LIKELIHOOD_STD = 2.5
LIKELIHOOD_K = 0.05
NUMBER_PARTICLES = 100
