import brickpi3

BP = brickpi3.BrickPi3()

RIGHT_WHEEL_PORT = BP.PORT_A
LEFT_WHEEL_PORT = BP.PORT_D
SONAR_PORT = BP.PORT_4

POWER_LIMIT = 70
MAX_DPS = 360
TURN_DPS = 230
# Robot physical characteristics
TURN_PER_DEG = 272 / 90
FORWARD_PER_CM = 803 / 40
# Drawing constants
CENTER = (100, 700)
GRID_SCALE_FACTOR = 3
NEGATIVE_AXIS_LEN = 100
POSITIVE_AXIS_LEN = 500
# LIKELIHOOD PARAMETERS
LIKELIHOOD_STD = 2.5
LIKELIHOOD_K = 0.05
MAX_TRUE_D = 1023
SONAR_OUTSCORE = 5
NUMBER_PARTICLES = 100
DRAW = True
WAYPOINT_SUCCESS_THRESHOLD = 2
