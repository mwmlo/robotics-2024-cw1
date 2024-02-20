from Map import Map
from Constants import LIKELIHOOD_K, LIKELIHOOD_STD
import numpy as np
import math

den_lim = 0.001

def vec_angle_from_east(x1, y1, x2, y2):
    """Calculate the angle between the vector from (x1, y1) to (x2, y2) and the positive x-axis, measured counterclockwise from the east direction. Range [-pi, pi]."""
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)
    return angle

def will_hit_wall(pos_x, pos_y, pos_theta, wall):
    """Return true if the robot's trajectory intersects with the given wall."""
    x1, y1, x2, y2 = wall

    # Calculate absolute angles for vectors from the robot to the wall's endpoints
    angle1 = vec_angle_from_east(pos_x, pos_y, x1, y1)
    angle2 = vec_angle_from_east(pos_x, pos_y, x2, y2)

    # Check that the robot's orientation is within wall endpoints
    return (angle1 <= pos_theta <= angle2) or (angle2 <= pos_theta <= angle1)

def wall_distance(x, y, theta, terrain: Map):
    """Returns the distance from the robot's position to the closest wall in the given Map."""
    min_distance = 255 # If there is no nearby wall to hit, use maximum
    for wall in terrain.walls:
        if will_hit_wall(x, y, theta, wall):
            x1, y1, x2, y2 = wall
            dy = y2 - y1
            dx = x2 - x1
            numerator = (dy * (x1 - x) - dx * (y1 - y))
            denominator = dy * np.cos(theta) - dx * np.sin(theta)
            distance = numerator / denominator
            if distance >= 0:
                min_distance = min(distance, min_distance)
    return min_distance

# def wall_distance(x, y, theta, terrain: Map):
#     dmin = 255
#     for (x1, y1, x2, y2) in terrain.walls:
#         dy = y2 - y1
#         dx = x2 - x1
#         denom = dy * np.cos(theta) - dx * np.sin(theta)
#         if denom > den_lim or denom < -den_lim:
#             d = (dy * (x1 - x) - dx * (y1 - y)) / denom
#             if d >= 0 and d < dmin:
#                 # check if this wall is valid
#                 cross = np.array(x + d*np.cos(theta), y + d*np.sin(theta))
#                 vec1 = np.array([x1, y1]) - cross
#                 vec2 = np.array([x2, y2]) - cross
#                 r = vec2 * vec1
#                 #print("r: ", r)
#                 if r[0] <= 0 or r[1] <= 0:
#                     # replace the result if current wall is closer
#                     dmin = d

#     return dmin


def likelihood(d_measure, d_true):
    print(d_measure, d_true, LIKELIHOOD_STD)
    p = - pow(d_measure - d_true, 2) / (2 * pow(LIKELIHOOD_STD, 2))
    return np.exp(p) + LIKELIHOOD_K
