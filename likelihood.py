from Map import Map
from Constants import LIKELIHOOD_K, LIKELIHOOD_STD
import numpy as np

den_lim = 0.001

def wall_distance(x, y, theta, terrain: Map):
    dmin = 255
    for (x1, y1, x2, y2) in terrain.walls:
        dy = y2 - y1
        dx = x2 - x1
        denom = dy * np.cos(theta) - dx * np.sin(theta)
        if denom > den_lim or denom < -den_lim:
            d = (dy * (x1 - x) - dx * (y1 - y)) / denom
            if d >= 0 and d < dmin:
                # check if this wall is valid
                cross = np.array(x + d*np.cos(theta), y + d*np.sin(theta))
                vec1 = np.array([x1, y1]) - cross
                vec2 = np.array([x2, y2]) - cross
                r = vec2 * vec1
                #print("r: ", r)
                if r[0] <= 0 or r[1] <= 0:
                    # replace the result if current wall is closer
                    dmin = d

    return dmin


def likelihood(d_measure, d_true):
    print(d_measure, d_true, LIKELIHOOD_STD)
    p = - pow(d_measure - d_true, 2) / (2 * pow(LIKELIHOOD_STD, 2))
    return np.exp(p) + LIKELIHOOD_K
