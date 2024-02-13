from Map import Map
import numpy as np

def wall_distance(x,y,theta, terrain: Map):
    dmin = None
    for (x1,y1,x2,y2) in terrain.walls:
        dy = y2-y1 
        dx = x2-x1
        d = (dy * (x1-x) - dx * (y1-y)) / (dy * np.cos(theta) - dx * np.sin(theta))
        # check if this wall is valid
        cross = np.array(x + np.cos(theta), y + np.sin(theta))
        vec1 = np.array([x1, y1]) - cross
        vec2 = np.array([x2, y2]) - cross
        r = vec2 / vec1
        if r[0] <= 0 and r[1] <= 0:
            # replace the result if current wall is closer
            if dmin is None or d < dmin:
                dmin = d
            
    return dmin

def likelihood(d_measure, d_true, std, k):
    p = - pow(d_measure-d_true, 2) / (2*pow(std, 2))
    return np.exp(p) + k
    
