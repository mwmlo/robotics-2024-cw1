import numpy as np

def rad_simplify(rad):
    rad %= (2*np.pi)
    if rad > np.pi:
        rad -= 2*np.pi
    
    return rad

def rad_to_deg(rad):
    return rad/np.pi*180

def ang_diff(f, t):
    diff = t - rad_simplify(f)
    diff = rad_simplify(diff)
    return rad_to_deg(diff)

def direction(dx, dy):
    if dx == 0:
        if dy >= 0:
            return np.pi/2
        else:
            return -np.pi/2
    r = np.arctan(dy/dx)
    if r > 0 and dy < 0:
        return r-np.pi
    elif r < 0 and dy > 0:
        return r+np.pi
    return r
    
print(direction(-1,-1))

