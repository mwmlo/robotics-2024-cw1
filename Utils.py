import numpy as np


def deg_to_rad(deg):
    return deg / 180 * np.pi


def rad_to_deg(rad):
    return rad / np.pi * 180


def direction(dx, dy):
    if dx == 0:
        if dy >= 0:
            return np.pi / 2
        else:
            return -np.pi / 2
    r = np.arctan(dy / dx)
    if r > 0 > dy:
        return r - np.pi
    elif r < 0 < dy:
        return r + np.pi
    return r


def rad_simplify(rad):
    rad %= (2 * np.pi)
    if rad > np.pi:
        rad -= 2 * np.pi

    return rad


def ang_diff(f, t):
    diff = t - rad_simplify(f)
    diff = rad_simplify(diff)
    return rad_to_deg(diff)

if __name__ == "__main__":
    print(ang_diff(0, direction(15, 0)))