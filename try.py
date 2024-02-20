import numpy as np

def navigate(self, loc, x_targ, y_targ, draw=False):
        vec = np.array([x_targ, y_targ]) - loc[:2]
        distance = np.sqrt(pow(vec[0], 2) + pow(vec[1], 2))
        rad = direction(vec[0], vec[1])
        ang = ang_diff(loc[2], rad)
        if ang > np.pi/2:
            ang -= np.pi
            distance = -distance
        elif ang < -np.pi/2:
            ang += np.pi
            distance = -distance
        self.turn(ang, draw)
        time.sleep(0.5)
        self.forward(distance, draw)