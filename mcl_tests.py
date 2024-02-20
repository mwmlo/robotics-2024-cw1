import unittest

from Utils import deg_to_rad, direction, ang_diff
from Visualize import Visualize
from likelihood import wall_distance, likelihood
from mcl_resample import normalize, resample
from Map import Map, myMap
# from main import Robot

class TestMonteCarloMethods(unittest.TestCase):

    # NOTE: for testing, please comment out lines 1-9 in Constants.py
    def setUp(self):
        self.visualizer = Visualize(0.2, deg_to_rad(1), deg_to_rad(3))
        self.terrain = myMap(self.visualizer)

    def test_wall_distance_wp1_orientx(self):
        # Location: waypoint 1, facing along the x-axis (right)
        pos_x, pos_y, pos_theta = 84, 30, 0
        # Should be facing wall between points G, H
        expected_distance = abs(210 - pos_x)
        predicted_distance = wall_distance(pos_x, pos_y, pos_theta, self.terrain)
        self.assertEqual(expected_distance, predicted_distance)

    def test_wall_distance_wp1_orienty(self):
        # Location: waypoint 1, facing along the y-axis (down)
        pos_x, pos_y, pos_theta = 84, 30, -90
        # Should be facing wall between points O, H
        expected_distance = abs(0 - pos_y)
        predicted_distance = wall_distance(pos_x, pos_y, pos_theta, self.terrain)
        self.assertEqual(expected_distance, predicted_distance)

    def test_wall_distance_wp2_orientx(self):
        # Location: waypoint 2, facing along the x-axis (right)
        pos_x, pos_y, pos_theta = 180, 30, 0
        # Should be facing wall between points G, H
        expected_distance = 210 - pos_x
        predicted_distance = wall_distance(pos_x, pos_y, pos_theta, self.terrain)
        self.assertEqual(expected_distance, predicted_distance)

    def test_wall_distance_wp2_orienty(self):
        # Location: waypoint 2, facing along the y-axis (up)
        pos_x, pos_y, pos_theta = 180, 30, 90
        # Should be facing wall between points F, G
        expected_distance = 84 - pos_y
        predicted_distance = wall_distance(pos_x, pos_y, pos_theta, self.terrain)
        self.assertEqual(expected_distance, predicted_distance)

if __name__ == '__main__':
    unittest.main()