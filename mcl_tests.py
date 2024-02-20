import unittest

import numpy as np
from Utils import deg_to_rad, direction, ang_diff
from Visualize import Visualize
from likelihood import *
from mcl_resample import normalize, resample
from Map import Map, myMap
from main import Robot

class TestMonteCarloMethods(unittest.TestCase):

    # NOTE: for testing, please comment out lines 1-9 in Constants.py
    def setUp(self):
        self.visualizer = Visualize(0.5, deg_to_rad(1), deg_to_rad(3))
        self.terrain = myMap(self.visualizer)
        self.robot = Robot(self.visualizer, self.terrain)

    # Check that absolute angle relative to x-axis is correctly calculated
    def test_vec_angle_from_east(self):
        northwest = vec_angle_from_east(0, 0, -10, 10)
        self.assertGreaterEqual(northwest, np.pi / 2)
        self.assertLessEqual(northwest, np.pi)

        southeast = vec_angle_from_east(0, 0, 10, -10)
        self.assertGreaterEqual(southeast, -np.pi / 2)
        self.assertLessEqual(southeast, 0)
        
        northeast = vec_angle_from_east(0, 0, 10, 10)
        self.assertGreaterEqual(northeast, 0)
        self.assertLessEqual(northeast, np.pi / 2)
        
        southwest = vec_angle_from_east(0, 0, -10, -10)
        self.assertGreaterEqual(southwest, -np.pi)
        self.assertLessEqual(southwest, -np.pi / 2)

    def test_will_hit_wall_wp1_orientx(self):
        GH_wall = (210, 84, 210, 0)
        self.assertTrue(will_hit_wall(84, 30, 0, GH_wall))
        # Although OH is closer, the robot is not facing OH so it should not hit
        OH_wall = (0, 0, 210, 0)
        self.assertFalse(will_hit_wall(84, 30, 0, OH_wall))

    def test_will_hit_wall_corner(self):
        AB_wall = (0, 168, 84, 168)
        OA_wall = (0, 0, 0, 168)
        self.assertTrue(will_hit_wall(10, 158, 3 * np.pi/4, AB_wall))
        self.assertTrue(will_hit_wall(10, 158, 3 * np.pi/4, OA_wall))

    def test_will_hit_wall_facing_west(self):
        OA_wall = (0, 0, 0, 168)
        self.assertTrue(will_hit_wall(10, 158, np.pi, OA_wall))

    def test_wall_distance_wp1_orientx(self):
        # Location: waypoint 1, facing along the x-axis (right)
        pos_x, pos_y, pos_theta = 84, 30, 0
        # Should be facing wall between points G, H
        expected_distance = abs(210 - pos_x)
        predicted_distance = wall_distance(pos_x, pos_y, pos_theta, self.terrain)
        self.assertEqual(expected_distance, predicted_distance)

    def test_wall_distance_wp1_orienty(self):
        # Location: waypoint 1, facing along the y-axis (down)
        pos_x, pos_y, pos_theta = 84, 30, (-np.pi/2)
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
        pos_x, pos_y, pos_theta = 180, 30, (np.pi/2)
        # Should be facing wall between points F, G
        expected_distance = 84 - pos_y
        predicted_distance = wall_distance(pos_x, pos_y, pos_theta, self.terrain)
        self.assertEqual(expected_distance, predicted_distance)

    # Just to check that MCL is reasonable
    def test_recalc_sensor_from_first_waypoint(self):
        print("TEST_RECALC_SENSOR")
        # Start at waypoint 1 and move 20 cm -> (104, 30, 0)
        self.robot.v.particles_gen(5, 84, 30, 0)
        print_particles(self.robot.v.particles)

        print("MOTION UPDATE")
        self.robot.v.forward(20)
        print_particles(self.robot.v.particles)
        
        # Assume sonar sensor returns reading
        print("SENSOR UPDATE")
        d_measure = 126 # expecting 126 distance to wall
        for p in self.robot.v.particles:
            p.weight *= self.robot.calculate_likelihood(p.x, p.y, p.theta, d_measure)
        print_particles(self.robot.v.particles)

        print("NORMALIZE")
        normalize(self.robot.v)
        print_particles(self.robot.v.particles)

        print("RESAMPLE")
        resample(self.robot.v)
        print_particles(self.robot.v.particles)

        self.robot.loc = self.robot.v.estimate_location()
        print("Estimated location update", self.robot.loc)

def print_particles(particles):
    print("Current particles in visualization")
    str_particles = [str(p) for p in particles]
    print(str_particles)

if __name__ == '__main__':
    unittest.main()