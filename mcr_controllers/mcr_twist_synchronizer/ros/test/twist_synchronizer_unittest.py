#!/usr/bin/env python
"""
Test unit for the functions/methods used in twist_synchronizer.py module.

"""

import unittest
import rosunit
import mcr_twist_synchronizer_ros.twist_synchronizer \
    as twist_synchronizer

PKG = 'mcr_twist_synchronizer'


class TestTwistSynchronizer(unittest.TestCase):
    """
    Tests methods used in the twist_synchronizer.py module.

    """
    def test_calculate_max_time(self):
        """
        Tests that the 'calculate_max_time' function returns the correct value.

        """
        velocity_x = 0.02
        velocity_y = -0.8
        velocity_z = 1.96

        error_x = 0.2
        error_y = 0.2
        error_z = 0.2

        velocity = (velocity_x, velocity_y, velocity_z)
        error = (error_x, error_y, error_z)

        result_1 = 10
        result_2 = 0.25
        result_3 = 0.10204

        self.assertAlmostEqual(
            twist_synchronizer.calculate_max_time(error, velocity), result_1
        )

        velocity = (0.0001, velocity_y, velocity_z)
        self.assertAlmostEqual(
            twist_synchronizer.calculate_max_time(error, velocity), result_2
        )

        velocity = (0.0001, 0.0001, velocity_z)
        self.assertAlmostEqual(
            twist_synchronizer.calculate_max_time(error, velocity), result_3, places=4
        )

    def test_calculate_sync_velocity(self):
        """
        Tests that the 'calculate_sync_velocity' function returns the correct value.

        """
        velocity_x = 0.2
        velocity_y = -0.6
        velocity_z = 0.5

        error_x = 0.02
        error_y = -0.8
        error_z = 1.96

        max_time = 5

        velocity = (velocity_x, velocity_y, velocity_z)
        error = (error_x, error_y, error_z)

        expected_result = [0.004, -0.16, 0.392]

        result = twist_synchronizer.calculate_sync_velocity(
            error, velocity, max_time
        )

        self.assertAlmostEqual(result[0], expected_result[0], places=4)
        self.assertAlmostEqual(result[1], expected_result[1], places=4)
        self.assertAlmostEqual(result[2], expected_result[2], places=4)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_twist_synchronizer', TestTwistSynchronizer)
