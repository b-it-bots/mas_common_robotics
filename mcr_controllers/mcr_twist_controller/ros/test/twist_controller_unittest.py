#!/usr/bin/env python
"""
Test unit for the functions/methods used in relative_twist_controller.py module.

"""

PKG = 'mcr_relative_twist_controller'

import unittest
import rosunit
import mcr_relative_twist_controller_ros.relative_twist_controller \
    as relative_twist_controller


class TestRelativeTwistController(unittest.TestCase):
    """
    Tests methods used in the relative_twist_controller.py module.

    """
    def test_control_velocity_in_limit(self):
        """
        Tests that the 'calculate_control_velocity' function
        returns the correct value, when the control velocity is
        within the velocity limits and the distance error is
        greater than the epsilon.

        """
        error = 0.3
        p_gain = 2.0
        max_velocity = 2.0
        epsilon = 0.02

        result = 0.6

        self.assertAlmostEqual(
            relative_twist_controller.calculate_control_velocity(
                error, p_gain, max_velocity, epsilon), result
        )

    def test_control_velocity_outside_limit(self):
        """
        Tests that the 'calculate_control_velocity' function
        returns the correct value, when the control velocity is
        outside the velocity limits and the distance error is
        greater than the epsilon.

        """
        error = 0.9
        p_gain = 2.0
        max_velocity = 1.0
        epsilon = 0.02

        result = 1.0

        self.assertAlmostEqual(
            relative_twist_controller.calculate_control_velocity(
                error, p_gain, max_velocity, epsilon), result
        )

        error = -0.9
        max_velocity = 0.5

        result = -0.5

        self.assertAlmostEqual(
            relative_twist_controller.calculate_control_velocity(
                error, p_gain, max_velocity, epsilon), result
        )

    def test_control_velocity_set_to_zero(self):
        """
        Tests that the 'calculate_control_velocity' function
        returns a zero velocity, since the distance error is
        lesser than the epsilon.

        """
        error = 0.01
        p_gain = 2.0
        max_velocity = 2.0
        epsilon = 0.02

        result = 0.0

        self.assertAlmostEqual(
            relative_twist_controller.calculate_control_velocity(
                error, p_gain, max_velocity, epsilon), result
        )

    def test_calculate_max_time(self):
        """
        Tests that the 'calculate_max_time' function returns the maximum time
        required, for each axis, to reach its goal.

        """
        error_x = 0.2
        error_y = 0.0
        error_z = 2.0
        velocity_x = 0.4
        velocity_z = -1.0

        expected_result = 2.0

        self.assertAlmostEqual(
            relative_twist_controller.calculate_max_time(
                error_x, error_y, error_z, velocity_x, None, velocity_z),
            expected_result
        )

    def test_calculate_sync_velocity(self):
        """
        Tests that the 'calculate_sync_velocity' function returns velocities
        for the different axis to reach their goal at the same time.

        """
        error_x = 0.2
        error_y = 0.5
        error_z = 0.8
        velocity_x = 0.4
        velocity_z = -1.0
        max_time = 0.5

        expected_result = [0.4, 0.0, 1.6]

        self.assertAlmostEqual(
            relative_twist_controller.calculate_sync_velocity(
                error_x, error_y, error_z, max_time, velocity_x, None, velocity_z),
            expected_result
        )

        max_time = 0.0
        expected_result = [0.0, 0.0, 0.0]

        self.assertAlmostEqual(
            relative_twist_controller.calculate_sync_velocity(
                error_x, error_y, error_z, max_time, velocity_x, None, velocity_z),
            expected_result
        )


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_relative_twist_controller', TestRelativeTwistController)
