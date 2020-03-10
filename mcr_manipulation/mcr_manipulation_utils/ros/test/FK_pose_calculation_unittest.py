#!/usr/bin/env python
"""
Test unit for the functions in the link_pose_calculation.py module.

"""

import unittest

from mcr_manipulation_utils_ros.FK_pose_calculation import \
    CalculateLinkPose

PKG = "mcr_manipulation_utils_ros"


class TestLinkPoseCalculation(unittest.TestCase):

    def test_joint_configuration(self):

        joint_names = [
            "arm_joint_1", "arm_joint_2", "arm_joint_3",
            "arm_joint_4", "arm_joint_5"
        ]

        joint_values = [
            2.1642, 1.13446, -2.54818, 1.78896, 2.93075
        ]

        link_pose_calculator = CalculateLinkPose("arm_1")

        srdf_joint_values, srdf_joint_names = link_pose_calculator.get_group_state_values("candle")

        self.assertListEqual(srdf_joint_values, joint_values)
        self.assertListEqual(srdf_joint_names, joint_names)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_link_pose_calculator', TestLinkPoseCalculation)
