#!/usr/bin/env python
"""
Test integration for the pose_error_to_pose_converter node.

"""

import unittest
import rostest
import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import tf

PKG = 'mcr_pose_error_to_pose_converter'


class TestPoseErrorToPoseConverter(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        self.offset_x = rospy.get_param('~offset_x', 0.0)
        self.offset_y = rospy.get_param('~offset_y', 0.0)

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True
        )
        self.component_output = rospy.Publisher(
            '~component_output', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference
        )

        # subscribers
        self.component_input = rospy.Subscriber(
            '~component_input',
            geometry_msgs.msg.PoseStamped,
            self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.component_input.unregister()

    def test_pose_error_to_pose_converter(self):
        """
        Verifies that the node returns pose correctly.

        """
        pose_error = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        pose_error.linear.x = 0.251 
        pose_error.linear.y = -0.016
        pose_error.linear.z = 0.456
        pose_error.angular.x = 0.0
        pose_error.angular.y = 0.0
        pose_error.angular.z = 0.0

        expected_result = geometry_msgs.msg.PoseStamped()
        expected_result.header.frame_id = 'base_link'
        expected_result.pose.position.x = 0.251 + self.offset_x
        expected_result.pose.position.y = -0.016 + self.offset_y
        expected_result.pose.position.z = 0.0
        expected_result.pose.orientation.x = 0.0
        expected_result.pose.orientation.y = 0.0
        expected_result.pose.orientation.z = 0.0
        expected_result.pose.orientation.w = 0.0

        while not self.wait_for_result:
            self.component_output.publish(pose_error)
            self.event_out.publish('e_start')
        self.event_out.publish('e_stop')

        self.assertAlmostEqual(expected_result.pose.position.x, self.result.pose.position.x)
        self.assertAlmostEqual(expected_result.pose.position.y, self.result.pose.position.y)
        self.assertAlmostEqual(expected_result.pose.position.z, self.result.pose.position.z)
        self.assertAlmostEqual(expected_result.pose.orientation.x, self.result.pose.orientation.x)
        self.assertAlmostEqual(expected_result.pose.orientation.y, self.result.pose.orientation.y)
        self.assertAlmostEqual(expected_result.pose.orientation.z, self.result.pose.orientation.z)
        self.assertAlmostEqual(expected_result.pose.orientation.w, self.result.pose.orientation.w)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('pose_error_to_pose_converter_test')
    rostest.rosrun(PKG, 'pose_error_to_pose_converter_test', TestPoseErrorToPoseConverter)
