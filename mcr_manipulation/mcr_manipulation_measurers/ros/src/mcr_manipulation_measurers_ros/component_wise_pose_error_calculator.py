#!/usr/bin/env python
"""
This module contains a component that calculates
the component-wise error between two poses.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
import mcr_manipulation_msgs.msg
from math import sin, cos, atan2

class ComponentWisePoseErrorCalculator(object):
    """
    Calculates the error between two poses in three
    linear components and three angular components.

    """
    def __init__(self):
        # params
        self.tf_listener = tf.TransformListener()

        # linear offset applied to the result (a three-element list)
        self.linear_offset = rospy.get_param('~linear_offset', None)
        if self.linear_offset is not None:
            assert (
                isinstance(self.linear_offset, list) and len(self.linear_offset) == 3
            ), "If linear offset is specified, it must be a three-dimensional array."

        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

    def set_parameters(self, wait_for_transform):
        """
        Helper method for dynamic reconfiguration
        """
        self.wait_for_transform = wait_for_transform

    def get_component_wise_pose_error(self, pose_1, pose_2):
        if pose_1 and pose_2:
            transformed_pose = self.transform_pose(pose_1, pose_2)
            if transformed_pose:
                pose_error = calculate_component_wise_pose_error(
                    pose_1, transformed_pose, self.linear_offset
                )
                return pose_error

            else:
                return None
        else:
            self.reset_component_data()
            return None

    def transform_pose(self, reference_pose, target_pose):
        """
        Transforms the target pose into the frame of the reference pose.

        :param reference_pose: The reference pose.
        :type reference_pose: geometry_msgs.msg.PoseStamped

        :param target_pose: The current pose.
        :type target_pose: geometry_msgs.msg.PoseStamped

        :return: The target pose transformed to the frame of the reference pose.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            target_pose.header.stamp = self.tf_listener.getLatestCommonTime(
                target_pose.header.frame_id, reference_pose.header.frame_id
            )

            self.tf_listener.waitForTransform(
                target_pose.header.frame_id, reference_pose.header.frame_id,
                target_pose.header.stamp, rospy.Duration(self.wait_for_transform)
            )

            transformed_pose = self.tf_listener.transformPose(
                reference_pose.header.frame_id, target_pose,
            )

            return transformed_pose

        except tf.Exception as error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.pose_1 = None
        self.pose_2 = None


def calculate_component_wise_pose_error(current_pose, target_pose, offset=None):
    """
    Calculates the component-wise error between two 'PoseStamped' objects.
    It assumes that both poses are specified with respect of the same
    reference frame.

    :param current_pose: The current pose.
    :type current_pose: geometry_msgs.msg.PoseStamped

    :param target_pose: The target pose.
    :type target_pose: geometry_msgs.msg.PoseStamped

    :param offset: A linear offset in X, Y, Z.
    :type offset: list

    :return: The difference in the six components
    (three linear and three angular).
    :rtype: mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference

    """
    error = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
    error.header.frame_id = current_pose.header.frame_id

    # calculate linear distances
    error.linear.x = target_pose.pose.position.x - current_pose.pose.position.x
    error.linear.y = target_pose.pose.position.y - current_pose.pose.position.y
    error.linear.z = target_pose.pose.position.z - current_pose.pose.position.z

    current_quaternion = [
        current_pose.pose.orientation.x, current_pose.pose.orientation.y,
        current_pose.pose.orientation.z, current_pose.pose.orientation.w
    ]
    target_quaternion = [
        target_pose.pose.orientation.x, target_pose.pose.orientation.y,
        target_pose.pose.orientation.z, target_pose.pose.orientation.w
    ]

    # convert quaternions into roll, pitch, yaw angles
    current_angles = tf.transformations.euler_from_quaternion(current_quaternion)
    target_angles = tf.transformations.euler_from_quaternion(target_quaternion)

    # calculate angular distances
    error.angular.x = getShortestAngle(target_angles[0], current_angles[0])
    error.angular.y = getShortestAngle(target_angles[1], current_angles[1])
    error.angular.z = getShortestAngle(target_angles[2], current_angles[2])

    if offset is not None:
        offset = tuple(offset)
        error.linear.x += offset[0]
        error.linear.y += offset[1]
        error.linear.z += offset[2]

    return error

def getShortestAngle(angle1, angle2):
   return atan2(sin(angle1 - angle2), cos(angle1 - angle2));
