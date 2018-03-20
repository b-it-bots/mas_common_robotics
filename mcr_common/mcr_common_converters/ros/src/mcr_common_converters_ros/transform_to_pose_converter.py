#!/usr/bin/env python
"""
This module contains a component that converts the
ROS-TF transform between two frames into a ROS pose message.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
import dynamic_reconfigure.server
# import group3_direct_base_controller.cfg.TransformToPoseConverterConfig as TransformToPoseConverterConfig


class TransformToPoseConverter(object):
    """
    Converts a transform, between a target frame with
    respect to a specified frame, into a pose.

    """
    def __init__(self):
        # params
        self.reference_frame = rospy.get_param('~reference_frame', None)
        assert self.reference_frame is not None, "Reference frame must be defined."
        self.target_frame = rospy.get_param('~target_frame', None)
        assert self.target_frame is not None, "Target frame must be defined."

        self.listener = tf.TransformListener()

        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

    def set_parameters(self, reference_frame, target_frame, wait_for_transform):
        """
        Helper method for dynamic reconfiguration
        """
        self.reference_frame = reference_frame
        self.target_frame = target_frame
        self.waitForTransform = wait_for_transform

    def get_converted_pose(self):
        """
        Publishes the converted pose based on the transform
        between a target frame and the reference frame.

        """
        if self.target_frame and self.reference_frame:
            converted_pose = geometry_msgs.msg.PoseStamped()
            converted_pose.header.frame_id = self.reference_frame

            try:
                self.listener.waitForTransform(
                    self.reference_frame, self.target_frame,
                    rospy.Time(0), rospy.Duration(self.wait_for_transform)
                )

                (translation, rotation) = self.listener.lookupTransform(
                    self.reference_frame, self.target_frame, rospy.Time(0)
                )

                converted_pose.pose.position.x = translation[0]
                converted_pose.pose.position.y = translation[1]
                converted_pose.pose.position.z = translation[2]
                converted_pose.pose.orientation.x = rotation[0]
                converted_pose.pose.orientation.y = rotation[1]
                converted_pose.pose.orientation.z = rotation[2]
                converted_pose.pose.orientation.w = rotation[3]

                converted_pose.header.stamp = self.listener.getLatestCommonTime(
                    self.reference_frame, self.target_frame
                )

            except tf.Exception, error:
                rospy.logwarn("Exception occurred: {0}".format(error))
                converted_pose = None

            return converted_pose

        return None
