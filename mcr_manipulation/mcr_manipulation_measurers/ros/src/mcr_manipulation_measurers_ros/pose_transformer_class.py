#!/usr/bin/env python
'''
   This component transforms a give pose into specified target frame.
'''
#-*- encoding: utf-8 -*-
__author__ = 'shehzad'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf


class PoseTransformer(object):
    def __init__(self):
        self.listener = tf.TransformListener()

        # how long to wait for transform (in seconds)
        self.wait_for_transform = 0.1

        self.transform_tries = 5

        self.transformed_pose = None

    def get_transformed_pose(self, reference_pose, target_frame):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        for i in range(0, self.transform_tries):
            self.transformed_pose = self.transform_pose(reference_pose, target_frame)
            if self.transformed_pose:
                return self.transformed_pose
        self.transformed_pose = None
        return self.transformed_pose


    def transform_pose(self, reference_pose, target_frame):
        """
        Transforms a given pose into the target frame.

        :param reference_pose: The reference pose.
        :type reference_pose: geometry_msgs.msg.PoseStamped

        :param target_frame: The name of the taget frame.
        :type target_frame: String

        :return: The pose in the target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            common_time = self.listener.getLatestCommonTime(
                target_frame, reference_pose.header.frame_id
            )

            self.listener.waitForTransform(
                target_frame, reference_pose.header.frame_id,
                common_time, rospy.Duration(self.wait_for_transform)
            )
            reference_pose.header.stamp = common_time

            transformed_pose = self.listener.transformPose(
                target_frame, reference_pose,
            )

            return transformed_pose

        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None
