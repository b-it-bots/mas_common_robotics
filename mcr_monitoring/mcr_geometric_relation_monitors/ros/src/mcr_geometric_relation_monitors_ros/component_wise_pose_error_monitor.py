#!/usr/bin/env python
"""
This module contains a component that monitors
the component-wise pose error

"""

import rospy
import std_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_monitoring_msgs.msg

# -*- encoding: utf-8 -*-
__author__ = 'ramit'


class ComponentWisePoseErrorMonitor(object):
    """
    Monitors the error between two poses in three
    linear components and three angular components.

    """
    def __init__(self):
        self.feedback = mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback()
        self.threshold_linear_x = 0
        self.threshold_linear_y = 0
        self.threshold_linear_z = 0
        self.threshold_angular_x = 0
        self.threshold_angular_y = 0
        self.threshold_angular_z = 0

    def set_parameters(self, threshold_linear_x, threshold_linear_y, threshold_linear_z,
                       threshold_angular_x, threshold_angular_y, threshold_angular_z):
        self.threshold_linear_x = threshold_linear_x
        self.threshold_linear_y = threshold_linear_y
        self.threshold_linear_z = threshold_linear_z
        self.threshold_angular_x = threshold_angular_x
        self.threshold_angular_y = threshold_angular_y
        self.threshold_angular_z = threshold_angular_z

    def isComponentWisePoseErrorWithinThreshold(self, pose_error):

        self.feedback.is_linear_x_within_tolerance = abs(pose_error.linear.x) < self.threshold_linear_x
        self.feedback.is_linear_y_within_tolerance = abs(pose_error.linear.y) < self.threshold_linear_y
        self.feedback.is_linear_z_within_tolerance = abs(pose_error.linear.z) < self.threshold_linear_z
        self.feedback.is_angular_x_within_tolerance = abs(pose_error.angular.x) < self.threshold_angular_x
        self.feedback.is_angular_y_within_tolerance = abs(pose_error.angular.y) < self.threshold_angular_y
        self.feedback.is_angular_z_within_tolerance = abs(pose_error.angular.z) < self.threshold_angular_z
        return (self.feedback.is_linear_x_within_tolerance and
                self.feedback.is_linear_y_within_tolerance and
                self.feedback.is_linear_z_within_tolerance and
                self.feedback.is_angular_x_within_tolerance and
                self.feedback.is_angular_y_within_tolerance and
                self.feedback.is_angular_z_within_tolerance)
