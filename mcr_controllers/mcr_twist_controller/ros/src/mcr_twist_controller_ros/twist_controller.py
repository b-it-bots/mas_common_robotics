#!/usr/bin/env python
"""
This module contains a component that calculates a
twist to reduce the difference between two poses.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_algorithms.controller.pid_controller as pid_controller

# -*- encoding: utf-8 -*-
__author__ = 'jsanch'

# This value is used as the 'current_value' for the 'pid_controller'
# from 'mcr_algorithms.controller.pid_controller', since this twist
# controller only receives an error which is used as 'set_value'.
ZERO = 0.0


class TwistController(object):
    """
    Calculates a twist, based on the Cartesian distance
    error between two poses.

    """
    def __init__(self):
        # proportional gains for the Cartesian linear velocities
        self.p_gain_x = rospy.get_param('~p_gain_x', 0.0)
        self.p_gain_y = rospy.get_param('~p_gain_y', 0.0)
        self.p_gain_z = rospy.get_param('~p_gain_z', 0.0)
        self.p_gain_roll = rospy.get_param('~p_gain_roll', 0.0)
        self.p_gain_pitch = rospy.get_param('~p_gain_pitch', 0.0)
        self.p_gain_yaw = rospy.get_param('~p_gain_yaw', 0.0)

        # create controllers
        self.x_controller = pid_controller.p_controller(self.p_gain_x)
        self.y_controller = pid_controller.p_controller(self.p_gain_y)
        self.z_controller = pid_controller.p_controller(self.p_gain_z)
        self.roll_controller = pid_controller.p_controller(self.p_gain_roll)
        self.pitch_controller = pid_controller.p_controller(self.p_gain_pitch)
        self.yaw_controller = pid_controller.p_controller(self.p_gain_yaw)

    def set_parameters(self, p_gain_x, p_gain_y, p_gain_z, p_gain_roll, p_gain_pitch, p_gain_yaw):
        # proportional gains for the Cartesian linear velocities
        self.p_gain_x = p_gain_x
        self.p_gain_y = p_gain_y
        self.p_gain_z = p_gain_z
        self.p_gain_roll = p_gain_roll
        self.p_gain_pitch = p_gain_pitch
        self.p_gain_yaw = p_gain_yaw

        self.x_controller = pid_controller.p_controller(self.p_gain_x)
        self.y_controller = pid_controller.p_controller(self.p_gain_y)
        self.z_controller = pid_controller.p_controller(self.p_gain_z)
        self.roll_controller = pid_controller.p_controller(self.p_gain_roll)
        self.pitch_controller = pid_controller.p_controller(self.p_gain_pitch)
        self.yaw_controller = pid_controller.p_controller(self.p_gain_yaw)

    def get_cartesian_velocity(self, pose_error):
        """
        Calculates a synchronized Cartesian velocity, based on a position error,
        for the three linear velocities and the three angular velocities.

        :return: The Cartesian velocity to reduce the position error.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        cartesian_velocity = geometry_msgs.msg.TwistStamped()
        cartesian_velocity.header.frame_id = pose_error.header.frame_id
        cartesian_velocity.header.stamp = rospy.Time.now()

        velocity_x = self.x_controller.control(pose_error.linear.x, ZERO)
        velocity_y = self.y_controller.control(pose_error.linear.y, ZERO)
        velocity_z = self.z_controller.control(pose_error.linear.z, ZERO)
        velocity_roll = self.roll_controller.control(pose_error.angular.x, ZERO)
        velocity_pitch = self.pitch_controller.control(pose_error.angular.y, ZERO)
        velocity_yaw = self.yaw_controller.control(pose_error.angular.z, ZERO)

        cartesian_velocity.twist.linear.x = velocity_x
        cartesian_velocity.twist.linear.y = velocity_y
        cartesian_velocity.twist.linear.z = velocity_z
        cartesian_velocity.twist.angular.x = velocity_roll
        cartesian_velocity.twist.angular.y = velocity_pitch
        cartesian_velocity.twist.angular.z = velocity_yaw

        return cartesian_velocity
