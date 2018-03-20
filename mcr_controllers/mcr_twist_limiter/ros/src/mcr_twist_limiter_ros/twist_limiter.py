#!/usr/bin/env python
"""
This module contains a component that sets the individual components
of a twist (represented as a geometry_msgs/TwistStamped message) to
a specified maximum, if they exceed their respective limit.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_twist_limiter.limiter as limiter


class TwistLimiter(object):
    """
    Sets the individual components of a twist to a specified
    maximum, if they exceed their respective limit.

    """
    def __init__(self):
        # Maximum linear velocities in the axes X, Y and Z (in meters/second)
        self.max_velocity_x = rospy.get_param('~max_velocity_x', 0.1)
        self.max_velocity_y = rospy.get_param('~max_velocity_y', 0.1)
        self.max_velocity_z = rospy.get_param('~max_velocity_z', 0.1)

        # Maximum angular velocities around the axes X, Y and Z (in radians/second)
        self.max_velocity_roll = rospy.get_param('~max_velocity_roll', 0.1)
        self.max_velocity_pitch = rospy.get_param('~max_velocity_pitch', 0.1)
        self.max_velocity_yaw = rospy.get_param('~max_velocity_yaw', 0.6)

    def set_parameters(self, max_velocity_x, max_velocity_y, max_velocity_z, max_velocity_yaw, max_velocity_pitch, max_velocity_roll):
        """
        Helper method for dynamic reconfiguration
        """
        self.max_velocity_x = max_velocity_x
        self.max_velocity_y = max_velocity_y
        self.max_velocity_z = max_velocity_z
        self.max_velocity_yaw = max_velocity_yaw
        self.max_velocity_roll = max_velocity_roll
        self.max_velocity_pitch = max_velocity_pitch



    def get_limited_twist(self, twist):
        """
        Limits a twist if it exceeds the specified maximum.

        :return: The Cartesian velocity to reduce the position error.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        self.twist = twist
        limited_twist = geometry_msgs.msg.TwistStamped()
        limited_twist.header.frame_id = self.twist.header.frame_id
        limited_twist.header.stamp = self.twist.header.stamp

        limited_twist.twist.linear.x = limiter.limit_value(
            self.twist.twist.linear.x, self.max_velocity_x
        )
        limited_twist.twist.linear.y = limiter.limit_value(
            self.twist.twist.linear.y, self.max_velocity_y
        )
        limited_twist.twist.linear.z = limiter.limit_value(
            self.twist.twist.linear.z, self.max_velocity_z
        )
        limited_twist.twist.angular.x = limiter.limit_value(
            self.twist.twist.angular.x, self.max_velocity_roll
        )
        limited_twist.twist.angular.y = limiter.limit_value(
            self.twist.twist.angular.y, self.max_velocity_pitch
        )
        limited_twist.twist.angular.z = limiter.limit_value(
            self.twist.twist.angular.z, self.max_velocity_yaw
        )
        # limited_twist.twist.angular.z = self.twist.twist.angular.z
        # rospy.loginfo("actual {0}".format( self.twist.twist.angular.z))
        # rospy.loginfo("max {0}".format( self.max_velocity_yaw))
        # rospy.loginfo("final {0}".format( limited_twist.twist.angular.z))
        # rospy.loginfo("max", self.max_velocity_yaw)
        # rospy.loginfo("final", limited_twist.twist.angular.z)
        return limited_twist
