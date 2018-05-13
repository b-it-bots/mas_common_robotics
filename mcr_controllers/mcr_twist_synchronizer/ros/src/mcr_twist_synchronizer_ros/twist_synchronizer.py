#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This component synchronizes the velocities of a twist (represented as a
geometry_msgs/TwistStamped message), such that each component of a Cartesian error
(compensated by the twist's velocities) simultaneously reaches zero.

**Input(s):**
  * `twist`: The twist to be synchronized.
  * `pose_error`: The component-wise Cartesian difference (error).

**Output(s):**
  * `synchronized_twist`: The synchronized twist.

**Parameter(s):**
  * `angular_synchronization`: If True, it also synchronizes the angular and linear
  velocities. By default, it only synchronizes the linear velocities (bool).
  * `near_zero`: A value to prevent division by near-zero values.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_twist_synchronizer_ros.twist_synchronizer_utils as utils


class TwistSynchronizer(object):
    """
    Synchronizes a twist.

    """
    def __init__(self):
        # If True, it also synchronizes the angular and linear velocities.
        # By default, it only synchronizes the linear velocities.
        self.angular_synchronization = rospy.get_param('~angular_synchronization', True)

        # A value to prevent division by near-zero values.
        self.near_zero = rospy.get_param('~near_zero', 0.001)

    def set_angular_synchronization(self, angular_synchronization):
        """
        sets the angular_synchronization parameter
        If set to True, both angular and linear velocities are synchronized
        """
        self.angular_synchronization = angular_synchronization

    def synchronize_twist(self, twist, pose_error):
        """
        Synchronizes a twist to make its velocities finish at the same time.

        :return: The synchronized twist.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        synchronized_twist = geometry_msgs.msg.TwistStamped()
        synchronized_twist.header.frame_id = twist.header.frame_id
        synchronized_twist.header.stamp = twist.header.stamp

        if self.angular_synchronization:
            error = [
                pose_error.linear.x, pose_error.linear.y,
                pose_error.linear.z, pose_error.angular.x,
                pose_error.angular.y, pose_error.angular.z
            ]

            velocity = [
                twist.twist.linear.x, twist.twist.linear.y,
                twist.twist.linear.z, twist.twist.angular.x,
                twist.twist.angular.y, twist.twist.angular.z
            ]
        else:
            error = [
                pose_error.linear.x, pose_error.linear.y,
                pose_error.linear.z
            ]

            velocity = [
                twist.twist.linear.x, twist.twist.linear.y,
                twist.twist.linear.z
            ]

        # Calculate maximum time to reach the goal.
        max_time = utils.calculate_max_time(
            error, velocity, self.angular_synchronization, self.near_zero
        )

        # Calculate the velocities to reach the goal at the same time.
        sync_velocities = utils.calculate_sync_velocity(
            error, velocity, max_time, self.angular_synchronization
        )

        synchronized_twist.twist.linear.x = sync_velocities[0]
        synchronized_twist.twist.linear.y = sync_velocities[1]
        synchronized_twist.twist.linear.z = sync_velocities[2]
        if self.angular_synchronization:
            synchronized_twist.twist.angular.x = sync_velocities[3]
            synchronized_twist.twist.angular.y = sync_velocities[4]
            synchronized_twist.twist.angular.z = sync_velocities[5]

        return synchronized_twist
