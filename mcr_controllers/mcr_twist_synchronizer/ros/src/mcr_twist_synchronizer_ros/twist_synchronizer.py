#!/usr/bin/env python
"""
This module contains a component that synchronizes the velocities
of a twist (represented as a geometry_msgs/TwistStamped message),
such that each component of a Cartesian error (compensated by the
twist's velocities) simultaneously reaches zero.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg

# This value prevents division by near-zero values.
ZERO = 0.001


class TwistSynchronizer(object):
    """
    Synchronizes the velocities of a twist (represented as a
    geometry_msgs/TwistStamped message), such that each component
    of a Cartesian error (compensated by the twist's velocities)
    simultaneously reaches zero.

    """
    def __init__(self):
        # params
        self.event = None
        self.twist = None
        self.pose_error = None

        # node cycle time (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time')

        # publishers
        self.synchronized_twist = rospy.Publisher(
            '~synchronized_twist', geometry_msgs.msg.TwistStamped
        )

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~twist', geometry_msgs.msg.TwistStamped, self.twist_cb)
        rospy.Subscriber(
            '~pose_error',
            mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference,
            self.pose_error_cb
        )

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            rospy.sleep(self.cycle_time)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def twist_cb(self, msg):
        """
        Obtains the twist.

        """
        self.twist = msg

    def pose_error_cb(self, msg):
        """
        Obtains the pose error.

        """
        self.pose_error = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.twist and self.pose_error:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'RUNNING'
        elif self.event == 'e_stop':
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            return 'INIT'
        else:
            synchronized_twist = self.synchronize_twist()
            self.synchronized_twist.publish(synchronized_twist)

            return 'RUNNING'

    def synchronize_twist(self):
        """
        Synchronizes a twist to make its velocities finish at the same time.

        :return: The synchronized twist.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        synchronized_twist = geometry_msgs.msg.TwistStamped()
        synchronized_twist.header.frame_id = self.twist.header.frame_id
        synchronized_twist.header.stamp = self.twist.header.stamp

        error = (self.pose_error.linear.x, self.pose_error.linear.y,
                 self.pose_error.linear.z)

        velocity = (self.twist.twist.linear.x, self.twist.twist.linear.y,
                    self.twist.twist.linear.z)

        # Calculate maximum time to reach the goal.
        max_time = calculate_max_time(error, velocity)

        # Calculate the velocities to reach the goal at the same time.
        sync_velocities = calculate_sync_velocity(error, velocity, max_time)

        synchronized_twist.twist.linear.x = sync_velocities[0]
        synchronized_twist.twist.linear.y = sync_velocities[1]
        synchronized_twist.twist.linear.z = sync_velocities[2]

        return synchronized_twist


def calculate_max_time(error, velocity):
    """
    Calculates the maximum time, between all three velocities,
    required to reach the goal. If a velocity is not specified,
    it is assumed that that axis is not to be controlled.

    :param error: The distance error tuple in the X, Y and Z axis.
    :type error: ()float

    :param velocity: Velocities in the X, Y and Z axis.
    :type velocity: ()float

    :return: The maximum time required to reach the goal.
    :rtype: float

    """
    t = []
    for i, vel in enumerate(velocity):
        if abs(vel) >= ZERO:
            time = abs(float(error[i]) / vel)
        else:
            time = 0.0
        t.append(time)

    return max(t)


def calculate_sync_velocity(error, velocity, max_time):
    """
    Calculates the synchronized velocity for all velocities to reach
    their goal at the same time.

    :param error: The distance error tuple in the X, Y and Z axis.
    :type error: ()float

    :param velocity: Velocity in the X, Y and Z axis.
    :type velocity: ()float

    :param max_time: The maximum time required, between the velocities,
    to reach their goal.
    :type max_time: float

    :return: The synchronized velocities.
    :rtype: float

    """
    sync_velocity = []

    for i, vel in enumerate(velocity):
        if max_time and vel:
            temp_velocity = abs(float(error[i]) / max_time) * cmp(vel, 0)
        else:
            temp_velocity = 0.0

        sync_velocity.append(temp_velocity)

    return sync_velocity


def main():
    rospy.init_node('twist_synchronizer', anonymous=True)
    twist_synchronizer = TwistSynchronizer()
    twist_synchronizer.start()
