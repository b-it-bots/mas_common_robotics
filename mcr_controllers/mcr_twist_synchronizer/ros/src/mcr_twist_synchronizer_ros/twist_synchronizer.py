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



class TwistSynchronizer(object):
    """
    Synchronizes the velocities of a twist (represented as a
    geometry_msgs/TwistStamped message), such that each component
    of a Cartesian error (compensated by the twist's velocities)
    simultaneously reaches zero.

    """
    def __init__(self):
        # Params
        self.event = None
        self.twist = None
        self.pose_error = None

        # If True, it also synchronizes the angular and linear velocities.
        # By default, it only synchronizes the linear velocities.
        self.angular_synchronization = rospy.get_param('~angular_synchronization', False)

        # A value to prevent division by near-zero values.
        self.near_zero = rospy.get_param('~near_zero', 0.001)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)
        self.synchronized_twist = rospy.Publisher(
            '~synchronized_twist', geometry_msgs.msg.TwistStamped
        )

        # Subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~twist', geometry_msgs.msg.TwistStamped, self.twist_cb)
        rospy.Subscriber(
            '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference,
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
            self.loop_rate.sleep()

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
        if self.event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.twist and self.pose_error:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            synchronized_twist = self.synchronize_twist()
            if synchronized_twist:
                self.synchronized_twist.publish(synchronized_twist)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

            self.reset_component_data()
            return 'IDLE'

    def synchronize_twist(self):
        """
        Synchronizes a twist to make its velocities finish at the same time.

        :return: The synchronized twist.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        synchronized_twist = geometry_msgs.msg.TwistStamped()
        synchronized_twist.header.frame_id = self.twist.header.frame_id
        synchronized_twist.header.stamp = self.twist.header.stamp

        if self.angular_synchronization:
            error = [
                self.pose_error.linear.x, self.pose_error.linear.y,
                self.pose_error.linear.z, self.pose_error.angular.x,
                self.pose_error.angular.y, self.pose_error.angular.z
            ]

            velocity = [
                self.twist.twist.linear.x, self.twist.twist.linear.y,
                self.twist.twist.linear.z, self.twist.twist.angular.x,
                self.twist.twist.angular.y, self.twist.twist.angular.z
            ]
        else:
            error = [
                self.pose_error.linear.x, self.pose_error.linear.y,
                self.pose_error.linear.z
            ]

            velocity = [
                self.twist.twist.linear.x, self.twist.twist.linear.y,
                self.twist.twist.linear.z
            ]

        # Calculate maximum time to reach the goal.
        max_time = calculate_max_time(
            error, velocity, self.angular_synchronization, self.near_zero
        )

        # Calculate the velocities to reach the goal at the same time.
        sync_velocities = calculate_sync_velocity(
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

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.twist = None
        self.pose_error = None


def calculate_max_time(error, velocity, angular_synchronization=False, zero=0.001):
    """
    Calculates the maximum time, between all the velocities, required to reach
    the goal. By default, it only synchronizes linear velocities.
    If angular_synchronization is True, then it also synchronizes for angular
    velocities.

    :param error: The distance error tuple in the X, Y and Z axis.
    :type error: list

    :param velocity: Velocities in the X, Y and Z axis.
    :type velocity: list

    :param zero: Value to prevent division by near-zero values.
    :type zero: float

    :return: The maximum time required to reach the goal.
    :rtype: float

    """
    if angular_synchronization:
        assert len(error) == len(velocity) == 6
    else:
        assert len(error) == len(velocity) == 3

    calculate_duration = lambda distance, speed: abs(float(distance) / speed)

    durations = [
        calculate_duration(ee, vv) if (abs(vv) >= zero) else 0.0
        for ee, vv in zip(error, velocity)
    ]

    return max(durations)


def calculate_sync_velocity(error, velocity, max_time, angular_synchronization=False):
    """
    Calculates the synchronized velocity for all velocities to reach their goal
    at the same time. By default, it only synchronizes linear velocities.
    If angular_synchronization is True, then it also synchronizes for angular
    velocities.

    :param error: The distance error tuple in the X, Y and Z axis. It should either
        be three-dimensional (i.e. only linear) or six-dimensional (i.e. linear
        and angular). Its dimension must match that of the velocity argument.
    :type error: list

    :param velocity: Velocity in the X, Y and Z axis. It should either
        be three-dimensional (i.e. only linear) or six-dimensional (i.e. linear
        and angular). Its dimension must match that of the error argument.
    :type velocity: list

    :param max_time: The maximum time required, between the velocities,
    to reach their goal.
    :type max_time: float

    :param angular_synchronization: If True, the twist is synchronized for linear
        and angular velocities. Otherwise the twist is only synchronized for linear
        velocities.
    :type angular_synchronization: bool

    :return: The synchronized velocities.
    :rtype: list

    """
    if angular_synchronization:
        assert len(error) == len(velocity) == 6
    else:
        assert len(error) == len(velocity) == 3

    # A velocity is computed to cover a distance (dist) in a given time (max_time),
    # where max_time is the same for all distances.
    synchronize_velocity = lambda dist, vel: abs(float(dist) / max_time) * cmp(vel, 0)

    return [
        synchronize_velocity(ee, vv) if (max_time and vv) else 0.0
        for ee, vv in zip(error, velocity)
    ]


def main():
    rospy.init_node('twist_synchronizer', anonymous=True)
    twist_synchronizer = TwistSynchronizer()
    twist_synchronizer.start()
