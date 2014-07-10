#!/usr/bin/env python
"""
This module contains a component that controls
the arm, in Cartesian Space, to reduce the
difference between two poses.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg


class TwistController(object):
    """
    Controls the arm in Cartesian space, based on the Cartesian distance
    error between two poses.

    """
    def __init__(self):
        # params
        self.event = None
        self.pose_error = None

        # Proportional gain for the Cartesian linear velocity in the X axis.
        self.p_gain_x = rospy.get_param('~p_gain_x', -0.8)
        # Proportional gain for the Cartesian linear velocity in the Y axis.
        self.p_gain_y = rospy.get_param('~p_gain_y', -0.8)
        # Proportional gain for the Cartesian linear velocity in the Z axis.
        self.p_gain_z = rospy.get_param('~p_gain_z', -0.8)
        # Maximum Cartesian linear velocity allowed in the X axis (in meters).
        self.epsilon_x = rospy.get_param('~epsilon_x', 0.02)
        # Maximum Cartesian linear velocity allowed in the Y axis (in meters).
        self.epsilon_y = rospy.get_param('~epsilon_y', 0.02)
        # Maximum Cartesian linear velocity allowed in the Z axis (in meters).
        self.epsilon_z = rospy.get_param('~epsilon_z', 0.02)

        # node cycle rate (in seconds)
        self.loop_rate = rospy.get_param('~loop_rate')

        # publishers
        self.controlled_velocity = rospy.Publisher(
            '~controlled_velocity', geometry_msgs.msg.TwistStamped
        )

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
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
            rospy.sleep(self.loop_rate)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

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
        if self.pose_error:
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
            self.pose_error = None
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
            self.pose_error = None
            return 'INIT'
        else:
            cartesian_velocity = self.calculate_cartesian_velocity()
            self.controlled_velocity.publish(cartesian_velocity)

            return 'RUNNING'

    def calculate_cartesian_velocity(self):
        """
        Calculates a synchronized Cartesian velocity, based on a position error,
        for all three linear velocities.

        :return: The Cartesian velocity to reduce the position error.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        cartesian_velocity = geometry_msgs.msg.TwistStamped()
        cartesian_velocity.header.frame_id = self.pose_error.header.frame_id
        cartesian_velocity.header.stamp = rospy.Time.now()

        velocity_x = calculate_control_velocity(
            self.pose_error.linear.x, self.p_gain_x, self.epsilon_x
        )
        velocity_y = calculate_control_velocity(
            self.pose_error.linear.y, self.p_gain_y, self.epsilon_y
        )
        velocity_z = calculate_control_velocity(
            self.pose_error.linear.z, self.p_gain_z, self.epsilon_z
        )

        cartesian_velocity.twist.linear.x = velocity_x
        cartesian_velocity.twist.linear.y = velocity_y
        cartesian_velocity.twist.linear.z = velocity_z

        return cartesian_velocity


def calculate_control_velocity(error, p_gain, epsilon):
    """
    Calculates a control velocity, based on a position error.

    :param error: The distance error.
    :type error: float

    :param p_gain: Proportional gain.
    :type p_gain: float

    :param max_velocity: Maximum velocity allowed.
    :type max_velocity: float

    :param epsilon: Minimum error, i.e. the minimum distance required
    to send a zero velocity.
    :type epsilon: float

    :return: The control velocity.
    :rtype: float

    """
    control_velocity = error * p_gain

    if abs(error) < epsilon:
        control_velocity = 0

    return control_velocity


def main():
    rospy.init_node('twist_controller', anonymous=True)
    twist_controller = TwistController()
    twist_controller.start()
