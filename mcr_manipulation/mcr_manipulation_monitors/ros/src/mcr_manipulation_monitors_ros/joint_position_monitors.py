#!/usr/bin/env python
"""
This module contains components that monitor
the joint position of a manipulator.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import sensor_msgs.msg


class JointPositionMonitor(object):
    """
    Monitors the joint positions of a manipulator.

    """
    def __init__(self):
        # params
        self.monitor_event = None
        self.desired_joint_positions = None
        self.current_joint_positions = None

        # node cycle rate (in seconds)
        self.loop_rate = rospy.get_param('~loop_rate')
        # tolerance for the joint positions
        self.epsilon = rospy.get_param('~epsilon')

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~configuration", sensor_msgs.msg.JointState,
                         self.configure_joint_monitor_cb)
        rospy.Subscriber("~joints_states", sensor_msgs.msg.JointState,
                         self.read_joint_positions_cb)

    def start_joint_position_monitor(self):
        """
        Starts the joint position monitor.

        """
        rospy.loginfo("Joint position monitor ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                if self.current_joint_positions and \
                   self.desired_joint_positions:
                    state = 'IDLE'
            elif state == 'IDLE':
                if self.monitor_event == 'e_start':
                    state = 'RUNNING'
            elif state == 'RUNNING':
                if check_joint_positions(self.current_joint_positions,
                                         self.desired_joint_positions,
                                         self.epsilon):
                    self.event_out.publish('e_done')

                if self.monitor_event == 'e_stop':
                    self.current_joint_positions = None
                    self.desired_joint_positions = None
                    state = 'INIT'

            rospy.sleep(self.loop_rate)

    def event_in_cb(self, msg):
        """
        Obtains an event for the joint position monitor.

        """
        self.monitor_event = msg.data

    def read_joint_positions_cb(self, msg):
        """
        Obtains the current joint positions values of each joint.

        """
        self.current_joint_positions = msg

    def configure_joint_monitor_cb(self, msg):
        """
        Obtains the desired joint positions.

        """
        self.desired_joint_positions = msg


def check_joint_positions(actual, reference, tolerance):
    """
    Detects if all joints have reached their desired
    joint position within a tolerance.

    :param actual: The current joint positions.
    :type actual: sensor_msgs.msg.JointState

    :param reference: The reference joint positions.
    :type reference: sensor_msgs.msg.JointState

    :param tolerance: The tolerance allowed between
                      reference and actual joint positions.
    :type tolerance: Float

    :return: The status of all joints in their desired positions.
    :rtype: Boolean

    """
    for j, desired in enumerate(reference.name):
        for i, current in enumerate(actual.name):
            if desired == current:
                below_limit = (actual.position[i] >= reference.position[j] + tolerance)
                above_limit = (actual.position[i] <= reference.position[j] - tolerance)

                if below_limit or above_limit:
                    return False

    return True


def main():
    rospy.init_node("joint_position_monitors", anonymous=True)
    joint_position_monitor = JointPositionMonitor()
    joint_position_monitor.start_joint_position_monitor()
