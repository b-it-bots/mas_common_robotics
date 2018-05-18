#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains components that monitor
the joint position of a manipulator.
"""

import numpy
import rospy
import std_msgs.msg
import sensor_msgs.msg

__author__ = 'jsanch'


class JointPositionMonitor(object):
    """
    Monitors the joint positions of a manipulator.

    """
    def __init__(self):
        # params
        self.desired_joint_positions = None
        self.current_joint_positions = None
        self.mapping_table = {}

        # tolerance for the joint positions
        self.epsilon = 0.08
        # target joint names
        self.target_joint_names = None

    def set_epsilon(self, epsilon):
        """
        Set tolerance for joint positions

        """
        self.epsilon = epsilon

    def set_target_joint_names(self, target_joint_names):
        self.target_joint_names = target_joint_names
        # create a mapping table of the target joint names and their indices
        self.current_joint_positions = [None for _ in self.target_joint_names]
        for index, joint in enumerate(self.target_joint_names):
            self.mapping_table[joint] = index


    def get_mapped_current_joint_positions(self, msg):
        """
        Get the current joint positions values of the desired joints based 
        on the mapping table

        """
        for desired in self.target_joint_names:
            for i, current in enumerate(msg.name):
                if current == desired:
                    index = self.mapping_table[desired]
                    self.current_joint_positions[index] = msg.position[i]
        mapped_current_joint_positions = sensor_msgs.msg.JointState()
        mapped_current_joint_positions.name = [joint for joint in self.target_joint_names]
        mapped_current_joint_positions.position = [
            self.current_joint_positions[self.mapping_table[joint]]
            for joint in self.target_joint_names
        ]
        return mapped_current_joint_positions

    def set_desired_joint_positions(self, msg):
        """
        Sets the desired joint positions.

        """
        self.desired_joint_positions = msg

    def joint_positions_reached(self, current_joint_positions_msg):
        """
        Checks if all the desired joint positions have been reached
        (within a tolerance).

        :return: True, if all the desired joint positions have reached
            their target values; otherwise False is returned.
        :rtype: bool

        """
        current_joint_positions = self.get_mapped_current_joint_positions(current_joint_positions_msg)

        sorted_joint_positions = sort_joint_values(
            current_joint_positions, self.desired_joint_positions
        )

        return check_joint_positions(
            self.current_joint_positions, sorted_joint_positions, self.epsilon
        )

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.current_joint_positions = [None for _ in self.target_joint_names]
        self.desired_joint_positions = None
        self.monitor_event = None


def sort_joint_values(actual, reference):
    """
    Sorts a set of joint values matching the order of the 'actual'
    joint values.

    :param actual: The current joint positions.
    :type actual: sensor_msgs.msg.JointState

    :param reference: The reference joint positions.
    :type reference: sensor_msgs.msg.JointState

    :return: The joint positions values sorted with the order of the 'actual'
        joint values.
    :rtype: []Float

    """
    assert len(actual.name) == len(reference.name), \
        "The length of the current and reference joint must be the same."

    return [
        reference.position[ii] for a_name in actual.name
        for ii, r_name in enumerate(reference.name) if a_name == r_name
    ]


def check_joint_positions(actual, reference, tolerance):
    """
    Detects if all joints have reached their desired
    joint position within a tolerance.

    :param actual: The current joint positions.
    :type actual: []Float

    :param reference: The reference joint positions.
    :type reference: []Float

    :param tolerance: The tolerance allowed between
                      reference and actual joint positions.
    :type tolerance: Float

    :return: The status of all joints in their desired positions.
    :rtype: Boolean

    """
    for i, desired in enumerate(reference):
        below_limit = (actual[i] >= desired + tolerance)
        above_limit = (actual[i] <= desired - tolerance)
        if below_limit or above_limit:
            return False

    return True
