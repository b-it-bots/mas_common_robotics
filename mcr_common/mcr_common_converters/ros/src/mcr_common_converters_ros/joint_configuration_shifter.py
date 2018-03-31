#!/usr/bin/env python
"""
This component computes a shifted joint configuration, by a specified offset, from a input joint configuration.

"""
#-*- encoding: utf-8 -*-

import copy
import itertools
import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf

class JointConfigurationShifter(object):
    """
    Computes a shifted joint configuration, by a specified offset, from a input joint configuration.

    """
    def __init__(self, joint_limits):
        # TODO: implement joint limit checking
        self.joint_limits = joint_limits

    def shift_joint_configuration(self, joints_in, offset):
        """
        Computes the shifted joint configuration from 'joints_in' given the specified offset.

        :param joints_in: The joint configuration to be used as reference for the shifted joint configuration.
        :type joints_in: List

        :return: A shifted joint configuration.
        :rtype: List

        """
        joints_out = []
        for joint, joint_offset in zip(joints_in, offset):
            joint += joint_offset
            joints_out.append(joint)

        return joints_out
