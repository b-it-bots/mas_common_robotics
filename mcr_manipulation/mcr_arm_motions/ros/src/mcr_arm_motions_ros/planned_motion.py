#!/usr/bin/env python
"""
This component moves a robotic arm, in a planned manner,
using MoveIt!

"""
#-*- encoding: utf-8 -*-

import rospy
import moveit_commander
import std_msgs.msg
import brics_actuator.msg


class PlannedMotion(object):
    """
    Components that move the arm in a planned motion.

    """
    def __init__(self):
        # params
        self.event = None
        self.target_configuration = None

        # the name of the group to move
        arm = rospy.get_param('~arm', None)
        assert arm is not None

        # set up MoveIt!
        self.arm = moveit_commander.MoveGroupCommander(arm)

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~target_configuration", brics_actuator.msg.JointPositions,
            self.target_configuration_cb
        )

    def target_configuration_cb(self, msg):
        """
        Obtains the joint configuration where the arm will be moved.

        """
        self.target_configuration = msg

    def event_in_cb(self, msg):
        """
        Starts a planned motion based on the specified arm position.

        """
        self.event = msg.data

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

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.target_configuration:
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
            self.target_configuration = None
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
            self.arm.stop()
            self.target_configuration = None
            return 'INIT'
        else:
            move_status = self.move_arm(self.target_configuration)

            if move_status:
                self.event_out.publish('e_success')

            self.target_configuration = None
            return 'INIT'

    def move_arm(self, joint_configuration, wait=True):
        """
        Moves the arm to a specified joint configuration.

        :param joint_configuration: The target joint configuration.
        :type joint_configuration: brics_actuator.msg.JointPositions

        :param wait: Wait for the execution of the trajectory to complete,
        :type wait: bool

        :return: False if the target configuration cannot be achieved.
        :rtype: bool

        """
        joint_list = brics_joint_positions_to_list(joint_configuration)
        self.arm.set_joint_value_target(joint_list)
        status = self.arm.go(wait=wait)

        return status


def brics_joint_positions_to_list(joint_configuration):
    """
    Converts a BRICS message of joint positions into a list of real values.

    :param joint_configuration: Joint configuration as a BRICS message.
    :type joint_configuration: brics_actuator.msg.JointPositions

    :return: A list of real values representing the joints as specified
             by the BRICS message.
    :rtype: list

    """
    return [joint.value for joint in joint_configuration.positions]


def main():
    rospy.init_node("planned_motion", anonymous=True)
    planned_motion = PlannedMotion()
    planned_motion.start()
