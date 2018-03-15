#!/usr/bin/env python
"""
This component moves a robotic manipulator, in a planned manner, to a specified
joint configuration using MoveIt!.

**Input(s):**
  * `target_configuration`: The joint configuration to which the manipulator will
  be moved.

**Parameter(s):**
  * `move_group`: MoveIt! interface.
  * `arm`: Name of the group to move.
  * `loop_rate`: Node cycle rate (in hz).

"""
#-*- encoding: utf-8 -*-

import rospy
import actionlib
import moveit_commander
import sensor_msgs.msg
import std_msgs.msg
import moveit_msgs.msg
import brics_actuator.msg
import mcr_arm_motions_ros.planned_motion_utils as utils
import mcr_manipulation_monitors_ros.joint_position_monitors_node
import mcr_topic_tools_ros.transformer_utils as topic_utils


class PlannedMotion(object):
    """
    Components that move the arm in a planned motion.

    """
    def __init__(self):
        # Params
        self.event = None
        self.target_configuration = None
        self.current_joint_position = None
        self.moving = False

        # MoveIt! interface
        move_group = rospy.get_param('~move_group', None)
        assert move_group is not None, "Move group must be specified."

        # Wait for MoveIt!
        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # Name of the group to move
        arm = rospy.get_param('~arm', None)
        assert arm is not None, "The group to be moved must be specified (e.g. arm)."

        # Set up MoveIt!
        self.arm = moveit_commander.MoveGroupCommander(arm)

        # Whether MoveIt! should wait for the arm to be stopped.
        self.wait_for_motion = rospy.get_param('~wait_for_motion', True)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        self.joint_position_monitor = \
                mcr_manipulation_monitors_ros.joint_position_monitors_node.JointPositionMonitor()

        epsilon = rospy.get_param('~epsilon')
        target_joint_names = rospy.get_param('~target_joint_names')
        self.joint_position_monitor.set_epsilon(epsilon)
        self.joint_position_monitor.set_target_joint_names(target_joint_names)

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~target_configuration", brics_actuator.msg.JointPositions,
            self.target_configuration_cb
        )
        rospy.Subscriber("~joint_states", sensor_msgs.msg.JointState,
                         self.current_joint_positions_cb)

    def target_configuration_cb(self, msg):
        """
        Obtains the joint configuration where the arm will be moved.

        """
        self.target_configuration = msg
        configuration_out = topic_utils.brics_joints_to_joint_state(
            self.target_configuration
        )
        self.joint_position_monitor.set_desired_joint_positions(configuration_out)

    def current_joint_positions_cb(self, msg):
        """
        Obtains the current joint positions values of the desired joints.

        """
        self.current_joint_position = msg

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
        elif self.target_configuration:
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
            self.arm.stop()
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            if not self.moving:
                self.moving = True
                move_status = self.move_arm(self.target_configuration, self.wait_for_motion)

            if self.joint_position_monitor.joint_positions_reached(self.current_joint_position):
                self.moving = False
                self.event_out.publish('e_success')
                self.reset_component_data()
                return 'INIT'
            else:
                self.event_out.publish('e_failure')
                    
            return 'RUNNING'

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
        joint_list = utils.brics_joint_positions_to_list(joint_configuration)
        self.arm.set_joint_value_target(joint_list)
        status = self.arm.go(wait=wait)

        return status

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.target_configuration = None
        self.moving = False


def main():
    rospy.init_node("planned_motion", anonymous=True)
    planned_motion = PlannedMotion()
    planned_motion.start()
