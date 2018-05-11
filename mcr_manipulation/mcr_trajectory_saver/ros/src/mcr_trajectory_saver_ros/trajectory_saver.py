#!/usr/bin/env python
"""
This module contains a component that saves...TODO

"""
#-*- encoding: utf-8 -*-
__authors__ = 'Djordje Vukcevic, Abhishek Padalkar'

import rospy
import roslib
import actionlib
import re
import std_msgs.msg
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import moveit_commander
import mcr_trajectory_saver.saver as saver

class TrajectorySaver(object):
    """
    Saves trajectory data, computed by the planner used in MoveIt!

    """

    def __init__(self):

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        #Start and end state of the robot
        self.start_state = rospy.get_param('~start_state', None)
        self.goal_state = rospy.get_param('~goal_state', None)

        # MoveIt! interface
        arm = rospy.get_param('~arm', None)
        assert arm is not None, "The group to be moved must be specified (e.g. arm)."

        # Set up MoveIt!
        self.move_group = moveit_commander.MoveGroupCommander(arm)

        self.file_path = rospy.get_param('~file_path')


    def start(self):
        """
        Starts the component.

        """

        self.move_group.clear_pose_targets()

        '''
        Setting start state in move_group with only name(string) is not possible.
        Thus the workaround is:
            1) Set target configuration with the name of robot start state.
            2) Extract/Get joint values of this state.
            3) Use these joint values to set start state in moveit group.
            4) Set target configuration with name of true goal state.
        '''

        #1) Set target configuration with the name of robot start state.
        try:
            self.move_group.set_named_target(self.start_state)
        except Exception as e:
            rospy.logerr('unable to set start configuration: %s' % (str(e)))

        #2) Extract/Get joint values of this state.
        robot_state = self.move_group.get_joint_value_target()

        #Robot configuration needs to be translated in moveit_msgs/RobotState
        joint_state = JointState()
        joint_state.header = std_msgs.msg.Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.move_group.get_joints()
        joint_state.position = robot_state

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state

        #3) Use these joint values to set start state in moveit group.
        self.move_group.set_start_state(moveit_robot_state)

        self.move_group.clear_pose_targets()

        #4) Set target configuration with name of true goal state.
        try:
            self.move_group.set_named_target(self.goal_state)
        except Exception as e:
            rospy.logerr('unable to set goal configuration: %s' % (str(e)))

        #Save trajectory in yaml file
        result = saver.save_trajectory(self.move_group.plan(),
                                       self.start_state,
                                       self.goal_state,
                                       self.file_path)
        if result:
            rospy.loginfo("Trajectory saved!")

def main():
    rospy.init_node('trajectory_saver', anonymous=True)
    trajectory_saver = TrajectorySaver()
    trajectory_saver.start()