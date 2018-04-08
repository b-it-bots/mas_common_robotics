#!/usr/bin/env python

"""
This module contains function to execute saved MoveIt! plan/trajectory.
"""

#-*- encoding: utf-8 -*-
__authors__ = 'Djordje Vukcevic, Abhishek Padalkar'

import rospy
import roslib
import std_msgs.msg
import moveit_msgs.msg
import moveit_commander
import mcr_arm_motions.trajectory_extractor as extractor

class TrajectoryExecutor(object):
    """
    Executes trajectory data, computed by the MoveIt! planner
    and saved in yaml files.
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

        #Get path where trajectory is saved
        self.file_path = rospy.get_param('~file_path')


    def start(self):
        """
        Starts the component.

        """

        trajectory = extractor.extract_trajectory(self.start_state,
                                                  self.goal_state,
                                                  self.file_path)

        execution_result = self.move_group.execute(trajectory)

        if execution_result:
            rospy.loginfo("Trajectory executed!")
        else:
            rospy.logerr('Unable to execute given trajectory')

def main():
    rospy.init_node('trajectory_executor', anonymous=True)
    trajectory_executor = TrajectoryExecutor()
    trajectory_executor.start()
