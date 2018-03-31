#!/usr/bin/env python
"""
This module contains a component that publishes arm joint configuration 
for desired Pose or list of poses.

"""
#-*- encoding: utf-8 -*-

import rospy
import actionlib
import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import brics_actuator.msg
import moveit_commander
import mcr_manipulation_utils_ros.kinematics as kinematics
import mcr_manipulation_pose_selector_ros.reachability_pose_selector_utils as pose_selector_utils


class PoseSelector(object):
    """
    Publishes a Joint Configuration based desired pose.

    """
    def __init__(self):

        # params
        self.goal_pose = None
        self.goal_pose_array = None

        # wait for MoveIt!
        move_group = rospy.get_param('~move_group', None)
        assert move_group is not None, "Move group must be specified."
        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # name of the group to compute the inverse kinematics
        self.arm = rospy.get_param('~arm', None)
        assert self.arm is not None, "Group to move (e.g. arm) must be specified."

        group = moveit_commander.MoveGroupCommander(self.arm)
        # joints to compute the inverse kinematics
        self.joint_uris = group.get_joints()

        # units of the joint position values
        self.units = 'rad'

        # linear offset for the X, Y and Z axis.
        self.linear_offset = None

        # kinematics class to compute the inverse kinematics
        self.kinematics = kinematics.Kinematics(self.arm)

        # Time allowed for the IK solver to find a solution (in seconds).
        self.ik_timeout = 0.5

        # node cycle rate (in hz)
        self.loop_rate = 10.0

    def get_reachable_pose_and_configuration(self, goal_pose_array, linear_offset):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        poses = self.group_goal_poses(goal_pose_array)
        solution = self.select_reachable_pose(poses, linear_offset)
        if solution is not None:
            pose = solution[0]
            joint_values = solution[1]
            if pose.header.stamp:
                configuration = pose_selector_utils.list_to_brics_joints(
                    joint_values, self.joint_uris, time_stamp=pose.header.stamp,
                    unit=self.units
                )
            else:
                configuration = pose_selector_utils.list_to_brics_joints(
                    joint_values, self.joint_uris, unit=self.units
                )
            return pose, configuration, solution[1]
        else:
            return None, None

    def group_goal_poses(self, goal_pose_array):
        """
        Returns a list of PoseStamped objects based on the input to the node.

        :return: A list of the goal poses.
        :rtype: list or None

        """
        poses = None
        if goal_pose_array:
            poses = pose_selector_utils.pose_array_to_list(goal_pose_array)
        if self.goal_pose:
            if poses is None:
                poses = [self.goal_pose]
            else:
                poses.append(self.goal_pose)
        return poses

    def select_reachable_pose(self, poses, offset):
        """
        Given a list of poses, it returns the first pose that returns a
        solution and the joint configuration for that solution.

        :param poses: A list of geometry_msgs.msg.PoseStamped objects.
        :type poses: list

        :param offset: A linear offset for the X, Y and Z axis.
        :type offset: list

        :return: The pose for which a solution exists and the joint configuration
            that reaches that pose (of the requested poses).
        :rtype: (geometry_msgs.msg.PoseStamped, list) or None

        """
        for ii, pose in enumerate(poses):
            rospy.logdebug("IK solver attempt number: {0}".format(ii))
            if offset:
                solution = self.kinematics.inverse_kinematics(
                    pose_selector_utils.add_linear_offset_to_pose(pose, offset),
                    timeout=self.ik_timeout
                )
            else:
                solution = self.kinematics.inverse_kinematics(
                    pose, timeout=self.ik_timeout
                )
            if solution:
                return pose, solution
        print("No solution found")
        return None
