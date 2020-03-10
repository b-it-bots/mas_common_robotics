#! /usr/bin/env python

from __future__ import print_function

import rospy
from srdfdom.srdf import SRDF
from std_msgs.msg import Header

from kinematics import Kinematics


class CalculateLinkPose():
    """ Class to calculate the pose of any link
    (for e.g.: end-effector, arm_link_1) for any
    any valid joint configuration from the SRDF with
    respect to any valid reference frame using moveit
    """

    def __init__(self, group_name="arm_1"):
        """ Class constructor to initialize moveit
        group name

        Parameters
        ----------
        group_name : string, optional
            moveit group name, by default "arm_1"
        """

        self.group_name = group_name

    def get_group_state_values(self, group_state_name):
        """ Class method to parse the SRDF to obtain the
        joint configuration and names for a given group state

        Parameters
        ----------
        group_state_name : string
            valid group state name from the SRDF for the given group

        Returns
        -------
        tuple
            Returns a tuple of two lists - one containing the joint
            configuration values and the other containing the joint names
        """

        robot = SRDF.from_parameter_server()
        joint_values = [
            joint.value[0] for joint in robot.group_state_map[group_state_name].joints
            ]
        joint_names = [
            joint.name for joint in robot.group_state_map[group_state_name].joints
            ]
        rospy.loginfo("Joint names {}".format(joint_names))
        rospy.loginfo("Joint values {}".format(joint_values))

        return joint_values, joint_names

    def get_link_pose(self,
                      target_link_names,
                      joint_values=None,
                      joint_names=None,
                      target_frame="/base_link"):
        """ Class method to calculate the pose of the
        give link(s) with respect to given target_frame
        frame

        Parameters
        ----------
        target_link_names : list
            list of valid link/frame names (obtained from tf) whose
            transformation to another frame is required
        joint_values : list
            list of joint values (obtained from srdf) for the given
            group, by default current joint configuration is taken
        joint_names : list
            list of joint names (obtained from srdf) for the given
            group, by default current joint names defined for the group
            is used
        target_frame : str, optional
            frame with reference to which pose of the given
            link(s) has to be calculated, by default "/base_link"

        Returns
        -------
        list
            List of geometry_msgs/PoseStamped messages consists of position
            and orientation of the given link(s)
        """

        rospy.loginfo(
            "Computing pose of links {} with respect \
            to {} frame".format(target_link_names, target_frame)
            )

        header = Header()
        header.frame_id = target_frame
        header.stamp = rospy.Time.now()

        kinematics = Kinematics(self.group_name)

        link_poses = kinematics.forward_kinematics(
            header,
            configuration=joint_values,
            joint_names=joint_names,
            link_names=target_link_names
            )

        return link_poses
