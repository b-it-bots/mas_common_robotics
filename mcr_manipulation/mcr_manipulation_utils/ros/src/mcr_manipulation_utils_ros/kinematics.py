#!/usr/bin/env python
"""
Kinematics module.

"""
#-*- encoding: utf-8 -*-

import actionlib
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
from srdfdom.srdf import SRDF
from std_msgs.msg import Header

import extractors


class Kinematics:
    def __init__(self, group_name, move_group='move_group'):
        # wait for MoveIt! to come up
        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        self.group_name = group_name
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.commander = moveit_commander.RobotCommander()
        self.state = moveit_commander.RobotState()
        self.joint_names = self.commander.get_joint_names(self.group_name)
        self.link_names = self.commander.get_link_names(self.group_name)

        # service clients
        rospy.loginfo("Waiting for 'compute_ik' service")
        rospy.wait_for_service('/compute_ik')
        self.ik_client = rospy.ServiceProxy('/compute_ik',
                                            moveit_msgs.srv.GetPositionIK)
        rospy.loginfo("Found service 'compute_ik'")

        rospy.loginfo("Waiting for 'compute_fk' service")
        rospy.wait_for_service('/compute_fk')
        self.fk_client = rospy.ServiceProxy('/compute_fk',
                                            moveit_msgs.srv.GetPositionFK)
        rospy.loginfo("Found service 'compute_fk'")

    def inverse_kinematics(self, goal_pose, configuration=None, timeout=0.5, attempts=1):
        """
        Calls the IK solver to calculate the joint configuration to reach the
        goal pose. The configuration parameter is the start position of the
        joints. If no configuration is provided, the current joint values are
        used as the initial joint configuration.

        :param goal_pose: The pose for which the inverse kinematics is solved
        :type goal_pose: geometry_msgs.msg.PoseStamped

        :param configuration: The initial manipulator's configuration
        :type configuration: Float[]

        :param timeout: Time allowed for the IK solver to find a solution.
        :type timeout: Float

        :param attempts: Number of  attempts the IK solver tries to find a solution.
        :type attempts: Int

        :return: The joint configuration that reaches the requested pose
        :rtype: Float[] or None

        """
        if not configuration:
            configuration = self.group.get_current_joint_values()

        if len(self.joint_names) != len(configuration):
            return None

        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.timeout = rospy.Duration(timeout)
        req.ik_request.attempts = attempts
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = configuration
        req.ik_request.pose_stamped = goal_pose
        try:
            resp = self.ik_client(req)
        except rospy.ServiceException, e:
            rospy.logerr('Service did not process request: %s', str(e))
            return None

        if resp.error_code.val == resp.error_code.SUCCESS:
            return extractors.extract_positions(
                resp.solution.joint_state, self.joint_names
            )
        else:
            return None

    def forward_kinematics(
            self, reference_header, configuration=None,
            joint_names=None, link_names=None
    ):
        """
        Calls the FK solver to calculate the poses of a list of links, given a
        joint configuration. If no configuration is provided, the current joint
        values are used as the joint configuration.

        :param reference_header: The forward kinematics are computed with respect
                            to this header (i.e. frame and time stamp).
        :type reference_header: str

        :param configuration: The robot's configuration. If none is provided,
                         the current joint values are used.
        :type configuration: Float[]

        :param joint_names: A list of joint names for the specified configuration.
                        If none is provided, the group's joints are used.
        :type joint_names: str[]

        :param link_names: A list of link names for which the forward kinematics must
                        be computed. If none is provided, the group's links are used.
        :type link_names: str[]

        :return: The poses of the requested links.
        :rtype: geometry_msgs.msg.PoseStamped[] or None

        """
        if configuration is None:
            configuration = self.group.get_current_joint_values()

        if joint_names is None:
            joint_names = self.joint_names

        if link_names is None:
            link_names = self.link_names

        req = moveit_msgs.srv.GetPositionFKRequest()
        req.header = reference_header
        req.fk_link_names = link_names

        req.robot_state.joint_state.header = reference_header
        req.robot_state.joint_state.name = joint_names
        req.robot_state.joint_state.position = configuration

        try:
            resp = self.fk_client(req)
            if resp.error_code.val == resp.error_code.SUCCESS:
                return resp.pose_stamped
            else:
                return None

        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: {0}".format(e))
            return None


def get_group_state_values(group_state_name):
    """ Function to parse the SRDF to obtain the
    joint configuration and names for a given group state

    Parameters
    ----------
    group_state_name : str
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


def get_link_pose(target_link_names,
                  group_name,
                  joint_values=None,
                  joint_names=None,
                  target_frame="/base_link"):
    """ Function to calculate the pose of the
    give link(s) with respect to given target_frame
    frame

    Parameters
    ----------
    target_link_names : list
        list of valid link/frame names (obtained from tf) whose
        transformation to another frame is required
    group_name : str
        valid group name from SRDF
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

    kinematics = Kinematics(group_name)

    link_poses = kinematics.forward_kinematics(
        header,
        configuration=joint_values,
        joint_names=joint_names,
        link_names=target_link_names
    )

    return link_poses
