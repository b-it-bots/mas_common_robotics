#!/usr/bin/env python
"""
This module contains a component that computes pose from given pose error.

"""
#-*- encoding: utf-8 -*-
__author__ = 'padmaja'

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import mcr_manipulation_msgs.msg


class PoseErrorToPoseConverter(object):

    def __init__(self):
        # params
        self.monitor_event = None
        # node cycle rate (in seconds)
        self.loop_rate = rospy.get_param('~loop_rate')
        self.offset_x = rospy.get_param('~offset_x', 0.0)
        self.offset_y = rospy.get_param('~offset_y', 0.0)
        self.frame_id = rospy.get_param('~frame_id', '/base_link')

        self.pose_error_linear = None
        self.r = rospy.Rate(self.loop_rate) 

        # publishers
        self.pose_pub = rospy.Publisher('~pose', PoseStamped)
        self.event_out_pub = rospy.Publisher('~event_out', String)
        
        # subscribers
        rospy.Subscriber('~event_in', String, self.event_in_cb)
        rospy.Subscriber('~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference, self.pose_error_cb)

    def start(self):
        """
        Starts pose error to pose calculator.

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
            self.r.sleep()

    def event_in_cb(self, msg):
        """
        Obtains an event for the pose computation.

        """
        self.monitor_event = msg.data

    def pose_error_cb(self, msg):
        """
        Stores pose error.

        """
        self.pose_error_linear = msg.linear

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.pose_error_linear:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_start':
            return 'RUNNING'
        elif self.monitor_event == 'e_stop':
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            return 'INIT'
        else:
            pose_from_pose_error = self.pose_error_to_pose_converter()

            self.pose_pub.publish(pose_from_pose_error)
            event_out_msg = String()
            event_out_msg.data = 'e_done'
            self.event_out_pub.publish(event_out_msg)

            return 'RUNNING'

    def pose_error_to_pose_converter(self):
        """
        Computes pose from pose error and offset.

        :return: pose from pose error.
        :rtype: geometry_msgs.msg.PoseStamped

        """

        pose_x = self.pose_error_linear.x + self.offset_x
        pose_y = self.pose_error_linear.y + self.offset_y

        pose_from_pose_error = PoseStamped()
        pose_from_pose_error.header.stamp = rospy.Time.now()
        pose_from_pose_error.header.frame_id = self.frame_id

        pose_from_pose_error.pose.position.x = pose_x
        pose_from_pose_error.pose.position.y = pose_y

        return pose_from_pose_error

def main():
    rospy.init_node('pose_error_to_pose_converter', anonymous=True)
    pose_error_to_pose_converter = PoseErrorToPoseConverter()
    pose_error_to_pose_converter.start()
