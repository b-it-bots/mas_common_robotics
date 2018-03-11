#!/usr/bin/env python
"""
This module contains a component that saves...TODO

"""
#-*- encoding: utf-8 -*-
__author__ = 'Djordje Vukcevic'

import rospy
import roslib
import std_msgs.msg
import geometry_msgs.msg
import mcr_saved_trajectories.saver as saver

class TrajectorySaver(object):
    """
    Saves trajectory data, computed by the planner used in MoveIt!

    """

    def __init__(self):
        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():
            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()
            saver.save_trajectory()
            state = "Running"

def main():
    rospy.init_node('trajectory_saver', anonymous=True)
    trajectory_saver = TrajectorySaver()
    trajectory_saver.start()
