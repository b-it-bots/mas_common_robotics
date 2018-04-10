#!/usr/bin/env python

"""
This module contains function to extract trajectory from yaml file,
that is computed by the MoveIt! planner.

"""

#-*- encoding: utf-8 -*-
__authors__ = 'Djordje Vukcevic, Abhishek Padalkar'

import rospy
import os
import rospkg
import yaml

def extract_trajectory(start_state, goal_state, file_path):

    file_name = start_state + "-" + goal_state + "_" + 'plan.yaml'

    full_path = os.path.join(file_path, file_name)

    if not os.path.exists(full_path):
        rospy.logerr('Unable to find requested file path')

    with open(full_path, 'r') as file_open:
        trajectory = yaml.load(file_open)

    rospy.loginfo("Trajectory extracted!")

    return trajectory
