#!/usr/bin/env python

"""
This module contains function to save trajectory
that is computed by the MoveIt! planner.

"""

#-*- encoding: utf-8 -*-
__authors__ = 'Djordje Vukcevic, Abhishek Padalkar'

import rospy
import os
import rospkg
import yaml

def save_trajectory(trajectory, start_state, goal_state, file_path):
    """
    MoveIt! plan will be saved in mcr_arm_motions package, under ros/config folder.
    From aforementioned package, saved trajectory should extracted and extecuted.
    Plan will be saved in yaml file.

    """

    file_name = start_state + "-" + goal_state + "_" + 'plan.yaml'

    if not os.path.exists(file_path):
        os.makedirs(file_path)

    with open(os.path.join(file_path, file_name), "w") as save_file:
        yaml.dump(trajectory, save_file, default_flow_style=True)

    return True
