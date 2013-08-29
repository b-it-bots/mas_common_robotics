#!/usr/bin/env python

import geometry_msgs.msg


class GeometryTransformer:

    def __init__(self):
        self.listener = tf.TransformListener()


    def transformWrench(self, wrench_in, target_frame = "base_link"):
        '''
        Transform the provided wrench to the target frame. Throws an exception
        if the transformation failed.
        
        :param wrench_in: The wrench which should be transformed.
        :type wrench_in: geometry_msgs.msg.WrenchStamped]
        
        :param target_frame: The frame into which the wrench should be
        transformed.
        :type target_frame: String
        
        :return: The transformed wrench.
        :rtype: geometry_msgs.msg.WrenchStamped
        '''
        return geometry_msgs.msg.WrenchStamped()
