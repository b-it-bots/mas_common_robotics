#!/usr/bin/env python

import geometry_msgs.msg


class GeometryTransformer:

    def __init__(self):
        self.listener = tf.TransformListener()


    '''
    Transform the provided wrench to the target frame. Throws an exception if
    the transformation failed.
    
    :param wrench_in: geometry_msgs.msg.WrenchStamped
    :param target_frame: String
    :return: geometry_msgs.msg.WrenchStamped
    '''
    def transformWrench(self, wrench_in, target_frame = "base_link"):
        return geometry_msgs.msg.WrenchStamped()
