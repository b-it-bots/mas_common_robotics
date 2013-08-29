def transform_wrench(transform, wrench):
    '''
    Apply a transform to a wrench. It is assumed that the reference point and
    reference frame are collapsed into a single coordinate frame. (See also
    
    :param transform: The desired transform that should be applied.
    [numpy.matrix[4][4]]
    
    :param wrench_in: The wrench to which the transform should be applied.
    [geometry_msgs.msg.WrenchStamped]
    
    :return: The transformed wrench.
    [geometry_msgs.msg.WrenchStamped]
    '''
    
    return None