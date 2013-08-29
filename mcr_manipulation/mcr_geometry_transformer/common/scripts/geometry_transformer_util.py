def transform_wrench(transform, wrench):
    '''
    Apply a transform to a wrench. It is assumed that the reference point and
    reference frame are collapsed into a single coordinate frame. (See also
    http://www.ros.org/wiki/tf/Reviews/2010-03-12_API_Review)
    
    :param transform: The desired transform that should be applied.
    :type transform: numpy.matrix[4][4]
    
    :param wrench_in: The wrench to which the transform should be applied.
    :type wrench_in: geometry_msgs.msg.WrenchStamped]
    
    :return: The transformed wrench.
    :rtype: geometry_msgs.msg.WrenchStamped
    '''
    
    return None