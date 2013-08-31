#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


def zoom():
    rospy.init_node('brsu_test_zoom')
    pub = rospy.Publisher('/brsu_rgb_camera/zoom/velocity', Int32)

    rospy.sleep(1)

    velocities = [5, -3, 7, -7, 0]
    sleep_seconds = 2

    for velocity in velocities:
        rospy.loginfo("Set velocity: " + str(velocity))
        pub.publish(Int32(velocity))

        rospy.loginfo("Wait for X seconds: " + str(sleep_seconds))    
        rospy.sleep(sleep_seconds);

   

if __name__ == '__main__':
    try:
        zoom()
    except rospy.ROSInterruptException:
        pass
