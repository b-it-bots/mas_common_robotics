#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


def zoom():
    rospy.init_node('mcr_test_zoom')
    pub = rospy.Publisher('/mcr_rgb_camera/zoom/position', Int32)

    rospy.sleep(1)

    positions = [10000, 500, 20000, 0]
    sleep_seconds = 5

    for pos in positions:
        rospy.loginfo("Set position: " + str(pos))
        pub.publish(Int32(pos))

        rospy.loginfo("Wait for X seconds: " + str(sleep_seconds))    
        rospy.sleep(sleep_seconds);


if __name__ == '__main__':
    try:
        zoom()
    except rospy.ROSInterruptException:
        pass
