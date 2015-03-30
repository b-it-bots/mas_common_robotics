/*
 * twist_to_motion_direction_conversion_node.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: fred
 */

#include <mcr_common_converters/twist_to_motion_direction_conversion_node.h>

TwistToMotionDirectionConversionNode::TwistToMotionDirectionConversionNode()
{
    ros::NodeHandle nh("~");

    sub_twist_ = nh.subscribe("input/twist", 1, &TwistToMotionDirectionConversionNode::twistCallback, this);
    pub_pose_ = nh.advertise < geometry_msgs::PoseStamped > ("output/pose", 1);

    nh.param < std::string > ("frame_id", frame_id_, "/base_link");
}

TwistToMotionDirectionConversionNode::~TwistToMotionDirectionConversionNode()
{
    sub_twist_.shutdown();
    pub_pose_.shutdown();
}

void TwistToMotionDirectionConversionNode::twistCallback(const geometry_msgs::TwistPtr &msg)
{
    if (pub_pose_.getNumSubscribers() <= 0)
        return;

    double motion_direction = 0.0;
    geometry_msgs::PoseStamped pose;

    motion_direction = getMotionDirectionFromTwist2D(msg->linear.x, msg->linear.y, msg->angular.z);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;

    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, motion_direction);

    pub_pose_.publish(pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_to_motion_direction_conversion");
    ros::NodeHandle nh("~");

    TwistToMotionDirectionConversionNode conversion_node = TwistToMotionDirectionConversionNode();

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
