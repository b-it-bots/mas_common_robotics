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
    pub_point_ = nh.advertise < geometry_msgs::PointStamped > ("output/point", 1);

    nh.param < std::string > ("frame_id", frame_id_, "/base_link");
    nh.param<double>("distance_to_frame", distance_to_frame_, 1.5);
}

TwistToMotionDirectionConversionNode::~TwistToMotionDirectionConversionNode()
{
    sub_twist_.shutdown();
    pub_pose_.shutdown();
    pub_point_.shutdown();
}

void TwistToMotionDirectionConversionNode::twistCallback(const geometry_msgs::TwistPtr &msg)
{
    if (pub_pose_.getNumSubscribers() <= 0)
        return;

    double motion_direction = 0.0;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PointStamped point;

    motion_direction = getMotionDirectionFromTwist2D(msg->linear.x, msg->linear.y, msg->angular.z);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;

    point.header = pose.header;

    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, motion_direction);

    point.point.x = distance_to_frame_ * cos(motion_direction);
    point.point.y = distance_to_frame_ * sin(motion_direction);
    point.point.z = 0.0;

    pub_pose_.publish(pose);
    pub_point_.publish(point);
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
