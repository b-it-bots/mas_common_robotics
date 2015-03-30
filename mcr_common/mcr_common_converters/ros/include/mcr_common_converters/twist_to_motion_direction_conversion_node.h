/*
 * twist_to_motion_direction_conversion_node.h
 *
 *  Created on: Mar 28, 2015
 *      Author: fred
 */

#ifndef TWIST_TO_MOTION_DIRECTION_CONVERSION_NODE_H_
#define TWIST_TO_MOTION_DIRECTION_CONVERSION_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>

#include <mcr_common_converters/motion_direction_calculation.h>


class TwistToMotionDirectionConversionNode
{
public:
    TwistToMotionDirectionConversionNode();
    ~TwistToMotionDirectionConversionNode();

private:
    void twistCallback(const geometry_msgs::TwistPtr &msg);

    ros::Subscriber sub_twist_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_point_;

    std::string frame_id_;
    double distance_to_frame_;
};

#endif /* TWIST_TO_MOTION_DIRECTION_CONVERSION_NODE_H_ */
