#ifndef COMPONENTWISEPOSEERRORMONITORNODE_H_
#define COMPONENTWISEPOSEERRORMONITORNODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <mcr_manipulation_msgs/ComponentWiseCartesianDifference.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <mcr_geometric_relation_monitors/ComponentWisePoseErrorMonitorConfig.h>
#include <math.h>


class ComponentWisePoseErrorMonitorNode
{

    public:
        ComponentWisePoseErrorMonitorNode(ros::NodeHandle &nh);
        virtual ~ComponentWisePoseErrorMonitorNode();
        void dynamicReconfigCallback(mcr_geometric_relation_monitors::ComponentWisePoseErrorMonitorConfig &config, uint32_t level);
        void eventCallback(const std_msgs::String &event_command);
        void errorCallback(const mcr_manipulation_msgs::ComponentWiseCartesianDifference::ConstPtr &pose_error);
        void states();
        void initState();
        void idleState();
        void runState();
        bool isComponentWisePoseErrorWithinThreshold();

    private:
        enum States {
            INIT,
            IDLE,
            RUNNING
        };

    private:
        ros::NodeHandle node_handler_;
        dynamic_reconfigure::Server<mcr_geometric_relation_monitors::ComponentWisePoseErrorMonitorConfig> dynamic_reconfig_server_;
        ros::Subscriber event_sub_;
        ros::Subscriber error_sub_;
        ros::Publisher event_pub_;
        bool pose_error_sub_status_;
        bool start_pose_error_monitor_;
        double threshold_linear_x_;
        double threshold_linear_y_;
        double threshold_linear_z_;
        double threshold_angular_x_;
        double threshold_angular_y_;
        double threshold_angular_z_;
        mcr_manipulation_msgs::ComponentWiseCartesianDifference error_;
        std_msgs::String status_msg_;
        States run_state_;

};

#endif /* COMPONENTWISEPOSEERRORMONITORNODE_H_ */
