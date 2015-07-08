#ifndef TWIST_DEMULTIPLEXER_NODE_H_
#define TWIST_DEMULTIPLEXER_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <mcr_monitoring_msgs/ComponentWisePoseErrorMonitorFeedback.h>

class TwistDemultiplexerNode
{

    public:
        TwistDemultiplexerNode(ros::NodeHandle &nh);
        virtual ~TwistDemultiplexerNode();
        void eventCallback(const std_msgs::String &event_command);
        void twistStampedCallback(const geometry_msgs::TwistStamped &msg);
        void errorFeedbackCallback(const mcr_monitoring_msgs::ComponentWisePoseErrorMonitorFeedback &error_feedback);
        void states();
        void initState();
        void idleState();
        void runState();
        void demultiplexTwistWithErrorFeedback();
        void demultiplexTwistWithoutErrorFeedback();

    private:
        enum States {
            INIT,
            IDLE,
            RUNNING
        };

    private:
        ros::NodeHandle node_handler_;
        ros::Subscriber event_sub_;
        ros::Subscriber twist_sub_;
        ros::Subscriber error_feedback_sub_;
        ros::Publisher event_pub_;
        ros::Publisher arm_twist_stamped_pub_;
        ros::Publisher base_twist_pub_;
        geometry_msgs::TwistStamped input_twist_;
        mcr_monitoring_msgs::ComponentWisePoseErrorMonitorFeedback error_feedback_msg_;
        bool has_twist_data_;
        bool has_error_feedback_data_;
        std::string base_tf_;
        std::string arm_tf_;
        std_msgs::String event_in_msg_;
        States current_state_;
        bool is_base_linear_x_enabled_;
        bool is_base_linear_y_enabled_;
        bool is_base_linear_z_enabled_;
        bool is_base_angular_x_enabled_;
        bool is_base_angular_y_enabled_;
        bool is_base_angular_z_enabled_;
        bool is_arm_linear_x_enabled_;
        bool is_arm_linear_y_enabled_;
        bool is_arm_linear_z_enabled_;
        bool is_arm_angular_x_enabled_;
        bool is_arm_angular_y_enabled_;
        bool is_arm_angular_z_enabled_;
        bool is_error_monitor_enabled_;
        geometry_msgs::TwistStamped arm_twist_;
        geometry_msgs::TwistStamped base_twist_;

};

#endif /* TWIST_DEMULTIPLEXER_NODE_H_ */
