#ifndef TWIST_DEMULTIPLEXER_NODE_H_
#define TWIST_DEMULTIPLEXER_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>


class TwistDemultiplexerNode
{

    public:
        TwistDemultiplexerNode(ros::NodeHandle &nh);
        virtual ~TwistDemultiplexerNode();
        void eventCallback(const std_msgs::String &event_command);
        void twistStampedCallback(const geometry_msgs::TwistStamped &msg);
        void states();
        void initState();
        void idleState();
        void runState();

    private:
        enum States {
            INIT,
            IDLE,
            RUNNING
        };


    private:
        void demultiplexTwist();


    private:
        ros::NodeHandle node_handler_;

        ros::Subscriber event_sub_;
        ros::Subscriber twist_sub_;

        ros::Publisher event_pub_;
        ros::Publisher arm_twist_stamped_pub_;
        ros::Publisher base_twist_pub_;

        geometry_msgs::TwistStamped input_twist_;

        bool start_twist_demultiplexer_;
        bool twist_sub_status_;

        std::string base_tf_;
        std::string arm_tf_;

        States run_state_;

};

#endif
