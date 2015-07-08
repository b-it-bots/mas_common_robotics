#include <mcr_twist_demultiplexer/twist_demultiplexer_node.h>

TwistDemultiplexerNode::TwistDemultiplexerNode(ros::NodeHandle &nh) : node_handler_(nh)
{
    node_handler_.param<std::string>("base_tf", base_tf_,"/tower_cam3d_rgb_optical_frame");
    node_handler_.param<std::string>("arm_tf", arm_tf_,"/base_link");
    node_handler_.param<bool>("is_base_linear_x_enabled", is_base_linear_x_enabled_,"false");
    node_handler_.param<bool>("is_base_linear_y_enabled", is_base_linear_y_enabled_,"false");
    node_handler_.param<bool>("is_base_linear_z_enabled", is_base_linear_z_enabled_,"false");
    node_handler_.param<bool>("is_base_angular_x_enabled", is_base_angular_x_enabled_,"false");
    node_handler_.param<bool>("is_base_angular_y_enabled", is_base_angular_y_enabled_,"false");
    node_handler_.param<bool>("is_base_angular_z_enabled", is_base_angular_z_enabled_,"false");
    node_handler_.param<bool>("is_arm_linear_x_enabled", is_arm_linear_x_enabled_,"false");
    node_handler_.param<bool>("is_arm_linear_y_enabled", is_arm_linear_y_enabled_,"false");
    node_handler_.param<bool>("is_arm_linear_z_enabled", is_arm_linear_z_enabled_,"false");
    node_handler_.param<bool>("is_arm_angular_x_enabled", is_arm_angular_x_enabled_,"false");
    node_handler_.param<bool>("is_arm_angular_y_enabled", is_arm_angular_y_enabled_,"false");
    node_handler_.param<bool>("is_arm_angular_z_enabled", is_arm_angular_z_enabled_,"false");
    node_handler_.param<bool>("is_error_monitor_enabled", is_error_monitor_enabled_,"false");

    arm_twist.header.frame_id = arm_tf_;
    base_twist.header.frame_id = base_tf_;

    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    arm_twist_stamped_pub_ = node_handler_.advertise<geometry_msgs::TwistStamped>("arm_twist", 1);
    base_twist_pub_ = node_handler_.advertise<geometry_msgs::Twist>("base_twist", 1);

    event_sub_ = node_handler_.subscribe("event_in", 1, &TwistDemultiplexerNode::eventCallback, this);
    twist_sub_ = node_handler_.subscribe("input_twist", 1, &TwistDemultiplexerNode::twistStampedCallback, this);
    error_feedback_sub_ = node_handler_.subscribe("error_feedback", 1, &TwistDemultiplexerNode::errorFeedbackCallback, this)

    current_state_ = INIT;
    has_twist_data_ = false;
}

TwistDemultiplexerNode::~TwistDemultiplexerNode()
{
    event_sub_.shutdown();
    twist_sub_.shutdown();
    arm_twist_stamped_pub_.shutdown();
    base_twist_pub_.shutdown();
    event_pub_.shutdown();
}

void TwistDemultiplexerNode::eventCallback(const std_msgs::String &event_command)
{
    event_in_msg_ = event_command;
}

void TwistDemultiplexerNode::twistStampedCallback(const geometry_msgs::TwistStamped &msg)
{
    input_twist_ = msg;
    has_twist_data_ = true;
}

void TwistDemultiplexerNode::errorFeedbackCallback(const mcr_monitoring_msgs::ComponentWiseErrorMonitorFeedback &error_feedback)
{
    error_feedback_msg_ = error_feedback;
    has_error_feedback_data_ = true;
}

void TwistDemultiplexerNode::states()
{
    switch (current_state_) {
        case INIT:
            initState();
            break;
        case IDLE:
            idleState();
            break;
        case RUNNING:
            runState();
            break;
        default:
            initState();
    }
}

void TwistDemultiplexerNode::initState()
{
    if(event_in_msg_.data == "e_start"){
        current_state_ = IDLE;
        event_in_msg_.data = "";
        has_twist_data_ = false;
    } else {
        current_state_ = INIT;
    }
}

void TwistDemultiplexerNode::idleState()
{
    if (event_in_msg_.data == "e_stop") {
        current_state_ = INIT;
        event_in_msg_.data = "";
    } else if (is_error_monitor_enabled_) {
        if (has_twist_data_ && has_error_feedback_data_) {
            current_state_ = RUNNING;
            has_twist_data_ = false;
            has_error_feedback_data_ = false;
        } else {
            current_state_ = IDLE;
        }
    } else {
        if (has_twist_data_) {
        current_state_ = RUNNING;
        has_twist_data_ = false;
        } else {
            current_state_ = IDLE;
        }
    }     
}

void TwistDemultiplexerNode::runState()
{
    if (is_error_monitor_enabled_){
        demultiplexTwistWithErrorFeedback();
    } else {
        demultiplexTwistWithoutErrorFeedback();
    }
    
    arm_twist_stamped_pub_.publish(arm_twist);
    base_twist_pub_.publish(base_twist.twist);
    current_state_ = IDLE;
}

void TwistDemultiplexerNode::demultiplexTwistWithErrorFeedback()
{
    arm_twist_(input_twist_);
    base_twist_(input_twist_);

    if (!is_base_linear_x_enabled || error_feedback_msg_.is_linear_x_within_tolerance) {
        base_twist_.twist.linear.x = 0.0;
    }

    if (!is_base_linear_y_enabled || error_feedback_msg_.is_linear_y_within_tolerance) {
        base_twist_.twist.linear.y = 0.0;
    }

    if (!is_base_linear_z_enabled || error_feedback_msg_.is_linear_z_within_tolerance) {
        base_twist_.twist.linear.z = 0.0;
    }

    if (!is_base_angular_x_enabled || error_feedback_msg_.is_angular_x_within_tolerance) {
        base_twist_.twist.angular.x = 0.0;
    }

    if (!is_base_angular_y_enabled || error_feedback_msg_.is_angular_y_within_tolerance) {
        base_twist_.twist.angular.y = 0.0;
    }

    if (!is_base_angular_z_enabled || error_feedback_msg_.is_angular_z_within_tolerance) {
        base_twist_.twist.angular.z = 0.0;
    }
    
    if (!is_arm_linear_x_enabled || error_feedback_msg_.is_linear_x_within_tolerance) {
        arm_twist_.twist.linear.x = 0.0;
    }

    if (!is_arm_linear_y_enabled || error_feedback_msg_.is_linear_y_within_tolerance) {
        arm_twist_.twist.linear.y = 0.0;
    }

    if (!is_arm_linear_z_enabled || error_feedback_msg_.is_linear_z_within_tolerance) {
        arm_twist_.twist.linear.z = 0.0;
    }

    if (!is_arm_angular_x_enabled || error_feedback_msg_.is_angular_x_within_tolerance) {
        arm_twist_.twist.angular.x = 0.0;
    }

    if (!is_arm_angular_y_enabled || error_feedback_msg_.is_angular_y_within_tolerance) {
        arm_twist_.twist.angular.y = 0.0;
    }

    if (!is_arm_angular_z_enabled || error_feedback_msg_.is_angular_z_within_tolerance) {
        arm_twist_.twist.angular.z = 0.0;
    }
}

void TwistDemultiplexerNode::demultiplexTwistWithoutErrorFeedback()
{
    arm_twist_(input_twist_);
    base_twist_(input_twist_);

    if (!is_base_linear_x_enabled) {
        base_twist_.twist.linear.x = 0.0;
    }

    if (!is_base_linear_y_enabled) {
        base_twist_.twist.linear.y = 0.0;
    }

    if (!is_base_linear_z_enabled) {
        base_twist_.twist.linear.z = 0.0;
    }

    if (!is_base_angular_x_enabled) {
        base_twist_.twist.angular.x = 0.0;
    }

    if (!is_base_angular_y_enabled) {
        base_twist_.twist.angular.y = 0.0;
    }

    if (!is_base_angular_z_enabled) {
        base_twist_.twist.angular.z = 0.0;
    }
    
    if (!is_arm_linear_x_enabled) {
        arm_twist_.twist.linear.x = 0.0;
    }

    if (!is_arm_linear_y_enabled) {
        arm_twist_.twist.linear.y = 0.0;
    }

    if (!is_arm_linear_z_enabled) {
        arm_twist_.twist.linear.z = 0.0;
    }

    if (!is_arm_angular_x_enabled) {
        arm_twist_.twist.angular.x = 0.0;
    }

    if (!is_arm_angular_y_enabled) {
        arm_twist_.twist.angular.y = 0.0;
    }

    if (!is_arm_angular_z_enabled) {
        arm_twist_.twist.angular.z = 0.0;
    }
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_demultiplexer");
    ros::NodeHandle nh("~");
    ROS_INFO("[twist demultiplexer] node started");
    TwistDemultiplexerNode twist_demultiplexer_node(nh);

    int loop_rate = 30;
    nh.param<int>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);

    while (ros::ok()) {
        ros::spinOnce();
        twist_demultiplexer_node.states();
        rate.sleep();
    }
    return 0;
}
