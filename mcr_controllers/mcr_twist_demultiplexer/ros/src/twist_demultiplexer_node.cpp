#include <mcr_twist_demultiplexer/twist_demultiplexer_node.h>

TwistDemultiplexerNode::TwistDemultiplexerNode(ros::NodeHandle &nh) : node_handler_(nh)
{
    node_handler_.getParam("base_tf", base_tf_);
    node_handler_.getParam("arm_tf", arm_tf_);

    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    arm_twist_stamped_pub_ = node_handler_.advertise<geometry_msgs::TwistStamped>("arm_twist", 1);
    base_twist_pub_ = node_handler_.advertise<geometry_msgs::Twist>("base_twist", 1);

    event_sub_ = node_handler_.subscribe("event_in", 1, &TwistDemultiplexerNode::eventCallback, this);
    twist_sub_ = node_handler_.subscribe("input_twist", 1, &TwistDemultiplexerNode::twistStampedCallback, this);

    run_state_ = INIT;
    start_twist_demultiplexer_ = false;
    twist_sub_status_ = false;
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
    if (event_command.data == "e_start") {
        start_twist_demultiplexer_ = true;
        ROS_INFO("Twist Demultiplexer ENABLED");
    } else if (event_command.data == "e_stop") {
        start_twist_demultiplexer_ = false;
        ROS_INFO("Twist Demultiplexer DISABLED");
    }
}

void TwistDemultiplexerNode::twistStampedCallback(const geometry_msgs::TwistStamped &msg)
{
    input_twist_ = msg;
    twist_sub_status_ = true;
}

void TwistDemultiplexerNode::states()
{
    switch (run_state_) {
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
    if (twist_sub_status_) {
        run_state_ = IDLE;
        twist_sub_status_ = false;
    }
}

void TwistDemultiplexerNode::idleState()
{
    if (start_twist_demultiplexer_) {
        run_state_ = RUNNING;
    } else {
        run_state_ = INIT;
    }
}

void TwistDemultiplexerNode::runState()
{
    demultiplexTwist();
    run_state_ = INIT;
}

void TwistDemultiplexerNode::demultiplexTwist()
{
    // copy input twist message
    geometry_msgs::TwistStamped arm_twist(input_twist_);
    geometry_msgs::TwistStamped base_twist(input_twist_);

    // transform twists to required frames
    //geometry_msgs::TwistStamped transformed_arm_twist = geometry_transformer_.transformTwist(arm_tf_, arm_twist);
    //geometry_msgs::TwistStamped transformed_base_twist = geometry_transformer_.transformTwist(base_tf_, base_twist);

    // set linear arm motions to zero and angular x,y to zero
    // set x,y angular arm motions to zero
    arm_twist.twist.angular.x = 0.0;
    arm_twist.twist.angular.y = 0.0;
    arm_twist.twist.linear.x = 0.0;
    arm_twist.twist.linear.y = 0.0;
    arm_twist.twist.linear.z = 0.0;

    arm_twist.header.frame_id = arm_tf_;

    // set angular base motions to zero and linear z to zero
    base_twist.twist.angular.x = 0.0;
    base_twist.twist.angular.y = 0.0;
    base_twist.twist.angular.z = 0.0;
    base_twist.twist.linear.z = 0.0;

/*    double minimum_base_velocity_x = 0.0;
    double minimum_base_velocity_y = 0.0;

    node_handler_.getParam("minimum_base_velocity_x", minimum_base_velocity_x);
    node_handler_.getParam("minimum_base_velocity_y", minimum_base_velocity_y);

    // if base velocity is too low, move the arm instead
    if (std::abs(transformed_base_twist.twist.linear.x) < minimum_base_velocity_x)
    {
        transformed_base_twist.twist.linear.x = 0.0;
    }
    else
    {
        transformed_arm_twist.twist.linear.x = 0.0;
    }

    if (std::abs(transformed_base_twist.twist.linear.y) < minimum_base_velocity_y)
    {
        transformed_base_twist.twist.linear.y = 0.0;
    }
    else
    {
        transformed_arm_twist.twist.linear.y = 0.0;
    }*/

    arm_twist_stamped_pub_.publish(arm_twist);
    base_twist_pub_.publish(base_twist.twist);

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
