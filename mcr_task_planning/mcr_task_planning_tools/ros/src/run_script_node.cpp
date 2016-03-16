/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Wrapper around common class RunScript which calls system() function 
 * for calling external bash scripts from code
 * 
 */

#include <run_script_node.h>
#include <string>
#include <vector>

RunScriptNode::RunScriptNode() : nh_("~"), 
{
      // set initial member variables values
    callback_received_ = false;
    are_args_available_ = false;
    
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &RunScriptNode::runScript, this);

    // publications
    pub_event_in_ = nh_.advertise<std_msgs::String>("event_out", 2);
    
    getParams();
    
    // set script path
    script_handler_.setScriptPath(full_path_to_script_);

    // set script arguments
    if (are_args_available_)
    {
        script_handler_.setScriptArgs(script_arguments_);
    }
    
     ROS_INFO("Run script node initialized...");
}

RunScriptNode::~RunScriptNode()
{
    // shut down publishers and subscribers
    sub_event_in_.shutdown();
    pub_event_in_.shutdown();
}

void RunScriptNode::getParams()
{
    // setup script default arguments
    std::vector<std::string> default_args;
    default_args.push_back("no_args");

    // getting required parameters from parameter server
    
    nh_.param<std::string>("script_path", full_path_to_script_, "/home/user/my_script.sh");
    nh_.param<std::vector<std::string> >("script_arguments", script_arguments_, default_args);

    // informing the user about the parameters which will be used
    ROS_INFO("Script path : %s", full_path_to_script_.c_str());

    if (script_arguments_.at(0) == std::string("no_args"))
    {
        are_args_available_ = false;
        ROS_INFO("Script will run with no arguments");
    }
    else
    {
        are_args_available_ = true;
        std::string args;
        for (int i =0 ; i < script_arguments_.size() ; i++)
        {
            args += script_arguments_.at(i);
            args += std::string(" ");
        }
        ROS_INFO("Script will run with the following arguments : %s", args.c_str());
    }
}

void RunScriptNode::runScriptCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    callback_received_ = true;
}

void RunScriptNode::update()
{
    // listen to callbacks
    ros::spinOnce();

    if (callback_received_)
    {
        // lower flag
        callback_received_ = false;

        // checking for event in msg content
        if (event_in_msg_.data == "e_trigger")
        {
            // run script
            if (script_handler_.run())
            {
                ROS_INFO("Script succesfully called !");
                // publish even_out : "e_success"
                even_out_msg_.data = std::string("e_success");
                pub_.publish(even_out_msg_);
            }
            else
            {
                // publish even_out : "e_failure"
                even_out_msg_.data = std::string("e_failure");
                pub_.publish(even_out_msg_);
                ROS_ERROR("Failed to run script, does it exist? is it executable?");
            }
        }
        else
        {
            // publish even_out : "e_failure"
            even_out_msg_.data = std::string("e_failure");
            pub_.publish(even_out_msg_);
            ROS_ERROR("event_in message received not known, admissible strings are : e_trigger");
        }
    }
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "run_script_node");

    // create object of this node class
    RunScriptNode run_script_node;

    // setup node frequency
    double node_frequency = 10.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
    ros::Rate loop_rate(node_frequency);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);

    while (ros::ok())
    {
        // main loop function
        run_script_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
