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

RunScriptNode::RunScriptNode() : nh_("~")
{
    // subscriptions
    sub_ = nh_.subscribe("event_in", 1, &RunScriptNode::runScript, this);

    // publications
    pub_ = nh_.advertise<std_msgs::String>("event_out", 2);
}

RunScriptNode::~RunScriptNode()
{
    // shut down publishers and subscribers
    sub_.shutdown();
    pub_.shutdown();
}

void RunScriptNode::init()
{
    // set initial member variables values
    callback_received_ = false;
    are_args_available_ = false;
    node_frequency_ = 0.0;
    ROS_INFO("Run script node initialized...");
}

void RunScriptNode::getParams()
{
    // setup script default arguments
    std::vector<std::string> default_args;
    default_args.push_back("no_args");

    // getting required parameters from parameter server
    nh_.param("node_frequency", node_frequency_, 10.0);
    nh_.param<std::string>("script_path", full_path_to_script_, "/home/user/my_script.sh");
    nh_.param<std::vector<std::string> >("script_arguments", script_arguments_, default_args);

    // informing the user about the parameters which will be used
    ROS_INFO("Node will run at : %lf [hz]", node_frequency_);
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

void RunScriptNode::oneTimeNodeSetup()
{
    // set script path
    script_handler_.setScriptPath(full_path_to_script_);

    // set script arguments
    if (are_args_available_)
    {
        script_handler_.setScriptArgs(script_arguments_);
    }
}

void RunScriptNode::runScript(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    callback_received_ = true;
}

void RunScriptNode::update()
{
    // setting the frequency at which the node will run
    ros::Rate loop_rate(node_frequency_);

    while (ros::ok())
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

        // sleep to control the node frequency
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "run_script_node");

    // create object of this node class
    RunScriptNode run_script_node;

    // initialize
    run_script_node.init();

    // get parameters
    run_script_node.getParams();

    // one time node setup
    run_script_node.oneTimeNodeSetup();

    // main loop function
    run_script_node.update();

    return 0;
}
