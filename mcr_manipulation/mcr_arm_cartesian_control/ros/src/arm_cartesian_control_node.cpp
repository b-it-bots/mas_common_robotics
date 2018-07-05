/*
 * ros_arm_cartesian_control.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 */

#include <mcr_manipulation_utils/ros_urdf_loader.h>
#include <mcr_arm_cartesian_control/arm_cartesian_control.h>

#include <sensor_msgs/JointState.h>
#include <kdl/kdl.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <brics_actuator/JointVelocities.h>
#include <tf/transform_listener.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>

KDL::Chain arm_chain;
std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits;

KDL::JntArray joint_positions;
std::vector<bool> joint_positions_initialized;

Eigen::VectorXd sigma;

KDL::Twist targetVelocity;

KDL::ChainIkSolverVel* ik_solver;
Eigen::MatrixXd weight_ts;
Eigen::MatrixXd weight_js;

ros::Publisher cmd_vel_publisher;
ros::Publisher sigma_publisher;

tf::TransformListener *tf_listener;

bool active = false;
ros::Time t_last_command;

brics_actuator::JointVelocities jointMsg;

std::string root_name = "DEFAULT_CHAIN_ROOT";
int nrOfJoints;


void jointstateCallback(sensor_msgs::JointStateConstPtr joints)
{

    for (unsigned i = 0; i < joints->position.size(); i++)
    {

        const char* joint_uri = joints->name[i].c_str();

        for (unsigned int j = 0; j < arm_chain.getNrOfJoints(); j++)
        {
            const char* chainjoint =
                arm_chain.getSegment(j).getJoint().getName().c_str();

            if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0)
            {
                joint_positions.data[j] = joints->position[i];
                joint_positions_initialized[j] = true;
            }
        }
    }
}

void wtsCallback(std_msgs::Float32MultiArray weights)
{
    weight_ts.resize(6, 6);
    weight_ts.setIdentity();
    weight_ts(0, 0) = weights.data[0];
    weight_ts(1, 1) = weights.data[1];
    weight_ts(2, 2) = weights.data[2];
    weight_ts(3, 3) = weights.data[3];
    weight_ts(4, 4) = weights.data[4];
    weight_ts(5, 5) = weights.data[5];
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightTS(weight_ts);
}

void wjsCallback(std_msgs::Float32MultiArray weights)
{
    weight_js.resize(nrOfJoints, nrOfJoints);
    weight_js.setIdentity();
    for (int i = 0; i < nrOfJoints; i++)
    {
        weight_js(i,i) = weights.data[i];
    }
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightJS(weight_js);
}

void ccCallback(geometry_msgs::TwistStampedConstPtr desiredVelocity)
{

    for (size_t i = 0; i < joint_positions_initialized.size(); i++)
    {
        if (!joint_positions_initialized[i])
        {
            std::cout << "joints not initialized" << std::endl;
            return;
        }
    }

    if (!tf_listener) return;

    geometry_msgs::Vector3Stamped linear_in;
    geometry_msgs::Vector3Stamped linear_out;
    linear_in.header = desiredVelocity->header;
    linear_in.vector = desiredVelocity->twist.linear;

    try
    {
        tf_listener->transformVector(root_name, linear_in, linear_out);
    }
    catch (...)
    {
        ROS_ERROR("Could not transform frames %s -> %s for linear transformation", desiredVelocity->header.frame_id.c_str(), root_name.c_str());
        return;
    }
    geometry_msgs::Vector3Stamped angular_in;
    geometry_msgs::Vector3Stamped angular_out;
    angular_in.header = desiredVelocity->header;
    angular_in.vector = desiredVelocity->twist.angular;

    try
    {
        tf_listener->transformVector(root_name, angular_in, angular_out);
    }
    catch (...)
    {
        ROS_ERROR("Could not transform frames %s -> %s for angular transformation", desiredVelocity->header.frame_id.c_str(), root_name.c_str());
        return;
    }
    targetVelocity.vel.data[0] = linear_out.vector.x;
    targetVelocity.vel.data[1] = linear_out.vector.y;
    targetVelocity.vel.data[2] = linear_out.vector.z;

    targetVelocity.rot.data[0] = angular_out.vector.x;
    targetVelocity.rot.data[1] = angular_out.vector.y;
    targetVelocity.rot.data[2] = angular_out.vector.z;

    t_last_command = ros::Time::now();

    active = true;
}


void init_ik_solver()
{

    if (ik_solver != 0)
    {
        return;
    }

    ik_solver = new KDL::ChainIkSolverVel_wdls(arm_chain);
    weight_ts.resize(6, 6);
    weight_ts.setIdentity();

    weight_ts(0, 0) = 1;
    weight_ts(1, 1) = 1;
    weight_ts(2, 2) = 1;
    weight_ts(3, 3) = 0.4;
    weight_ts(4, 4) = 0.4;
    weight_ts(5, 5) = 0.4;
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightTS(weight_ts);

    weight_js = (Eigen::MatrixXd::Identity(arm_chain.getNrOfJoints(),
                                           arm_chain.getNrOfJoints()));
    //weight_js(0, 0) = 0.5;
    //weight_js(1,1) = 1;
    //weight_js(2,2) = 1;
    //weight_js(3,3) = 1;
    //weight_js(4,4) = 0.1;
    //((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightJS(weight_js);


    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setLambda(10000.0);
}

void init_joint_msgs()
{
    joint_positions_initialized.resize(arm_chain.getNrOfJoints(), false);
    jointMsg.velocities.resize(arm_chain.getNrOfJoints());
    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++)
    {
        jointMsg.velocities[i].joint_uri =
            arm_chain.getSegment(i).getJoint().getName();
        jointMsg.velocities[i].unit = "s^-1 rad";
    }
}

void publishJointVelocities(KDL::JntArrayVel& joint_velocities)
{

    for (unsigned int i = 0; i < joint_velocities.qdot.rows(); i++)
    {
        jointMsg.velocities[i].value = joint_velocities.qdot(i);
        ROS_DEBUG("%s: %.5f %s", jointMsg.velocities[i].joint_uri.c_str(),
                  jointMsg.velocities[i].value, jointMsg.velocities[i].unit.c_str());
        if (isnan(jointMsg.velocities[i].value))
        {
            ROS_ERROR("invalid joint velocity: nan");
            return;
        }
        if (fabs(jointMsg.velocities[i].value) > 1.0)
        {
            ROS_ERROR("invalid joint velocity: too fast");
            return;
        }
    }
    cmd_vel_publisher.publish(jointMsg);
}


void stopMotion()
{

    for (unsigned int i = 0; i < jointMsg.velocities.size(); i++)
    {
        jointMsg.velocities[i].value = 0.0;

    }
    cmd_vel_publisher.publish(jointMsg);
}


bool watchdog()
{

    double watchdog_time = 0.3;
    if (active == false)
    {
        return false;
    }

    ros::Time now = ros::Time::now();

    ros::Duration time = (now - t_last_command);

    if (time > ros::Duration(watchdog_time))
    {
        active = false;
        stopMotion();
        return false;
    }

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_cartesian_control");
    ros::NodeHandle node_handle("~");
    tf_listener = new tf::TransformListener();

    double rate = 50;

    //TODO: read from param
    std::string velocity_command_topic = "joint_velocity_command";
    std::string sigma_values_topic = "sigma_values";
    std::string weight_ts_topic = "weight_task_space";
    std::string weight_js_topic = "weight_joint_space";
    std::string joint_state_topic = "/joint_states";
    std::string cart_control_topic = "cartesian_velocity_command";

    std::string tooltip_name = "DEFAULT_CHAIN_TIP";

    if (!node_handle.getParam("root_name", root_name))
    {
        ROS_ERROR("No parameter for root_name specified");
        return -1;
    }
    ROS_INFO("Using %s as chain root [param: root_name]", root_name.c_str());

    if (!node_handle.getParam("tip_name", tooltip_name))
    {
        ROS_ERROR("No parameter for tip_name specified");
        return -1;
    }
    ROS_INFO("Using %s as tool tip [param: tip_name]", tooltip_name.c_str());




    //load URDF model
    ROS_URDF_Loader loader;
    loader.loadModel(node_handle, root_name, tooltip_name, arm_chain, joint_limits);
    
    //init
    nrOfJoints = arm_chain.getNrOfJoints();
    joint_positions.resize(nrOfJoints);

    std_msgs::Float32MultiArray sigma_array;

    init_ik_solver();

    init_joint_msgs();

    //fk_solver = new KDL::ChainFkSolverPos_recursive(arm_chain);
    //jnt2jac = new KDL::ChainJntToJacSolver(arm_chain);

    //sigma values publisher 
    sigma_publisher = node_handle.advertise<std_msgs::Float32MultiArray>(
                            sigma_values_topic, 1);

    //register publisher
    cmd_vel_publisher = node_handle.advertise<brics_actuator::JointVelocities>(
                            velocity_command_topic, 1);

    //register subscriber
    ros::Subscriber sub_joint_states = node_handle.subscribe(joint_state_topic,
                                       1, jointstateCallback);

    ros::Subscriber sub_wjs = node_handle.subscribe(weight_js_topic, 1,
                             wjsCallback);
    
    ros::Subscriber sub_wts = node_handle.subscribe(weight_ts_topic, 1,
                             wtsCallback);

    ros::Subscriber sub_cc = node_handle.subscribe(cart_control_topic, 1,
                             ccCallback);
    

    arm_cc::Arm_Cartesian_Control control(&arm_chain, ik_solver);
    std::vector<double> upper_limits;
    std::vector<double> lower_limits;

    for (unsigned int i = 0; i < joint_limits.size(); i++)
    {
        upper_limits.push_back(joint_limits[i]->upper);
        lower_limits.push_back(joint_limits[i]->lower);
    }
    control.setJointLimits(lower_limits, upper_limits);

    KDL::JntArrayVel cmd_velocities(nrOfJoints);

    //loop with 50Hz
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {

        ros::spinOnce();

        if (watchdog())
        {
            control.process(1 / rate, joint_positions, targetVelocity, cmd_velocities, sigma);
            
            sigma_array.data.clear();
            if (sigma.size() != 0)
            {
                for (int i = 0; i < nrOfJoints; i++)
                {
                    sigma_array.data.push_back(sigma[i]);
                }
            }
            
            sigma_publisher.publish(sigma_array);
            publishJointVelocities(cmd_velocities);
        }


        loop_rate.sleep();
    }

    delete tf_listener;

    return 0;
}
