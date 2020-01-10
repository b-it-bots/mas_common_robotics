/*
 * arm_cartesian_control.h
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 */

#ifndef ARMCARTESIANCONTROL_H_
#define ARMCARTESIANCONTROL_H_

#include <kdl/kdl.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

namespace arm_cc
{

class Arm_Cartesian_Control
{
protected:

    KDL::ChainIkSolverVel* ik_solver;

    KDL::Chain* arm_chain;

    std::vector<double> upper_joint_limits;
    std::vector<double> lower_joint_limits;

    double max_lin_frame_velocity = 0.1;  // m/s
    double max_joint_vel = 0.25; // radian/s
    double eps_velocity = 0.0001;

public:
    Arm_Cartesian_Control(KDL::Chain* arm_chain,
                          KDL::ChainIkSolverVel* ik_solver);

    virtual ~Arm_Cartesian_Control();


    void checkLimits(double dt, KDL::JntArray& joint_positions,
                     KDL::JntArray& jntVel);

    //bool watchdog();

    //void stopMotion();

    void process(double dt, KDL::JntArray& position, KDL::Twist& targetVelocity, KDL::JntArrayVel& out_jnt_velocities, Eigen::VectorXd& sigma);

    void setJointLimits(std::vector<double> lower, std::vector<double> upper);

    void setCartVelLimit(double limit);

    void setJointVelLimit(double limit);
};

} /* namespace arm_cc */
#endif /* ARMCARTESIANCONTROL_H_ */
