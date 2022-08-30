/** ----------------------------------------------------------------------------
 * @file: 6dof_antsmc.hpp
 * @date: August 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF adaptive non-singular terminal sliding mode controller class
 * -----------------------------------------------------------------------------
 * */

#ifndef __ANTSMC6DOF_H__
#define __ANTSMC6DOF_H__

#include "antsmc.hpp"
#include "vanttec_msgs/ThrustControl.h"
#include "vanttec_msgs/SystemDynamics.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class ANTSMC6DOF
{
    private:
        ros::NodeHandle handle;
        // bool functs_arrived;

        float MAX_FORCE_X;
        float MAX_FORCE_Y;
        float MAX_FORCE_Z;
        float MAX_TORQUE_K;
        float MAX_TORQUE_M;
        float MAX_TORQUE_N; 
        
    public:
        Eigen::VectorXf u;              // Control
        Eigen::VectorXf ua;             // Auxiliary Control
        Eigen::VectorXf delta;
        Eigen::VectorXf f;
        Eigen::MatrixXf g;
        Eigen::VectorXf q_dot_dot;      //ref_dot_dot
        vanttec_msgs::ThrustControl  thrust;

        ANTSMC ANTSMC_x;
        ANTSMC ANTSMC_y;
        ANTSMC ANTSMC_z;
        ANTSMC ANTSMC_phi;
        ANTSMC ANTSMC_theta;
        ANTSMC ANTSMC_psi;
        
        ANTSMC6DOF(const float sample_time_s, const float alpha[6], const float beta[6], const float K2[6], const float K_alpha[6], const float K_min[6],  const float K_min_init[6], const float mu[6]);
        ~ANTSMC6DOF();

        void SetMaxThrust(const float* MAX_TAU);
        void UpdateDynamics(const vanttec_msgs::SystemDynamics& non_linear_functions);
        void UpdatePose(const vanttec_msgs::EtaPose& current);
        void UpdateSetPoints(const vanttec_msgs::EtaPose& set_points);
        void CalculateManipulation();

        // void UpdateTwist(const geometry_msgs::Twist& twist);
        
        // void UpdateControlLaw();
        // void UpdateThrustOutput();

        // void PublishAccel();
};

#endif