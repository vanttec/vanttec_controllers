/** ----------------------------------------------------------------------------
 * @file: 6dof_pid.hpp
 * @date: April 10, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: 6-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __6DOF_PID_H__
#define __6DOF_PID_H__

#include "pid.hpp"
#include "vanttec_msgs/ThrustControl.h"
#include "vanttec_msgs/SystemDynamics.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class PID6DOF
{
    private:
        ros::NodeHandle handle;
        bool functs_arrived;
        float MAX_FORCE_X;
        float MAX_FORCE_Y;
        float MAX_FORCE_Z;
        float MAX_TORQUE_K;
        float MAX_TORQUE_M;
        float MAX_TORQUE_N; 
        
    public:
        Eigen::VectorXf u;              // Control
        Eigen::VectorXf ua;             // Auxiliary Control
        Eigen::VectorXf f;
        Eigen::MatrixXf g;
        Eigen::VectorXf ref_dot_dot;
        vanttec_msgs::ThrustControl  thrust;

        PID PID_x;
        PID PID_y;
        PID PID_z;
        PID PID_phi;
        PID PID_theta;
        PID PID_psi;

        PID6DOF(const float &_sample_time_s, const float *_k_p, const float *_k_i, const float *_k_d, const DOFControllerType_E *_type);
        ~PID6DOF();

        void SetMaxThrust(const float* MAX_TAU);
        void CalculateManipulations();
        void UpdateDynamics(const vanttec_msgs::SystemDynamics& _non_linear_functions);
        void UpdateSetPoints(const vanttec_msgs::EtaPose& _set_points);
        void UpdatePose(const vanttec_msgs::EtaPose& _current);
};

#endif