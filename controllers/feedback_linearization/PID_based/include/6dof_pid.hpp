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
        ros::NodeHandle handle_;
        // bool functs_arrived_;
        float MAX_FORCE_X_;
        float MAX_FORCE_Y_;
        float MAX_FORCE_Z_;
        float MAX_TORQUE_K_;
        float MAX_TORQUE_M_;
        float MAX_TORQUE_N_; 
        
    public:
        Eigen::VectorXf u_;              // Control
        Eigen::VectorXf ua_;             // Auxiliary Control
        Eigen::VectorXf f_;
        Eigen::MatrixXf g_;
        Eigen::VectorXf ref_dot_dot_;
        vanttec_msgs::ThrustControl  thrust_;

        PID PID_x_;
        PID PID_y_;
        PID PID_z_;
        PID PID_phi_;
        PID PID_theta_;
        PID PID_psi_;

        PID6DOF(const float &sample_time, const float *k_p, const float *k_i, const float *k_d, const DOFControllerType_E *type);
        ~PID6DOF();

        void setMaxThrust(const float* MAX_TAU);
        void calculateManipulations();
        void updateDynamics(const vanttec_msgs::SystemDynamics& non_linear_functions);
        void updateSetPoints(const vanttec_msgs::EtaPose& set_points);
        void updatePose(const vanttec_msgs::EtaPose& current);
};

#endif