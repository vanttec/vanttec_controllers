/** ----------------------------------------------------------------------------
 * @file: uuv_4dof_pid.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: 4-DOF controller class, using different, decoupled controller for
 *         each DOF.
           OUTDATED.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_4DOF_PID_CONTROLLER_H__
#define __UUV_4DOF_PID_CONTROLLER_H__

#include "pid.hpp"
#include "vtec_u3_parameters.hpp"
#include "vanttec_msgs/ThrustControl.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class UUV4DOFController
{
    public:
        geometry_msgs::Pose         local_pose;
        geometry_msgs::Twist        local_twist;
        
        vanttec_msgs::ThrustControl  thrust;

        float yaw_psi_angle;

        PID surge_speed_controller;
        PID sway_speed_controller;
        PID depth_controller;
        PID heading_controller;

        Eigen::Vector4f f_x;
        Eigen::Vector4f g_x;

        UUV4DOFController(float sample_time_, const float _kpid_u[3], const float _kpid_v[3], const float _kpid_z[3], const float _kpid_psi[3]);
        ~UUV4DOFController();

        void updatePose(const geometry_msgs::Pose& _pose);
        void updateTwist(const geometry_msgs::Twist& _twist);
        void updateSetPoints(const geometry_msgs::Twist& _set_points);
        
        void updateControlLaw();
        void updateThrustOutput();

        void publishAccel();
    
    private:

        Eigen::Vector4f upsilon;
        Eigen::Matrix4f M_rb_;
        Eigen::Matrix4f M_a_;
        Eigen::Matrix4f C_rb_;
        Eigen::Matrix4f C_a_;
        Eigen::Matrix4f D_lin_;
        Eigen::Matrix4f D_qua_;
        Eigen::Vector4f G_eta_;

        ros::NodeHandle handle;
        ros::Publisher v_dot_pub = handle.advertise<geometry_msgs::Twist>("/uuv_accel",1000);
};

#endif