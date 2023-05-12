/** ----------------------------------------------------------------------------
 * @file: fblin_6dof_model.hpp
 * @date: April 23, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a base feedback linearized 6 DOF dynamic model for any vehicle.
 * -----------------------------------------------------------------------------
 **/

#ifndef __FBLINMODEL__
#define __FBLINMODEL__

#include "vanttec_msgs/ThrustControl.h"
#include "vanttec_msgs/EtaPose.h"
#include "utils/utils.hpp"

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

class FBLinDynModel
{
    public:
        float sample_time_;

        /* Transformation matrix */

        Eigen::MatrixXf J_;
        Eigen::Matrix3f R_;
        Eigen::Matrix3f T_;
        Eigen::MatrixXf J_dot_;
        Eigen::Matrix3f R_dot_;
        Eigen::Matrix3f T_dot_;
        Eigen::MatrixXf J_inv_;

        /* System states */

        Eigen::VectorXf eta_;            // x, y, z, phi, theta_, psi
        Eigen::VectorXf eta_dot_;
        Eigen::VectorXf eta_dot_prev_;
        Eigen::VectorXf eta_dot_dot_;
        Eigen::VectorXf eta_dot_dot_prev_;
        Eigen::VectorXf nu_;             // u, v, w, p, q, r
        Eigen::VectorXf nu_dot_;
        Eigen::VectorXf nu_dot_prev_;

        /* Non-linear functions */
        Eigen::VectorXf f_x_;
        Eigen::MatrixXf g_x_;

        FBLinDynModel(float sample_time);
        ~FBLinDynModel();

};

#endif