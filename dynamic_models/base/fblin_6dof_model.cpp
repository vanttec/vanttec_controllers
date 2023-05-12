/** ----------------------------------------------------------------------------
 * @file: fblin_6dof_model.cpp
 * @date: April 25, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a base feedback linearized 6 DOF dynamic model for any vehicle.
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/base/fblin_6dof_model.hpp"

FBLinDynModel::FBLinDynModel(float sample_time)
{
    sample_time_ = sample_time;

    J_ = Eigen::MatrixXf::Zero(6, 6);
    R_ = Eigen::Matrix3f::Zero(3,3);
    T_ = Eigen::Matrix3f::Zero(3,3);
    J_dot_ = Eigen::MatrixXf::Zero(6, 6);
    R_dot_ = Eigen::Matrix3f::Zero(3,3);
    T_dot_ = Eigen::Matrix3f::Zero(3,3);
    J_inv_ = Eigen::MatrixXf::Zero(6, 6);

    eta_ = Eigen::MatrixXf::Zero(6,1);            // x, y, z, phi, theta, psi
    eta_dot_ = Eigen::MatrixXf::Zero(6,1);
    eta_dot_prev_ = Eigen::MatrixXf::Zero(6,1);
    eta_dot_dot_ = Eigen::MatrixXf::Zero(6,1);
    eta_dot_dot_prev_ = Eigen::MatrixXf::Zero(6,1);
    nu_ = Eigen::MatrixXf::Zero(6,1);             // u, v, w, p, q, r
    nu_dot_ = Eigen::MatrixXf::Zero(6,1);
    nu_dot_prev_ = Eigen::MatrixXf::Zero(6,1);

    f_x_ = Eigen::MatrixXf::Zero(6,1);
    g_x_ = Eigen::MatrixXf::Zero(6,6);

}

FBLinDynModel::~FBLinDynModel(){}