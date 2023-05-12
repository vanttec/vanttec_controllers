/** ----------------------------------------------------------------------------
 * @file: car_3dof_dynamic_model.cpp
 * @date: November 29, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of generic 3-DOF car model in the non-inertial frame with
           Euler Angles.
 * -----------------------------------------------------------------------------
 **/

#include "car_3dof_dynamic_model.hpp"

CarDynamicModel::CarDynamicModel(const float sample_time){
    sample_time_ = sample_time;

    // State vectors
    eta_ = Eigen::Vector3f::Zero();   // x, y, psi
    eta_dot_ = Eigen::Vector3f::Zero();
    eta_dot_prev_ = Eigen::Vector3f::Zero();
    nu_ = Eigen::Vector3f::Zero();    // u, v, r
    nu_dot_ = Eigen::Vector3f::Zero();
    nu_dot_prev_ = Eigen::Vector3f::Zero();
    theta_ = 0.0;

    // Rotation matrix
    Eigen::Matrix3f R_ = Eigen::Matrix3f::Zero();

    /* Model forces */
    F_grav_ = 0.0;      // Force due to gravity
    F_brake_ = 0.0;     // Braking force
    F_throttle_ = 0.0;  // Throttle force
    F_drag_x_ = 0.0;      // Air drag force
    F_rr_ = 0.0;        // Rolling resistance force
    F_fy_ = 0.0;        // Frontal lateral force
    F_ry_ = 0.0;        // Rear lateral force

    /* Control inputs */
    B_ = 0.0;           // Steering command
    D_ = 0.0;           // Throttle command
    delta_ = 0.0;       // Steering angle

    g_x_ = Eigen::Matrix3f::Zero();   // x, y, psi
    f_x_ = Eigen::Vector3f::Zero();
    u_ = Eigen::Vector3f::Zero();
}

CarDynamicModel::~CarDynamicModel(){}

void CarDynamicModel::calculateRotation(){
    float psi = eta_(2);
    R_ << std::cos(psi), std::sin(psi), 0,
         -std::sin(psi), std::cos(psi), 0,
          0,             0,             1;
}

void CarDynamicModel::calculateStates(){
    
    calculateRotation();
    
    float u = nu_(0);
    float v = nu_(1);
    float r = nu_(2);
    float fx = 0;
    float fy = 0;
    float fz = 0;

    nu_dot_prev_ = nu_dot_;
    eta_dot_prev_ = eta_dot_;
    // float u_dot_brake = std::min(0,); // Braking pedal model acceleration

    F_grav_ = m_*utils::g*std::sin(theta_);
    F_drag_x_ = 0.5*utils::air_rho*A_*Cd_*std::pow(u,2);
    F_rr_ = m_*utils::g*fr_*cos(theta_);       /* I don't like this friction model.
                                                Does not account when the vehicle is
                                                at rest. Must find alternative. */
    
    // Next condition was set so the vehicle does not move backwards when
    // the throttle force is less than the resistance
    if(u_(0) < F_rr_){
        F_rr_ = 0;
        u_(0) = 0;
    }

    alpha_f_x_ = std::atan2(v + len_f_*r,u) - delta_;
    alpha_r_ = std::atan2(v - len_r_*r,u);
    F_fy_ = -C_alpha_*alpha_f_;
    F_ry_ = -C_alpha_*alpha_r_;

    // ROS_INFO_STREAM("F drag = " << F_drag_);
    // ROS_INFO_STREAM("F rr = " << F_rr_);
    // ROS_INFO_STREAM("F fy = " << F_fy_);
    // ROS_INFO_STREAM("F ry = " << F_ry_);
    
    fx = -(F_drag_ + F_rr_ + F_grav_ + F_fy_*std::sin(delta_) - m_*v*r);
    fy = F_ry_ + F_fy_*std::cos(delta_) - m_*u*r;
    fz = F_fy_*len_f_*std::cos(delta_) - F_ry_*len_r_;

    f_ << fx/m_,
          fy/m_,
          fz/Iz_;

    g_(0,0) = 1/m_;

    /* 3-DOF state calculation */
    nu_dot_ = f_x_ + g_x_*u_;

    /* Integrating acceleration to get velocities */
    nu_ += (nu_dot_prev_ + nu_dot_) / 2 * sample_time_;

    /* Changing frames */
    eta_dot_ = R_*nu_;

    /* Integrating velocities to get positions */
    eta_ += (eta_dot_prev_ + eta_dot_) / 2 * sample_time_;

    if (std::fabs(eta_(2)) > M_PI){
        eta_(2) = (eta_(2) / std::fabs(eta_(2))) * (std::fabs(eta_(2)) - 2 * M_PI);
    }
    
    /* update ROS Messages */

    accelerations_.linear.x = nu_dot_(0);
    accelerations_.linear.y = nu_dot_(1);
    accelerations_.angular.z = nu_dot_(2);

    velocities_.linear.x = nu_(0);
    velocities_.linear.y = nu_(1);
    velocities_.angular.z = nu_(2);
    
    eta_pose_.x = eta_(0);
    eta_pose_.y = eta_(1);
    eta_pose_.psi = eta_(2);
}

void CarDynamicModel::setForceInput(const vanttec_msgs::ThrustControl& thrust){
    u_ << thrust.tau_x,
          0,
          0;
}

void CarDynamicModel::setSteeringInput(const std_msgs::Float32& delta){
    delta_ = delta.data;
    if(u_(0) < 0.1){
        delta_ = 0;
    }
}