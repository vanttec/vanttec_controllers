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

#include <iostream>
#include "utils/utils.hpp"

CarDynamicModel::CarDynamicModel(float sample_time){
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
    F_drag_ = 0.0;      // Air drag force
    F_rr_ = 0.0;        // Rolling resistance force
    F_fy_ = 0.0;        // Frontal lateral force
    F_ry_ = 0.0;        // Rear lateral force

    /* Control inputs */
    B_ = 0.0;           // Steering command
    D_ = 0.0;           // Throttle command
    delta_ = 0.0;       // Steering angle

    g_ = Eigen::Matrix3f::Zero();   // x, y, psi
    f_ = Eigen::Vector3f::Zero();
    u_ = Eigen::Vector3f::Zero();
}

CarDynamicModel::~CarDynamicModel(){}

void CarDynamicModel::setInitPose(const std::vector<float>& eta)
{
    eta_(0) = eta[0];
    eta_(1) = eta[1];
    eta_(2) = eta[2];
}

void CarDynamicModel::setOffsets(float rr_offset, float t_offset)
{
    rr_offset_ = rr_offset;
    t_offset_ = t_offset;
}

void CarDynamicModel::setMotorConstants(float Cm1, float Cm2)
{
    Cm1_ = Cm1;
    Cm2_ = Cm2;
}

void CarDynamicModel::calculateStates(){
    
    utils::calculate2DRotation(R_, eta_(2));
    
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
    F_drag_ = 0.5*utils::air_rho*A_*Cd_*std::pow(u,2);
    F_rr_ = m_*utils::g*fr_*cos(theta_);       /* I don't like this friction model.
                                                Does not account when the vehicle is
                                                at rest. An alternative could be found
                                                in the future */
    F_rr_ -= rr_offset_;    // To compensate for model error

    std::cout << std::endl;
    std::cout << "F grav = " << F_grav_ << std::endl;
    std::cout << "F drag = " << F_drag_ << std::endl;
    std::cout << "F rr = " << F_rr_ << std::endl;

    F_throttle_ = (Cm1_ - Cm2_*u)*static_cast<int>(D_);

    std::cout << "Cm1 = " << Cm1_ << std::endl;
    std::cout << "Cm2 = " << Cm2_ << std::endl;
    std::cout << "D = " << static_cast<int>(D_) << std::endl;
    std::cout << "F throttle = " << F_throttle_ << std::endl;

    if(D_ > 0)
        F_throttle_ -= t_offset_;
    else {
        F_throttle_ = 0;
    }
    std::cout << "Throttle offset = " << t_offset_ << std::endl;

    std::cout << "F rr = " << F_rr_ << std::endl;
    std::cout << "F throttle = " << F_throttle_ << std::endl;

    u_(0) = F_throttle_;// + F_brake_;
    
    // Next condition was set so the vehicle does not move backwards when
    // the throttle force is less than the resistance
    if(F_rr_ >= u_(0)  && u < 0.01) {
        u_(0) = 0;
        F_rr_ = 0;
    }

    alpha_f_ = std::atan2(v + len_f_*r,u) - delta_;
    alpha_r_ = std::atan2(v - len_r_*r,u);
    F_fy_ = -C_alpha_*alpha_f_;
    F_ry_ = -C_alpha_*alpha_r_;

    fx = -(F_drag_ + F_rr_ + F_grav_ + F_fy_*std::sin(delta_) - m_*v*r);
    fy = F_ry_ + F_fy_*std::cos(delta_) - m_*u*r;
    fz = F_fy_*len_f_*std::cos(delta_) - F_ry_*len_r_;

    std::cout << "F fy = " << F_fy_ << std::endl;
    std::cout << "F ry = " << F_ry_ << std::endl;

    std::cout << "fx = " << fx << std::endl;
    std::cout << "fy = " << fy << std::endl;
    std::cout << "fz = " << fz << std::endl;

    f_ << fx/m_,
          fy/m_,
          fz/Iz_;

    g_(0,0) = 1/m_;

    /* 3-DOF state calculation */
    nu_dot_ = f_ + g_*u_;

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

void CarDynamicModel::setThrottle(uint8_t D){
    D_ = D;
    // u_ << u,  // throttle + break
    //       0,
    //       0;
}

void CarDynamicModel::setSteeringInput(float delta){
    delta_ = delta;
    if(u_(0) < 0.1){
        delta_ = 0;
    }
}

// void CarDynamicModel::manualControl(const sdv_msgs::msg::VehicleControl &manual)
// {
    
//     //RCLCPP_WARN(node_->get_logger(), "Could not create directory!");
//     F_throttle_ = (manual.throttle==1) ? F_throttle_+10 : F_throttle_;
//     F_throttle_ = (manual.brake==1) ? F_throttle_-10 : F_throttle_;
//     F_throttle_ = (F_throttle_>=Cm_) ? Cm_ : F_throttle_;
//     F_throttle_ = (F_throttle_<=0) ? 0 : F_throttle_;
//     u_ << F_throttle_,
//         0,
//         0;
//     delta_ = manual.steer;
//     if (u_(0) < 0.1)
//     { 
//         delta_ = 0;
//     }