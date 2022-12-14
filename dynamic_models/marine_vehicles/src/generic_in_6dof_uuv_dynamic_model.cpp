/** ----------------------------------------------------------------------------
 * @file: generic_in_6dof_uuv_model.hpp
 * @date: August 31, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a generic 6dof UUV model in the inertial frame with
           Euler Angles.
   @todo: Modify matrices for the true general case of non-diagonal matrices.
          Include ALL terms. Also, include offset vector in the case the origin
          is no the COM_.
 * -----------------------------------------------------------------------------
 **/

#include "generic_in_6dof_uuv_dynamic_model.hpp"

GenericIn6DOFUUVDynamicModel::GenericIn6DOFUUVDynamicModel(const float& sample_time)
{
    sample_time_ = sample_time;

    J_.resize(6,6);
    M_.resize(6,6);
    M_rb_.resize(6,6);
    M_a_.resize(6,6);
    C_.resize(6,6);
    C_rb_.resize(6,6);
    C_a_.resize(6,6);
    D_.resize(6,6);
    D_lin_.resize(6,6);
    D_qua_.resize(6,6);
    g_.resize(6,6);

    eta_.resize(6,1);            // x, y, z, phi, theta, psi
    eta_dot_.resize(6,1);
    eta_dot_prev_.resize(6,1);
    eta_dot_dot_.resize(6,1);
    eta_dot_dot_prev_.resize(6,1);
    nu_.resize(6,1);             // u, v, w, p, q, r
    nu_dot_.resize(6,1);
    nu_dot_prev_.resize(6,1);
    g_eta_.resize(6,1);
    tau_.resize(6,1);
    u_.resize(6,1);
    f_.resize(6,1);

    J_ = Eigen::MatrixXf::Zero(6, 6);
    R_ = Eigen::Matrix3f::Zero(3,3);
    T_ = Eigen::Matrix3f::Zero(3,3);
    J_dot_ = Eigen::MatrixXf::Zero(6, 6);
    R_dot_ = Eigen::Matrix3f::Zero(3,3);
    T_dot_ = Eigen::Matrix3f::Zero(3,3);
    J_inv_ = Eigen::MatrixXf::Zero(6, 6);

    M_ = Eigen::MatrixXf::Zero(6, 6);
    M_rb_ = Eigen::MatrixXf::Zero(6, 6);
    M_a_ = Eigen::MatrixXf::Zero(6, 6);
    C_ = Eigen::MatrixXf::Zero(6, 6);
    C_rb_ = Eigen::MatrixXf::Zero(6, 6);
    C_a_ = Eigen::MatrixXf::Zero(6, 6);
    D_ = Eigen::MatrixXf::Zero(6, 6);
    D_lin_ = Eigen::MatrixXf::Zero(6, 6);
    D_qua_ = Eigen::MatrixXf::Zero(6, 6);
    g_eta_ = Eigen::MatrixXf::Zero(6, 1);

    eta_ << 0,
           0,
           0,
           0,
           0,
           0;
     
    eta_dot_ << 0,
               0,
               0,
               0,
               0,
               0;

    eta_dot_prev_ << 0,
                    0,
                    0,
                    0,
                    0,
                    0;

    eta_dot_dot_ << 0,
               0,
               0,
               0,
               0,
               0;

    eta_dot_dot_prev_ << 0,
                    0,
                    0,
                    0,
                    0,
                    0;
    
    nu_ << 0,
          0,
          0,
          0,
          0,
          0;
    
    nu_dot_ << 0,
              0,
              0,
              0,
              0,
              0;
    
    nu_dot_prev_ << 0,
                   0,
                   0,
                   0,
                   0,
                   0;

    f_ << 0,
         0,
         0,
         0,
         0,
         0;

    g_ = Eigen::MatrixXf::Zero(6, 6);

    tau_ << 0,
           0,
           0,
           0,
           0,
           0;

    u_ << 0,
           0,
           0,
           0,
           0,
           0;

    MAX_FORCE_X_ = 0.0;
    MAX_FORCE_Y_ = 0.0;
    MAX_FORCE_Z_ = 0.0;
    MAX_TORQUE_K_ = 0.0;
    MAX_TORQUE_M_ = 0.0;
    MAX_TORQUE_N_ = 0.0;

    eta_pose_.x = 0;
    eta_pose_.y = 0;
    eta_pose_.z = 0;
    eta_pose_.phi = 0;
    eta_pose_.theta = 0;
    eta_pose_.psi = 0;

    velocities_.linear.x = 0;
    velocities_.linear.y = 0;
    velocities_.linear.z = 0;
    velocities_.angular.x = 0;
    velocities_.angular.y = 0;
    velocities_.angular.z = 0;

    accelerations_.linear.x = 0;
    accelerations_.linear.y = 0;
    accelerations_.linear.z = 0;
    accelerations_.angular.x = 0;
    accelerations_.angular.y = 0;
    accelerations_.angular.z = 0;
}

GenericIn6DOFUUVDynamicModel::~GenericIn6DOFUUVDynamicModel(){}

void GenericIn6DOFUUVDynamicModel::calculateTransformation()
{
    float phi = eta_(3);
    float theta = eta_(4);
    float psi = eta_(5);

    float phi_dot = eta_dot_(3);
    float theta_dot_ = eta_dot_(4);
    float psi_dot = eta_dot_(5);

    Eigen::Vector3f vect;
    vect << phi,
            theta,
            psi;

    R_ <<   std::cos(psi)*std::cos(theta),      -std::sin(psi)*std::cos(phi) + std::cos(psi)*std::sin(theta)*std::sin(phi),     std::sin(psi)*std::sin(phi) + std::cos(psi)*std::cos(phi)*std::sin(theta),
            std::sin(psi)*std::cos(theta),       std::cos(psi)*std::cos(phi) + std::sin(phi)*std::sin(theta)*std::sin(psi),    -std::cos(psi)*std::sin(phi) + std::sin(theta)*std::sin(psi)*std::cos(phi),
           -std::sin(theta),                     std::cos(theta)*std::sin(phi),                                                 std::cos(theta)*std::cos(phi);

    T_ <<   1,     std::sin(phi)*std::tan(theta),  std::cos(phi)*std::tan(theta),
            0,     std::cos(phi),                  -std::sin(phi),
            0,     std::sin(phi)/std::cos(theta),  std::cos(phi)/std::cos(theta);

    J_ << R_,                            Eigen::Matrix3f::Zero(3, 3),
          Eigen::Matrix3f::Zero(3, 3),   T_;

    R_dot_ = R_*common::Skew(vect);
    T_dot_ << 0,         std::sin(phi)*common::secant(theta)*common::secant(theta)*theta_dot_ + std::cos(phi)*std::tan(theta)*phi_dot,                   std::cos(phi)*common::secant(theta)*common::secant(theta)*theta_dot_ - std::sin(phi)*std::tan(theta)*phi_dot,
              0,        -std::sin(phi)*phi_dot,                                                                                                  -std::cos(phi)*phi_dot,
              0,         (std::cos(theta)*std::cos(phi)*phi_dot + std::sin(phi)*std::sin(theta)*theta_dot_)/(std::cos(theta)*std::cos(theta)),    (-std::cos(theta)*std::sin(phi)*phi_dot + std::cos(phi)*std::sin(theta)*theta_dot_)/(std::cos(theta)*std::cos(theta));

    J_dot_ << R_dot_,                             Eigen::Matrix3f::Zero(3, 3),
              Eigen::Matrix3f::Zero(3, 3),        T_dot_;

    J_inv_ = J_.inverse();
}

void GenericIn6DOFUUVDynamicModel::calculateCoriolis()
{
    Eigen::MatrixXf aux = J_inv_*eta_dot_;
    /* Rigid Body Coriolis Matrix */

    float m_u = m_ * aux(0);
    float m_v = m_ * aux(1);
    float m_w = m_ * aux(2);
    float ixx_p = Ixx_ * aux(3);
    float iyy_q = Iyy_ * aux(4);
    float izz_r = Izz_ * aux(5);
    
    C_rb_ << 0,   0,   0,   0,    m_w,    -m_v,
            0,   0,   0,   -m_w,  0,     m_u,
            0,   0,   0,   m_v,   -m_u,   0,
            0,   m_w,  -m_v, 0,    -izz_r, iyy_q,
            -m_w, 0 ,  m_u,  izz_r, 0,     -ixx_p,
            m_v,  -m_u, 0,   -iyy_q, ixx_p,  0;

    /* Hydrodynamic Added Mass Coriolis Matrix */

    float a1 = X_u_dot_ * aux(0);
    float a2 = Y_v_dot_ * aux(1);
    float a3 = Z_w_dot_ * aux(2);
    float a4 = K_p_dot_ * aux(3);
    float a5 = M_q_dot_ * aux(4);
    float a6 = N_r_dot_ * aux(5);
    
    C_a_ << 0,     0,    0,    0,  -a3,  a2,
           0,     0,    0,   a3,   0,  -a1,
           0,     0,    0,  -a2,  a1,   0,
           0,   -a3,  a2,   0,  -a6,  a5,
           a3,   0,  -a1,  a6,   0,  -a4,
           -a2,  a1,   0,  -a5,   a4,  0;
    
    C_= -M_*J_dot_*J_inv_ + (C_rb_ + C_a_)*J_inv_;
    // std::cout << "C:" << C << std::endl;

}

void GenericIn6DOFUUVDynamicModel::calculateDamping()
{    
    Eigen::MatrixXf aux = J_inv_*eta_dot_;
    /* Hydrodynamic Damping */

    D_lin_ << -(X_u_), 0,      0,    0,   0,    0,
                0,   -(Y_v_), 0,    0,   0,    0,
                0,   0,    -(Z_w_), 0,   0,    0,
                0,   0,      0, K_p_,   0,    0,
                0,   0,      0,    0, M_q_,   0,
                0,   0,      0,    0,   0, -(N_r_);

    D_qua_ << -(X_uu_ * std::fabs(aux(0))), 0, 0, 0, 0, 0,
             0, -(Y_vv_ * std::fabs(aux(1))), 0, 0, 0, 0,
             0, 0, -(Z_ww_ * std::fabs(aux(2))), 0, 0, 0,
             0, 0, 0, -(K_pp_ * std::fabs(aux(3))), 0, 0,
             0, 0, 0, 0, -(M_qq_ * std::fabs(aux(4))), 0,
             0, 0, 0, 0, 0, -(N_rr_ * std::fabs(aux(5)));
    
    D_= (D_lin_ + D_qua_)*J_inv_;
    // std::cout << "D:" << D << std::endl;

}

void GenericIn6DOFUUVDynamicModel::thrustCallbacK(const vanttec_msgs::ThrustControl& thrust)
{
    tau_ << thrust.tau_x,
            thrust.tau_y,
            thrust.tau_z,
            thrust.tau_phi,
            thrust.tau_theta,
            thrust.tau_psi;
}

void GenericIn6DOFUUVDynamicModel::calculateStates()
{
    calculateTransformation();
    nu_dot_prev_ = nu_dot_;
    eta_dot_prev_ = eta_dot_;
    eta_dot_dot_prev_ = eta_dot_dot_;

    /* Rigid Body Mass Matrix */

    M_rb_ << m_, 0, 0, 0, 0, 0,
            0, m_, 0, 0, 0, 0,
            0, 0, m_, 0, 0, 0,
            0, 0, 0, Ixx_, 0, 0,
            0, 0, 0, 0, Iyy_, 0,
            0, 0, 0, 0, 0, Izz_;

    /* Hydrodynamic Added Mass Matrix */

    M_a_ << X_u_dot_, 0, 0, 0, 0, 0,
           0, Y_v_dot_, 0, 0, 0, 0,
           0, 0, Z_w_dot_, 0, 0, 0,
           0, 0, 0, K_p_dot_, 0, 0,
           0, 0, 0, 0, M_q_dot_, 0,
           0, 0, 0, 0, 0, N_r_dot_;

    M_ = (M_rb_ + M_a_)*J_inv_;

    calculateCoriolis();

    calculateDamping();

    /* Restoring Forces */
    
    g_eta_ <<  (W_ - B_)*sin(eta_(4)),
              -(W_ - B_)*cos(eta_(4))*sin(eta_(3)),
              -(W_ - B_)*cos(eta_(4))*cos(eta_(3)),
              rb_y_*B_*cos(eta_(4))*cos(eta_(4)) - rb_z_*B_*cos(eta_(4))*sin(eta_(4)),
              -rb_z_*B_*sin(eta_(4)) - rb_x_*B_*cos(eta_(4))*cos(eta_(4)),
              rb_x_*cos(eta_(4))*sin(eta_(4)) + rb_y_*B_*sin(eta_(4));

    /* 6 DoF State Calculation */

    g_ = M_.inverse();
    f_ = -M_.inverse() * ((C_ * eta_dot_) + (D_ * eta_dot_) + g_eta_);

    // std::cout << "f:" << f_ << std::endl;
    // std::cout << "g:" << g << std::endl;

    u_ = tau_;
    eta_dot_dot_ = f_ + g_*u_;

    nu_ = J_inv_*eta_dot_;
    nu_dot_ = J_inv_*(eta_dot_dot_ - J_dot_*J_inv_*eta_dot_);

    /* Integrating Acceleration to get velocities_ */

    // Non-inertial frame
    Eigen::VectorXf nu_dot_sum = nu_dot_ + nu_dot_prev_;
    nu_ = (nu_dot_sum / 2 * sample_time_) + nu_;
    
    // Inertial frame
    Eigen::VectorXf eta_dot_dot_sum = eta_dot_dot_ + eta_dot_dot_prev_;
    eta_dot_ = (eta_dot_dot_sum / 2 * sample_time_) + eta_dot_;

    /* Integrating velocities_ to get Position */

    Eigen::VectorXf eta_dot_sum = eta_dot_ + eta_dot_prev_;
    eta_ = (eta_dot_sum / 2 * sample_time_) + eta_;

    if (std::fabs(eta_(3)) > M_PI)
    {
        eta_(3) = (eta_(3) / std::fabs(eta_(3))) * (std::fabs(eta_(3)) - 2 * M_PI);
    }
    if (std::fabs(eta_(4)) > M_PI)
    {
        eta_(4) = (eta_(4) / std::fabs(eta_(4))) * (std::fabs(eta_(4)) - 2 * M_PI);
    }
    if (std::fabs(eta_(5)) > M_PI)
    {
        eta_(5) = (eta_(5) / std::fabs(eta_(5))) * (std::fabs(eta_(5)) - 2 * M_PI);
    }

    /* Transform Euler Angles to Quaternions : 3-2-1 convention */
    // double psi = eta_(3);  // yaw
    // double eta_(4) = 0.0;         // pitch
    // double eta_(4) = 0.0;           // roll

    // double C_11 = cos(eta_(4))*cos(Theta(2));
    // double C_12 = cos(eta_(4))*sin(Theta(2));
    // double C_13 = -sin(eta_(4));
    // double C_21 = sin(eta_(4))*sin(eta_(4))*cos(Theta(2))-cos(eta_(4))*sin(Theta(2));
    // double C_22 = sin(eta_(4))*sin(eta_(4))*sin(Theta(2))+cos(eta_(4))*cos(Theta(2));
    // double C_23 = sin(eta_(4))*cos(eta_(4));
    // double C_31 = cos(eta_(4))*sin(eta_(4))*cos(Theta(2))+sin(eta_(4))*sin(Theta(2));
    // double C_32 = cos(eta_(4))*sin(eta_(4))*sin(Theta(2))-sin(eta_(4))*cos(Theta(2));
    // double C_33 = cos(eta_(4))*cos(eta_(4));
    // double trace = C_11+C_22+C_33;
    
    // C_rot <<  C_11, C_12, C_13,
    //                 C_21, C_22, C_23,
    //                 C_31, C_32, C_33;

    // Elements are squared their value
    // quat << 0.25*(1+trace),
    //               0.25*(1+2*C_11-trace),
    //               0.25*(1+2*C_22-trace),
    //               0.25*(1+2*C_33-trace);
    
    // char i;
    // quat.maxCoeff(&i);

    // switch (i){
    // case 0:
    //     quat(0) = sqrt(quat(0));
    //     quat(1) = (C_23-C_32)/4/quat(0);
    //     quat(2) = (C_31-C_13)/4/quat(0);
    //     quat(3) = (C_12-C_21)/4/quat(0);
    //     break;
    // case 1:
    //     quat(1) = sqrt(quat(1));
    //     quat(0) = (C_23-C_32)/4/quat(1);
    //     quat(2) = (C_12+C_21)/4/quat(1);
    //     quat(3) = (C_31+C_13)/4/quat(1);
    //     break;
    // case 2:
    //     quat(2) = sqrt(quat(2));
    //     quat(0) = (C_31-C_13)/4/quat(2);
    //     quat(1) = (C_12+C_21)/4/quat(2);
    //     quat(3) = (C_23+C_32)/4/quat(2);
    //     break;
    // case 3:
    //     quat(3) = sqrt(quat(3));
    //     quat(0) = (C_12-C_21)/4/quat(3);
    //     quat(1) = (C_31+C_13)/4/quat(3);
    //     quat(2) = (C_23+C_32)/4/quat(3);
    //     break;
    // default:
    //     break;
    // }

    // std::cout << "Cuaterniones:" <<std::endl;
    // std::cout << quat[1] << std::endl;
    // std::cout << quat[2] << std::endl;
    // std::cout << quat[3] << std::endl;
    // std::cout << quat[0] << std::endl;
    // std::cout << "Yaw:" <<std::endl;
    // std::cout << eta_(3) << std::endl;

    /* update ROS Messages */

    accelerations_.linear.x = nu_dot_(0);
    accelerations_.linear.y = nu_dot_(1);
    accelerations_.linear.z = nu_dot_(2);
    accelerations_.angular.x = nu_dot_(0);
    accelerations_.angular.y = nu_dot_(1);
    accelerations_.angular.z = nu_dot_(2);

    velocities_.linear.x = nu_(0);
    velocities_.linear.y = nu_(1);
    velocities_.linear.z = nu_(2);
    velocities_.angular.x = nu_(3);
    velocities_.angular.y = nu_(4);
    velocities_.angular.z = nu_(5);
    
    eta_pose_.x = eta_(0);
    eta_pose_.y = eta_(1);
    eta_pose_.z = eta_(2);
    eta_pose_.phi = eta_(3);
    eta_pose_.theta = eta_(4);
    eta_pose_.psi = eta_(5);
}