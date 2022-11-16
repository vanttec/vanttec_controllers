/** ----------------------------------------------------------------------------
 * @file: 6dof_pid.cpp
 * @date: April 10, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: 6-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#include "6dof_pid.hpp"

PID6DOF::PID6DOF(const float &sample_time, const float *k_p, const float *k_i, const float *k_d, const DOFControllerType_E *type) : 
                    PID_x_    (sample_time, k_p[0], k_i[0], k_d[0], type[0]),
                    PID_y_    (sample_time, k_p[1], k_i[1], k_d[1], type[1]),
                    PID_z_    (sample_time, k_p[2], k_i[2], k_d[2], type[2]),
                    PID_phi_  (sample_time, k_p[3], k_i[3], k_d[3], type[3]),
                    PID_theta_(sample_time, k_p[4], k_i[4], k_d[4], type[4]),
                    PID_psi_  (sample_time, k_p[5], k_i[5], k_d[5], type[5])
{

    u_.resize(6,1);              // Control
    ua_.resize(6,1);             // Auxiliary Control
    f_.resize(6,1);
    g_.resize(6,6);
    ref_dot_dot_.resize(6,1);

    u_ << 0,
         0,
         0,
         0,
         0,
         0;

    ua_ << 0,
         0,
         0,
         0,
         0,
         0;

    ref_dot_dot_ << 0,
                   0,
                   0,
                   0,
                   0,
                   0;

    g_ = Eigen::MatrixXf::Zero(6,6);

    f_ << 0,
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

    thrust_.tau_x = 0;
    thrust_.tau_y = 0;
    thrust_.tau_z = 0;
    thrust_.tau_phi = 0;
    thrust_.tau_theta = 0;
    thrust_.tau_psi = 0;
    
}

PID6DOF::~PID6DOF(){};


void PID6DOF::setMaxThrust(const float* MAX_TAU){
    MAX_FORCE_X_ = MAX_TAU[0];
    MAX_FORCE_Y_ = MAX_TAU[1];
    MAX_FORCE_Z_ = MAX_TAU[2];
    MAX_TORQUE_K_ = MAX_TAU[3];
    MAX_TORQUE_M_ = MAX_TAU[4];
    MAX_TORQUE_N_ = MAX_TAU[5];
}

void PID6DOF::updateSetPoints(const vanttec_msgs::EtaPose& set_points)
{
    PID_x_.updateSetPoint(set_points.x);
    PID_y_.updateSetPoint(set_points.y);
    PID_z_.updateSetPoint(set_points.z);
    PID_phi_.updateSetPoint(set_points.phi);
    PID_theta_.updateSetPoint(set_points.theta);
    PID_psi_.updateSetPoint(set_points.psi);
}

void PID6DOF::updatePose(const vanttec_msgs::EtaPose& eta_)
{
    PID_x_.calculateManipulation(eta_.x);
    PID_y_.calculateManipulation(eta_.y);
    PID_z_.calculateManipulation(eta_.z);
    PID_phi_.calculateManipulation(eta_.phi);
    PID_theta_.calculateManipulation(eta_.theta);
    PID_psi_.calculateManipulation(eta_.psi);
}

void PID6DOF::calculateManipulations()
{
    Eigen::FullPivLU<Eigen::MatrixXf> glu(g_);
    // if (functs_arrived) {
    if (glu.isInvertible()) {
        ua_ << PID_x_.manipulation_,
            PID_y_.manipulation_,
            PID_z_.manipulation_,
            PID_phi_.manipulation_,
            PID_theta_.manipulation_,
            PID_psi_.manipulation_;

        u_ << g_.inverse()*(ref_dot_dot_ - f_ + ua_);


        // Saturate for maximum thrust in each degree of freedom
        if(std::fabs(u_(0)) > MAX_FORCE_X_)  u_(0) = u_(0) / std::fabs(u_(0)) * MAX_FORCE_X_;
        if(std::fabs(u_(1)) > MAX_FORCE_Y_)  u_(1) = u_(1) / std::fabs(u_(1)) * MAX_FORCE_Y_;
        if(std::fabs(u_(2)) > MAX_FORCE_Z_)  u_(2) = u_(2) / std::fabs(u_(2)) * MAX_FORCE_Z_;
        if(std::fabs(u_(3)) > MAX_TORQUE_K_) u_(3) = u_(3) / std::fabs(u_(3)) * MAX_TORQUE_K_;
        if(std::fabs(u_(4)) > MAX_TORQUE_M_) u_(4) = u_(4) / std::fabs(u_(4)) * MAX_TORQUE_M_;
        if(std::fabs(u_(5)) > MAX_TORQUE_N_) u_(5) = u_(5) / std::fabs(u_(5)) * MAX_TORQUE_N_;
    
        thrust_.tau_x = u_(0);
        thrust_.tau_y = u_(1);
        thrust_.tau_z = u_(2);
        thrust_.tau_phi = u_(3);
        thrust_.tau_theta = u_(4);
        thrust_.tau_psi = u_(5);
    }
}

void PID6DOF::updateDynamics(const vanttec_msgs::SystemDynamics& non_linear_functions)
{
    uint8_t stride =  (uint8_t) non_linear_functions.g.layout.dim[0].stride;
    uint8_t offset =  (uint8_t) non_linear_functions.g.layout.data_offset;

    for(int i=0; i<6; ++i){
        for(int j=0; j<6; ++j){
            g_(i,j) = non_linear_functions.g.data[offset + i*stride + j];
        }
    }

    // g << non_linear_functions.g[0][0], non_linear_functions.g[0][1], non_linear_functions.g[0][2], non_linear_functions.g[0][3], non_linear_functions.g[0][4], non_linear_functions.g[0][5],
    //      non_linear_functions.g[1][0], non_linear_functions.g[1][1], non_linear_functions.g[1][2], non_linear_functions.g[1][3], non_linear_functions.g[1][4], non_linear_functions.g[1][5],
    //      non_linear_functions.g[2][0], non_linear_functions.g[2][1], non_linear_functions.g[2][2], non_linear_functions.g[2][3], non_linear_functions.g[2][4], non_linear_functions.g[2][5],
    //      non_linear_functions.g[3][0], non_linear_functions.g[3][1], non_linear_functions.g[3][2], non_linear_functions.g[3][3], non_linear_functions.g[3][4], non_linear_functions.g[3][5],
    //      non_linear_functions.g[4][0], non_linear_functions.g[4][1], non_linear_functions.g[4][2], non_linear_functions.g[4][3], non_linear_functions.g[4][4], non_linear_functions.g[4][5],
    //      non_linear_functions.g[5][0], non_linear_functions.g[5][1], non_linear_functions.g[5][2], non_linear_functions.g[5][3], non_linear_functions.g[5][4], non_linear_functions.g[5][5];

    f_ << non_linear_functions.f[0],
         non_linear_functions.f[1],
         non_linear_functions.f[2],
         non_linear_functions.f[3],
         non_linear_functions.f[4],
         non_linear_functions.f[5];

    // std::cout << "f:" << f_ << std::endl;
    // std::cout << "g:" << g << std::endl;
}