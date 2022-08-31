/** ----------------------------------------------------------------------------
 * @file: 6dof_antsmc.cpp
 * @date: August 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF adaptive non-singular terminal sliding mode controller class
 * -----------------------------------------------------------------------------
 * */

#include "6dof_antsmc.hpp"

ANTSMC6DOF::ANTSMC6DOF(const float sample_time_s, const float alpha[6], const float beta[6], const float K2[6], const float K_alpha[6], const float K_min[6], const float K1_init[6], const float mu[6])
                    : ANTSMC_x(sample_time_s, alpha[0], beta[0], K2[0], K_alpha[0], K_min[0], K1_init[0], mu[0], LINEAR_DOF)
                    , ANTSMC_y(sample_time_s, alpha[1], beta[1], K2[1], K_alpha[1], K_min[1], K1_init[1], mu[1], LINEAR_DOF)
                    , ANTSMC_z(sample_time_s, alpha[2], beta[2], K2[2], K_alpha[2], K_min[2], K1_init[2], mu[2], LINEAR_DOF)
                    , ANTSMC_phi(sample_time_s, alpha[3], beta[3], K2[3], K_alpha[3], K_min[3], K1_init[3], mu[3], ANGULAR_DOF)
                    , ANTSMC_theta(sample_time_s, alpha[4], beta[4], K2[4], K_alpha[4], K_min[4], K1_init[4], mu[4], ANGULAR_DOF)
                    , ANTSMC_psi(sample_time_s, alpha[5], beta[5], K2[5], K_alpha[5], K_min[5], K1_init[5], mu[5], ANGULAR_DOF)
{
    u.resize(6,1);              // Control
    ua.resize(6,1);             // Auxiliary Control
    delta.resize(6,1);
    f.resize(6,1);
    g.resize(6,6);
    q_dot_dot.resize(6,1);

    u << 0,
         0,
         0,
         0,
         0,
         0;

    ua << 0,
          0,
          0,
          0,
          0,
          0;

    delta << 0,
             0,
             0,
             0,
             0,
             0;

    q_dot_dot << 0,
                   0,
                   0,
                   0,
                   0,
                   0;

    g = Eigen::MatrixXf::Zero(6,6);

    f << 0,
         0,
         0,
         0,
         0,
         0;

    MAX_FORCE_X = 0.0;
    MAX_FORCE_Y = 0.0;
    MAX_FORCE_Z = 0.0;
    MAX_TORQUE_K = 0.0;
    MAX_TORQUE_M = 0.0;
    MAX_TORQUE_N = 0.0;

    thrust.tau_x = 0;
    thrust.tau_y = 0;
    thrust.tau_z = 0;
    thrust.tau_phi = 0;
    thrust.tau_theta = 0;
    thrust.tau_psi = 0;
    
    // functs_arrived = 0;
}

ANTSMC6DOF::~ANTSMC6DOF(){}

void ANTSMC6DOF::SetMaxThrust(const float* MAX_TAU){
    MAX_FORCE_X = MAX_TAU[0];
    MAX_FORCE_Y = MAX_TAU[1];
    MAX_FORCE_Z = MAX_TAU[2];
    MAX_TORQUE_K = MAX_TAU[3];
    MAX_TORQUE_M = MAX_TAU[4];
    MAX_TORQUE_N = MAX_TAU[5];
}


void ANTSMC6DOF::UpdateSetPoints(const vanttec_msgs::EtaPose& q_d)//, const vanttec_msgs::EtaPose& q_dot_d)
{
    ANTSMC_x.UpdateSetPoint(q_d.x, 0.0);        //, q_dot_d.x, 0.0);
    ANTSMC_y.UpdateSetPoint(q_d.y, 0.0);        //, q_dot_d.y, 0.0);
    ANTSMC_z.UpdateSetPoint(q_d.z, 0.0);        //, q_dot_d.z, 0.0);
    ANTSMC_phi.UpdateSetPoint(q_d.phi, 0.0);        //, q_dot_d.phi);
    ANTSMC_theta.UpdateSetPoint(q_d.theta, 0.0);        //, q_dot_d.theta);
    ANTSMC_psi.UpdateSetPoint(q_d.psi, 0.0);        //, q_dot_d.psi);
}

void ANTSMC6DOF::UpdatePose(const vanttec_msgs::EtaPose& q)//, const vanttec_msgs::EtaPose& q_dot)
{
    ANTSMC_x.CalculateAuxControl(q.x, 0.0);     //, q_dot.x);
    ANTSMC_y.CalculateAuxControl(q.y, 0.0);     //, q_dot.y);
    ANTSMC_z.CalculateAuxControl(q.z, 0.0);     //, q_dot.z);
    ANTSMC_phi.CalculateAuxControl(q.phi, 0.0);     //, q_dot.phi);
    ANTSMC_theta.CalculateAuxControl(q.theta, 0.0);     //, q_dot.theta);
    ANTSMC_psi.CalculateAuxControl(q.psi, 0.0);     //, q_dot.psi);
}

void ANTSMC6DOF::CalculateManipulation()
{
    Eigen::FullPivLU<Eigen::MatrixXf> g_lu(g);
    // if(functs_arrived){
    if (g_lu.isInvertible()) {
        ua << ANTSMC_x._ua,
              ANTSMC_y._ua,
              ANTSMC_z._ua,
              ANTSMC_phi._ua,
              ANTSMC_theta._ua,
              ANTSMC_psi._ua;
        delta << ANTSMC_x._delta,
                 ANTSMC_y._delta,
                 ANTSMC_z._delta,
                 ANTSMC_phi._delta,
                 ANTSMC_theta._delta,
                 ANTSMC_psi._delta;
        u << g.inverse()*(q_dot_dot - f + delta - ua);

        // Saturate for maximum thrust in each degree of freedom
        if(fabs(u(0)) > MAX_FORCE_X)  u(0) = u(0) / fabs(u(0)) * MAX_FORCE_X;
        if(fabs(u(1)) > MAX_FORCE_Y)  u(1) = u(1) / fabs(u(1)) * MAX_FORCE_Y;
        if(fabs(u(2)) > MAX_FORCE_Z)  u(2) = u(2) / fabs(u(2)) * MAX_FORCE_Z;
        if(fabs(u(3)) > MAX_TORQUE_K) u(3) = u(3) / fabs(u(3)) * MAX_TORQUE_K;
        if(fabs(u(4)) > MAX_TORQUE_M) u(4) = u(4) / fabs(u(4)) * MAX_TORQUE_M;
        if(fabs(u(5)) > MAX_TORQUE_N) u(5) = u(5) / fabs(u(5)) * MAX_TORQUE_N;
    
        thrust.tau_x = u(0);
        thrust.tau_y = u(1);
        thrust.tau_z = u(2);
        thrust.tau_phi = u(3);
        thrust.tau_theta = u(4);
        thrust.tau_psi = u(5);
    }
}

void ANTSMC6DOF::UpdateDynamics(const vanttec_msgs::SystemDynamics& _non_linear_functions)
{
    uint8_t stride =  (uint8_t) _non_linear_functions.g.layout.dim[0].stride;
    uint8_t offset =  (uint8_t) _non_linear_functions.g.layout.data_offset;

    for(int i=0; i<6; ++i){
        for(int j=0; j<6; ++j){
            g(i,j) = _non_linear_functions.g.data[offset + i*stride + j];
        }
    }

    f << _non_linear_functions.f[0],
         _non_linear_functions.f[1],
         _non_linear_functions.f[2],
         _non_linear_functions.f[3],
         _non_linear_functions.f[4],
         _non_linear_functions.f[5];

    // std::cout << "f:" << f << std::endl;
    // std::cout << "g:" << g << std::endl;

    // functs_arrived = 1;
}
