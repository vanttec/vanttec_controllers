/** ----------------------------------------------------------------------------
 * @file: vtec_u4_6dof_in_pid.cpp
 * @date: April 27, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a 6DOF PID Controller for the VTec U4 inertial dyn model
 * -----------------------------------------------------------------------------
 **/

#include "controllers/feedback_linearization/model_based_controllers/UUVs/vtec_u4_6dof_in_pid.hpp"

VTEC_U4_6DOF_PID::VTEC_U4_6DOF_PID(float sample_time, const std::vector<float>& k_p, const std::vector<float>& k_i, 
        const std::vector<float>& k_d, const std::array<float,6>& u_max,
        const std::array<DOFControllerType_E,6>& type) :
        VTecU4InDynamicModel(sample_time), PID6DOF(sample_time, k_p, k_i, k_d, u_max, type)
{}

VTEC_U4_6DOF_PID::~VTEC_U4_6DOF_PID(){}

void VTEC_U4_6DOF_PID::updateNonLinearFunctions()
{
    PID6DOF::f_x_ = VTecU4InDynamicModel::f_x_;
    PID6DOF::g_x_ = VTecU4InDynamicModel::g_x_;
}

void VTEC_U4_6DOF_PID::calculateControlSignals()
{
    std::array<float,6> chi1, chi2;

    chi1[0] = eta_(0);
    chi1[1] = eta_(1);
    chi1[2] = eta_(2);
    chi1[3] = eta_(3);
    chi1[4] = eta_(4);
    chi1[5] = eta_(5);

    chi1[0] = eta_dot_(0);
    chi1[1] = eta_dot_(1);
    chi1[2] = eta_dot_(2);
    chi1[3] = eta_dot_(3);
    chi1[4] = eta_dot_(4);
    chi1[5] = eta_dot_(5);

    calculateManipulations(chi1, chi2);
}

void VTEC_U4_6DOF_PID::updateControlSignals()
{
    VTecU4InDynamicModel::tau_ = PID6DOF::u_;
}