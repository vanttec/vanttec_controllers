/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_pid.cpp
 * @date: August 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: Description of a single DOF PID Controller for the VTec SDC1 dynamic
 *model
 * -----------------------------------------------------------------------------
 **/

#include "vtec_sdc1_pid.hpp"

VTEC_SDC1_1DOF_PID::VTEC_SDC1_1DOF_PID(const PIDParameters &params)
    : VTecSDC1DynamicModel(params.kDt, 255),
      PIDLin(params) {}

VTEC_SDC1_1DOF_PID::~VTEC_SDC1_1DOF_PID() {}

void VTEC_SDC1_1DOF_PID::updateNonLinearFunctions(double f_x, double g_x) {
  PIDLin::f_x_ = f_x;
  PIDLin::g_x_ = g_x;
}

// void VTEC_SDC1_1DOF_PID::calculateControlSignals() {
//   calculateManipulations(nu_(0));
// }

double VTEC_SDC1_1DOF_PID::calculateControlSignals(double surge, double surge_d) {
  return std::clamp(calculateManipulations(surge, surge_d), 0., 255.);
}

void VTEC_SDC1_1DOF_PID::updateControlSignals() {
  VTecSDC1DynamicModel::u_(0) = PIDLin::u_;
}

// void VTEC_SDC1_1DOF_PID::updateCurrentReference(float chi1_d,
//                                                 float chi1_dot_d) {
//   updateReferences(chi1_d, chi1_dot_d);
// }