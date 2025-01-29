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

VTEC_SDC1_1DOF_PID::VTEC_SDC1_1DOF_PID(const PIDParameters &params, std::shared_ptr<VTecSDC1DynamicModel> model)
    : PIDLin(params.kUMax, params.kUMin, params), sdc1_model_(model) {}

VTEC_SDC1_1DOF_PID::~VTEC_SDC1_1DOF_PID() {}

void VTEC_SDC1_1DOF_PID::updateNonLinearFunctions() {
  PIDLin::f_x_ = sdc1_model_->f_(0);
  PIDLin::g_x_ = sdc1_model_->g_(0);
}

double VTEC_SDC1_1DOF_PID::calculateControlSignals(double chi1, double chi1_d, double chi1_dot_d) {
  // Only in the case of the car, the next condition must be considered, as
  // achieving reverse is not done by computing negative control signals. This
  // must not be programed in any of the base controllers classes, as in the
  // case of the boat and submarine, reverse is straightforward
  return std::max(calculateManipulations(chi1, chi1_d, chi1_dot_d), 0.);
}

void VTEC_SDC1_1DOF_PID::updateControlSignals() {
  sdc1_model_->u_(0) = PIDLin::u_;
}