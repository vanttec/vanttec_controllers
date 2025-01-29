/** ----------------------------------------------------------------------------
 * @file: pid.hpp
 * @date: April 26, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: Single DOF PID Second Order Controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/PID/second_order/pid.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

// PID::PID(float sample_time,float k_p,float k_i,float k_d, float u_max, const DOFControllerType_E& type)
// {
//     k_p_            = k_p;
//     k_i_            = k_i;
//     k_d_            = k_d;

//     error_          = 0;
//     prev_error_     = 0;
//     chi1_d_         = 0;
//     chi2_d_         = 0;
//     u_              = 0;

//     U_MAX_ = u_max;

//     controller_type_   = type;
// }

PID::PID(const PIDParameters &params)
{ 
  params_ = params;
}

PID::~PID(){}


double PID::update(double chi1, double chi2, double chi1_d, double chi2_d)
{
    double error = chi1_d - chi1;
    double error_d = chi2_d - chi2;

    if (params_.controller_type == ANGULAR_DOF)
        if (std::abs(error) > M_PI)
            // error = error - ((error / std::abs(error)) * 2 * M_PI);
            error = (error / std::abs(error)) * (std::abs(error) - 2 * M_PI);

    double i = ((error + prev_error_) / 2 * params_.kDt) + error;
    prev_error_ = error;

    double u = params_.kP * error + params_.kI * i + params_.kD * error_d;

  // If ramp rate is disabled, or if we are within ramp rate, go to U.
  if (!params_.enable_ramp_rate_limit ||
      std::abs((set_u_ - u)) < params_.ramp_rate * params_.kDt) {
        set_u_ = u;
  } else {
    // Ramp rate is enabled, and we can only increase by ramp rate.
    set_u_ += std::copysign(params_.ramp_rate * params_.kDt, u - set_u_);
  }

  return set_u_;
}

void PID::updateSaturated(double chi1, double chi2, double chi1_d, double chi2_d)
{
  return std::clamp(update(chi1, chi2, chi1_d, chi2_d), params_.kUMin, params_.kUMax);
}