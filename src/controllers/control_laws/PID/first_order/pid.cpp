/** ----------------------------------------------------------------------------
 * @file: pid.cpp
 * @date: April 26, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: Single DOF First Order PID Controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/PID/first_order/pid.hpp"
#include <algorithm>
#include <cmath>

PID::PID(const PIDParameters &params)
{ 
  params_ = params;
}

double PID::update(double chi1, double chi1_d)
{
  double error = chi1_d - chi1;

  if (params_.controller_type == ANGULAR_DOF)
      if (std::abs(error) > M_PI)
          error = (error / std::abs(error)) * (std::abs(error) - 2 * M_PI);

  double d = (error - prev_error_) / params_.kDt;
  double i = ((error + prev_error_) / 2 * params_.kDt) + error;
  prev_error_ = error;

  double u = params_.kP * error + params_.kI * i + params_.kD * d;

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

double PID::updateSaturated(double chi1, double chi1_d)
{
  return std::clamp(update(chi1, chi1_d), params_.kUMin, params_.kUMax);
}