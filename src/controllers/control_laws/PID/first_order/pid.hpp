/** ----------------------------------------------------------------------------
 * @file: pid.hpp
 * @date: April 26, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: Single DOF First Order PID Controller class.
 * -----------------------------------------------------------------------------
 * */

#pragma once

#include <limits>

struct PIDParameters {
    double kP{0}, kI{0}, kD{0};
    double kDt{0.01};
    
    double kUMax{std::numeric_limits<double>::max()};
    double kUMin{std::numeric_limits<double>::min()};

    bool enable_ramp_rate_limit{false};
    double ramp_rate{1}; // 
};

class PID {
public:
  PID(const PIDParameters &params);

  double update(double measurement, double desired);

private:
  PIDParameters params_;
  double prev_error_{0};

  double set_u_{0}; // Used to limit ramp rate.
};
