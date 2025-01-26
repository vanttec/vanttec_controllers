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
#include <utils/utils.hpp>

class PID {
  public:
    PID(const PIDParameters &params);

    double update(double chi1, double chi1_d);

    // In model based controllers, you want to saturate the end computed control signal, not the auxiliar
    // control signal (PID in this case)
    // updateSaturated method is intended to be used in applications where a FBLin PID is not required,
    // as FBLin base classes already saturate the control signals.
    double updateSaturated(double chi1, double chi1_d);

  private:
    PIDParameters params_;
    double prev_error_{0};

    double set_u_{0}; // Used to limit ramp rate.
};
