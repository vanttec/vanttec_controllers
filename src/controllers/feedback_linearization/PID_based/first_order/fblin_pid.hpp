/** ----------------------------------------------------------------------------
 * @file: fblin_pid.hpp
 * @date: August 13, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: 1-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __FBLIN_PID_H__
#define __FBLIN_PID_H__

#include "controllers/control_laws/PID/first_order/pid.hpp"
#include "controllers/feedback_linearization/base/fb_lin_control.hpp"
#include "utils/utils.hpp"

class PIDLin : public FBLin {
public:
  PIDLin(const PIDParameters &params);
  ~PIDLin();

  double calculateManipulations(double surge, double surge_d);

private:
  PID control_law_;
};

#endif