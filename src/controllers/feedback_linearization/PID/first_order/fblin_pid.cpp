/** ----------------------------------------------------------------------------
 * @file: fblin_pid.cpp
 * @date: August 13, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 1-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/feedback_linearization/PID/first_order/fblin_pid.hpp"
#include <iostream>

PIDLin::PIDLin(double FB_LIN_UMAX, double FB_LIN_UMIN, const PIDParameters &params) : 
               FBLin (FB_LIN_UMAX, FB_LIN_UMIN),
               control_law_ (params)
{}

PIDLin::~PIDLin(){};

double PIDLin::calculateManipulations(double chi1, double chi1_d, double chi1_dot_d)
{
    chiX_dot_d_ = chi1_dot_d;
    u_aux_ = -control_law_.update(chi1, chi1_d);
    updateControlSignal();
    return u_;
}