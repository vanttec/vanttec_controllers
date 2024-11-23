/** ----------------------------------------------------------------------------
 * @file: fblin_pid.cpp
 * @date: August 13, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 1-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/feedback_linearization/PID_based/first_order/fblin_pid.hpp"
#include <iostream>

PIDLin::PIDLin(const PIDParameters &params) : 
                FBLin (255),
                control_law_ (params)
{}

PIDLin::~PIDLin(){};

double PIDLin::calculateManipulations(double surge, double surge_d)
{
    // u_aux_ = -control_law_.update(surge, surge_d);
    u_aux_ = control_law_.update(surge, surge_d);
    std::cout << "u_aux_: " << u_aux_ << std::endl;

    // u_aux_ = -control_law_.u_;

    updateControlSignal();
    return u_;
}
