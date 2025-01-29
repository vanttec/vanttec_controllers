/** ----------------------------------------------------------------------------
 * @file: fb_lin_control.cpp
 * @date: August 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: 1-DOF Feedback linearization class definition for any controller.
 * -----------------------------------------------------------------------------
 * */

#include <cmath>

#include "controllers/feedback_linearization/base/fb_lin_control.hpp"

FBLin::FBLin(double FB_LIN_UMAX, double FB_LIN_UMIN)
        : U_MAX_(FB_LIN_UMAX), U_MIN_(FB_LIN_UMIN) {}

FBLin::~FBLin(){}

void FBLin::updateControlSignal(){

    if(std::isnormal(g_x_))
    {
        u_ = (chiX_dot_d_ - f_x_ + u_n_ - u_aux_)/g_x_;
    }

    // std::fabs(u_) > U_MAX_ ? u_ / std::fabs(u_) * U_MAX_ : u_;
    u_ = std::clamp(u_, U_MIN_, U_MAX_);
}