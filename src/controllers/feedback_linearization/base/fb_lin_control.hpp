/** ----------------------------------------------------------------------------
 * @file: fb_lin_control.cpp
 * @date: August 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: 1-DOF Feedback linearization class definition for any controller.
 * -----------------------------------------------------------------------------
 * */


#ifndef __FBLIN__
#define __FBLIN__

#include <algorithm>

class FBLin
{
    public:
        double f_x_{0};
        double g_x_{0};

        double u_{0};
        double U_MAX_{255};
        double U_MIN_{-255};
        double u_aux_{0};
        double u_n_{0};
        double chiX_dot_d_{0};

        // u_ = g_x_^(-1)*(chi1_dot_d - f_x_ + u_n - u_aux)
        FBLin(double FB_LIN_UMAX, double FB_LIN_UMIN);
        ~FBLin();

        void updateControlSignal();
};

#endif