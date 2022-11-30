/** ----------------------------------------------------------------------------
 * @file: asmc_guidance.cpp
 * @date: June 20, 2021
 * @author: Carlos Medina
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ASMC Waypoint Guidance Class
 * -----------------------------------------------------------------------------
 * */

#include "asmc_guidance.hpp"

ASMC_GUIDANCE::ASMC_GUIDANCE(double sample_time_, const double _Ka,  const double K2_, const double _Kalpha, const double _Kmin, const double _miu, const DOFControllerType_E _type)
                            :ASMC(sample_time_,K2_,_Kalpha,_Kmin,_miu,_type)
{
    Ka = _Ka;
    error_i = 0;
    u_ = 0;
    Uax = 0;
    desired_dot_error = 0;
}

ASMC_GUIDANCE::~ASMC_GUIDANCE(){}

void ASMC_GUIDANCE::reset()
{
    error1 = 0.0;
    prev_error_1_ = 0.0;
    error2 = 0.0;
    prev_error_2_ = 0.0;
    error_i = 0.0;
    prev_error_i = 0.0;
    Uax = 0.0;
}

void ASMC_GUIDANCE::manipulation_(double _current)
{
    double sign = 0.0;
    prev_error_1_ = error1;
    prev_error_2_ = error2;
    prevdot_K1_ = dotK1_;

    error1 = set_point - _current_pos;
    error2 = 0.0       - _current_vel;

    if (controller_type == ANGULAR_DOF)
    {
        if (std::abs(error1) > PI)
        {
            error11 = (error1 / std::abs(error1)) * (std::abs(error1) - 2 * PI);
        }
        if (std::abs(error2) > PI)
        {
            error2 = (error2 / std::abs(error2)) * (std::abs(error2) - 2 * PI);
        }
    }

    s = error2 + lambda*error1;
    if (std::abs(s) - mu != 0.0)
    {
        sign = (s - mu) / (std::abs(s) - mu);
    } else
    {
        sign = 0;
    }
    dotK1_ = K1>K_min ?  K_alpha*common::sign:K_min;
    K1 += (dotK1_+prevdot_K1_)/2*sample_time_;
    ua = -K1*common::sign - K2*s;
}