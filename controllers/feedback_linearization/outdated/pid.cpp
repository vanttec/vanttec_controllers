/** ----------------------------------------------------------------------------
 * @file: pid.cpp
 * @date: July 30, 2020
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Single DOF PID Controller class.
 * -----------------------------------------------------------------------------
 * */

#include "pid.hpp"

PID::PID(const float sample_time, const float k_p, const float k_i, const float k_d, const float u_max, const DOFControllerType_E& type)
{
    sample_time_ = sample_time;
    k_p_ = k_p;
    k_i_ = k_i;
    k_d_ = k_d;
    controller_type_ = type;
    U_MAX_ = u_max;
    f_ = 0;
    g_ = 0;
    u_ = 0;
    a_ = 0;
    u_aux_ = 0;
    set_point_ = 0;
    error_ = 0;
    prev_error_ = 0;
}

PID::~PID(){}

void PID::updateFunctions(const float f, const float g){
    f_ = f;
    g_ = g;
}

void PID::updateSetpoint(const float set_point, const float a)
{
    set_point_ = set_point;
    a_ = a;
}

void PID::calculateManipulation(const float current_value)
{
    prev_error_    = error_;
    error_         = set_point_ - current_value;

    if (controller_type_ == ANGULAR_DOF)
    {
        if (std::abs(error_) > M_PI)
        {
            error_ = (error_ / std::abs(error_)) * (std::abs(error_) - 2 * M_PI);
        }
    }

    const float error_d = (error_ - prev_error_) / sample_time_;
    const float error_i = ((error_ + prev_error_) / 2 * sample_time_) + error_;

    // manipulation_  = (1 / g_x) * (-f_x + k_p_ * error_ + k_i_ * error_i + k_d_ * error_d);
    u_aux_  = k_p_ * error_ + k_i_ * error_i + k_d_ * error_d;
    if(std::isnormal(g_)){
        u_ = (-f_ + a_ + u_aux_)/g_;
    }
}