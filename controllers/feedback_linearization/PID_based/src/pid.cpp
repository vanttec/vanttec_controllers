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

PID::PID(const float& sample_time, const float& k_p, const float& k_i, const float& k_d, const DOFControllerType_E& type)
{
    sample_time_     = sample_time;
    k_p_               = k_p;
    k_i_               = k_i;
    k_d_               = k_d;

    error_             = 0;
    prev_error_        = 0;
    set_point_         = 0;
    manipulation_      = 0;
    
    // f_x               = 0;
    // g_x               = 0;

    controller_type_   = type;
}

PID::~PID(){}

void PID::calculateManipulation(const float& current_value)
{
    prev_error_    = error_;
    error_         = set_point_ - current_value;

    if (controller_type_ == ANGULAR_DOF)
    {
        if (std::abs(error_) > M_PI)
        {
            // error_ = error_ - ((error_ / std::abs(error_)) * 2 * M_PI);
            error_ = (error_ / std::abs(error_)) * (std::abs(error_) - 2 * M_PI);
        }
    }

    const float error_d       = (error_ - prev_error_) / sample_time_;
    const float error_i       = ((error_ + prev_error_) / 2 * sample_time_) + error_;

    // manipulation_  = (1 / g_x) * (-f_x + k_p_ * error_ + k_i_ * error_i + k_d_ * error_d);
    manipulation_  = k_p_ * error_ + k_i_ * error_i + k_d_ * error_d;
}

void PID::updateSetPoint(const float& set_point)
{
    set_point_ = set_point;
}