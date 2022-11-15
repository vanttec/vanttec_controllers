/** ----------------------------------------------------------------------------
 * @file: pid.hpp
 * @date: July 30, 2020
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Single DOF PID Controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __PID_H__
#define __PID_H__

#include "common.hpp"
#include "vanttec_msgs/EtaPose.h"
#include <std_msgs/Float32.h>
#include <cmath>

class PID
{
    public:
        float sample_time_;
        
        float set_point_;
        float manipulation_;
        float error_;
        float prev_error_;

        float k_p_;
        float k_i_;
        float k_d_;
        
        // float f_x_;
        // float g_x_;

        DOFControllerType_E controller_type_;
        
        PID(const float& sample_time, const float& _k_p, const float& _k_i, const float& _k_d, const DOFControllerType_E& _type);
        ~PID();
        
        void updateSetPoint(const float& set_point);
        void calculateManipulation(const float& _current_value);
};

#endif