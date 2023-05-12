/** ----------------------------------------------------------------------------
 * @file: pid.hpp
 * @date: November 29, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Single DOF PID Controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __PID_H__
#define __PID_H__

#include "utils/utils.hpp"
#include "vanttec_msgs/EtaPose.h"
#include <std_msgs/Float32.h>
#include <cmath>

class PID
{
    public:
        float sample_time_;
        
        float f_;
        float g_;

        float u_;
        float a_;
        float u_aux_;

        float set_point_;
        float error_;
        float prev_error_;

        float k_p_;
        float k_i_;
        float k_d_;

        float U_MAX_;

        DOFControllerType_E controller_type_;
        
        PID(const float sample_time, const float k_p, const float k_i, const float k_d, const float u_max, const DOFControllerType_E& _type);
        ~PID();
        
        void updateFunctions(const float f, const float g);
        void updateSetpoint(const float set_point, const float a_);
        void calculateManipulation(const float _current_value);
};

#endif