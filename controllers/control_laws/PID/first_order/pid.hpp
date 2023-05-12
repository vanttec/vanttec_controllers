/** ----------------------------------------------------------------------------
 * @file: pid.hpp
 * @date: April 26, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Single DOF First Order PID Controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __PID_H__
#define __PID_H__

#include "utils/utils.hpp"

class PID
{
    private:
        float sample_time_;
        
        float error_;
        float prev_error_;

        float k_p_;
        float k_i_;
        float k_d_;

        float U_MAX_;

        DOFControllerType_E controller_type_;
        
        PID(float sample_time, float k_p, float k_i, float k_d, float u_max, const DOFControllerType_E& type);
        ~PID();
        
    public:
        float u_;
        float chi1_d_;

        void updateReferences(float chi1_d);
        void calculateManipulation(float chi1);
        void saturateManipulation(float chi1);
};

#endif