/** ----------------------------------------------------------------------------
 * @file: asmc.hpp
 * @date: June 17, 2021
 * @date: June 4, 2022
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: First Order Adaptive Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ASMC_H__
#define __ASMC_H__

#include "utils/utils.hpp"
#include "sdv_msgs/msg/eta_pose.hpp"
// #include "vanttec_msgs/EtaPose.h"
#include <cmath>

class ASMC
{
    private:
        float sample_time_;

        float q_d_;             // Setpoint
        float q_dot_d_;

        float error_1_;
        float error_2_;
        
        float prev_error_1_;
        float prev_error_2_;
        
        // Auxiliar control
        float u_;

        // Sliding surface
        float lambda_;
        float s_;

        // Gains
        float K1_;
        float K2_;
        float dot_K1_;
        float prev_dot_K1_;

        // Adaptive law
        float K_min_;
        float K_alpha_;
        float mu_;

        DOFControllerType_E controller_type_;
    public:
        // Constructor
        ASMC(float sample_time,float lambda, float K2,float K_alpha,float K1_init,float K_min,float mu, const DOFControllerType_E& type);

        // Destructor
        ~ASMC();

        void reset();
        void updateSetPoint(float q_d,float q_dot_d);
        void calculateAuxControl(float q,float q_dot);
        
        // friend class ASMC6DOF;
};

#endif