/** ----------------------------------------------------------------------------
 * @file: asmc.hpp
 * @date: June 17, 2021
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ASMC_H__
#define __ASMC_H__

#include "common.hpp"
#include "vanttec_msgs/EtaPose.h"
#include <cmath>

class ASMC
{
    private:
        float sample_time_;
        float q_d_;         // Setpoint
        float q_dot_d_;
        float error_1_;
        float error_2_;
        float prev_error_1_;
        float prev_error_2_;
        
        // Auxiliar control
        float ua_;

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
        ASMC(const float& sample_time, const float& lambda,  const float& K2, const float& K_alpha, const float& K1_init, const float& K_min, const float& mu, const DOFControllerType_E& type);

        // Destructor
        ~ASMC();

        void reset();
        void updateSetPoint(const float& q_d, const float& q_dot_d);
        void calculateAuxControl(const float& q, const float& q_dot);
        
        friend class ASMC6DOF;
};

#endif