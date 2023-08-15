/** ----------------------------------------------------------------------------
 * @file: asmc.hpp
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Second Order Adaptive Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ASMC_H__
#define __ASMC_H__

#include "utils/utils.hpp"
#include "vanttec_msgs/EtaPose.h"
#include <cmath>

class ASMC
{
    protected:
        float sample_time_;
        
        float error_;
        float prev_error_;

        /* Setpoint */
        float q_d_;
        float q_dot_d_;
        
        /* Auxiliar control */
        float u_;

        /* Sliding surface */
        float lambda_;
        float s_;

        /* Gains */
        float K1_;
        float K2_;
        float dot_K1_;
        float prev_dot_K1_;

        /* Adaptive law */
        float K_min_;
        float K_alpha_;
        float mu_;

        DOFControllerType_E controller_type_;

    public:

        // Constructor
        ASMC(float sample_time,float lambda, float K2,float K_alpha, float K1_init, float K_min,float mu, const DOFControllerType_E& type);

        // Destructor
        ~ASMC();

        void reset();
        void updateSetPoint(float q_d, float q_dot_d);
        void calculateManipulation(float q, float q_dot);
};

#endif