/** ----------------------------------------------------------------------------
 * @file: antsmc.hpp
 * @date: August 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ANTSMC_H__
#define __ANTSMC_H__

#include "common.hpp"
#include "vanttec_msgs/EtaPose.h"
#include <cmath>

class ANTSMC
{
    private:
        float _sample_time_s;
        float _q_d;         // Setpoint
        float _q_dot_d;
        float _error_1;
        float _error_2;
        float _prev_error_1;
        float _prev_error_2;
        
        // Auxiliar control
        float _ua;

        // Control parameters
        float _alpha;
        float _beta;
        float _s;
        float _delta;

        // Gains
        float _K1;
        float _K2;
        float _dot_K1;
        float _prev_dot_K1;

        // Adaptive law
        float _K_min;
        float _K_alpha;
        float _mu;

        DOFControllerType_E _controller_type;
    public:
        // Constructor
        ANTSMC(const float sample_time_s, const float alpha, const float beta, const float K2, const float K_alpha, const float K_min, const float K_min_init, const float mu, const DOFControllerType_E type);

        // Destructor
        ~ANTSMC();

        void Reset();
        void UpdateSetPoint(const float q_d, const float q_dot_d);
        void CalculateAuxControl(float q, float q_dot);
        float sign(const float e);
        float sig(const float e, const float a);

        friend class ANTSMC6DOF;
};

#endif