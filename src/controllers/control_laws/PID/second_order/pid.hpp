/** ----------------------------------------------------------------------------
 * @file: pid.hpp
 * @date: April 26, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Single DOF PID Second Order Controller class.
 * -----------------------------------------------------------------------------
 * */

#pragma once

#include <limits>
#include <utils/utils.hpp>

class PID
{
    private:
        PIDParameters params_;
        double prev_error_{0};
        double set_u_{0}; // Used to limit ramp rate.
        
        // float error_;
        // float prev_error_;
        // float error_d_;

    public:
        // PID(float sample_time, float k_p, float k_i, float k_d, float u_max, const DOFControllerType_E& type);
        PID(const PIDParameters &params);
        ~PID();

        // void updateReferences(float chi1_d, float chi2_d);
        // void calculateManipulation(float chi1, float chi2);
        // void saturateManipulation(float chi1, float chi2);

        double update(double chi1, double chi2, double chi1_d, double chi2_d);

        // In model based controllers, you want to saturate the end computed control signal, not the auxiliar
        // control signal (PID in this case)
        // updateSaturated method is intended to be used in applications where a FBLin PID is not required,
        // as FBLin base classes already saturate the control signals.
        double updateSaturated(double chi1, double chi2, double chi1_d, double chi2_d);
};