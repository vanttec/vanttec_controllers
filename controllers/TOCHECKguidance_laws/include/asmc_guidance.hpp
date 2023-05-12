/** ----------------------------------------------------------------------------
 * @file: asmc_guidance.hpp
 * @date: June 20, 2021
 * @author: Carlos Medina
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ASMC Waypoint Guidance Class
 * -----------------------------------------------------------------------------
 * */


#ifndef __UUV_ASMC_GUIDANCE_H__
#define __UUV_ASMC_GUIDANCE_H__

#include "asmc.hpp"

class ASMC_GUIDANCE : public ASMC
{
    public:
        double Ka;
        double error_i;
        double prev_error_i;
        double desired_dot_error;
        double U;
        double Uax;

        // Constructor
        ASMC_GUIDANCE(double sample_time_, const double _Ka, const double K2_, const double _Kalpha, const double _Kmin, const double _miu, const DOFControllerType_E _type);

        // Destructor
        ~ASMC_GUIDANCE();

        void reset();
        void manipulation_(double _current);
};

#endif