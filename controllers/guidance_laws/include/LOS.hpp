/** ----------------------------------------------------------------------------
 * @file: LOS.hpp
 * @date: November 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: LOS class
 * -----------------------------------------------------------------------------
 * */

#ifndef __LOS__
#define __LOS__

#include <stdio.h>
#include <ros/console.h>
#include "vanttec_msgs/EtaPose.h"

class LOS
{
    public:
        float DELTA_MAX_;       // Max steering
        float delta_;           // Desired steering
        float psi_;             // Current heading
        float e_;               // Crosstrack error
        float vel_;             // velocity vector norm
        float ak_;              // path angle
        float beta_;            // desired heading
        float kappa_;           // look ahead distance
        float KAPPA_MAX_;           // look ahead distance
        vanttec_msgs::EtaPose vehicle_pose_;

        LOS(float delta_max, float kappa);
        ~LOS();

        void calculateCrosstrackError(float x0, float y0, float x1, float y1);
        void setHeading(const vanttec_msgs::EtaPose& pose);
        void calculateSteering(float vel, float L);
};

#endif