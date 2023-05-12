/** ----------------------------------------------------------------------------
 * @file: el_rasho.hpp
 * @date: November 29, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * @author: Montserrat Cruz
 * 
 * @brief: Description of 3-DOF car model in the non-inertial frame with
           Euler Angles for Applied Robotics class (con Montserrat). 
           The nonlinear bycicle model was used.
           This model works for low speed movement and considers no
           longitudinal slip.
 * -----------------------------------------------------------------------------
 **/

#ifndef __CAFE__
#define __CAFE__

#include "car_3dof_dynamic_model.hpp"

class Cafe : public CarDynamicModel {
    public:
        /* Control inputs */
        float B_;           // Steering command
        float D_;           // Throttle command

        /* Constructor and destructor */
        Cafe(const float sample_time);
        ~Cafe();
        
        /* Class methods */
        // void calculateControlInputs();
};

#endif