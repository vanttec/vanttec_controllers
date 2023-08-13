/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1.hpp
 * @date: August 10, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * @author: Montserrat Cruz
 * 
 * @brief: Description of 3-DOF VTec Self-Driving Car model in the non-inertial frame with
           Euler Angles for Applied Robotics class (con Montserrat). 
           The nonlinear bycicle model was used.
           This model works for low speed movement and considers no
           longitudinal slip.
 * -----------------------------------------------------------------------------
 **/

#ifndef __VTEC_SDC1__
#define __VTEC_SDC1__

#include "base/car_3dof_dynamic_model.hpp"
#include "rclcpp/rclcpp.hpp"

class VTecSDC1 : public CarDynamicModel {
    private:
        float rr_offset_ {0};
        float throttle_offset_ {0};

        uint8_t D_MAX_{255};
        uint8_t D_MIN_{0};

    public:
        /* Constructor and destructor */
        VTecSDC1(float sample_time);
        ~VTecSDC1();

        void calculateModelParams();
        void updateDBSignals();
        
        /* Class methods */
        // void calculateControlInputs();
};

#endif