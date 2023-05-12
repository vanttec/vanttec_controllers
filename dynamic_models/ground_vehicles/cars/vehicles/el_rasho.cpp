/** ----------------------------------------------------------------------------
 * @file: el_rasho.cpp
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

#include "el_rasho.hpp"

Cafe::Cafe(const float sample_time) : CarDynamicModel(sample_time){

    /* Vehicle physical parameters */
    m_ = 1120;   
    Iz_ = 1908.94666;
    A_ = 3.426;
    Cm_ = 340;            // Chosen so max speed = 35 km/h = 9.722 m/s
    Cd_ = 0.9;            // Cd was chosen based on the book 'fundamentals of vehicle dynamics'
    len_f_x_ = 1.58;
    len_r_ = 2.69;
    car_len_ = 4.27;    
    MAX_R_ = 4.5;
    // u_dot_brake_ = 0.0;
    fr_ = 0.015;          // For passenger cars, from fundamentals of vehicle dynamics book
    C_alpha_ = 51935.3434;// m*g*fr*cos(theta) -> from fundamentals of vehicle dynamics book
    alpha_f_x_ = 0.0;
    alpha_r_ = 0.0;
}

Cafe::~Cafe(){}