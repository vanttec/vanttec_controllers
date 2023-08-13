/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1.cpp
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

#include "vtec_sdc1.hpp"

VTecSDC1::VTecSDC1(float sample_time) : CarDynamicModel(sample_time){

    /* Vehicle physical parameters */
    m_ = 1120;   
    Iz_ = 1908.94666;
    A_ = 3.426;
    Cm_ = 340;            // Chosen so max speed = 35 km/h = 9.722 m/s
    Cd_ = 0.9;            // Cd was chosen based on the book 'fundamentals of vehicle dynamics'
    len_f_ = 1.58;
    len_r_ = 2.69;
    car_len_ = 4.27;    
    MAX_R_ = 4.5;
    // u_dot_brake_ = 0.0;
    fr_ = 0.015;          // For passenger cars, from fundamentals of vehicle dynamics book
    C_alpha_ = 51935.3434;// m*g*fr*cos(theta) -> from fundamentals of vehicle dynamics book
    alpha_f_ = 0.0;
    alpha_r_ = 0.0;
}

VTecSDC1::~VTecSDC1(){}

void VTecSDC1::calculateModelParams(){
    float rr_off = -0.0178*std::pow(D_,2) + 1.4012*D_ + 1018;
    float t_off = -0.0769*std::pow(D_,2) + 8.1533*D_ + 1038.7;

    setOffsets(rr_off, t_off);

    float Cm1 = 0.0113*std::pow(D_,2)-2.1552*D_+142.22;
    float Cm2 = 0.0028*std::pow(D_,2)-0.4629*D_+22.399;

    setMotorConstants(Cm1, Cm2);
}

void VTecSDC1::updateDBSignals(){

    /* THROTTLE*/
    /* Find roots of throttle eq. */
    float a = 0.0113-0.0028*nu_(0);
    float b = 0.4629*nu_(0)-2.0783;
    float c = 134.067-22.399*nu_(0);
    float d = -1038.7-u_(0);
    bool has_real_root;
    float real_root;
    int root = 0;

    // Create a cubic polynomial using Eigen's PolynomialSolver
    Eigen::Matrix<float, 4, 1> coeffs;
    coeffs << d, c, b, a;
    Eigen::PolynomialSolver<float, 3> solver(coeffs);

    // Compute the roots
    real_root = solver.smallestRealRoot(has_real_root, 1e-6);

    if(has_real_root)
        root = static_cast<int>(std::round(real_root));
    
    D_ = root>D_MAX_? D_MAX_:root<D_MIN_? D_MIN_:D_;

    /* BRAKING */

}