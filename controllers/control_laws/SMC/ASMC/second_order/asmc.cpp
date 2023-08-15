/** ----------------------------------------------------------------------------
 * @file: asmc.cpp
 * @date: June 17, 2021
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: First Order Adaptive Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */

#include "controllers/control_laws/SMC_based/ASMC/asmc.hpp"

ASMC::ASMC(float sample_time,float lambda,float K2,float K_alpha,float K1_init,float K_min,float mu, const DOFControllerType_E& type)
{
    sample_time_ = sample_time;
    q_d_ = 0.0;
    q_dot_d_ = 0.0;
    error_1_ = 0.0;
    error_2_ = 0.0;
    prev_error_1_ = 0.0;
    prev_error_2_ = 0.0;
    
    // Auxiliar control
    u_ = 0.0;

    // Sliding surface
    lambda_ = lambda;
    s_ = 0.0;

    // Gains
    K1_ = K1_init;
    K2_ = K2;
    dot_K1_ = 0.0;
    prev_dot_K1_ = 0.0;

    // Adaptive law
    K_min_ = K_min;
    K_alpha_ = K_alpha;
    mu_ = mu;

    controller_type_ = type;
}

ASMC::~ASMC(){}

void ASMC::reset()
{
    error_1_ = 0.0;
    prev_error_1_ = 0.0;
    error_2_ = 0.0;
    prev_error_2_ = 0.0;
    u_ = 0.0;
    K1_ = 0.0;
}

void ASMC::updateSetPoint(float q_d,float q_dot_d)
{
    q_d_ = q_d;
    q_dot_d_ = q_dot_d;
}


void ASMC::calculateAuxControl(float q,float q_dot)
{
    prev_error_1_ = error_1_;
    prev_error_2_ = error_2_;
    prev_dot_K1_ = dot_K1_;

    error_1_ = q_d_ - q;
    error_2_ = q_dot_d_ - q_dot;

    if (controller_type_ == ANGULAR_DOF)
    {
        if (std::fabs(error_1_) > M_PI)
        {
            error_1_ = (error_1_ / std::fabs(error_1_)) * (std::fabs(error_1_) - 2 * M_PI);
        }
        if (std::fabs(error_2_) > M_PI)
        {
            error_2_ = (error_2_ / std::fabs(error_2_)) * (std::fabs(error_2_) - 2 * M_PI);
        }
    }

    // Checar que calc de sign est[e bien]
    s_ = error_2_ + lambda_*error_1_;

    dot_K1_ = K1_ > K_min_ ?  K_alpha_*static_cast<float>(utils::sign(std::fabs(s_) - mu_)) : K_min_;

    K1_ += (dot_K1_ + prev_dot_K1_) / 2 * sample_time_;

    u_ = -(K1_*utils::sig(s_, 0.5) + K2_*s_);       // that "-" comes from fback lin theory:
                                                    // u_  = (1 / g_x) * (-f_x + K1... + K2*s);
}