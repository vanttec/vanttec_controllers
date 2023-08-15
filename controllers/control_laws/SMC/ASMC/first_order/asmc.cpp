/** ----------------------------------------------------------------------------
 * @file: asmc.hpp
 * @date: August 15, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Second Order Adaptive Sliding Mode Controller class.
 * -----------------------------------------------------------------------------
 * */
#include "controllers/control_laws/SMC_based/ASMC/asmc.hpp"

ASMC::ASMC(float sample_time,float lambda,float K2,float K_alpha,float K1_init,float K_min,float mu, const DOFControllerType_E& type)
{
    sample_time_ = sample_time;
    
    error_ = 0.0;
    prev_error_ = 0.0;

    /* Setpoint */
    q_d_ = 0.0;
    q_dot_d_ = 0.0;
    
    /* Auxiliar control */
    u_ = 0.0;

    /* Sliding surface */
    lambda_ = lambda;
    s_ = 0.0;

    /* Gains */
    K1_ = K1_init;
    K2_ = K2;
    dot_K1_ = 0.0;
    prev_dot_K1_ = 0.0;

    /* Adaptive law */
    K_min_ = K_min;
    K_alpha_ = K_alpha;
    mu_ = mu;

    controller_type_ = type;
}

ASMC::~ASMC(){}

void ASMC::reset()
{
    error_ = 0.0;
    prev_error_ = 0.0;
    u_ = 0.0;
    K1_ = 0.0;
}

void ASMC::updateSetPoint(float q_d, float q_dot_d)
{
    q_d_ = q_d;
    q_dot_d_ = q_dot_d;
}

void ASMC::calculateManipulation(float q,float q_dot)
{
    float error_dot;
    prev_error_ = error_;
    prev_dot_K1_ = dot_K1_;

    error_ = q_d_ - q;

    if (controller_type_ == ANGULAR_DOF)
    {
        if (std::fabs(error_) > M_PI)
        {
            error_ = (error_ / std::fabs(error_)) * (std::fabs(error_) - 2 * M_PI);
        }
    }

    error_dot = (error_ - prev_error_) / sample_time_;

    // Checar que calc de sign est[e bien]
    s_ = error_dot + lambda_*error_;

    // Adaptive law
    dot_K1_ = K1_ > K_min_ ?  K_alpha_*utils::sign(std::fabs(s_) - mu_) : K_min_;
    
    K1_ += (dot_K1_ + prev_dot_K1_) / 2 * sample_time_;
    u_ = -K1_*utils::sig(s_, 0.5) - K2_*s_;
}