/** ----------------------------------------------------------------------------
 * @file: antsmc.hpp
 * @date: August 30, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#include "antsmc.hpp"

ANTSMC::ANTSMC(const float& sample_time, const float& alpha, const float& beta, const float& K2, const float& K_alpha, const float& K_min, const float& K1_init, const float& mu, const DOFControllerType_E& type)
{
    sample_time_ = sample_time_;
    q_d_ = 0.0;
    q_dot_d_ = 0.0;
    error_1_ = 0.0;
    error_2_ = 0.0;
    prev_error_1_ = 0.0;
    prev_error_2_ = 0.0;
    
    // Auxiliar control
    ua_ = 0.0;

    // Control parameters
    alpha_ = alpha;
    beta_ = beta;
    s_ = 0.0;
    delta_ = 0.0;

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

ANTSMC::~ANTSMC(){}

void ANTSMC::reset()
{
    error_1_ = 0.0;
    prev_error_1_ = 0.0;
    error_2_ = 0.0;
    prev_error_2_ = 0.0;
    ua_ = 0.0;
    K1_ = 0.0;
}

void ANTSMC::updateSetPoint(const float& q_d, const float& q_dot_d)
{
    q_d_ = q_d;
    q_dot_d_ = q_dot_d;
}

void ANTSMC::calculateAuxControl(const float& q, const float& q_dot)
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

    delta_ = common::sig(error_2_, 2-beta_)/(alpha_*beta_);

    s_ = error_1_ + alpha_*common::sig(error_2_, beta_);
    
    dot_K1_ = K1_ > K_min_ ?  K_alpha_*common::sign(std::fabs(s_) - mu_) : K_min_;
    K1_ += (dot_K1_ + prev_dot_K1_) / 2 * sample_time_;
    ua_ = -K1_*common::sig(s_, 0.5) - K2_*s_;

}