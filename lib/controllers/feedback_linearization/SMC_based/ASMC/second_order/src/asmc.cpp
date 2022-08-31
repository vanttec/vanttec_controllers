/** ----------------------------------------------------------------------------
 * @file: asmc.cpp
 * @date: June 17, 2021
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#include "asmc.hpp"

ASMC::ASMC(const float sample_time_s, const float lambda, const float K2, const float K_alpha, const float K1_init, const float K_min, const float mu, const DOFControllerType_E type)
{
    _sample_time_s = sample_time_s;
    _q_d = 0.0;
    _q_dot_d = 0.0;
    _error_1 = 0.0;
    _error_2 = 0.0;
    _prev_error_1 = 0.0;
    _prev_error_2 = 0.0;
    
    // Auxiliar control
    _ua = 0.0;

    // Sliding surface
    _lambda = lambda;
    _s = 0.0;

    // Gains
    _K1 = K1_init;
    _K2 = K2;
    _dot_K1 = 0.0;
    _prev_dot_K1 = 0.0;

    // Adaptive law
    _K_min = K_min;
    _K_alpha = K_alpha;
    _mu = mu;

    _controller_type = type;
}

ASMC::~ASMC(){}

void ASMC::Reset()
{
    _error_1 = 0.0;
    _prev_error_1 = 0.0;
    _error_2 = 0.0;
    _prev_error_2 = 0.0;
    _ua = 0.0;
    _K1 = 0.0;
}

void ASMC::UpdateSetPoint(const float q_d, const float q_dot_d)
{
    _q_d = q_d;
    _q_dot_d = q_dot_d;
}

float ASMC::sign(const float e)
{
    float sign = 0.0;
    if (e != 0.0)
    {
        sign = e / fabs(e);
    } else
    {
        sign = 0;
    }
    return sign;
}

float ASMC::sig(const float e, const float a)
{
    return sign(e)*pow(fabs(e),a);
}

void ASMC::CalculateAuxControl(float q, float q_dot)
{
    _prev_error_1 = _error_1;
    _prev_error_2 = _error_2;
    _prev_dot_K1 = _dot_K1;

    _error_1 = _q_d - q;
    _error_2 = _q_dot_d - q_dot;

    if (_controller_type == ANGULAR_DOF)
    {
        if (fabs(_error_1) > M_PI)
        {
            _error_1 = (_error_1 / fabs(_error_1)) * (fabs(_error_1) - 2 * M_PI);
        }
        if (fabs(_error_2) > M_PI)
        {
            _error_2 = (_error_2 / fabs(_error_2)) * (fabs(_error_2) - 2 * M_PI);
        }
    }

    // Checar que calc de sign est[e bien]
    _s = _error_2 + _lambda*_error_1;

    _dot_K1 = _K1 > _K_min ?  _K_alpha*sign(fabs(_s) - _mu) : _K_min;
    _K1 += (_dot_K1 + _prev_dot_K1) / 2 * _sample_time_s;
    _ua = -_K1*sig(_s, 0.5) - _K2*_s;
}