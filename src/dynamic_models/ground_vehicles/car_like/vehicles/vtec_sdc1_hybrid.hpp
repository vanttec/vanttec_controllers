/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_hybrid.hpp
 * @date: Feb 6, 2025
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of 3-DOF VTec Self-Driving Car hybrid model in the non-inertial frame with
           Euler Angles for Applied Robotics class (con Montserrat). 
           The first-principles nonlinear bycicle model is fused with a Gaussian Process residuals model.
 * -----------------------------------------------------------------------------
 **/

#ifndef __VTEC_SDC1_HYBRID__
#define __VTEC_SDC1_HYBRID__

#include <torch/script.h>  // PyTorch C++ API for inference
#include <Eigen/Dense>

#include "dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.hpp"

class VTecSDC1HybridModel : public VTecSDC1DynamicModel {
    public:
        VTecSDC1HybridModel(float sample_time, uint8_t D_MAX, const std::string& model_path);
        Eigen::Vector3f computeResidualDynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& control);
        void computeState() override;

    private:
        torch::jit::script::Module model_;
        torch::Device device_;
        float psi_dot_prev_ = 0.0;
        float psi_dot_prev2_ = 0.0;

        Eigen::MatrixXf control_history_;
        Eigen::MatrixXf state_history_;
};
#endif