/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_hybrid.cpp
 * @date: Feb 6, 2025
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of 3-DOF VTec Self-Driving Car hybrid model in the non-inertial frame with
           Euler Angles for Applied Robotics class (con Montserrat). 
           The first-principles nonlinear bycicle model is fused with a Gaussian Process residuals model.
 * -----------------------------------------------------------------------------
 **/

#include <torch/torch.h>
#include "dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1_hybrid.hpp"
#include <iostream>

VTecSDC1HybridModel::VTecSDC1HybridModel(float sample_time, uint8_t D_MAX, const std::string& model_path):
                    VTecSDC1DynamicModel(sample_time, D_MAX),
                    device_(torch::Device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU))
{
    try {
        model_ = torch::jit::load(model_path);
        model_.to(device_);
    } catch (const c10::Error& e) {
        std::cerr << "Error loading PyTorch model: " << e.what() << std::endl;
    }
    // Initialize state and control history (sequence of 10 time steps)
    state_history_ = Eigen::MatrixXf::Zero(10, 9);  // 10 time steps, 9 state features
    control_history_ = Eigen::MatrixXf::Zero(10, 2); // 10 time steps, 2 control inputs
}

Eigen::Vector3f VTecSDC1HybridModel::computeResidualDynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& control) {
    // Shift history: Remove oldest entry, push latest state & control
    state_history_.block(0, 0, 9, 9) = state_history_.block(1, 0, 9, 9);
    state_history_.row(9) = state.cast<float>();

    control_history_.block(0, 0, 9, 2) = control_history_.block(1, 0, 9, 2);
    control_history_.row(9) = control.cast<float>();

    // Convert Eigen matrices to Torch tensors
    torch::Tensor state_tensor = torch::from_blob(state_history_.data(), {1, 10, 9}, torch::kFloat32).to(device_);
    torch::Tensor control_tensor = torch::from_blob(control_history_.data(), {1, 10, 2}, torch::kFloat32).to(device_);

    // Forward pass through the GRU model
    torch::Tensor output_tensor = model_.forward({state_tensor, control_tensor}).toTensor();

    output_tensor = output_tensor.to(torch::kCPU);

    // Convert tensor output to Eigen vector
    Eigen::VectorXd residuals(output_tensor.size(1));
    std::memcpy(residuals.data(), output_tensor.data_ptr(), output_tensor.numel() * sizeof(float));

    return residuals.cast<float>(); // Ensure correct type conversion

/*
    // Ensure inputs are correctly shaped as sequences (batch=1, seq_len=10, features)
    torch::Tensor state_tensor = torch::from_blob((void*)state.data(), {10, 9}, torch::dtype(torch::kFloat32)).to(device_);
    torch::Tensor control_tensor = torch::from_blob((void*)control.data(), {10, 2}, torch::dtype(torch::kFloat32)).to(device_);

    // Reshape tensors to match model expectations (batch=1)
    state_tensor = state_tensor.unsqueeze(0);  // (1, 10, 9)
    control_tensor = control_tensor.unsqueeze(0);  // (1, 10, 2)

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(state_tensor);
    inputs.push_back(control_tensor);

    torch::Tensor output_tensor = model_.forward(inputs).toTensor();
    output_tensor = output_tensor.to(torch::kCPU);

    Eigen::VectorXf residuals(output_tensor.size(1));
    std::memcpy(residuals.data(), output_tensor.data_ptr(), output_tensor.numel() * sizeof(float));

    return residuals;
*/
}

void VTecSDC1HybridModel::computeState() {
    float psi_ddot = (velocities_(2) - 2 * psi_dot_prev_ + psi_dot_prev2_) / (sample_time_ * sample_time_);
    
    psi_dot_prev2_ = psi_dot_prev_;
    psi_dot_prev_ = velocities_(2);

    // State vector for GP Model:
    // Acceleration: (ax, ay) in the body frame.
    // Velocity: (vx, vy) in the body frame.
    // Position: (x, y) in the NED frame.
    // Yaw: ψ (yaw angle) in the NED frame.
    // Yaw Rate: r (yaw rate).

    // order is important
    Eigen::VectorXf state(9);
    state << eta_pose_(0), eta_pose_(1), eta_pose_(2),
            velocities_(0), velocities_(1), velocities_(2),
            accelerations_(0), accelerations_(1), psi_ddot;

    // Control Vector for GP Model:
    // Throttle
    // Steering

    Eigen::VectorXf control(2);
    control << D_, delta_;
    Eigen::Vector3f r = computeResidualDynamics(state.cast<float>(), control.cast<float>());


    /* 3-DOF state calculation */
    nu_dot_ = f_ + g_*u_ + r;

    /* Integrating acceleration to get velocities */
    // nu_ += (nu_dot_prev_ + nu_dot_) / 2 * sample_time_;
    Eigen::Vector3f k1_nu = nu_dot_;  // Use the calculated nu_dot_
    Eigen::Vector3f k2_nu = f_ + g_*(u_ + 0.5*sample_time_*k1_nu);
    Eigen::Vector3f k3_nu = f_ + g_*(u_ + 0.5*sample_time_*k2_nu);
    Eigen::Vector3f k4_nu = f_ + g_*(u_ + sample_time_*k3_nu);

    nu_ += (k1_nu + 2*k2_nu + 2*k3_nu + k4_nu) / 6 * sample_time_;
    
    // So the model doesn't do weird things without moving forward 
    // if(delta_ == 0.0){
    //     nu_(1) = 0.0;
    //     nu_(2) = 0.0;
    // }

    /* Changing frames */
    eta_dot_ = R_*nu_;

    /* Integrating velocities to get positions */
    // eta_ += (eta_dot_prev_ + eta_dot_) / 2 * sample_time_;
    Eigen::Vector3f k1_eta = eta_dot_;  // Use the calculated eta_dot_
    Eigen::Vector3f k2_eta = R_*(nu_ + 0.5*sample_time_*k1_eta);
    Eigen::Vector3f k3_eta = R_*(nu_ + 0.5*sample_time_*k2_eta);
    Eigen::Vector3f k4_eta = R_*(nu_ + sample_time_*k3_eta);

    eta_ += (k1_eta + 2*k2_eta + 2*k3_eta + k4_eta) / 6 * sample_time_;

    if (std::fabs(eta_(2)) > M_PI) {
        eta_(2) = (eta_(2) / std::fabs(eta_(2))) * (std::fabs(eta_(2)) - 2 * M_PI);
    }

    /* Change of coordinate frame convention (from DYN_MODEL to BASE_LINK):
        - x (front) -> x (front)
        - y (left)  -> y (right)
        - z (up)    -> z (down)
    */

    accelerations_ << nu_dot_(0), -nu_dot_(1), -nu_dot_(2);
    std::cout << "accelerations_: " << accelerations_ << std::endl;
    velocities_ << nu_(0), -nu_(1), -nu_(2);
    std::cout << "velocities_: " << velocities_ << std::endl;
    eta_pose_ << eta_(0), eta_(1), -eta_(2);
}