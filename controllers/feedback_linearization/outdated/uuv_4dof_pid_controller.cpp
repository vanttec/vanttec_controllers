/** ----------------------------------------------------------------------------
 * @file: uuv_4dof_pid.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: 4-DOF controller class, using different, decoupled controllers for
 *         each DOF.
           OUTDATED.
 * -----------------------------------------------------------------------------
 * */

#include "uuv_4dof_pid.hpp"

UUV4DOFController::UUV4DOFController(float sample_time_, const float _kpid_u[3], const float _kpid_v[3], const float _kpid_z[3], const float _kpid_psi[3])
                                    : surge_speed_controller(sample_time_, _kpid_u, LINEAR_DOF_PID)
                                    , sway_speed_controller(sample_time_, _kpid_v, LINEAR_DOF_PID)
                                    , depth_controller(sample_time_, _kpid_z, LINEAR_DOF_PID)
                                    , heading_controller(sample_time_, _kpid_psi, ANGULAR_DOF_PID)
{
    this->g_x << (1 / (mass - X_u_dot)),
                 (1 / (mass - Y_v_dot)),
                 (1 / (mass - Z_w_dot)),
                 (1 / (Izz - N_r_dot_));

    this->surge_speed_controller.g_x    = g_x(0);
    this->sway_speed_controller.g_x     = g_x(1);
    this->depth_controller.g_x          = g_x(2);
    this->heading_controller.g_x        = g_x(3);

    this->f_x << 0,
                 0,
                 0,
                 0;
}

UUV4DOFController::~UUV4DOFController(){}

void UUV4DOFController::publishAccel(){
    // ros::Rate loop_rate(10);

    geometry_msgs::Twist v_dot_msg;
    v_dot_msg.linear.x =  this->f_x(0); 
    v_dot_msg.linear.y =  this->f_x(1); 
    v_dot_msg.linear.z =  this->f_x(2); 
    v_dot_msg.angular.x =  0; 
    v_dot_msg.angular.y =  0; 
    v_dot_msg.angular.z =  this->f_x(3); 

    // while (ros::ok())
    // {
    v_dot_pub.publish(v_dot_msg);
    //     loop_rate.sleep();
    // }
}

void UUV4DOFController::updatePose(const geometry_msgs::Pose& _pose)
{
    this->local_pose.position.x     = _pose.position.x;
    this->local_pose.position.y     = _pose.position.y;
    this->local_pose.position.z     = _pose.position.z;
    
    // double w = _pose.orientation.w; 
    // double x = _pose.orientation.x; 
    // double y = _pose.orientation.y; 
    // double z = _pose.orientation.z; 
    // double C_11 = pow(w,2)+pow(x,2)-pow(y,2)-pow(z,2);
    // double C_12 = 2*(x*y + w*z);
    // double C_13 = 2*(x*z - w*y);
    // double C_23 = 2*(y*z + w*x);
    // double C_33 = pow(w,2)-pow(x,2)-pow(y,2)+pow(z,2);
    // //3-2-1 convention
    // double yaw = atan2(C_12,C_11);
    // double pitch = -asin(C_13);
    // double roll = atan2(C_23,C_33);
    // this->yaw_psi_angle = yaw;

    this->yaw_psi_angle             = _pose.orientation.z;
}

void UUV4DOFController::updateTwist(const geometry_msgs::Twist& _twist)
{
    this->local_twist.linear.x = _twist.linear.x;
    this->local_twist.linear.y = _twist.linear.y;
    this->local_twist.linear.z = _twist.linear.z;
    this->local_twist.angular.x = _twist.angular.x;
    this->local_twist.angular.y = _twist.angular.y;
    this->local_twist.angular.z = _twist.angular.z;

}

void UUV4DOFController::updateSetPoints(const geometry_msgs::Twist& _set_points)
{
    this->surge_speed_controller.set_point      = (float) _set_points.linear.x;
    this->sway_speed_controller.set_point       = (float) _set_points.linear.y;
    this->depth_controller.set_point            = (float) _set_points.linear.z;

    float uncorrected_angle                     = (float) _set_points.angular.z;

    if (std::abs(uncorrected_angle) > 3.1416)
    {
        this->heading_controller.set_point      = (uncorrected_angle / std::abs(uncorrected_angle)) * (std::abs(uncorrected_angle) - (2 * 3.1416));
    }
    else
    {
        this->heading_controller.set_point      = uncorrected_angle;
    } 
}

void UUV4DOFController::updateControlLaw()
{
    this->upsilon << ((float) this->local_twist.linear.x),
                     ((float) this->local_twist.linear.y),
                     ((float) this->local_twist.linear.z),
                     ((float) this->local_twist.angular.z);
    
    
    /* Rigid Body Mass Matrix */

    this->M_rb_ << mass, 0, 0, 0,
                  0, mass, 0, 0,
                  0, 0, mass, 0,
                  0, 0, 0, Izz;

    /* Hydrodynamic Added Mass Matrix */

    this->M_a_ << X_u_dot, 0, 0, 0,
                 0, Y_v_dot, 0, 0,
                 0, 0, Z_w_dot, 0,
                 0, 0, 0, N_r_dot_;

    /* Rigid Body Coriolis Matrix */

    float rb_a_1 = mass * this->upsilon(0);
    float rb_a_2 = mass * this->upsilon(1);
    
    this->C_rb_ << 0, 0, 0, -rb_a_2,
                  0, 0, 0, rb_a_1,
                  0, 0, 0, 0,
                  rb_a_2, -rb_a_1, 0, 0;

    /* Hydrodynamic Added Mass Coriolis Matrix */

    float a_a_1 = X_u_dot * this->upsilon(0);
    float a_a_2 = Y_v_dot * this->upsilon(1);
    
    this->C_a_ << 0, 0, 0, a_a_2,
                 0, 0, 0, -a_a_1,
                 0, 0, 0, 0,
                 -a_a_2, a_a_1, 0, 0;
    
    /* Hydrodynamic Damping */

    this->D_lin_ << -(X_u_), 0, 0, 0,
                   0, -(Y_v_), 0, 0,
                   0, 0, -(Z_w_), 0,
                   0, 0, 0, -(N_r_);

    this->D_qua_ << -(X_uu * std::fabs(this->upsilon(0))), 0, 0, 0,
                   0, -(Y_vv * std::fabs(this->upsilon(1))), 0, 0,
                   0, 0, -(Z_ww * std::fabs(this->upsilon(2))), 0,
                   0, 0, 0, -(N_rr * std::fabs(this->upsilon(3)));

    /* Restoring Forces */
    
    this->G_eta_ << (weight - buoyancy) * sin(theta_b),
                   -(weight - buoyancy) * cos(theta_b) * sin(phi_b),
                   -(weight - buoyancy) * cos(theta_b) * cos(phi_b),
                   0;

    /* 4 DoF State Calculation */

    Eigen::Matrix4f M_ = this->M_rb_ - this->M_a_;
    Eigen::Matrix4f C_= this->C_rb_ + this->C_a_;
    Eigen::Matrix4f D_= this->D_lin_ + this->D_qua_;

    this->f_x = M_.inverse() * (- (C_ * this->upsilon) - (D_ * this->upsilon) - this->G_eta_);

    this->surge_speed_controller.f_x    = f_x(0);
    this->sway_speed_controller.f_x     = f_x(1);
    this->depth_controller.f_x          = f_x(2);
    this->heading_controller.f_x        = f_x(3);
}

void UUV4DOFController::updateThrustOutput()
{
    /* calculate Controller Ouput/manipulation_ */
    this->surge_speed_controller.calculateManipulation(this->local_twist.linear.x);
    this->sway_speed_controller.calculateManipulation(this->local_twist.linear.y);
    this->depth_controller.calculateManipulation(this->local_pose.position.z);
    this->heading_controller.calculateManipulation(this->yaw_psi_angle);

    /* Saturate Controller Ouput/manipulation_ */

    if (std::fabs(this->surge_speed_controller.manipulation_) > MAX_THRUST_SURGE)
    {
        this->thrust.tau_x = (this->surge_speed_controller.manipulation_ / std::fabs(this->surge_speed_controller.manipulation_)) * (MAX_THRUST_SURGE);
    }
    else
    {
        this->thrust.tau_x = this->surge_speed_controller.manipulation_;
    }

    if (std::fabs(this->sway_speed_controller.manipulation_) > MAX_THRUST_SWAY)
    {
        this->thrust.tau_y = (this->sway_speed_controller.manipulation_ / std::fabs(this->sway_speed_controller.manipulation_)) * (MAX_THRUST_SWAY);
    }
    else
    {
        this->thrust.tau_y = this->sway_speed_controller.manipulation_;
    }

    if (std::fabs(this->depth_controller.manipulation_) > MAX_THRUST_HEAVE)
    {
        this->thrust.tau_z = (this->depth_controller.manipulation_ / std::fabs(this->depth_controller.manipulation_)) * (MAX_THRUST_HEAVE);
    }
    else
    {
        this->thrust.tau_z = this->depth_controller.manipulation_;
    }

    if (std::fabs(this->heading_controller.manipulation_) > MAX_THRUST_YAW)
    {
        this->thrust.tau_psi = (this->heading_controller.manipulation_ / std::fabs(this->heading_controller.manipulation_)) * (MAX_THRUST_YAW);
    }
    else
    {
        this->thrust.tau_psi = this->heading_controller.manipulation_;
    }
}

