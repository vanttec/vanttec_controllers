/** ----------------------------------------------------------------------------
 * @file: stanley_controller.cpp
 * @date: November 30, 2022
 * @date: August 18, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: Stanley Controller class
 * -----------------------------------------------------------------------------
 * */

#include "stanley_controller.hpp"

StanleyController::StanleyController(float delta_max, float k, float k_soft)
{
    DELTA_MAX_ = delta_max;
    psi_ = 0;
    k_ = k;
    k_soft_ = k_soft;
}

StanleyController::~StanleyController(){}

// void StanleyController::calculateCrosstrackError(float x, float y, float p1.x, float p1.y, float p2.x, float p2.y){
void StanleyController::calculateCrosstrackError(const Point& vehicle_pos, const Point& p1, const Point& p2){
    // float x = vehicle_pose_.x;
    // float y = vehicle_pose_.y;

    float m1;
    float m2;
    float b;
    float c;
    float xp;
    float yp;

    float ex = p2.x - p1.x;
    float ey = p2.y - p1.y;

    // Angle of path frame
    ak_ = std::atan2(ey,ex);

    if(std::isnormal(ex) && std::isnormal(ey)){
        // Slope of path
        m1 = ex/ey;
        b = p2.x - m1*p2.y;

        // Slope of normal line to the path
        m2 = -1/m1;
        c = vehicle_pos.x - m2*vehicle_pos.y;

        // Obtain intersection point
        yp = (c - b)/(m1 - m2);
        xp = m1*yp + b;

    } else {
        if(!std::isnormal(ex)){
            yp = vehicle_pos.y;
            xp = p2.x; // or x2
        }
        if(!std::isnormal(ey)){
            yp = p2.y; // or y2
            xp = vehicle_pos.x;
        }
    }

    // Crosstrack error in path frame
    e_ = -(vehicle_pos.x-xp)*std::sin(ak_) + (vehicle_pos.y-yp)*std::cos(ak_);

    // ROS_INFO_STREAM("xp = " << xp);
    // ROS_INFO_STREAM("yp = " << yp);
    // ROS_INFO_STREAM("x = " << x);
    // ROS_INFO_STREAM("y = " << y);
    // ROS_INFO_STREAM("Crosstrack error = " << e_);
    // ROS_INFO_STREAM("ak = " << ak_);
}

void StanleyController::setYawAngle(float psi){
    psi_ = psi;
}

void StanleyController::calculateSteering(float vel){
    vel_ = vel;
    float phi = psi_ - ak_;
    // delta_ = -(phi + std::atan2(k_*e_,k_soft_ + vel_));
    delta_ = phi + std::atan2(k_*e_,k_soft_ + vel_);
    // ROS_INFO_STREAM("Delta = " << delta_);

    if (delta_ > DELTA_MAX_)
        delta_ = DELTA_MAX_;
    else if (delta_ < -DELTA_MAX_)
        delta_ = -DELTA_MAX_;
}