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

#include <stdio.h>
#include <iostream>
#include <cmath>

#include "stanley_controller.hpp"

StanleyController::StanleyController(const std::vector<float>& delta_sat, float k, float k_soft)
{
    DELTA_SAT_ = delta_sat;
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
            xp = p2.x; // or x1
        }
        if(!std::isnormal(ey)){
            yp = p2.y; // or y1
            xp = vehicle_pos.x;
        }
    }

    // Crosstrack and along-track errors in path frame
    ex_ = (p2.x - xp)*std::cos(ak_) + (p2.y - yp)*std::sin(ak_);
    ey_ = -(vehicle_pos.x - xp)*std::sin(ak_) + (vehicle_pos.y - yp)*std::cos(ak_);

    std::cout << "xp = " << xp << std::endl;
    std::cout << "yp = " << yp << std::endl;
    std::cout << "x = " << vehicle_pos.x << std::endl;
    std::cout << "y = " << vehicle_pos.y << std::endl;
    std::cout << "Along-track error = " << ex_ << std::endl;
    std::cout << "Crosstrack error = " << ey_ << std::endl;
    std::cout << "ak = " << ak_ << std::endl;
}

void StanleyController::setYawAngle(float psi){
    psi_ = psi;
}

void StanleyController::calculateSteering(float vel){
    vel_ = vel;
    float phi = psi_ - ak_;
    // delta_ = -(phi + std::atan2(k_*ey_,k_soft_ + vel_));
    delta_ = phi + std::atan2(k_*ey_,k_soft_ + vel_);

    if (delta_ > DELTA_SAT_[0])
        delta_ = DELTA_SAT_[0];
    else if (delta_ < DELTA_SAT_[1])
        delta_ = DELTA_SAT_[1];

    std::cout << "psi = " << psi_ << std::endl;
    std::cout << "Delta max = " << DELTA_SAT_[0] << "Delta min = " << DELTA_SAT_[1] << std::endl;
    std::cout << "Delta = " << delta_ << std::endl;
}