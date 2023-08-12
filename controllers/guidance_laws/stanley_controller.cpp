/** ----------------------------------------------------------------------------
 * @file: stanley_controller.cpp
 * @date: November 30, 2022
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

void StanleyController::calculateCrosstrackError(float x0, float y0, float x1, float y1){
    float x = vehicle_pose_.x;
    float y = vehicle_pose_.y;

    float ex = x1 - x0;
    float ey = y1 - y0;

    float m1;
    float m2;
    float b;
    float c;
    float xp;
    float yp;

    // Angle of path frame
    ak_ = std::atan2(ey,ex);

    if(std::isnormal(ex) && std::isnormal(ey)){
        // Slope of path
        m1 = ex/ey;
        b = x1 - m1*y1;

        // Slope of normal line to the path
        m2 = -1/m1;
        c = x - m2*y;

        // Obtain intersection point
        yp = (c - b)/(m1 - m2);
        xp = m1*yp + b;

    } else {
        if(!std::isnormal(ex)){
            yp = y;
            xp = x1; // or x2
        }
        if(!std::isnormal(ey)){
            yp = y1; // or y2
            xp = x;
        }
    }

    // Crosstrack error in path frame
    e_ = -(x-xp)*std::sin(ak_) + (y-yp)*std::cos(ak_);

    // ROS_INFO_STREAM("xp = " << xp);
    // ROS_INFO_STREAM("yp = " << yp);
    // ROS_INFO_STREAM("x = " << x);
    // ROS_INFO_STREAM("y = " << y);
    // ROS_INFO_STREAM("Crosstrack error = " << e_);
    // ROS_INFO_STREAM("ak = " << ak_);
}

void StanleyController::setHeading(const sdv_msgs::msg::EtaPose& pose){
    vehicle_pose_ = pose;
    psi_ = pose.psi;
}

void StanleyController::calculateSteering(float vel){
    vel_ = vel;
    delta_ = -((-ak_ + psi_) +std::atan2(k_*e_,k_soft_ + vel_));
    // ROS_INFO_STREAM("Delta = " << delta_);

    if (delta_ >= DELTA_MAX_)
        delta_ = DELTA_MAX_;
    else if (delta_ <= -DELTA_MAX_)
        delta_ = -DELTA_MAX_;
}