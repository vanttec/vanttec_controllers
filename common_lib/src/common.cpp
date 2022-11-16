/** ----------------------------------------------------------------------------
 * @file: common.cpp
 * @date: March 2, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Common functions and constants used throughout the package.
 * -----------------------------------------------------------------------------
 * */

#include "common.hpp"

namespace common
{
    vanttec_msgs::GuidanceWaypoints GenerateCircle(const float& _radius, const float& _x_center, const float& _y_center, const float& _z_center, const float& _angle_offset)
    {
        vanttec_msgs::GuidanceWaypoints _waypoints;
        
        float angle = _angle_offset;
        uint8_t counter = 0;
        
        while (angle <= (_angle_offset + M_PI))
        {
            _waypoints.waypoint_list_x.push_back(_radius * std::cos(angle) + _x_center);
            _waypoints.waypoint_list_y.push_back(_radius * std::sin(angle) + _y_center);
            _waypoints.waypoint_list_z.push_back(_z_center);
            angle += M_PI / 6;
            counter++;
        }
        
        angle = _angle_offset - M_PI;

        while (angle <= _angle_offset)
        {
            _waypoints.waypoint_list_x.push_back(_radius * std::cos(angle) + _x_center);
            _waypoints.waypoint_list_y.push_back(_radius * std::sin(angle) + _y_center);
            _waypoints.waypoint_list_z.push_back(_z_center);
            angle += M_PI / 6;
            counter++;
        }
        
        _waypoints.waypoint_list_length = counter;

        return _waypoints;
    }


    Eigen::MatrixXf calculateRotation(const double& phi, const double& theta_, const double& psi)
    {
        Eigen::Matrix3f R;
        R <<   std::cos(psi)*std::cos(theta_),      -std::sin(psi)*std::cos(phi) + std::cos(psi)*std::sin(theta_)*std::sin(phi),     std::sin(psi)*std::sin(phi) + std::cos(psi)*std::cos(phi)*std::sin(theta_),
                std::sin(psi)*std::cos(theta_),       std::cos(psi)*std::cos(phi) + std::sin(phi)*std::sin(theta_)*std::sin(psi),    -std::cos(psi)*std::sin(phi) + std::sin(theta_)*std::sin(psi)*std::cos(phi),
               -std::sin(theta_),                     std::cos(theta_)*std::sin(phi),                                                 std::cos(theta_)*std::cos(phi);

        return R;
    }

    // Eigen::MatrixXf calculateTransformation(const double& phi, const double& theta_, const double& psi)
    // {
    //     Eigen::Matrix3f R;
    //     R_ << std::cos(theta_)*std::cos(psi),                                                 std::cos(theta_)*std::sin(psi),                                                  -std::sin(theta_),
    //          std::sin(phi)*std::sin(theta_)*std::cos(psi) - std::cos(phi)*std::sin(psi),     std::sin(phi)*std::sin(theta_)*std::sin(psi) + std::cos(phi)*std::cos(psi),      std::sin(phi)*std::cos(theta_),
    //          std::cos(phi)*std::sin(theta_)*std::cos(psi) + std::sin(phi)*std::sin(psi),     std::cos(phi)*std::sin(theta_)*std::sin(psi) - std::sin(phi)*std::cos(psi),      std::cos(phi)*std::cos(theta_);

    //     Eigen::Matrix3f T;
    //     T_ << 1,     std::sin(phi)*std::tan(theta_),  std::cos(phi)*std::tan(theta_),
    //          0,     std::cos(phi),                  -std::sin(phi),
    //          0,     std::sin(phi)/std::cos(theta_),  std::cos(phi)/std::cos(theta_);

    //     Eigen::Matrix3f zero;
    //     zero << 0,0,0,
    //             0,0,0,
    //             0,0,0;

    //     Eigen::MatrixXf J(6,6);
    //     J_ << R,     zero,
    //          zero,  T;

    //     return J;
    // }

    Eigen::Matrix3f Skew(const Eigen::Vector3f& vect)
    {
        Eigen::Matrix3f skew;
        skew << 0,       -vect(3),  vect(2),
                vect(3),  0,       -vect(1),
                -vect(2), vect(1), 0;
        
        return skew;
    }

    float secant(const float& angle)
    {
        return 1/std::cos(angle);
    }
    
    float sign(const float& e)
    {
        float sign = 0.0;
        if (e != 0.0)
        {
            sign = e / std::fabs(e);
        } else
        {
            sign = 0;
        }
        return sign;
    }

    float sig(const float& e, const float& a)
    {
        return sign(e)*pow(std::fabs(e),a);
    }

}