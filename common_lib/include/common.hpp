/** ----------------------------------------------------------------------------
 * @file: common.hpp
 * @date: March 2, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Common functions and constants used throughout the package.
 * -----------------------------------------------------------------------------
 * */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <vanttec_msgs/GuidanceWaypoints.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

typedef enum Side_E
{
    LEFT = 0,
    RIGHT = 1,
} Side_E;

typedef enum DOFControllerType_E
{
    LINEAR_DOF = 0,
    ANGULAR_DOF = 1,
} DOFControllerType_E;

namespace common
{
    /* Helper constants */
    static const float rho = 1000;
    static const float g = 9.81;
    
    /* Helper functions */
    vanttec_msgs::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center, float _angle_offset);
    // Eigen::MatrixXf                calculateTransformation(double phi, double theta_, double psi);
    Eigen::MatrixXf calculateRotation(double phi, double theta_, double psi);
    Eigen::Matrix3f Skew(const Eigen::Vector3f& vect);
    float secant(const float& angle);
    float sign(const float& e);
    float sig(const float& e, const float& a);
}

#endif
