#include <gtest/gtest.h>
#include "controllers/feedback_linearization/PID/first_order/fblin_pid.hpp"

TEST(PIDLin, SimpleModel){

  PIDParameters params;
  params.kP = 10;
  params.kI = 10;
  params.kD = 0.1;
  params.kDt = 0.01;

  double FB_LIN_UMAX = 255;
  double FB_LIN_UMIN = -255;
  double g_x = 1;
  double accel_d = 1;

  double position = 0;
  double velocity = 0;

  double position_d = 5;
  double distance = 0.1;

  PIDLin pid(FB_LIN_UMAX, FB_LIN_UMIN, params);

  pid.g_x_ = g_x;

  // u_ = g_x_^(-1)*(chi1_dot_d - f_x_ + u_n - u_aux)
  // Run a simple simulation, check that setpoint is reached with full PID impl.
  for(int i = 0; i < 100; i++){
    double u = pid.calculateManipulations(position, position_d, accel_d);
    position += (velocity + u) * params.kDt;
    velocity += 0.1 * params.kDt;
  }

  // Position should be near setpoint.
  EXPECT_NEAR(position, position_d, distance);

  pid = PIDLin(FB_LIN_UMAX, FB_LIN_UMIN, params);
  position = 0;
  velocity = 0;

  // Run a simple simulation, check that setpoint is reached with full PID impl.
  for(int i = 0; i < 1000; i++){
    double u = pid.calculateManipulations(position, -position_d, accel_d);
    position -= (velocity + u) * params.kDt;
    velocity += 0.1 * params.kDt;
  }

  EXPECT_NEAR(position, -position_d, distance);
}

TEST(PIDLin, ClampU) {

  PIDParameters params;
  params.kP = 100;
  params.kDt = 0.01;

  double FB_LIN_UMAX = 10;
  double FB_LIN_UMIN = -10;
  double g_x = 1;
  double accel_d = 1;

  double vel = 0;
  double vel_d = 5;
  double distance = 0.1;

  PIDLin pid(FB_LIN_UMAX, FB_LIN_UMIN, params);

  EXPECT_LE(pid.calculateManipulations(vel, vel_d, accel_d), FB_LIN_UMAX);
  EXPECT_GE(pid.calculateManipulations(vel, -vel_d, accel_d), FB_LIN_UMIN);

  FB_LIN_UMIN = 0;
  pid = PIDLin(FB_LIN_UMAX, FB_LIN_UMIN, params);

  EXPECT_GE(pid.calculateManipulations(vel, -vel_d, accel_d), FB_LIN_UMIN);
}