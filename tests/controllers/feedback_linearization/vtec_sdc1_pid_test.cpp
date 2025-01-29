// #include <gtest/gtest.h>
// #include "controllers/feedback_linearization/base/fb_lin_control.hpp"

// TEST(FBLin, SimpleModel){
//   double f_x{5};
//   double g_x{1};

//   double U_MAX{255};
//   double U_MIN{-255};
//   double u_aux{10};
//   double u_n{2};
//   double chiX_dot_d{1};

//   FBLin controller(U_MAX, U_MIN);
//   controller.f_x_ = f_x;
//   controller.g_x_ = g_x;
  
//   controller.u_n_ = u_n;
//   controller.u_aux_ = u_aux;
//   controller.chiX_dot_d_ = chiX_dot_d;

//   controller.updateControlSignal();
//   // u_ = g_x_^(-1)*(chi1_dot_d - f_x_ + u_n - u_aux)

//   EXPECT_EQ(controller.u_, -12);
// }

// TEST(FBLin, ClampU){
//   double f_x{5};
//   double g_x{1};

//   double U_MAX{255};
//   double U_MIN{-255};
//   double u_aux{300};
//   double u_n{2};
//   double chiX_dot_d{1};

//   FBLin controller(U_MAX, U_MIN);
//   controller.f_x_ = f_x;
//   controller.g_x_ = g_x;
  
//   controller.u_n_ = u_n;
//   controller.u_aux_ = u_aux;
//   controller.chiX_dot_d_ = chiX_dot_d;

//   controller.updateControlSignal();
//   // u_ = g_x_^(-1)*(chi1_dot_d - f_x_ + u_n - u_aux)

//   EXPECT_LE(controller.u_, U_MAX);
//   EXPECT_GE(controller.u_, U_MIN);
// }