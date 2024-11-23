/** ----------------------------------------------------------------------------
 * @file: vtec_sdc1_pid.hpp
 * @date: August 13, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 *
 * @brief: Description of a single DOF PID Controller for the VTec SDC1 dynamic
 *model
 * -----------------------------------------------------------------------------
 **/

#include "controllers/feedback_linearization/PID_based/first_order/fblin_pid.hpp"
#include "dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.hpp"
#include "utils/utils.hpp"

class VTEC_SDC1_1DOF_PID : public VTecSDC1DynamicModel, public PIDLin {
public:
  VTEC_SDC1_1DOF_PID(const PIDParameters &params);
  ~VTEC_SDC1_1DOF_PID();

  // void updateNonLinearFunctions();
  void updateNonLinearFunctions(double f_x, double g_x);

  // For simulations
  // void calculateControlSignals();

  // For real world
  double calculateControlSignals(double surge, double surge_d);

  void updateControlSignals();

  // void updateCurrentReference(float chi1_d, float chi1_dot_d);
};