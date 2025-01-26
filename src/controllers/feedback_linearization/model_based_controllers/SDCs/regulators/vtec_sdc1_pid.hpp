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

#include "controllers/feedback_linearization/PID/first_order/fblin_pid.hpp"
#include "dynamic_models/ground_vehicles/car_like/vehicles/vtec_sdc1.hpp"
#include "utils/utils.hpp"

class VTEC_SDC1_1DOF_PID : public PIDLin {
  public:
    VTEC_SDC1_1DOF_PID(const PIDParameters &params, VTecSDC1DynamicModel* model);
    ~VTEC_SDC1_1DOF_PID();

    void updateNonLinearFunctions();

    double calculateControlSignals(double chi1, double chi1_d, double chi1_dot_d);

    void updateControlSignals();

    VTecSDC1DynamicModel* model;
};