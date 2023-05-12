/** ----------------------------------------------------------------------------
 * @file: vtec_u4_6dof_in_pid.hpp
 * @date: April 27, 2023
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a 6DOF PID Controller for the VTec U4 inertial dyn model
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/marine_vehicles/underwater/vehicles/vtec_u4_in_6dof_dynamic_model.hpp"
#include "controllers/feedback_linearization/PID_based/second_order/6dof_pid.hpp"

class VTEC_U4_6DOF_PID : public VTecU4InDynamicModel, public PID6DOF
{
    public:
        VTEC_U4_6DOF_PID(float sample_time, const std::vector<float>& k_p, const std::vector<float>& k_i, 
                const std::vector<float>& k_d, const std::array<float,6>& u_max,
                const std::array<DOFControllerType_E,6>& type);
        ~VTEC_U4_6DOF_PID();

        void updateNonLinearFunctions();

        void calculateControlSignals();

        void updateControlSignals();
};