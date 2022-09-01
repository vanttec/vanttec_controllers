/** ----------------------------------------------------------------------------
 * @file: vtec_u4_6dof_dynamic_model.hpp
 * @date: March 20, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of VTec U4 6dof dynamic model in the non-inertial frame
           with Euler Angles.
 * -----------------------------------------------------------------------------
 **/

#ifndef __VTEC_U4_6DOF_DYNAMIC_MODEL__
#define __VTEC_U4_6DOF_DYNAMIC_MODEL__

#include "generic_6dof_uuv_dynamic_model.hpp"

class VTecU4DynamicModel : public Generic6DOFUUVDynamicModel
{
    public:
        VTecU4DynamicModel(const float& sample_time);
        ~VTecU4DynamicModel();

        void updateThrust();
        
    private:
        Eigen::MatrixXf L_;          // Thrust mapping matrix
        Eigen::VectorXf Thrust_;     // Thrust vector (6 elements, 1 per thruster)

        /* Thruster configuration parameters */

        float beta_;
        float var_epsilon_;
        float alpha_;
        float delta_;
        float gamma_;
        float rh_x_;
        float rh_y_;
        float rh_z_;
        float rv_x_;
        float rv_y_;
        float rv_z_;
};

#endif