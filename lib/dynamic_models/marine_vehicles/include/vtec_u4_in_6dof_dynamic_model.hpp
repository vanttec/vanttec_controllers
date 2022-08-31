/** ----------------------------------------------------------------------------
 * @file: vtec_u4_in_6dof_dynamic_model.hpp
 * @date: August 31, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of VTec U4 6dof dynamic model in the inertial frame
           with Euler Angles.
 * -----------------------------------------------------------------------------
 **/

#ifndef __VTEC_U4_IN_6DOF_DYNAMIC_MODEL__
#define __VTEC_U4_IN_6DOF_DYNAMIC_MODEL__

#include "generic_in_6dof_uuv_dynamic_model.hpp"

class VTecU4InDynamicModel : public GenericIn6DOFUUVDynamicModel
{
    public:
        VTecU4InDynamicModel(float _sample_time_s);
        ~VTecU4InDynamicModel();

        void UpdateThrust();
        
    private:
        Eigen::MatrixXf L;          // Thrust mapping matrix
        Eigen::VectorXf Thrust;     // Thrust vector (6 elements, 1 per thruster)

        /* Thruster configuration parameters */

        float beta;
        float var_epsilon;
        float alpha;
        float delta;
        float gamma;
        float rh_x;
        float rh_y;
        float rh_z;
        float rv_x;
        float rv_y;
        float rv_z;
};

#endif