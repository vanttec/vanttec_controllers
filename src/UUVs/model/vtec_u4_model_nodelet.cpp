/** ----------------------------------------------------------------------------
 * @file: vtec_u4_model_nodelet.cpp
 * @date: August 30, 2022
 * @author: Sebastian Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: VTec U4 model nodelet. Does not currently work. For some reason the
 *         dynamic_cast of the current file can not be compiled.
 * -----------------------------------------------------------------------------
**/

#include <pluginlib/class_list_macros.h>
#include "vtec_u4_model_nodelet.hpp"

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(VTecU4ModelNodelet, nodelet::Nodelet)

VTecU4ModelNodelet::VTecU4ModelNodelet(){}
VTecU4ModelNodelet::~VTecU4ModelNodelet(){}

void VTecU4ModelNodelet::onInit()
{
    static const float SAMPLE_TIME_S = 0.01;
    NODELET_DEBUG("Initializing VTec U4 model control nodelet...");
    ros::NodeHandle nh = getNodeHandle();
        
    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));
    VTecU4DynamicModel      uuv_model(SAMPLE_TIME_S);
    vanttec_msgs::SystemDynamics  uuv_functions;
    
    ros::Publisher  uuv_accel     = nh.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_acc", 1000);
    ros::Publisher  uuv_vel       = nh.advertise<geometry_msgs::Twist>("/uuv_simulation/dynamic_model/vel", 1000);
    ros::Publisher  uuv_eta_pose  = nh.advertise<vanttec_msgs::EtaPose>("/uuv_simulation/dynamic_model/eta_pose", 1000);
    ros::Publisher  uuv_dynamics  = nh.advertise<vanttec_msgs::SystemDynamics>("/uuv_dynamics/non_linear_functions", 10);

    ros::Subscriber uuv_thrust_input = nh.subscribe("/uuv_control/uuv_control_node/thrust", 
                                                    10, 
                                                    &Generic6DOFUUVDynamicModel::ThrustCallback,
                                                    dynamic_cast<Generic6DOFUUVDynamicModel*> (&uuv_model));
            
    uuv_functions.g.layout.dim.push_back(std_msgs::MultiArrayDimension());
    uuv_functions.g.layout.dim.push_back(std_msgs::MultiArrayDimension());
    uuv_functions.g.layout.dim[0].label = "rows";
    uuv_functions.g.layout.dim[1].label = "cols";
    uuv_functions.g.layout.dim[0].size = 6;
    uuv_functions.g.layout.dim[1].size = 6;
    uuv_functions.g.layout.dim[0].stride = 6;
    uuv_functions.g.layout.data_offset = 0;

    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* Calculate Model States */
        uuv_model.CalculateStates();

        /* Publish Odometry */
        uuv_accel.publish(uuv_model.accelerations);
        uuv_vel.publish(uuv_model.velocities);
        uuv_eta_pose.publish(uuv_model.eta_pose);
        
        /* Publish nonlinear functions */

        uuv_functions.f = {uuv_model.f(0), uuv_model.f(1), uuv_model.f(2), uuv_model.f(3), uuv_model.f(4), uuv_model.f(5)};
        uuv_functions.g.data = { uuv_model.g(0,0), uuv_model.g(0,1), uuv_model.g(0,2), uuv_model.g(0,3), uuv_model.g(0,4), uuv_model.g(0,5),
                                 uuv_model.g(1,0), uuv_model.g(1,1), uuv_model.g(1,2), uuv_model.g(1,3), uuv_model.g(1,4), uuv_model.g(1,5),
                                 uuv_model.g(2,0), uuv_model.g(2,1), uuv_model.g(2,2), uuv_model.g(2,3), uuv_model.g(2,4), uuv_model.g(2,5),
                                 uuv_model.g(3,0), uuv_model.g(3,1), uuv_model.g(3,2), uuv_model.g(3,3), uuv_model.g(3,4), uuv_model.g(3,5),
                                 uuv_model.g(4,0), uuv_model.g(4,1), uuv_model.g(4,2), uuv_model.g(4,3), uuv_model.g(4,4), uuv_model.g(4,5),
                                 uuv_model.g(5,0), uuv_model.g(5,1), uuv_model.g(5,2), uuv_model.g(5,3), uuv_model.g(5,4), uuv_model.g(5,5) };
        
        uuv_dynamics.publish(uuv_functions);

        /* Sleep for 10ms */
        cycle_rate.sleep();
    }
}