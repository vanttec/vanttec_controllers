/** ----------------------------------------------------------------------------
 * @file: uuv_control_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS control node for the UUV. Uses uuv_control library.
 * -----------------------------------------------------------------------------
 **/

#include "6dof_pid.hpp"
#include "vtec_u4_6dof_dynamic_model.hpp"
#include "vanttec_msgs/EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle nh("~");
    
    ros::Rate      cycle_rate(int(1 / SAMPLE_TIME_S));
    std::vector<float> g_p;
    std::vector<float> g_i;
    std::vector<float> g_d;

    nh.getParam("k_p", g_p);
    nh.getParam("k_i", g_i);
    nh.getParam("k_d", g_d);

    float* k_p = &g_p[0];
    float* k_i = &g_i[0];
    float* k_d = &g_d[0];

    DOFControllerType_E types[6] = {LINEAR_DOF, LINEAR_DOF, LINEAR_DOF, ANGULAR_DOF, ANGULAR_DOF, ANGULAR_DOF};

    SixDOFPID   system_controller(SAMPLE_TIME_S, k_p, k_i, k_d, types);
    
    ros::Publisher  uuv_thrust      = nh.advertise<vanttec_msgs::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1000);
    ros::Subscriber uuv_dynamics    = nh.subscribe("/uuv_simulation/dynamic_model/non_linear_functions", 10, 
                                                    &SixDOFPID::UpdateDynamics,
                                                    &system_controller);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/eta_pose", 10,
                                                    &SixDOFPID::UpdatePose,
                                                    &system_controller);

    ros::Subscriber uuv_setpoint    = nh.subscribe("/uuv_control/uuv_control_node/setpoint", 10,
                                                    &SixDOFPID::UpdateSetPoints,
                                                    &system_controller); 

    int counter = 0;
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Update Parameters with new info */ 
        system_controller.CalculateManipulations();
       
        /* Publish Odometry */
        // Current way: if no functions arrive through the subscriber, the last computed thrusts are published.
        // An option could be to use ros::topic::waitForMessage to publish once the nonlinear functioncs arrive, in order to
        // avoid publishing garbage.
        uuv_thrust.publish(system_controller.thrust);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}