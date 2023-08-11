/** ----------------------------------------------------------------------------
 * @file: tf2_6dof_broadcaster.hpp
 * @date: August 10, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Used to publish the current pose of the simulated vehicle and correctly
 *         represent it in RViz.
 * -----------------------------------------------------------------------------
 **/

#ifndef __TF2_6DOF_BROADCASTER2_H__
#define __TF2_6DOF_BROADCASTER2_H__

#include <math.h>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <string.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sdv_msgs/msg/eta_pose.hpp"

class TF2Broadcaster
{
    public:
        
        TF2Broadcaster(rclcpp::Node::SharedPtr node,const std::string& _parent, const std::string& _child);
        virtual ~TF2Broadcaster();
        
        nav_msgs::msg::Path  path_;

        std::string parent_frame;
        std::string child_frame;
        /* Class methods */
        void BroadcastTransform(const sdv_msgs::msg::EtaPose& msg);
    private:
        rclcpp::Node::SharedPtr node_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
        rclcpp::Subscription<sdv_msgs::msg::EtaPose>::SharedPtr sub_;

};

#endif