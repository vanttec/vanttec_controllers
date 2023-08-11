/** ----------------------------------------------------------------------------
 * @file: tf2_6dof_broadcaster.cpp
 * @date: August 10, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Used to publish the current pose of the simulated vehicle and correctly
 *         represent it in RViz.
 * https://github.com/ros-visualization/rviz/issues/597
 * -----------------------------------------------------------------------------
 **/
#include <memory>
#include <chrono>
#include "simulation/tf2_6dof_broadcaster_ros2.hpp"

TF2Broadcaster::TF2Broadcaster(rclcpp::Node::SharedPtr node,const std::string& _parent, const std::string& _child): node_(node), parent_frame(_parent), child_frame(_child)
{

br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
sub_ = node_->create_subscription<sdv_msgs::msg::EtaPose>("/car_simulation/dynamic_model/eta_pose",  10,std::bind(&TF2Broadcaster::BroadcastTransform, this, std::placeholders::_1));
}

TF2Broadcaster::~TF2Broadcaster(){}

void TF2Broadcaster::BroadcastTransform(const vanttec_msgs::EtaPose& _pose)
{    
    geometry_msgs::TransformStamped transformStamped;
    
    // From NED to NWU RViz reference frame (x forward, y left, z up): negate y and z
    transformStamped.header.stamp               = node_->now();
    transformStamped.header.frame_id            = this->parent_frame;
    transformStamped.child_frame_id             = this->child_frame;
    transformStamped.transform.translation.x    = _pose.x;
    transformStamped.transform.translation.y    = -_pose.y;
    transformStamped.transform.translation.z    = -_pose.z;

    tf2::Quaternion q;
    q.setRPY(_pose.phi, -_pose.theta, -_pose.psi);
    q.normalize();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    this->br_.sendTransform(transformStamped);

    geometry_msgs::PoseStamped      pose;

    pose.header.stamp       = node_->now();
    pose.header.frame_id    = this->parent_frame;
    pose.pose.position.x    = _pose.x;
    pose.pose.position.y    = -_pose.y;
    pose.pose.position.z    = -_pose.z;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    this->path_.header.stamp     = node_->now();
    this->path_.header.frame_id  = this->parent_frame;
    this->path_.poses.push_back(pose);
}