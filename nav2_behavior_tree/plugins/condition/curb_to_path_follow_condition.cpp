// Copyright (c) 2020 Aitor Miguel Blanco
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include "nav2_behavior_tree/plugins/condition/curb_to_path_follow_condition.hpp"

namespace nav2_behavior_tree
{

CurbToPathFollowCondition::CurbToPathFollowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), 
  initialized_(false)
{}

BT::NodeStatus CurbToPathFollowCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  nav_msgs::msg::Odometry odometry_gps;
  double max_position_covariance;
  double max_angle_covariance;
  nav2_msgs::msg::Waypoint waypoint;
  geometry_msgs::msg::PoseStamped curb_traction_point;
  double gps_recovery_hold_time;

  getInput("odometry_gps", odometry_gps);
  getInput("max_position_covariance", max_position_covariance);
  getInput("max_angle_covariance", max_angle_covariance);
  getInput("waypoint", waypoint);
  getInput("curb_traction_point", curb_traction_point);
  getInput("gps_recovery_hold_time", gps_recovery_hold_time);

  /* ------------------------------------------------------------ */
  if(waypoint.option_curb_traction_fix == false) {
    RCLCPP_INFO(node_->get_logger(), "[CurbToPathFollowCondition] 当前路径不支持路牙跟随");
    return BT::NodeStatus::SUCCESS;
  }

  /* ------------------------------------------------------------ */
  if(curb_traction_point == last_curb_traction_point_) {
    RCLCPP_INFO(node_->get_logger(), "[CurbToPathFollowCondition] 实时路牙没有更新");

    // Get the current position in the odom frame.
    geometry_msgs::msg::PoseStamped current_pose;
    global_frame_ = "odom";
    robot_base_frame_ = "base_link";
    if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
      RCLCPP_INFO(node_->get_logger(), "[CurbToPathFollowCondition] Current robot pose is not available.");
      return BT::NodeStatus::SUCCESS;
    }

    double dx = current_pose.pose.position.x - curb_traction_point.pose.position.x;
    double dy = current_pose.pose.position.y - curb_traction_point.pose.position.y;
    if((dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_)) {
      RCLCPP_INFO(node_->get_logger(), "[CurbToPathFollowCondition] 到达路牙引导点并且没有更新新的路牙引导点");
      return BT::NodeStatus::SUCCESS;
    }
  }
  last_curb_traction_point_ = curb_traction_point;

  /* ------------------------------------------------------------ */
  if(odometry_gps.pose.covariance[0] < max_position_covariance &&
    odometry_gps.pose.covariance[7] < max_position_covariance &&
    odometry_gps.pose.covariance[14] < max_position_covariance &&
    odometry_gps.pose.covariance[21] < max_angle_covariance &&
    odometry_gps.pose.covariance[28] < max_angle_covariance &&
    odometry_gps.pose.covariance[35] < max_angle_covariance) {
    auto now_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - gps_lost_time_);

    RCLCPP_INFO(node_->get_logger(), "[CurbToPathFollowCondition] 检测到GPS恢复，持续恢复时间为 %f 秒", duration.count() / 1000.0);
    if(duration.count() > gps_recovery_hold_time * 1000) {
      RCLCPP_INFO(node_->get_logger(), "[CurbToPathFollowCondition] GPS恢复时间超过 %f 秒，切换到路径跟随", gps_recovery_hold_time);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    gps_lost_time_ = std::chrono::high_resolution_clock::now();
  }

  return BT::NodeStatus::FAILURE;
}

void CurbToPathFollowCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_reached_tol",
    rclcpp::ParameterValue(0.25));
  node_->get_parameter_or<double>("goal_reached_tol", goal_reached_tol_, 0.25);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_);

  initialized_ = true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CurbToPathFollowCondition>("CurbToPathFollow");
}
