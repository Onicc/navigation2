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
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/plugins/condition/path_to_curb_follow_condition.hpp"

namespace nav2_behavior_tree
{

PathToCurbFollowCondition::PathToCurbFollowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus PathToCurbFollowCondition::tick()
{
  nav_msgs::msg::Odometry odometry_gps;
  double max_position_covariance;
  double max_angle_covariance;
  nav2_msgs::msg::Waypoint waypoint;
  geometry_msgs::msg::PoseStamped curb_traction_point;

  getInput("odometry_gps", odometry_gps);
  getInput("max_position_covariance", max_position_covariance);
  getInput("max_angle_covariance", max_angle_covariance);
  getInput("waypoint", waypoint);
  getInput("curb_traction_point", curb_traction_point);

  // Check if the odometry is valid.
  if(odometry_gps.pose.covariance[0] < max_position_covariance &&
    odometry_gps.pose.covariance[7] < max_position_covariance &&
    odometry_gps.pose.covariance[14] < max_position_covariance &&
    odometry_gps.pose.covariance[21] < max_angle_covariance &&
    odometry_gps.pose.covariance[28] < max_angle_covariance &&
    odometry_gps.pose.covariance[35] < max_angle_covariance) {
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(), "[PathToCurbFollowCondition] Poor GPS quality, meets switching conditions.");

  // Check if the curb following option for the waypoint points is turned on.
  if(waypoint.option_curb_traction_fix == false) {
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(), "[PathToCurbFollowCondition] Curb traction option is on at the waypoints, meets switching conditions.");

  // Check if the real-time curb traction points are being updated.
  if(curb_traction_point == last_curb_traction_point_) {
    RCLCPP_INFO(node_->get_logger(), "[PathToCurbFollowCondition] 实时路牙不满足切换条件");
    return BT::NodeStatus::FAILURE;
  }
  last_curb_traction_point_ = curb_traction_point;
  RCLCPP_INFO(node_->get_logger(), "[PathToCurbFollowCondition] 实时路牙满足切换条件");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathToCurbFollowCondition>("PathToCurbFollow");
}
