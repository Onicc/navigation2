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
#include "nav2_behavior_tree/plugins/condition/path_to_bypass_obstacle_condition.hpp"

namespace nav2_behavior_tree
{

PathToBypassObstacleCondition::PathToBypassObstacleCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus PathToBypassObstacleCondition::tick()
{
  nav_msgs::msg::Odometry odometry_gps;
  double max_position_covariance;
  double max_angle_covariance;
  nav2_msgs::msg::Waypoint waypoint;

  getInput("odometry_gps", odometry_gps);
  getInput("max_position_covariance", max_position_covariance);
  getInput("max_angle_covariance", max_angle_covariance);
  getInput("waypoint", waypoint);

  if(waypoint.option_bypass_obstacle == false) {
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(), "[PathToBypassObstacleCondition] This waypoint allows obstacle avoidance.");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathToBypassObstacleCondition>("PathToBypassObstacle");
}
