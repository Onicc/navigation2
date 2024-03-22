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
#include "nav2_behavior_tree/plugins/condition/obstacle_option_condition.hpp"

namespace nav2_behavior_tree
{

ObstacleOptionCondition::ObstacleOptionCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus ObstacleOptionCondition::tick()
{
  nav2_msgs::msg::Waypoint waypoint;
  std::string obstacle_mode;

  getInput("waypoint", waypoint);
  getInput("obstacle_mode", obstacle_mode);

  if(obstacle_mode == "auto") {
    return BT::NodeStatus::SUCCESS;
  } else if(obstacle_mode == "bypass") {
    if(waypoint.option_bypass_obstacle == true) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  } else if(obstacle_mode == "stop") {
    if(waypoint.option_stop_obstacle == true) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  } else if(obstacle_mode == "off") {
    if(waypoint.option_stop_obstacle == false && waypoint.option_bypass_obstacle == false) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }

  // RCLCPP_INFO(node_->get_logger(), "[ObstacleOptionCondition] This waypoint allows obstacle avoidance.");

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ObstacleOptionCondition>("ObstacleOption");
}
