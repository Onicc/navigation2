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
#include "nav2_behavior_tree/plugins/condition/traffic_light_detect_condition.hpp"

namespace nav2_behavior_tree
{

TrafficLightCondition::TrafficLightCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus TrafficLightCondition::tick()
{
  nav2_msgs::msg::Waypoint waypoint;
  std::string detect_traffic_light;
  std::string goal_traffic_light;

  getInput("waypoint", waypoint);
  getInput("detect_traffic_light", detect_traffic_light);
  getInput("goal_traffic_light", goal_traffic_light);

  // RCLCPP_INFO(node_->get_logger(), "[TrafficLightCondition] Current traffic light is %s", detect_traffic_light.c_str());

  if(waypoint.option_traffic_light == true) {
    if(goal_traffic_light == goal_traffic_light) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TrafficLightCondition>("TrafficLight");
}
