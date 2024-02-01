// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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

#include "nav2_behavior_tree/plugins/condition/navigation_state_comparison_condition.hpp"

namespace nav2_behavior_tree
{

NavigationStateComparisonCondition::NavigationStateComparisonCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  current_navigation_state_("none"),
  navigation_state_("none")
{
  getInput("current_navigation_state", current_navigation_state_);
  getInput("navigation_state", navigation_state_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  navigation_state_pub_ = node_->create_publisher<std_msgs::msg::String>("/bt/navigation_state", qos);
}

BT::NodeStatus NavigationStateComparisonCondition::tick()
{
  getInput("current_navigation_state", current_navigation_state_);
  getInput("navigation_state", navigation_state_);

  std_msgs::msg::String msg;
  msg.data = current_navigation_state_;
  navigation_state_pub_->publish(msg);

  if (current_navigation_state_ == navigation_state_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::NavigationStateComparisonCondition>("NavigationStateComparison");
}
