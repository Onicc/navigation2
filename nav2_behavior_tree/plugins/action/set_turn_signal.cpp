// Copyright (c) 2021 Samsung Research America
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
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/set_turn_signal.hpp"

namespace nav2_behavior_tree
{

SetTurnSignal::SetTurnSignal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  turn_signal_pub_ = node_->create_publisher<std_msgs::msg::String>("vehicle/command/turn_signal", qos);
}

inline BT::NodeStatus SetTurnSignal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  std::string turn_signal;
  getInput("turn_signal", turn_signal);

  std_msgs::msg::String msg;
  msg.data = turn_signal;
  turn_signal_pub_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SetTurnSignal>("SetTurnSignal");
}
