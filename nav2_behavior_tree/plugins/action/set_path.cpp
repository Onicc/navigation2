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

#include "nav2_behavior_tree/plugins/action/set_path.hpp"

namespace nav2_behavior_tree
{

SetPath::SetPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  current_time_ = node_->now();
  last_time_ = node_->now();

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  bypass_path_now_ =
    node_->create_publisher<nav_msgs::msg::Path>("/bypass_path_now", qos);
  bypass_path_last_ =
    node_->create_publisher<nav_msgs::msg::Path>("/bypass_path_last", qos);
}

inline BT::NodeStatus SetPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  getInput("input_path", input_path_);
  getInput("interval", time_interval_);

  bypass_path_now_->publish(input_path_);

  current_time_ = node_->now();
  if ((current_time_ - last_time_).seconds() > time_interval_) {
    last_time_ = current_time_;
    bypass_path_last_->publish(input_path_);
    setOutput("output_path", input_path_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SetPath>("SetPath");
}
