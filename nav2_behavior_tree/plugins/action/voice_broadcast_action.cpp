// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/action/voice_broadcast_action.hpp"

namespace nav2_behavior_tree
{

VoiceBroadcast::VoiceBroadcast(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.transient_local().reliable();
  voice_text_pub_ = 
    node->create_publisher<std_msgs::msg::String>("/voice", qos);
}

inline BT::NodeStatus VoiceBroadcast::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  std::string text;
  getInput("text", text);

  std_msgs::msg::String msg;
  msg.data = text;
  voice_text_pub_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::VoiceBroadcast>("VoiceBroadcast");
}
