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

#include "nav2_behavior_tree/plugins/action/toggle_forward_backward_mode_by_cmd.hpp"

namespace nav2_behavior_tree
{

ToggleForwardBackwardModeByCmd::ToggleForwardBackwardModeByCmd(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  set_robot_frame_pub_ = 
    node->create_publisher<std_msgs::msg::String>("/set_robot_frame", qos);
}

inline BT::NodeStatus ToggleForwardBackwardModeByCmd::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  geometry_msgs::msg::Twist cmd_vel;
  std::string base_link_frame;

  getInput("cmd_vel", cmd_vel);
  getInput("base_link_frame", base_link_frame);

  if(cmd_vel.linear.x > 0.1) {
    if(base_link_frame != "front_base_link") {
      std_msgs::msg::String msg;
      msg.data = "front_base_link";
      set_robot_frame_pub_->publish(msg);

      // RCLCPP_INFO(node_->get_logger(), "Switching to front_base_link");

      return BT::NodeStatus::FAILURE;
    }
  }
  if(cmd_vel.linear.x < -0.1) {
    if(base_link_frame != "rear_base_link") {
      std_msgs::msg::String msg;
      msg.data = "rear_base_link";
      set_robot_frame_pub_->publish(msg);

      // RCLCPP_INFO(node_->get_logger(), "Switching to rear_base_link");

      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ToggleForwardBackwardModeByCmd>("ToggleForwardBackwardModeByCmd");
}
