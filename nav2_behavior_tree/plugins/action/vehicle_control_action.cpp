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

#include "nav2_behavior_tree/plugins/action/vehicle_control_action.hpp"

namespace nav2_behavior_tree
{

VehicleControl::VehicleControl(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.transient_local().reliable();
  set_ros2_control_pub_ = 
    node->create_publisher<std_msgs::msg::Bool>("vehicle/command/ros2_control", qos);
  set_cruise_control_pub_ = 
    node->create_publisher<std_msgs::msg::Float32>("vehicle/command/cruise_control", qos);
  set_steering_pub_ = 
    node->create_publisher<std_msgs::msg::Int32>("vehicle/command/steering", qos);
}

inline BT::NodeStatus VehicleControl::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  int control;
  getInput("control", control);

  if(control == 0) {
    set_ros2_control_pub_->publish(std_msgs::msg::Bool().set__data(false));
    set_ros2_control_pub_->publish(std_msgs::msg::Bool().set__data(false));
    set_cruise_control_pub_->publish(std_msgs::msg::Float32().set__data(0.0));
    set_cruise_control_pub_->publish(std_msgs::msg::Float32().set__data(0.0));
    set_steering_pub_->publish(std_msgs::msg::Int32().set__data(0));
    set_steering_pub_->publish(std_msgs::msg::Int32().set__data(0));
  } else {
    set_ros2_control_pub_->publish(std_msgs::msg::Bool().set__data(true));
    set_ros2_control_pub_->publish(std_msgs::msg::Bool().set__data(true));
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::VehicleControl>("VehicleControl");
}
