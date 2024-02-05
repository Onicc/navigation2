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

#include "nav2_behavior_tree/plugins/action/exception_action.hpp"

#define GET_INFO_STRING(file, line, func) ((std::string("File: ") + file + ", Line: " + std::to_string(line) + ", Function: " + func))

namespace nav2_behavior_tree
{

ExceptionAction::ExceptionAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.transient_local().reliable();
  excption_pub_ = 
    node_->create_publisher<nav2_msgs::msg::Exception>("/bt/excption", qos);
}

inline BT::NodeStatus ExceptionAction::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  std::string message;
  getInput("message", message);

  RCLCPP_INFO(node_->get_logger(), "Exception: %s", message.c_str());

  nav2_msgs::msg::Exception exception;
  exception.header.stamp = node_->now();
  exception.message = message;
  exception.error_code = 1;
  const char* fileName = __FILE__;
  int lineNumber = __LINE__;
  const char* functionName = __func__;
  exception.traceback_info = GET_INFO_STRING(fileName, lineNumber, functionName);

  excption_pub_->publish(exception);
  setOutput("exception", exception);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ExceptionAction>("Exception");
}
