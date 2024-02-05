// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include "nav2_behavior_tree/plugins/condition/is_exception_updated_condition.hpp"

namespace nav2_behavior_tree
{

IsExceptionUpdatedCondition::IsExceptionUpdatedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus IsExceptionUpdatedCondition::tick()
{
  nav2_msgs::msg::Exception exception;
  if (!config().blackboard->get("exception", exception)) {
    RCLCPP_ERROR(node_->get_logger(), "Exception message not found");
    return BT::NodeStatus::FAILURE;
  }

  double update_time;
  getInput("update_time", update_time);

  const auto age_of_topic = node_->now() - exception.header.stamp;
  if (age_of_topic < rclcpp::Duration::from_seconds(update_time)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsExceptionUpdatedCondition>("IsExceptionUpdated");
}