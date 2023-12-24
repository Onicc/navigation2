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

#include "nav2_behavior_tree/plugins/condition/command_match_condition.hpp"

namespace nav2_behavior_tree
{

CommandMatchCondition::CommandMatchCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  command_.data = "none0";
  match_command_ = "none1";

  getInput("command", command_);
  getInput("match_command", match_command_);
}

BT::NodeStatus CommandMatchCondition::tick()
{
  getInput("command", command_);
  getInput("match_command", match_command_);

  // std::cout << "command: " << command_.data << std::endl;
  // std::cout << "match_command: " << match_command_ << std::endl;

  if (command_.data == match_command_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CommandMatchCondition>("CommandMatch");
}
