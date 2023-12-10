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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_GOAL_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_GOAL_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class UpdateGoalAction : public BT::ActionNodeBase
{
public:
  UpdateGoalAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::InputPort<int>("input_index", -1, "Input goal path index"),
      BT::InputPort<double>("increment", 1.0, "Single distance increment value"),
      BT::InputPort<int>("replanning_count", 0, "Number of replans"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "Destination to plan to"),
      BT::OutputPort<int>("output_index", "Output the index of the path corresponding to the goal"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  int retry_count = 0;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
