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

#include "nav2_behavior_tree/plugins/action/update_goal_action.hpp"

namespace nav2_behavior_tree
{

UpdateGoalAction::UpdateGoalAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus UpdateGoalAction::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path path;
  int input_index;
  double increment;
  int replanning_count;

  getInput("input_path", path);
  getInput("input_index", input_index);
  getInput("increment", increment);
  getInput("replanning_count", replanning_count);

  geometry_msgs::msg::PoseStamped output_goal;
  int output_index;
  if (input_index >= 0 && input_index < static_cast<int>(path.poses.size())) {
    for (size_t i = input_index; i < path.poses.size(); i++) {
      double curr_dist = nav2_util::geometry_utils::euclidean_distance(
                  path.poses[input_index], path.poses[i]);
      if (curr_dist > 6+increment*replanning_count) {
        output_goal = path.poses[i];
        output_index = i;
        break;
      }
    }
    setOutput("output_goal", output_goal);
    setOutput("output_index", output_index);

    std::cout << "replanning_count: " << replanning_count << std::endl;
    std::cout << "output_goal x: " << output_goal.pose.position.x << std::endl;
    std::cout << "output_goal y: " << output_goal.pose.position.y << std::endl;

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::UpdateGoalAction>("UpdateGoal");
}
