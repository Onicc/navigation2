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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_FARTHEST_OBSTACLE_POINT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_FARTHEST_OBSTACLE_POINT_ACTION_HPP_

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

class FindFarthestObstaclePoint : public BT::ActionNodeBase
{
public:
  FindFarthestObstaclePoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::InputPort<int>("input_index", -1, "Input goal path index"),
      BT::InputPort<double>("resolution", 0.1, "Single distance increment value"),
      BT::InputPort<int>("replanning_count", 0, "Number of replans"),
      BT::InputPort<double>("find_distance", 10.0, "furthest search range"),
      BT::InputPort<double>("back_find_distance", 2.0, "furthest search range"),
      BT::InputPort<double>("safe_distance", 4.0, "furthest search range"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("farthest_obstacle_point", "Destination to plan to"),
      // BT::OutputPort<int>("farthest_obstacle_index", "Output the index of the path corresponding to the goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("recovery_point", "Destination to plan to"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  int retry_count = 0;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_FARTHEST_OBSTACLE_POINT_ACTION_HPP_
