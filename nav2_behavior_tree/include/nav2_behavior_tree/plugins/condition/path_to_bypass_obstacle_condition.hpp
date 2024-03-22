// Copyright (c) 2020 Aitor Miguel Blanco
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_TO_BYPASS_OBSTACLE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_TO_BYPASS_OBSTACLE_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/waypoint.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when goal is
 * updated on the blackboard and FAILURE otherwise
 */
class PathToBypassObstacleCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PathToBypassObstacleCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PathToBypassObstacleCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  PathToBypassObstacleCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Odometry>("odometry_gps", "GPS position"),
      BT::InputPort<double>("max_position_covariance", 1.0, "Maximum value of GPS position covariance"),
      BT::InputPort<double>("max_angle_covariance", 1.0, "Maximum value of GPS angle covariance"),
      BT::InputPort<nav2_msgs::msg::Waypoint>("waypoint", "The nearest waypoint"),
      BT::InputPort<std::string>("obstacle_mode", std::string("audo"), "Mode selection when encountering obstacles"),
    };
  }
  

private:
  rclcpp::Node::SharedPtr node_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_UPDATED_CONDITION_HPP_
