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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CURB_TO_PATH_FOLLOW_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CURB_TO_PATH_FOLLOW_CONDITION_HPP_

#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
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
class CurbToPathFollowCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::CurbToPathFollowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  CurbToPathFollowCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  CurbToPathFollowCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

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
      BT::InputPort<geometry_msgs::msg::PoseStamped>("curb_traction_point", "Curb traction points in the odom frame"),
      BT::InputPort<double>("gps_recovery_hold_time", 1.0, "Seconds to hold position when recovering from a bad GPS reading"),
    };
  }
  

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool initialized_;
  double goal_reached_tol_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;

  geometry_msgs::msg::PoseStamped last_curb_traction_point_;

  std::chrono::high_resolution_clock::time_point gps_lost_time_;  // time when GPS was lost
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_UPDATED_CONDITION_HPP_
