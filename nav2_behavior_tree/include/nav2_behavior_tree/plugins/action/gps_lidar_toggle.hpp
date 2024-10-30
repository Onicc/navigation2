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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GPS_LIDAR_TOGGLE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GPS_LIDAR_TOGGLE_ACTION_HPP_

#include <memory>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

#include "nav2_msgs/msg/waypoint_array.hpp"
#include "nav2_msgs/msg/waypoint.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class GpsLidarToggle : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief A nav2_behavior_tree::GpsLidarToggle constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GpsLidarToggle(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav2_msgs::msg::Waypoint>("waypoint", "Nearest waypoint to robot"),
      BT::InputPort<nav_msgs::msg::Odometry>("odometry_gps", "GPS position"),
      BT::InputPort<nav_msgs::msg::Odometry>("odometry_lidar", "Lidar position"),
      BT::InputPort<double>("gps_position_covariance_threshold", 0.0001, "Threshold for GPS position covariance"),
      BT::InputPort<double>("gps_orientation_covariance_threshold", 0.0001, "Threshold for GPS orientation covariance"),
      BT::InputPort<double>("lidar_position_covariance_threshold", 0.0001, "Threshold for Lidar position covariance"),
      BT::InputPort<double>("lidar_orientation_covariance_threshold", 0.0001, "Threshold for Lidar orientation covariance"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_location_selector_publisher;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_NEAREST_POINT_INDEX_ACTION_HPP_
