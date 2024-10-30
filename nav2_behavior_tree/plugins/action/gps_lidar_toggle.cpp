// Copyright (c) 2021 RoboTech Vision
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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_behavior_tree/plugins/action/gps_lidar_toggle.hpp"

namespace nav2_behavior_tree
{

GpsLidarToggle::GpsLidarToggle(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  set_location_selector_publisher = node->create_publisher<std_msgs::msg::String>("/location_selector/set", qos);
}

inline BT::NodeStatus GpsLidarToggle::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav2_msgs::msg::Waypoint waypoint;
  nav_msgs::msg::Odometry odometry_gps;
  nav_msgs::msg::Odometry odometry_lidar;
  double gps_position_covariance_threshold;
  double gps_orientation_covariance_threshold;
  double lidar_position_covariance_threshold;
  double lidar_orientation_covariance_threshold;
  getInput("waypoint", waypoint);
  getInput("odometry_gps", odometry_gps);
  getInput("odometry_lidar", odometry_lidar);
  getInput("gps_position_covariance_threshold", gps_position_covariance_threshold);
  getInput("gps_orientation_covariance_threshold", gps_orientation_covariance_threshold);
  getInput("lidar_position_covariance_threshold", lidar_position_covariance_threshold);
  getInput("lidar_orientation_covariance_threshold", lidar_orientation_covariance_threshold);

  int localization_method = waypoint.option_localization_method;  // 0: None, 1: GPS, 2: Lidar
  std::string localization_method_str;

  if (odometry_gps.pose.covariance[0] < gps_position_covariance_threshold &&
      odometry_gps.pose.covariance[7] < gps_position_covariance_threshold &&
      odometry_gps.pose.covariance[35] < gps_orientation_covariance_threshold) {
    if (localization_method == 1) {
      localization_method_str = "gps";
    } else if (localization_method == 2) {
      if (odometry_lidar.pose.covariance[0] < lidar_position_covariance_threshold &&
          odometry_lidar.pose.covariance[7] < lidar_position_covariance_threshold &&
          odometry_lidar.pose.covariance[35] < lidar_orientation_covariance_threshold) {
        localization_method_str = "lidar";
      } else {
        // RCLCPP_ERROR(
        //   node_->get_logger(),
        //   "Lidar localization is not reliable.");
        return BT::NodeStatus::FAILURE;
      }
    } else {
      // RCLCPP_ERROR(
      //   node_->get_logger(),
      //   "Invalid localization method: %d", localization_method);
      return BT::NodeStatus::FAILURE;
    }
  } else {
    if (odometry_lidar.pose.covariance[0] < lidar_position_covariance_threshold &&
        odometry_lidar.pose.covariance[7] < lidar_position_covariance_threshold &&
        odometry_lidar.pose.covariance[35] < lidar_orientation_covariance_threshold) {
      localization_method_str = "lidar";
    } else {
      // RCLCPP_ERROR(
      //   node_->get_logger(),
      //   "GPS localization is not reliable.");
      return BT::NodeStatus::FAILURE;
    }
  }

  std_msgs::msg::String msg;
  msg.data = localization_method_str;
  set_location_selector_publisher->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::GpsLidarToggle>(
    "GpsLidarToggle");
}
