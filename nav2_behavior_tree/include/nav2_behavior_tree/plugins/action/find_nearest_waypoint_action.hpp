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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_NEAREST_WAYPOINT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_NEAREST_WAYPOINT_ACTION_HPP_

#include <memory>
#include <string>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int32.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

#include "nav2_msgs/msg/waypoint_array.hpp"
#include "nav2_msgs/msg/waypoint.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class FindNearestWaypoint : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief A nav2_behavior_tree::FindNearestWaypoint constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  FindNearestWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav2_msgs::msg::WaypointArray>("waypoints", "Original Path"),
      BT::InputPort<double>(
        "max_search_dist", std::numeric_limits<double>::infinity(),
        "Maximum search distance forward and backward"),
      BT::InputPort<double>(
        "angular_distance_weight", 0.0,
        "Weight of angular distance relative to positional distance when finding which path "
        "pose is closest to robot. Not applicable on paths without orientations assigned"),
      BT::InputPort<int>("input_closest_index", -1, "Input closest waypoint index"),
      BT::InputPort<std::string>(
        "robot_frame", "base_link",
        "Robot base frame id"),
      BT::InputPort<double>(
        "transform_tolerance", 0.2,
        "Transform lookup tolerance"),
      BT::InputPort<double>(
        "distance_forward", 8.0,
        "Distance in forward direction"),
      BT::InputPort<double>(
        "distance_backward", 4.0,
        "Distance in backward direction"),
      BT::InputPort<double>(
        "goal_reached_tol", 0.3,
        "goal reached tol"),
      BT::OutputPort<int>("waypoint_index", "Outpuut closest waypoint index"),
      BT::OutputPort<Goals>("goals", "Goals with passed viapoints removed"),
      BT::OutputPort<nav2_msgs::msg::Waypoint>("waypoint", "Goals with passed viapoints removed"),
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

  /**
   * @brief Get either specified input pose or robot pose in path frame
   * @param path_frame_id Frame ID of path
   * @param pose Output pose
   * @return True if succeeded
   */
  bool getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief A custom pose distance method which takes angular distance into account
   * in addition to spatial distance (to improve picking a correct pose near cusps and loops)
   * @param pose1 Distance is computed between this pose and pose2
   * @param pose2 Distance is computed between this pose and pose1
   * @param angular_distance_weight Weight of angular distance relative to spatial distance
   * (1.0 means that 1 radian of angular distance corresponds to 1 meter of spatial distance)
   */
  static double poseDistance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2,
    const double angular_distance_weight);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_local_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr closest_index_pub_;

  int waypoints_size_;
  int last_waypoints_size_;
  int waypoint_section_index_ = 1;
  std::vector<int> waypoint_section_index_list_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_NEAREST_POINT_INDEX_ACTION_HPP_
