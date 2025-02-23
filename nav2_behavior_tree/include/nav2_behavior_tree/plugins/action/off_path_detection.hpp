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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__OFF_PATH_DETECTION_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__OFF_PATH_DETECTION_ACTION_HPP_

#include <memory>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

#include "nav2_msgs/msg/waypoint_array.hpp"
#include "nav2_msgs/msg/waypoint.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2/utils.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class OffPathDetection : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief A nav2_behavior_tree::OffPathDetection constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  OffPathDetection(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("goals", "The pose of the point on the path"),
      BT::InputPort<nav_msgs::msg::Path>("path", "Input path"),
      BT::InputPort<double>( "distance", 4.0, "Maximum value of deviation distance"),
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

  bool transformPoseToFrame(const std::string frame_id, const geometry_msgs::msg::PoseStamped & input_pose, geometry_msgs::msg::PoseStamped & transformed_pose);

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

  double calculate_distance_to_line(double x1, double y1, double x, double y, double yaw);


  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  Goals goals_;
  nav_msgs::msg::Path path_;
  double distance_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__OFF_PATH_DETECTION_ACTION_HPP_
