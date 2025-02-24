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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_OBSTACLE_CHECK_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_OBSTACLE_CHECK_ACTION_HPP_

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

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class PathObstacleCheck : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief A nav2_behavior_tree::PathObstacleCheck constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PathObstacleCheck(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
      BT::InputPort<double>("distance", 4.0, "The distance between the target point and the obstacle"),
      BT::InputPort<nav2_msgs::msg::Costmap>("costmap_msg", "Costmap to check for obstacles"),
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

  nav_msgs::msg::Path path_;
  double distance_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_NEAREST_POINT_INDEX_ACTION_HPP_
