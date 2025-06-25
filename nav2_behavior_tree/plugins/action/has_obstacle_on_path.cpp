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

#include "nav2_behavior_tree/plugins/action/has_obstacle_on_path.hpp"

namespace nav2_behavior_tree
{

HasObstacleOnPath::HasObstacleOnPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

inline BT::NodeStatus HasObstacleOnPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);


  /******************************* costmap ***********************************/
  nav2_msgs::msg::Costmap costmap_msg;
  getInput("costmap_msg", costmap_msg);

  const auto age_of_costmap = node_->now() - costmap_msg.header.stamp;
  if (age_of_costmap > rclcpp::Duration::from_seconds(3.0)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[HasObstacleOnPath] Costmap is too old: %f seconds",
      age_of_costmap.seconds());
    return BT::NodeStatus::FAILURE;
  }

  // costmap to costmap2d
  if (costmap_ == nullptr) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
      costmap_msg.metadata.size_x, costmap_msg.metadata.size_y,
      costmap_msg.metadata.resolution, costmap_msg.metadata.origin.position.x,
      costmap_msg.metadata.origin.position.y);
  } else if (costmap_->getSizeInCellsX() != costmap_msg.metadata.size_x ||  // NOLINT
    costmap_->getSizeInCellsY() != costmap_msg.metadata.size_y ||
    costmap_->getResolution() != costmap_msg.metadata.resolution ||
    costmap_->getOriginX() != costmap_msg.metadata.origin.position.x ||
    costmap_->getOriginY() != costmap_msg.metadata.origin.position.y)
  {
    // Update the size of the costmap
    costmap_->resizeMap(
      costmap_msg.metadata.size_x, costmap_msg.metadata.size_y,
      costmap_msg.metadata.resolution,
      costmap_msg.metadata.origin.position.x,
      costmap_msg.metadata.origin.position.y);
  }

  unsigned char * master_array = costmap_->getCharMap();
  unsigned int index = 0;
  for (unsigned int i = 0; i < costmap_msg.metadata.size_x; ++i) {
    for (unsigned int j = 0; j < costmap_msg.metadata.size_y; ++j) {
      master_array[index] = costmap_msg.data[index];
      ++index;
    }
  }

  // get frame_id
  std::string frame_id = costmap_msg.header.frame_id;

  /******************************* transform pose to target frame ***********************************/
  Goals goals;
  Goals transformed_goals;
  getInput("goals", goals);
  for (auto & goal : goals) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (!transformPoseToFrame(frame_id, goal, transformed_pose)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[HasObstacleOnPath] Failed to transform pose to frame: %s",
        frame_id.c_str());
      return BT::NodeStatus::FAILURE;
    }
    transformed_pose.header.frame_id = frame_id;
    transformed_pose.header.stamp = node_->now();
    transformed_goals.push_back(transformed_pose);
  }

  /******************************* goal_inflation ***********************************/
  double goal_inflation_radius;
  getInput("goal_inflation_radius", goal_inflation_radius);

  // Check if any goal has obstacles within inflation radius
  for (const auto& goal : transformed_goals) {
    if (hasObstacleInInflationRadius(goal, goal_inflation_radius)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[HasObstacleOnPath] Obstacle found within inflation radius of goal at (%.2f, %.2f)",
        goal.pose.position.x, goal.pose.position.y);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

inline bool HasObstacleOnPath::hasObstacleInInflationRadius(
  const geometry_msgs::msg::PoseStamped& goal, 
  double inflation_radius)
{
  // Convert world coordinates to map coordinates
  unsigned int goal_mx, goal_my;
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    // Goal is outside the costmap
    return true;
  }
  
  // Convert inflation radius to cells
  int radius_cells = static_cast<int>(std::ceil(inflation_radius / costmap_->getResolution()));
  
  // Check cells within the circular region using efficient square iteration
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      // Check if point is within circle
      double distance = std::sqrt(dx * dx + dy * dy) * costmap_->getResolution();
      if (distance > inflation_radius) {
        continue;
      }
      
      // Calculate cell coordinates
      int check_x = static_cast<int>(goal_mx) + dx;
      int check_y = static_cast<int>(goal_my) + dy;
      
      // Check bounds
      if (check_x < 0 || check_x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
          check_y < 0 || check_y >= static_cast<int>(costmap_->getSizeInCellsY())) {
        continue;
      }
      
      // Check cost
      unsigned int cost = costmap_->getCost(static_cast<unsigned int>(check_x), 
                                           static_cast<unsigned int>(check_y));
      if (cost > 0) {
        return true;
      }
    }
  }
  
  return false;
}

inline bool HasObstacleOnPath::getRobotPose(
  std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose)
{
  if (!getInput("pose", pose)) {
    std::string robot_frame;
    if (!getInput("robot_frame", robot_frame)) {
      RCLCPP_ERROR(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Neither pose nor robot_frame specified for %s", name().c_str());
      return false;
    }
    double transform_tolerance;
    getInput("transform_tolerance", transform_tolerance);
    if (!nav2_util::getCurrentPose(
        pose, *tf_buffer_, path_frame_id, robot_frame, transform_tolerance))
    {
      RCLCPP_WARN(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Failed to lookup current robot pose for %s", name().c_str());
      return false;
    }
  }
  return true;
}

inline bool HasObstacleOnPath::transformPoseToFrame(
  const std::string frame_id, 
  const geometry_msgs::msg::PoseStamped & input_pose, 
  geometry_msgs::msg::PoseStamped & transformed_pose)
{
  double transform_tolerance;
  getInput("transform_tolerance", transform_tolerance);
  if (input_pose.header.frame_id == frame_id) {
    transformed_pose = input_pose;
    return true;
  } else {
    return nav2_util::transformPoseInTargetFrame(
      input_pose, transformed_pose, *tf_buffer_,
      frame_id, transform_tolerance);
  }
}

double
HasObstacleOnPath::poseDistance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2,
  const double angular_distance_weight)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  // taking angular distance into account in addition to spatial distance
  // (to improve picking a correct pose near cusps and loops)
  tf2::Quaternion q1;
  tf2::convert(pose1.pose.orientation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angular_distance_weight * std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::HasObstacleOnPath>(
    "HasObstacleOnPath");
}
