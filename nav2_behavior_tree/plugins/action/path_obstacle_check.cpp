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

#include "nav2_behavior_tree/plugins/action/path_obstacle_check.hpp"

namespace nav2_behavior_tree
{

PathObstacleCheck::PathObstacleCheck(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

inline BT::NodeStatus PathObstacleCheck::tick()
{
  getInput("path", path_);
  getInput("distance", distance_);

  setStatus(BT::NodeStatus::RUNNING);

  /******************************* costmap ***********************************/
  nav2_msgs::msg::Costmap costmap_msg;
  getInput("costmap_msg", costmap_msg);

  const auto age_of_costmap = node_->now() - costmap_msg.header.stamp;
  if (age_of_costmap > rclcpp::Duration::from_seconds(3.0)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[PathObstacleCheck] Costmap is too old: %f seconds",
      age_of_costmap.seconds());
    std::cout << "[PathObstacleCheck] Costmap is too old: " << age_of_costmap.seconds() << " seconds" << std::endl;
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
  Goals transformed_goals;
  if (path_.poses.size() == 0) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[PathObstacleCheck] Empty path");
    return BT::NodeStatus::SUCCESS;
  }

  // for (auto & goal : path_.poses) {
  //   geometry_msgs::msg::PoseStamped transformed_pose;
  //   if (!transformPoseToFrame(frame_id, goal, transformed_pose)) {
  //     RCLCPP_INFO(
  //       node_->get_logger(),
  //       "[PathObstacleCheck] Failed to transform pose to frame: %s",
  //       frame_id.c_str());
  //     std::cout << "[PathObstacleCheck] Failed to transform pose to frame: " << frame_id.c_str() << std::endl;
  //     return BT::NodeStatus::FAILURE;
  //   }
  //   transformed_pose.header.frame_id = frame_id;
  //   transformed_pose.header.stamp = node_->now();
  //   transformed_goals.push_back(transformed_pose);
  // }

  for (auto & goal : path_.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    transformed_pose.header.frame_id = frame_id;
    transformed_pose.header.stamp = node_->now();
    transformed_goals.push_back(goal);
  }

  /******************************* cal ***********************************/

  /** Find the target point n meters away from the robot **/
  int end_index = transformed_goals.size()-1;
  for(int i = 0; i < transformed_goals.size(); i++) {
    double end_distance = nav2_util::geometry_utils::euclidean_distance(
      transformed_goals[0], transformed_goals[i]);
    if (end_distance > distance_) {
      end_index = i;
      break;
    }
  }

  std::cout << "end_index: " << end_index << std::endl; 

  for(int i = 10; i < end_index; i++) {
    unsigned int mx = 0;
    unsigned int my = 0;
    costmap_->worldToMap(
      transformed_goals[i].pose.position.x,
      transformed_goals[i].pose.position.y, mx, my);
    unsigned int cost = costmap_->getCost(mx, my);
    if (cost > 180) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[PathObstacleCheck] The goal is in the obstacle");
      std::cout << "[PathObstacleCheck] i = " << i << " cost = " << cost << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

inline bool PathObstacleCheck::getRobotPose(
  std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose)
{
  if (!getInput("pose", pose)) {
    std::string robot_frame;
    if (!getInput("robot_frame", robot_frame)) {
      RCLCPP_INFO(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Neither pose nor robot_frame specified for %s", name().c_str());
      return false;
    }
    double transform_tolerance;
    getInput("transform_tolerance", transform_tolerance);
    if (!nav2_util::getCurrentPose(
        pose, *tf_buffer_, path_frame_id, robot_frame, transform_tolerance))
    {
      RCLCPP_INFO(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Failed to lookup current robot pose for %s", name().c_str());
      return false;
    }
  }
  return true;
}

inline bool PathObstacleCheck::transformPoseToFrame(
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

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::PathObstacleCheck>(
    "PathObstacleCheck");
}
