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

#include "nav2_behavior_tree/plugins/action/find_goal_from_costmap.hpp"

namespace nav2_behavior_tree
{

FindGoalFromCostmap::FindGoalFromCostmap(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("/obstacle/goal", qos);
}

inline BT::NodeStatus FindGoalFromCostmap::tick()
{
  setStatus(BT::NodeStatus::RUNNING);


  /******************************* costmap ***********************************/
  nav2_msgs::msg::Costmap costmap_msg;
  getInput("costmap_msg", costmap_msg);

  const auto age_of_costmap = node_->now() - costmap_msg.header.stamp;
  if (age_of_costmap > rclcpp::Duration::from_seconds(3.0)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[FindGoalFromCostmap] Costmap is too old: %f seconds",
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
        "[FindGoalFromCostmap] Failed to transform pose to frame: %s",
        frame_id.c_str());
      return BT::NodeStatus::FAILURE;
    }
    transformed_goals.push_back(transformed_pose);
  }

  /******************************* get robot pose ***********************************/
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(frame_id, robot_pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[FindGoalFromCostmap] Failed to get robot pose");
    return BT::NodeStatus::FAILURE;
  }

  /******************************* Get the furthest intersection point between the path and the obstacle ***********************************/
  int farthest_obstacle_index_in_goals = 0;
  for (int i = transformed_goals.size()-1; i >= 0; --i) {
    unsigned int mx = 0;
    unsigned int my = 0;
    costmap_->worldToMap(
      transformed_goals[i].pose.position.x,
      transformed_goals[i].pose.position.y, mx, my);
    unsigned int cost = costmap_->getCost(mx, my);
    if (cost > 0) {
      farthest_obstacle_index_in_goals = i;
      break;
    }
  }

  // The intersection point of the obstacle is translated n meters on the path as the target point.
  int goal_index_in_goals = transformed_goals.size()-1;
  double safe_distance;
  getInput("safe_distance", safe_distance);
  if(farthest_obstacle_index_in_goals > 0) {
    double distance = 0.0;
    for (size_t i = farthest_obstacle_index_in_goals; i < (transformed_goals.size()-1); ++i) {
      distance += nav2_util::geometry_utils::euclidean_distance(
        transformed_goals[i], transformed_goals[i+1]);
      if (distance > safe_distance) {
        goal_index_in_goals = i+1;
        break;
      }
    }
    goal_ = transformed_goals[goal_index_in_goals];
  }

  // Determine whether goal is one of the elements in goals
  bool is_goal_in_goals = false;
  for (auto & goal : transformed_goals) {
    if (poseDistance(goal, goal_, 0.0) < 0.01) {
      is_goal_in_goals = true;
      break;
    }
  }

  // Are there any obstacles at the target point?
  unsigned int mx = 0;
  unsigned int my = 0;
  costmap_->worldToMap(goal_.pose.position.x, goal_.pose.position.y, mx, my);
  unsigned int cost = costmap_->getCost(mx, my);
  if (cost > 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[FindGoalFromCostmap] The goal is in the obstacle");
    return BT::NodeStatus::FAILURE;
  }

  if (is_goal_in_goals) {
    pose_pub_->publish(goal_);
    setOutput("goal", goal_);
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[FindGoalFromCostmap] The goal is not in the transformed goals");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

inline bool FindGoalFromCostmap::getRobotPose(
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

inline bool FindGoalFromCostmap::transformPoseToFrame(
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
FindGoalFromCostmap::poseDistance(
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
  factory.registerNodeType<nav2_behavior_tree::FindGoalFromCostmap>(
    "FindGoalFromCostmap");
}
