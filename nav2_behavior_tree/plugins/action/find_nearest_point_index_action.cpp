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

#include "nav2_behavior_tree/plugins/action/find_nearest_point_index_action.hpp"

namespace nav2_behavior_tree
{

FindClosestPointIndex::FindClosestPointIndex(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");

  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  path_local_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("/goal/path_local", qos);
}

inline BT::NodeStatus FindClosestPointIndex::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path path;
  double max_search_dist, angular_distance_weight;
  int input_closest_index, closest_pose_index;
  double distance_forward, distance_backward;

  getInput("input_path", path);
  getInput("max_search_dist", max_search_dist);
  getInput("angular_distance_weight", angular_distance_weight);
  getInput("input_closest_index", input_closest_index);
  getInput("distance_forward", distance_forward);
  getInput("distance_backward", distance_backward);

  Goals goal_poses;

  geometry_msgs::msg::PoseStamped pose;
  if (!getRobotPose(path.header.frame_id, pose)) {
    return BT::NodeStatus::FAILURE;
  }

  if (input_closest_index == -1) {
    // find the closest pose on the path
    auto closest_pose = nav2_util::geometry_utils::min_by(
      path.poses.begin(), path.poses.end(),
      [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
        return poseDistance(pose, ps, angular_distance_weight);
      });
    closest_pose_index = std::distance(path.poses.begin(), closest_pose);
    setOutput("output_closest_index", closest_pose_index);

    auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      closest_pose, path.poses.end(), distance_forward);
    auto backward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      std::reverse_iterator(closest_pose + 1), path.poses.rend(), distance_backward);
    
    nav_msgs::msg::Path local_path;
    local_path.header = path.header;
    local_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
      backward_pose_it.base(), forward_pose_it);
    path_local_pub_->publish(local_path);

    for (size_t curr_idx = 0; curr_idx < local_path.poses.size(); ++curr_idx) {
      goal_poses.push_back(local_path.poses[curr_idx]);
    }
    setOutput("output_goals", goal_poses);
    
    return BT::NodeStatus::SUCCESS;
  } else {
    int begin_index, end_index;
    double sum_dist = 0;
    if(input_closest_index > 0) {
      for (int i = (input_closest_index-1); i >= 0; i--) {
        sum_dist += nav2_util::geometry_utils::euclidean_distance(
          path.poses[i+1].pose.position, path.poses[i].pose.position);
        if (sum_dist > max_search_dist) {
          begin_index = i;
          break;
        }
      }
    } else {
      begin_index = 0;
    }

    sum_dist = 0;
    if(input_closest_index < static_cast<int>(path.poses.size()-1)) {
      for (size_t i = (input_closest_index+1); i < path.poses.size(); i++) {
        sum_dist += nav2_util::geometry_utils::euclidean_distance(
          path.poses[i-1].pose.position, path.poses[i].pose.position);
        if (sum_dist > max_search_dist) {
          end_index = i;
          break;
        }
      }
    } else {
      end_index = path.poses.size()-1;
    }

    auto closest_pose = nav2_util::geometry_utils::min_by(
      path.poses.begin() + begin_index, path.poses.begin() + end_index,
      [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
        return poseDistance(pose, ps, angular_distance_weight);
      });
    closest_pose_index = std::distance(path.poses.begin(), closest_pose);
    setOutput("output_closest_index", closest_pose_index);
    
    auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      closest_pose, path.poses.end(), distance_forward);
    auto backward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      std::reverse_iterator(closest_pose + 1), path.poses.rend(), distance_backward);
    
    nav_msgs::msg::Path local_path;
    local_path.header = path.header;
    local_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
      backward_pose_it.base(), forward_pose_it);
    path_local_pub_->publish(local_path);

    for (size_t curr_idx = 0; curr_idx < local_path.poses.size(); ++curr_idx) {
      goal_poses.push_back(local_path.poses[curr_idx]);
    }
    setOutput("output_goals", goal_poses);

    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

inline bool FindClosestPointIndex::getRobotPose(
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

double
FindClosestPointIndex::poseDistance(
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
  factory.registerNodeType<nav2_behavior_tree::FindClosestPointIndex>(
    "FindClosestPointIndex");
}
