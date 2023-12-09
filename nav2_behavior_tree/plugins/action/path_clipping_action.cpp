// Copyright (c) 2021 Samsung Research America
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

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/path_clipping_action.hpp"

namespace nav2_behavior_tree
{

PathClipping::PathClipping(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5)
{
  getInput("radius", viapoint_achieved_radius_);

  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  clipped_path_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("/goal/clipped_path", qos);
}

inline BT::NodeStatus PathClipping::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  Path goal_path;
  getInput("input_goals", goal_path);

  if (goal_path.poses.empty()) {
    setOutput("output_goals", goal_path);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  try {
    // Get current path points
    nav_msgs::msg::Path current_path;
    getInput("input_goals", current_path);

    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx =
      [&current_pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

    auto closest_pose_idx = find_closest_pose_idx();
    goal_path.poses.erase(goal_path.poses.begin(), goal_path.poses.begin() + closest_pose_idx);
    current_path.poses.erase(current_path.poses.begin(), current_path.poses.begin() + closest_pose_idx);

    double path_length;
    getInput("length", path_length);

    auto find_pose_idx_by_dis = 
      [&path_length, &current_path]() {
        size_t distance_pose_idx = 0;
        double sum_dis = 0.0;
        for (size_t curr_idx = 0; curr_idx < (current_path.poses.size()-1); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_path.poses[curr_idx], current_path.poses[curr_idx+1]);
          sum_dis += curr_dist;
          distance_pose_idx = curr_idx;
          if(sum_dis > path_length) break;
        }
        return distance_pose_idx;
      };

    goal_path.poses.erase(goal_path.poses.begin()+find_pose_idx_by_dis(), goal_path.poses.end());

  } catch (...) {
    // Ignore
  }

  Goals goal_poses;
  for (size_t curr_idx = 0; curr_idx < (goal_path.poses.size()-1); ++curr_idx) {
    goal_poses.push_back(goal_path.poses[curr_idx]);
  }

  setOutput("output_path", goal_path);
  setOutput("output_goals", goal_poses);

  clipped_path_pub_->publish(goal_path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathClipping>("PathClipping");
}
