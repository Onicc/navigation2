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

#include "nav2_behavior_tree/plugins/action/find_nearest_waypoint_action.hpp"

namespace nav2_behavior_tree
{

FindNearestWaypoint::FindNearestWaypoint(
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
  closest_index_pub_ = 
    node->create_publisher<std_msgs::msg::Int32>("/goal/closest_index", qos);
}

inline BT::NodeStatus FindNearestWaypoint::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav2_msgs::msg::WaypointArray waypoints;
  nav_msgs::msg::Path path;
  double max_search_dist, angular_distance_weight;
  int input_closest_index, closest_pose_index;
  double distance_forward, distance_backward;
  double goal_reached_tol;

  getInput("waypoints", waypoints);
  getInput("max_search_dist", max_search_dist);
  getInput("angular_distance_weight", angular_distance_weight);
  getInput("input_closest_index", input_closest_index);
  getInput("distance_forward", distance_forward);
  getInput("distance_backward", distance_backward);
  getInput("goal_reached_tol", goal_reached_tol);

  Goals goal_poses;

  // waypoints to path
  path.header = waypoints.header;
  for (const auto &waypoint : waypoints.waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = waypoint.header;
    pose.pose = waypoint.pose;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!getRobotPose(path.header.frame_id, pose)) {
    return BT::NodeStatus::FAILURE;
  }

  // waypoints_size_ = waypoints.waypoints.size();
  // if(last_waypoints_size_ != waypoints_size_) {
  //   last_waypoints_size_ = waypoints_size_;
  //   input_closest_index = -1;
  // }

  if (input_closest_index == -1) {
    // find the closest pose on the path
    auto closest_pose = nav2_util::geometry_utils::min_by(
      path.poses.begin(), path.poses.end(),
      [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
        return poseDistance(pose, ps, angular_distance_weight);
      });
    closest_pose_index = std::distance(path.poses.begin(), closest_pose);
    setOutput("waypoint_index", closest_pose_index);
    setOutput("waypoint", waypoints.waypoints[closest_pose_index]);
    std_msgs::msg::Int32 closest_index;
    closest_index.data = closest_pose_index;
    closest_index_pub_->publish(closest_index);

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
    setOutput("goals", goal_poses);
    
    return BT::NodeStatus::SUCCESS;
  } else {
    int begin_index = 0;
    int end_index = path.poses.size()-1;
    double sum_dist = 0;

    int segment_waypoint_start_index = 0;
    int segment_waypoint_end_index = path.poses.size()-1;
    for(int i = input_closest_index; i >= 0; i--) {
      if(!((waypoints.waypoints[input_closest_index].option_speed >= 0 && waypoints.waypoints[i].option_speed >= 0) || 
           (waypoints.waypoints[input_closest_index].option_speed < 0 && waypoints.waypoints[i].option_speed < 0))) {
        segment_waypoint_start_index = i+1;
        break;
      }
    }
    for(size_t i = input_closest_index; i < path.poses.size(); i++) {
      if(!((waypoints.waypoints[input_closest_index].option_speed >= 0 && waypoints.waypoints[i].option_speed >= 0) || 
           (waypoints.waypoints[input_closest_index].option_speed < 0 && waypoints.waypoints[i].option_speed < 0))) {
        segment_waypoint_end_index = i-1;
        break;
      }
    }

    if(input_closest_index > 0) {
      for (int i = (input_closest_index-1); i >= segment_waypoint_start_index; i--) {
        sum_dist += nav2_util::geometry_utils::euclidean_distance(
          path.poses[i+1].pose.position, path.poses[i].pose.position);
        if (sum_dist > max_search_dist) {
          begin_index = i;
          break;
        }
        if (i == segment_waypoint_start_index) {
          begin_index = i;
        }
      }
    } else {
      begin_index = 0;
    }

    sum_dist = 0;
    if(input_closest_index < static_cast<int>(path.poses.size()-1) && 
       segment_waypoint_end_index < static_cast<int>(path.poses.size()-1)) {
      for (size_t i = (input_closest_index+1); i < segment_waypoint_end_index; i++) {
        sum_dist += nav2_util::geometry_utils::euclidean_distance(
          path.poses[i-1].pose.position, path.poses[i].pose.position);
        if (sum_dist > max_search_dist) {
          end_index = i;
          break;
        }
        if (i == segment_waypoint_end_index-1) {
          end_index = i;
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
    // 将waypoint向运行方向移动0.3m，因为有时候会出现机器人在路径上的情况导致引导向后运动
    double min_dist = 0.3;
    for(size_t i = closest_pose_index; i < path.poses.size(); i++) {
      if(nav2_util::geometry_utils::euclidean_distance(
        path.poses[closest_pose_index].pose.position, path.poses[i].pose.position) > min_dist) {
        closest_pose_index = i;
        closest_pose = std::next(path.poses.begin(), i);
        break;
      }
      if(i == path.poses.size()-1) {
        closest_pose_index = i;
        closest_pose = std::next(path.poses.begin(), i);
      }
    }

    // find the end waypoint
    int end_waypoint_index = path.poses.size()-1;
    for(size_t i = closest_pose_index; i < path.poses.size(); i++) {
      if(!((waypoints.waypoints[closest_pose_index].option_speed >= 0 && waypoints.waypoints[i].option_speed >= 0) || 
           (waypoints.waypoints[closest_pose_index].option_speed < 0 && waypoints.waypoints[i].option_speed < 0))) {
        end_waypoint_index = i-1;
        break;
      }
    }

    // calculate the remain distance from the closest waypoint to the end waypoint
    double remain_dist = 0;
    for(size_t i = closest_pose_index; i < end_waypoint_index; i++) {
      remain_dist += nav2_util::geometry_utils::euclidean_distance(
        path.poses[i].pose.position, path.poses[i+1].pose.position);
    }

    // if the remain distance is less than goal_reached_tol m, then the end waypoint is the closest waypoint
    if(remain_dist < goal_reached_tol && end_waypoint_index < path.poses.size()-2) {
      closest_pose_index = end_waypoint_index + 1;
      closest_pose = std::next(path.poses.begin(), end_waypoint_index);
      end_waypoint_index = path.poses.size()-1;
      for(size_t i = closest_pose_index; i < path.poses.size(); i++) {
        if(!((waypoints.waypoints[closest_pose_index].option_speed >= 0 && waypoints.waypoints[i].option_speed >= 0) || 
            (waypoints.waypoints[closest_pose_index].option_speed < 0 && waypoints.waypoints[i].option_speed < 0))) {
          end_waypoint_index = i-1;
          break;
        }
      }
    }
    closest_pose = std::next(path.poses.begin(), closest_pose_index);

    // // create a segment path
    // nav_msgs::msg::Path segment_path;
    // segment_path.header = path.header;
    // segment_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
    //   path.poses.begin() + closest_pose_index, path.poses.begin() + end_waypoint_index);

    setOutput("waypoint_index", closest_pose_index);
    setOutput("waypoint", waypoints.waypoints[closest_pose_index]);
    std_msgs::msg::Int32 closest_index;
    closest_index.data = closest_pose_index;
    closest_index_pub_->publish(closest_index);
    
    auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      closest_pose, path.poses.begin() + end_waypoint_index, distance_forward);
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
    setOutput("goals", goal_poses);

    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

inline bool FindNearestWaypoint::getRobotPose(
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
FindNearestWaypoint::poseDistance(
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
  factory.registerNodeType<nav2_behavior_tree::FindNearestWaypoint>(
    "FindNearestWaypoint");
}
