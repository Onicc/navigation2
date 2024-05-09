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

  // std::cout << "--------------------------1---------------------------" << std::endl;

  // 如果waypoints的数量发生变化，则重新计算最近的waypoint和路径分段
  waypoints_size_ = waypoints.waypoints.size();
  if(last_waypoints_size_ != waypoints_size_) {
    last_waypoints_size_ = waypoints_size_;
    input_closest_index = -1;
    waypoint_section_index_ = 0;

    // 根据速度计算路径分段, ( , ] 左开右闭区间为同速区间
    waypoint_section_index_list_.clear();
    for(size_t i = 0; i < waypoints_size_; i++) {
      if(i == 0) {
        waypoint_section_index_list_.push_back(i);
      } else {
        if(!((waypoints.waypoints[i].option_speed >= 0 && waypoints.waypoints[i-1].option_speed >= 0) || 
             (waypoints.waypoints[i].option_speed < 0 && waypoints.waypoints[i-1].option_speed < 0))) {
          waypoint_section_index_list_.push_back(i-1);
        }
        if(i == waypoints_size_-1) {
          waypoint_section_index_list_.push_back(i);
        }
      }
    }

    // for(auto d:waypoint_section_index_list_) {
    //   std::cout << " " << d << " |";
    // }
    // std::cout << std::endl;
  }

  // std::cout << "--------------------------2---------------------------" << std::endl;

  if (input_closest_index == -1) {
    // std::cout << "--------------------------A---------------------------" << std::endl;

    // find the closest pose on the path
    auto closest_pose = nav2_util::geometry_utils::min_by(
      path.poses.begin(), path.poses.end(),
      [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
        return poseDistance(pose, ps, angular_distance_weight);
      });
    closest_pose_index = std::distance(path.poses.begin(), closest_pose);
    // 根据最近的waypoint找到所在的路径分段
    for(size_t i = 1; i < waypoint_section_index_list_.size(); i++) {
      if(closest_pose_index <= waypoint_section_index_list_[i]) {
        waypoint_section_index_ = i;
        break;
      }
    }
    // waypoint_section_index_==0表明分段只有(0, 0]]
    if(waypoint_section_index_ == 0) {
      return BT::NodeStatus::FAILURE;
    }
    // 取出分段内的waypoints
    nav2_msgs::msg::WaypointArray section_waypoints;
    for(size_t i = waypoint_section_index_list_[waypoint_section_index_-1]+1; i <= waypoint_section_index_list_[waypoint_section_index_]; i++) {
      section_waypoints.waypoints.push_back(waypoints.waypoints[i]);
    }
    // 取出分段内的path
    nav_msgs::msg::Path section_path;
    section_path.header = waypoints.header;
    for(size_t i = waypoint_section_index_list_[waypoint_section_index_-1]+1; i <= waypoint_section_index_list_[waypoint_section_index_]; i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = waypoints.waypoints[i].header;
      pose.pose = waypoints.waypoints[i].pose;
      section_path.poses.push_back(pose);
    }
    // 计算分段内的最近点
    auto section_closest_pose = nav2_util::geometry_utils::min_by(
      section_path.poses.begin(), section_path.poses.end(),
      [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
        return poseDistance(pose, ps, angular_distance_weight);
      });
    // 计算分段内的最近点索引
    int section_closest_pose_index = std::distance(section_path.poses.begin(), section_closest_pose);

    setOutput("waypoint_index", closest_pose_index);
    setOutput("waypoint", section_waypoints.waypoints[section_closest_pose_index]);
    std_msgs::msg::Int32 closest_index;
    closest_index.data = closest_pose_index;
    closest_index_pub_->publish(closest_index);

    auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      section_closest_pose, section_path.poses.end(), distance_forward);
    auto backward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      std::reverse_iterator(section_closest_pose + 1), section_path.poses.rend(), distance_backward);
    
    nav_msgs::msg::Path local_path;
    local_path.header = section_path.header;
    local_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
      backward_pose_it.base(), forward_pose_it);
    path_local_pub_->publish(local_path);

    for (size_t curr_idx = 0; curr_idx < local_path.poses.size(); ++curr_idx) {
      goal_poses.push_back(local_path.poses[curr_idx]);
    }
    setOutput("goals", goal_poses);

    // std::cout << "---------------- closest_pose_index: " << closest_pose_index << "----------------" << std::endl;
    // std::cout << "---------------- waypoint_section_index_: " << waypoint_section_index_ << "----------------" << std::endl;
    
    return BT::NodeStatus::SUCCESS;
  } else {
    // std::cout << "--------------------------B---------------------------" << std::endl;

    // waypoint_section_index_==0表明分段只有(0, 0]]
    if(waypoint_section_index_ == 0) {
      return BT::NodeStatus::FAILURE;
    }

    int section_begin_index = 0;
    int section_end_index = waypoint_section_index_list_[waypoint_section_index_]-(waypoint_section_index_list_[waypoint_section_index_-1]+1);
    int input_section_closest_pose_index = input_closest_index-(waypoint_section_index_list_[waypoint_section_index_-1]+1);
    if(input_section_closest_pose_index < section_begin_index) input_section_closest_pose_index = section_begin_index;
    if(input_section_closest_pose_index > section_end_index) input_section_closest_pose_index = section_end_index;

    // std::cout << "---------------- section_begin_index: " << section_begin_index << "----------------" << std::endl;
    // std::cout << "---------------- section_end_index: " << section_end_index << "----------------" << std::endl;
    // std::cout << "---------------- input_section_closest_pose_index: " << input_section_closest_pose_index << "----------------" << std::endl;
    // std::cout << "---------------- input_closest_index: " << input_section_closest_pose_index << "----------------" << std::endl;

    // std::cout << "--------------------------B 1---------------------------" << std::endl;

    // 取出分段内的waypoints
    nav2_msgs::msg::WaypointArray section_waypoints;
    for(size_t i = waypoint_section_index_list_[waypoint_section_index_-1]+1; i <= waypoint_section_index_list_[waypoint_section_index_]; i++) {
      section_waypoints.waypoints.push_back(waypoints.waypoints[i]);
    }
    // 取出分段内的path
    nav_msgs::msg::Path section_path;
    section_path.header = waypoints.header;
    for(size_t i = waypoint_section_index_list_[waypoint_section_index_-1]+1; i <= waypoint_section_index_list_[waypoint_section_index_]; i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = waypoints.waypoints[i].header;
      pose.pose = waypoints.waypoints[i].pose;
      section_path.poses.push_back(pose);
    }

    // std::cout << "--------------------------B 2---------------------------" << std::endl;

    double sum_dist = 0;
    for (int i = input_section_closest_pose_index; i >= 0; i--) {
      sum_dist += nav2_util::geometry_utils::euclidean_distance(
        section_path.poses[i+1].pose.position, section_path.poses[i].pose.position);
      if (sum_dist > max_search_dist) {
        section_begin_index = i;
        break;
      }
    }
    sum_dist = 0;
    for (size_t i = input_section_closest_pose_index; i < section_path.poses.size(); i++) {
      sum_dist += nav2_util::geometry_utils::euclidean_distance(
        section_path.poses[i-1].pose.position, section_path.poses[i].pose.position);
      if (sum_dist > max_search_dist) {
        section_end_index = i;
        break;
      }
    }
    // std::cout << "---------------- section_begin_index: " << section_begin_index << "----------------" << std::endl;
    // std::cout << "---------------- section_end_index: " << section_end_index << "----------------" << std::endl;

    // std::cout << "--------------------------B 3---------------------------" << std::endl;

    // 查找最近的waypoint
    auto section_closest_pose = nav2_util::geometry_utils::min_by(
      section_path.poses.begin() + section_begin_index, section_path.poses.begin() + section_end_index,
      [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
        return poseDistance(pose, ps, angular_distance_weight);
      });
    int section_closest_pose_index = std::distance(section_path.poses.begin(), section_closest_pose);

    // std::cout << "---------------- section_closest_pose_index: " << section_closest_pose_index << "----------------" << std::endl;
    // std::cout << "--------------------------B 4---------------------------" << std::endl;

    // 将waypoint向运行方向移动0.3m，因为有时候会出现机器人在路径上的情况导致引导向后运动
    double min_dist = 0.3;
    for(size_t i = section_closest_pose_index; i < section_path.poses.size(); i++) {
      if(nav2_util::geometry_utils::euclidean_distance(
        section_path.poses[section_closest_pose_index].pose.position, section_path.poses[i].pose.position) > min_dist) {
        section_closest_pose_index = i;
        section_closest_pose = std::next(section_path.poses.begin(), i);
        break;
      }
      if(i == section_path.poses.size()-1) {
        section_closest_pose_index = i;
        section_closest_pose = std::next(section_path.poses.begin(), i);
      }
    }

    // std::cout << "---------------- section_closest_pose_index: " << section_closest_pose_index << "----------------" << std::endl;
    // std::cout << "--------------------------B 5---------------------------" << std::endl;

    closest_pose_index = section_closest_pose_index+waypoint_section_index_list_[waypoint_section_index_-1]+1;
    setOutput("waypoint_index", closest_pose_index);
    setOutput("waypoint", section_waypoints.waypoints[section_closest_pose_index]);

    // std::cout << "---------------- closest_pose_index: " << closest_pose_index << "----------------" << std::endl;
    // std::cout << "---------------- waypoint_section_index_: " << waypoint_section_index_ << "----------------" << std::endl;

    // 判断是否走完section路径
    double section_remain_dist = 0;
    for(size_t i = section_closest_pose_index; i < section_path.poses.size()-1; i++) {
      section_remain_dist += nav2_util::geometry_utils::euclidean_distance(
        section_path.poses[i].pose.position, section_path.poses[i+1].pose.position);
    }
    if(section_remain_dist < goal_reached_tol) {
      if(waypoint_section_index_ < waypoint_section_index_list_.size()-1) {
        waypoint_section_index_++;
      }
    }

    // std::cout << "--------------------------B 6---------------------------" << std::endl;

    std_msgs::msg::Int32 closest_index;
    closest_index.data = closest_pose_index;
    closest_index_pub_->publish(closest_index);

    auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      section_closest_pose, section_path.poses.end(), distance_forward);
    auto backward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
      std::reverse_iterator(section_closest_pose + 1), section_path.poses.rend(), distance_backward);
    
    // std::cout << "--------------------------B 7---------------------------" << std::endl;

    nav_msgs::msg::Path local_path;
    local_path.header = section_path.header;
    local_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
      backward_pose_it.base(), forward_pose_it);
    path_local_pub_->publish(local_path);

    // std::cout << "--------------------------B 8---------------------------" << std::endl;

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
