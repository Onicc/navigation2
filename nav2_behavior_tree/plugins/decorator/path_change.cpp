// Copyright (c) 2022 Neobotix GmbH
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
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/path_change.hpp"

namespace nav2_behavior_tree
{

PathChange::PathChange(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

bool PathChange::isPathUpdated(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path)
{
  return new_path != old_path && old_path.poses.size() != 0 &&
         new_path.poses.size() != 0 &&
         old_path.poses.back() == new_path.poses.back();
}

bool PathChange::isRobotInGoalProximity(
  nav_msgs::msg::Path & old_path,
  double & prox_leng)
{
  return nav2_util::geometry_utils::calculate_path_length(old_path, 0) < prox_leng;
}

bool PathChange::isNewPathLonger(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path,
  double & length_factor)
{
  return nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
         length_factor * nav2_util::geometry_utils::calculate_path_length(
    old_path, 0);
}

double PathChange::distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2) + std::pow(p2.z - p1.z, 2));
}

nav_msgs::msg::Path PathChange::crop_path_by_distance(const nav_msgs::msg::Path &original_path, double max_distance) {
    nav_msgs::msg::Path cropped_path;
    double total_distance = 0.0;

    // 需要确保原始路径至少有两个点
    if (original_path.poses.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Path has fewer than two points. Cannot crop.");
        return cropped_path;
    }

    // 复制路径的头部信息
    cropped_path.header = original_path.header;

    // 添加第一个点
    cropped_path.poses.push_back(original_path.poses[0]);

    // 遍历路径并计算距离
    for (size_t i = 1; i < original_path.poses.size(); ++i) {
        double segment_distance = distance(original_path.poses[i-1].pose.position, original_path.poses[i].pose.position);
        total_distance += segment_distance;

        // 如果总距离超过最大裁剪距离，停止裁剪
        if (total_distance > max_distance) {
            break;
        }

        // 否则，继续添加路径中的点
        cropped_path.poses.push_back(original_path.poses[i]);
    }

    return cropped_path;
}

double PathChange::calculatePathLength(const nav_msgs::msg::Path &path) {
  double total_length = 0.0;

  // 确保路径中至少有两个点
  if (path.poses.size() < 2) {
      return total_length;
  }

  // 遍历路径中的点，计算相邻点之间的距离并累加
  for (size_t i = 1; i < path.poses.size(); ++i) {
      total_length += distance(path.poses[i - 1].pose.position, path.poses[i].pose.position);
  }

  return total_length;
}

double PathChange::pose_distance(
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

double PathChange::calculate_pose_to_path_distance(
  const geometry_msgs::msg::PoseStamped &pose, 
  const nav_msgs::msg::Path &path, 
  const double angular_distance_weight)
{
  geometry_msgs::msg::PoseStamped nearest_pose;
  double min_distance = std::numeric_limits<double>::infinity();

  for (const auto & path_pose : path.poses) {
    double dist = distance(pose.pose.position, path_pose.pose.position);
    if (dist < min_distance) {
      min_distance = dist;
      nearest_pose = path_pose;
    }
  }

  return pose_distance(pose, nearest_pose, angular_distance_weight);
}

double PathChange::calculate_path_to_path_distance(
  const nav_msgs::msg::Path &path1, 
  const nav_msgs::msg::Path &path2, 
  const double angular_distance_weight)
{
  double path_to_path_distance = 0;

  int step = path1.poses.size() / 10;
  if (step == 0) {
    step = 1;
  }

  double index = 0.0;
  for (size_t i = 0; i < path1.poses.size(); i += step) {
    const auto & pose1 = path1.poses[i];
    double dist = calculate_pose_to_path_distance(pose1, path2, angular_distance_weight);
    path_to_path_distance += dist;
    index += 1.0;
  }

  return path_to_path_distance/index;
}

double PathChange::calculate_fitted_line_angle(const nav_msgs::msg::Path &path)
{
  // 确保路径有足够的点来拟合直线
  if (path.poses.size() < 2) {
    RCLCPP_INFO(node_->get_logger(), "At least two points are required to fit a line.");
    return 0.0;
  }

  double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
  int n = path.poses.size();

  // 对路径点进行最小二乘法拟合
  for (const auto &pose_stamped : path.poses) {
    double x = pose_stamped.pose.position.x;
    double y = pose_stamped.pose.position.y;
    sum_x += x;
    sum_y += y;
    sum_xy += x * y;
    sum_xx += x * x;
  }

  // 检查分母是否为零
  double denominator = n * sum_xx - sum_x * sum_x;
  if (std::abs(denominator) < 1e-9) {
    RCLCPP_INFO(node_->get_logger(), "Denominator is zero. Cannot calculate slope.");
    return 0.0;
  }

  // 最小二乘法计算斜率 m
  double m = (n * sum_xy - sum_x * sum_y) / denominator;

  // 计算直线角度
  double angle_radians = std::atan(m);
  double angle_degrees = angle_radians * 180.0 / M_PI;

  return angle_degrees;
}

double PathChange::calculate_path_to_path_dangle(
  const nav_msgs::msg::Path &path1, 
  const nav_msgs::msg::Path &path2)
{
  double angle1 = calculate_fitted_line_angle(path1);
  double angle2 = calculate_fitted_line_angle(path2);
  double dangle = angle1 - angle2;
  if(dangle > 90.0) dangle -= 180.0;
  if(dangle < -90.0) dangle += 180.0;

  return dangle;
}

inline BT::NodeStatus PathChange::tick()
{
  getInput("path", new_path_);
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);
  getInput("angular_distance_weight", angular_distance_weight_);

  // if (status() == BT::NodeStatus::IDLE) {
  //   // Reset the starting point since we're starting a new iteration of
  //   // PathChange (moving from IDLE to RUNNING)
  //   first_time_ = true;
  // }

  setStatus(BT::NodeStatus::RUNNING);

  // std::cout << "new_path_size: " << new_path_.poses.size() << std::endl;
  // std::cout << "old_path_size: " << old_path_.poses.size() << std::endl;
  // std::cout << "first_time: " << first_time_ << std::endl;

  if (!first_time_) {
    // 1. 如果新路径或者旧路径为空，则返回SUCCESS，表示不执行child_node
    if (new_path_.poses.size() < 2 || old_path_.poses.size() < 2) {
      old_path_ = new_path_;
      return BT::NodeStatus::SUCCESS;
    }
    // 2. 对新路径和旧路径进行长度计算，如果长度不满足要求返回SUCCESS，表示不执行child_node
    double new_path_length = calculatePathLength(new_path_);
    double old_path_length = calculatePathLength(old_path_);
    // std::cout << "new_path_length: " << new_path_length << std::endl;
    // std::cout << "old_path_length: " << old_path_length << std::endl;
    if (new_path_length < prox_len_ || old_path_length < prox_len_) {
      old_path_ = new_path_;
      return BT::NodeStatus::SUCCESS;
    }
    // 
    if (fabs(new_path_length - old_path_length) < 0.01) {
      old_path_ = new_path_;
      return BT::NodeStatus::SUCCESS;
    }
    // 3. 对新路径和旧路径进行裁剪
    nav_msgs::msg::Path cropped_new_path = crop_path_by_distance(new_path_, prox_len_);
    nav_msgs::msg::Path cropped_old_path = crop_path_by_distance(old_path_, prox_len_);
    // 4. 计算新路径和旧路径之间的距离
    // double path_distance = calculate_path_to_path_distance(cropped_new_path, cropped_old_path, angular_distance_weight_);
    // RCLCPP_INFO(node_->get_logger(), "Path distance: %f", path_distance);
    double dangel = calculate_path_to_path_dangle(cropped_new_path, cropped_old_path);
    RCLCPP_INFO(node_->get_logger(), "Path dangle: %f", dangel);
    double path_distance = fabs(dangel);
    // 5. 如果新路径和旧路径之间的距离大于prox_len_，则执行child_node
    if (path_distance > length_factor_) {
      const BT::NodeStatus child_state = child_node_->executeTick();
      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;
        case BT::NodeStatus::SUCCESS:
          old_path_ = new_path_;
          return BT::NodeStatus::SUCCESS;
        case BT::NodeStatus::FAILURE:
          old_path_ = new_path_;
          return BT::NodeStatus::FAILURE;
        default:
          old_path_ = new_path_;
          return BT::NodeStatus::FAILURE;
      }
    }
  }
  old_path_ = new_path_;
  first_time_ = false;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathChange>("PathChange");
}
