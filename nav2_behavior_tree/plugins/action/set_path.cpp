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

#include "nav2_behavior_tree/plugins/action/set_path.hpp"

namespace nav2_behavior_tree
{

SetPath::SetPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  current_time_ = node_->now();
  last_time_ = node_->now();

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  bypass_path_now_ =
    node_->create_publisher<nav_msgs::msg::Path>("/bypass_path_now", qos);
  bypass_path_last_ =
    node_->create_publisher<nav_msgs::msg::Path>("/bypass_path_last", qos);
}

inline BT::NodeStatus SetPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  getInput("input_path", input_path_);
  getInput("interval", time_interval_);

  bypass_path_now_->publish(input_path_);

  // double max_curvature = detect_maximum_curvature(input_path_);
  // RCLCPP_WARN(node_->get_logger(), "Path curvature is too high: %f", max_curvature);

  // // if (max_curvature > 0.5) {
  // //   RCLCPP_WARN(node_->get_logger(), "Path curvature is too high: %f", max_curvature);
  // //   return BT::NodeStatus::FAILURE;
  // // }

  current_time_ = node_->now();
  if ((current_time_ - last_time_).seconds() > time_interval_) {
    last_time_ = current_time_;
    // Trim the path to remove points that are too close to the start
    double distance_threshold = 1.0;
    double curvature_threshold = 50.0;
    nav_msgs::msg::Path trimmed_path = trim_path(input_path_, distance_threshold);
    double max_curvature = detect_maximum_curvature(trimmed_path);

    // Check if the maximum curvature exceeds the threshold
    if (max_curvature > curvature_threshold) {
      RCLCPP_WARN(node_->get_logger(), "Trimmed path curvature is too high: %f", max_curvature);
      bypass_path_last_->publish(trimmed_path);
      setOutput("output_path", trimmed_path);
      return BT::NodeStatus::FAILURE;
    }

    // Smooth the path
    // nav_msgs::msg::Path smoothed_path = smooth_path_spline(trimmed_path, 0.5);
    nav_msgs::msg::Path smoothed_path = smooth_path(trimmed_path, 30);

    bypass_path_last_->publish(smoothed_path);
    setOutput("output_path", smoothed_path);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::SUCCESS;
}

nav_msgs::msg::Path SetPath::trim_path(const nav_msgs::msg::Path& path, double distance_threshold) {
  nav_msgs::msg::Path trimmed_path;
  trimmed_path.header = path.header;

  // Check if the path has at least one point
  if (path.poses.empty()) {
    return trimmed_path; // Return empty path if no points
  }

  double distance = 0.0;
  size_t start_index = 0;

  // Calculate distance until we reach the specified threshold
  for (size_t i = 1; i < path.poses.size(); ++i) {
    distance += nav2_util::geometry_utils::euclidean_distance(
      path.poses[i-1].pose.position, path.poses[i].pose.position);
    if (distance >= distance_threshold) {
      start_index = i;
      break;
    }
  }

  // Copy remaining points to the new path
  for (size_t i = start_index; i < path.poses.size(); ++i) {
    trimmed_path.poses.push_back(path.poses[i]);
  }

  return trimmed_path;
}

/**
 * @brief Calculate the maximum curvature in a ROS2 path
 * 
 * This function computes the curvature at each segment of the path and returns the maximum value.
 * Curvature is calculated using the Menger curvature method with three consecutive points.
 * 
 * @param path The input path message
 * @return double The maximum curvature value found in the path
 */
double SetPath::detect_maximum_curvature(const nav_msgs::msg::Path& path) {
  // Need at least 3 points to calculate curvature
  if (path.poses.size() < 3) {
    RCLCPP_WARN(rclcpp::get_logger("path_utils"), "Path needs at least 3 points to calculate curvature");
    return 0.0;
  }

  double max_curvature = 0.0;

  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    // Get three consecutive points
    const auto& p1 = path.poses[i-1].pose.position;
    const auto& p2 = path.poses[i].pose.position;
    const auto& p3 = path.poses[i+1].pose.position;

    // Calculate vectors between points
    double v1x = p2.x - p1.x;
    double v1y = p2.y - p1.y;
    double v2x = p3.x - p2.x;
    double v2y = p3.y - p2.y;

    // Calculate distances between points
    double d1 = std::sqrt(v1x * v1x + v1y * v1y);
    double d2 = std::sqrt(v2x * v2x + v2y * v2y);
    
    // Avoid division by zero
    if (d1 < 1e-10 || d2 < 1e-10) {
      continue;
    }

    // Normalize vectors
    v1x /= d1;
    v1y /= d1;
    v2x /= d2;
    v2y /= d2;

    // Calculate the angle between vectors (using dot product)
    double dot_product = v1x * v2x + v1y * v2y;
    dot_product = std::clamp(dot_product, -1.0, 1.0); // Clamp to avoid numerical errors
    double angle = std::acos(dot_product);

    // Calculate the curvature (κ = |θ| / distance)
    // Using the average of the two segments as distance
    double curvature = std::abs(angle) / ((d1 + d2) / 2.0);
    
    max_curvature = std::max(max_curvature, curvature);
  }

  return max_curvature;
}

/**
 * @brief Smooth a ROS2 path using moving average filter
 * 
 * This function applies a moving average filter to smooth the path.
 * The window size determines how many points to include in the average.
 * 
 * @param path The input path message
 * @param window_size Number of points to include in the moving average
 * @return nav_msgs::msg::Path The smoothed path
 */
nav_msgs::msg::Path SetPath::smooth_path(const nav_msgs::msg::Path& path, int window_size) {
  if (path.poses.size() <= 1 || window_size <= 1) {
    return path; // Return original if no smoothing needed
  }

  // Make window size odd to have equal points before and after
  if (window_size % 2 == 0) {
    window_size++;
  }

  // Create output path with the same header
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = path.header;
  
  // Ensure window size doesn't exceed path length
  window_size = std::min(window_size, static_cast<int>(path.poses.size()));
  int half_window = window_size / 2;

  // Reserve space for efficiency
  smoothed_path.poses.reserve(path.poses.size());

  // Process each point
  for (size_t i = 0; i < path.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped smoothed_pose;
    smoothed_pose.header = path.poses[i].header;
    
    // Initialize accumulators
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    double sum_orientation_x = 0.0, sum_orientation_y = 0.0;
    double sum_orientation_z = 0.0, sum_orientation_w = 0.0;
    int count = 0;

    // Calculate window bounds
    int start = std::max(0, static_cast<int>(i) - half_window);
    int end = std::min(static_cast<int>(path.poses.size()) - 1, static_cast<int>(i) + half_window);

    // Sum up values in the window
    for (int j = start; j <= end; ++j) {
      sum_x += path.poses[j].pose.position.x;
      sum_y += path.poses[j].pose.position.y;
      sum_z += path.poses[j].pose.position.z;
      
      sum_orientation_x += path.poses[j].pose.orientation.x;
      sum_orientation_y += path.poses[j].pose.orientation.y;
      sum_orientation_z += path.poses[j].pose.orientation.z;
      sum_orientation_w += path.poses[j].pose.orientation.w;
      
      count++;
    }

    // Average the values
    smoothed_pose.pose.position.x = sum_x / count;
    smoothed_pose.pose.position.y = sum_y / count;
    smoothed_pose.pose.position.z = sum_z / count;

    // For orientation, we need to normalize after averaging
    double norm = std::sqrt(
      sum_orientation_x * sum_orientation_x +
      sum_orientation_y * sum_orientation_y +
      sum_orientation_z * sum_orientation_z +
      sum_orientation_w * sum_orientation_w
    );

    if (norm > 1e-10) {
      smoothed_pose.pose.orientation.x = sum_orientation_x / norm;
      smoothed_pose.pose.orientation.y = sum_orientation_y / norm;
      smoothed_pose.pose.orientation.z = sum_orientation_z / norm;
      smoothed_pose.pose.orientation.w = sum_orientation_w / norm;
    } else {
      // If normalization fails, just keep the original orientation
      smoothed_pose.pose.orientation = path.poses[i].pose.orientation;
    }

    smoothed_path.poses.push_back(smoothed_pose);
  }

  return smoothed_path;
}

/**
 * @brief Alternative approach for path smoothing using cubic spline interpolation
 * 
 * This function implements a more sophisticated smoothing approach using cubic splines.
 * Better for maintaining path characteristics while reducing noise.
 * 
 * @param path The input path message
 * @param smoothness Smoothness factor (0.0-1.0), higher means smoother
 * @return nav_msgs::msg::Path The smoothed path
 */
nav_msgs::msg::Path SetPath::smooth_path_spline(const nav_msgs::msg::Path& path, double smoothness) {
  if (path.poses.size() <= 2) {
    return path; // Return original if not enough points for spline
  }

  // Create output path with the same header
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = path.header;
  
  // Parameters for the algorithm
  smoothness = std::clamp(smoothness, 0.0, 1.0);
  double alpha = 6.0 * (1.0 - smoothness); // Control parameter
  
  // Copy first and last points directly (boundary conditions)
  smoothed_path.poses.push_back(path.poses.front());
  
  // Process internal points with cubic spline algorithm
  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    geometry_msgs::msg::PoseStamped smoothed_pose;
    smoothed_pose.header = path.poses[i].header;
    
    // Apply cubic spline formula for position
    const auto& p0 = path.poses[i-1].pose.position;
    const auto& p1 = path.poses[i].pose.position;
    const auto& p2 = path.poses[i+1].pose.position;
    
    // Cubic spline calculation
    smoothed_pose.pose.position.x = p1.x + (alpha * (p0.x - 2*p1.x + p2.x)) / 16.0;
    smoothed_pose.pose.position.y = p1.y + (alpha * (p0.y - 2*p1.y + p2.y)) / 16.0;
    smoothed_pose.pose.position.z = p1.z + (alpha * (p0.z - 2*p1.z + p2.z)) / 16.0;
    
    // SLERP for orientation (Spherical Linear Interpolation)
    // For simplicity, we'll just use the original orientation
    // A more accurate approach would use quaternion SLERP
    smoothed_pose.pose.orientation = path.poses[i].pose.orientation;
    
    smoothed_path.poses.push_back(smoothed_pose);
  }
  
  // Add the last point
  smoothed_path.poses.push_back(path.poses.back());
  
  return smoothed_path;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SetPath>("SetPath");
}
