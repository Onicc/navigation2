/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "nav2_curb_planner/curb_planner.hpp"

double quaternionToYaw(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf_q(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

namespace nav2_curb_planner
{

void CurbPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".roi_x", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".roi_x", roi_x_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".roi_y", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".roi_y", roi_y_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".roi_r", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".roi_r", roi_r_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_ratio", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".cost_ratio", cost_ratio_);
}

void CurbPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void CurbPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void CurbPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path CurbPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // // calculating the number of loops for current value of interpolation_resolution_
  // int total_number_of_loop = std::hypot(
  //   goal.pose.position.x - start.pose.position.x,
  //   goal.pose.position.y - start.pose.position.y) /
  //   interpolation_resolution_;
  // double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  // double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  // for (int i = 0; i < total_number_of_loop; ++i) {
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.pose.position.x = start.pose.position.x + x_increment * i;
  //   pose.pose.position.y = start.pose.position.y + y_increment * i;
  //   pose.pose.position.z = 0.0;
  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.0;
  //   pose.pose.orientation.w = 1.0;
  //   pose.header.stamp = node_->now();
  //   pose.header.frame_id = global_frame_;
  //   global_path.poses.push_back(pose);
  // }
  global_path.poses.push_back(start);
  global_path.poses.push_back(start);

  // geometry_msgs::msg::PoseStamped goal_pose = goal;
  // goal_pose.header.stamp = node_->now();
  // goal_pose.header.frame_id = global_frame_;
  // global_path.poses.push_back(goal_pose);

  double yaw = quaternionToYaw(goal.pose.orientation);
  double roi_local_x = roi_x_;
  double roi_local_y = roi_y_;
  double roi_center_x = start.pose.position.x + cos(yaw)*roi_local_x + cos(yaw + M_PI/2.0)*roi_local_y;
  double roi_centor_y = start.pose.position.y + sin(yaw)*roi_local_x + sin(yaw + M_PI/2.0)*roi_local_y;
  double roi_radius = roi_r_;
  double roi_cost_ratio = 0.0;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  unsigned int mx = 0;
  unsigned int my = 0;
  unsigned int cost = 0;
  unsigned int total_cost = 0;
  unsigned int total_cost_ratio = 0;
  for (double r = 0.0; r <= roi_radius; r += 0.05) {
    for (double theta = 0.0; theta <= 2.0*M_PI; theta += 0.017) {
      costmap_->worldToMap(
        roi_center_x + r*cos(theta),
        roi_centor_y + r*sin(theta), mx, my);
      cost = costmap_->getCost(mx, my);
      if (cost > 0) {
        total_cost++;
      }
      total_cost_ratio++;
    }
  }

  roi_cost_ratio = (double)total_cost / (double)total_cost_ratio;

  RCLCPP_INFO(node_->get_logger(), "[curb planner] roi_cost_ratio = %f", roi_cost_ratio);

  if (roi_cost_ratio < cost_ratio_) {
    RCLCPP_INFO(node_->get_logger(), "[curb planner] 无法满足路牙启动条件");
    nav_msgs::msg::Path path;
    return path;
  }

  // std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  // unsigned int mx = 0;
  // unsigned int my = 0;
  // for (size_t i = 0; i < global_path.poses.size(); ++i) {
  //   costmap_->worldToMap(
  //     global_path.poses[i].pose.position.x,
  //     global_path.poses[i].pose.position.y, mx, my);
  //   unsigned int cost = costmap_->getCost(mx, my);
  //   // if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
  //   //   cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  //   if (cost > 0)
  //   {
  //     RCLCPP_INFO(
  //       node_->get_logger(), "路径上有障碍物, cost = %d",
  //       cost);
  //     nav_msgs::msg::Path path;
  //     return path;
  //   }
  // }
  return global_path;
}

}  // namespace nav2_curb_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_curb_planner::CurbPlanner, nav2_core::GlobalPlanner)
