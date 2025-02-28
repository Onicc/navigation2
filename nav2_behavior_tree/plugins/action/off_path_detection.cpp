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

#include "nav2_behavior_tree/plugins/action/off_path_detection.hpp"

namespace nav2_behavior_tree
{

OffPathDetection::OffPathDetection(
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
}

inline BT::NodeStatus OffPathDetection::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  getInput("goals", goals_);
  getInput("path", path_);
  getInput("distance", distance_);

  std::string frame_id = "map";
  int step = path_.poses.size() / 30;

  if(step == 0) {
    return BT::NodeStatus::SUCCESS;
  }

  double max_distance = 0.0;
  double detection_distance = 2.0;
  double sum_distance = 0.0;
  for (size_t i = 0; i < goals_.size()-1; ++i) {
    if (sum_distance > detection_distance) {
      break;
    }
    sum_distance += nav2_util::geometry_utils::euclidean_distance(goals_[i], goals_[i+1]);

    double yaw = tf2::getYaw(goals_[i].pose.orientation) + M_PI/2.0;
    double x = goals_[i].pose.position.x;
    double y = goals_[i].pose.position.y;
    double min_distance = std::numeric_limits<double>::max();
    int min_distance_index = 0;
    for (size_t j = 0; j < path_.poses.size(); j += step) {
      double x1 = path_.poses[j].pose.position.x;
      double y1 = path_.poses[j].pose.position.y;
      double distance = calculate_distance_to_line(x1, y1, x, y, yaw);
      if (distance < min_distance) {
        min_distance = distance;
        min_distance_index = j;
      }
    }
    double max_distance_ = nav2_util::geometry_utils::euclidean_distance(goals_[i], path_.poses[min_distance_index]);
    std::cout << max_distance_ << ", ";
    if (max_distance_ > max_distance) {
      max_distance = max_distance_;
    }
  }
  std::cout << std::endl;

  // double max_distance = 0.0;
  // for (size_t i = 0; i < path_.poses.size(); ++i) {
  //   double distance = nav2_util::geometry_utils::euclidean_distance(goals_[0], path_.poses[i]);
  //   if (distance > max_distance) {
  //     max_distance = distance;
  //   }
  // }

  std::cout << "max_distance: " << max_distance << ", distance: " << distance_ << std::endl;

  if (max_distance > distance_) {
    RCLCPP_INFO(node_->get_logger(), "[OffPathDetection] The distance is greater than the maximum distance");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

double OffPathDetection::calculate_distance_to_line(double x1, double y1, double x, double y, double yaw)
{
  // 计算直线的斜率 m
  double m = std::tan(yaw);  // 假设 yaw 是弧度

  // 计算点到直线的距离
  double numerator = std::abs((y1 - y) - m * (x1 - x));
  double denominator = std::sqrt(1 + m * m);

  double distance = numerator / denominator;

  return distance;
}

inline bool OffPathDetection::getRobotPose(
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

inline bool OffPathDetection::transformPoseToFrame(
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
OffPathDetection::poseDistance(
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
  factory.registerNodeType<nav2_behavior_tree::OffPathDetection>(
    "OffPathDetection");
}
