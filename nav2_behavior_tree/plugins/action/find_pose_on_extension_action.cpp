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

#include "nav2_behavior_tree/plugins/action/find_pose_on_extension_action.hpp"

namespace nav2_behavior_tree
{

FindPoseOnExtension::FindPoseOnExtension(
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
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("/curb/pose_on_extension", qos);
}

inline BT::NodeStatus FindPoseOnExtension::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  std::string map_frame;
  if (!getInput("map_frame", map_frame)) {
    RCLCPP_ERROR(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "Neither pose nor map_frame specified for %s", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  
  geometry_msgs::msg::PoseStamped origin_pose;
  if (!getRobotPose(map_frame, origin_pose)) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose_on_path;
  double distance;

  getInput("pose_on_path", pose_on_path);
  getInput("distance", distance);

  geometry_msgs::msg::PoseStamped pose_on_extension;

  double distance_oa = sqrt(pow(pose_on_path.pose.position.x - origin_pose.pose.position.x, 2) + pow(pose_on_path.pose.position.y - origin_pose.pose.position.y, 2));
  RCLCPP_INFO(node_->get_logger(), "[FindPoseOnExtension] distance_oa=%f", distance_oa);
  pose_on_extension.pose.position.x = origin_pose.pose.position.x + distance * (pose_on_path.pose.position.x - origin_pose.pose.position.x) / distance_oa;
  pose_on_extension.pose.position.y = origin_pose.pose.position.y + distance * (pose_on_path.pose.position.y - origin_pose.pose.position.y) / distance_oa;
  // double yaw = std::atan2(pose_on_path.pose.position.y - origin_pose.pose.position.y, pose_on_path.pose.position.x - origin_pose.pose.position.x);
  // pose_on_extension.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
  pose_on_extension.pose.orientation = pose_on_path.pose.orientation;
  pose_on_extension.header.frame_id = "odom";
  pose_on_extension.header.stamp = node_->now();

  pose_pub_->publish(pose_on_extension);

  setOutput("pose_on_extension", pose_on_extension);

  return BT::NodeStatus::SUCCESS;
}

inline bool FindPoseOnExtension::getRobotPose(
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
FindPoseOnExtension::poseDistance(
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
  factory.registerNodeType<nav2_behavior_tree::FindPoseOnExtension>(
    "FindPoseOnExtension");
}
