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

#include "nav2_behavior_tree/plugins/action/generate_goal_for_following_curbs_action.hpp"

double quaternionToYaw(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf_q(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

namespace nav2_behavior_tree
{

GenerateGoalForFollowingCurbs::GenerateGoalForFollowingCurbs(
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
  curb_goal_pub_ = node->create_publisher<nav_msgs::msg::Path>("/curb/goal", qos);
}

inline BT::NodeStatus GenerateGoalForFollowingCurbs::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path path;
  int input_path_index;
  double local_x, local_y;
  double angular_distance_weight;
  double off_path_tolerance;

  getInput("input_path", path);
  getInput("input_path_index", input_path_index);
  getInput("x", local_x);
  getInput("y", local_y);
  getInput("angular_distance_weight", angular_distance_weight);
  getInput("off_path_tolerance", off_path_tolerance);

  geometry_msgs::msg::PoseStamped pose;
  if (!getRobotPose(path.header.frame_id, pose)) {
    return BT::NodeStatus::FAILURE;
  }

  // 当离路径的距离小的时候才使用路牙,此距离不只是欧式距离,还包含角度距离
  geometry_msgs::msg::PoseStamped nearest_pose_inpath = path.poses[input_path_index];
  double off_path_distance = poseDistance(pose, nearest_pose_inpath, angular_distance_weight);
  if(off_path_distance < off_path_tolerance) {
    geometry_msgs::msg::PoseStamped output_goal;
    double yaw = quaternionToYaw(pose.pose.orientation);
    output_goal.pose.position.x = pose.pose.position.x + local_x * std::cos(yaw) + std::cos(yaw + M_PI/2.0)*local_y;
    output_goal.pose.position.y = pose.pose.position.y + local_x * std::sin(yaw) + std::sin(yaw + M_PI/2.0)*local_y;
    setOutput("output_goal", output_goal);

    // 发布当前点到路牙目标点的连线
    nav_msgs::msg::Path curb_goal;
    curb_goal.header = path.header;
    curb_goal.poses.push_back(pose);
    curb_goal.poses.push_back(output_goal);
    curb_goal_pub_->publish(curb_goal);

    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

inline bool GenerateGoalForFollowingCurbs::getRobotPose(
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
GenerateGoalForFollowingCurbs::poseDistance(
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
  factory.registerNodeType<nav2_behavior_tree::GenerateGoalForFollowingCurbs>(
    "GenerateGoalForFollowingCurbs");
}
