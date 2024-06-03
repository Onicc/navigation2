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

#include "nav2_behavior_tree/plugins/action/vehicle_automatic_exit.hpp"

namespace nav2_behavior_tree
{

VehicleAutomaticExit::VehicleAutomaticExit(
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
  maximum_crusing_speed_pub_ =
    node->create_publisher<std_msgs::msg::Float32>("vehicle/command/max_cruise_control_speed", qos);
}

inline BT::NodeStatus VehicleAutomaticExit::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  geometry_msgs::msg::Twist cmd_vel;
  nav_msgs::msg::Odometry odometry;
  double remaining_distance;
  getInput("cmd_vel", cmd_vel);
  getInput("odometry", odometry);
  getInput("remaining_distance", remaining_distance);

  static int times = 0;

  if(std::fabs(cmd_vel.linear.x) <= 0.01 && std::fabs(odometry.twist.twist.linear.x) <= 0.01 && remaining_distance < 1.0) {
    times+=1;
  } else {
    times = 0;
  }

  if(times > 50) {
    // exit
    RCLCPP_INFO(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(), "-------退出自动运行-------");
    std::string command = "ros2 service call /bt/navigation_state nav2_msgs/srv/SetString \"{data: stop}\";ros2 service call /vehicle/command/ros2_control slv_msgs/srv/SetString \"{data: OFF}\";ros2 service call /vehicle/command/power slv_msgs/srv/SetString \"{data: OFF}\"";
    int result = system(command.c_str());
    times = 0;
  }

  return BT::NodeStatus::SUCCESS;
}

inline bool VehicleAutomaticExit::getRobotPose(
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
VehicleAutomaticExit::poseDistance(
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
  factory.registerNodeType<nav2_behavior_tree::VehicleAutomaticExit>(
    "VehicleAutomaticExit");
}