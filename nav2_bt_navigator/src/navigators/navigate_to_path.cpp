// Copyright (c) 2023 星熔科技技术（武汉）有限责任公司
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

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "nav2_bt_navigator/navigators/navigate_to_path.hpp"

namespace nav2_bt_navigator
{

bool
NavigateToPathNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();
  auto blackboard = bt_action_server_->getBlackboard();

  if (!node->has_parameter("goals_blackboard_id")) {
    node->declare_parameter("goals_blackboard_id", std::string("goals"));
  }
  goals_blackboard_id_ = node->get_parameter("goals_blackboard_id").as_string();

  if (!node->has_parameter("path_local_blackboard_id")) {
    node->declare_parameter("path_local_blackboard_id", std::string("path_local"));
  }
  path_local_blackboard_id_ = node->get_parameter("path_local_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  if (!node->has_parameter("navigation_state_blackboard_id")) {
    node->declare_parameter("navigation_state_blackboard_id", std::string("navigation_state"));
  }
  navigation_state_blackboard_id_ = node->get_parameter("navigation_state_blackboard_id").as_string();
  blackboard->set<std::string>(navigation_state_blackboard_id_, "stop");

  if (!node->has_parameter("waypoint_index_blackboard_id")) {
    node->declare_parameter("waypoint_index_blackboard_id", std::string("waypoint_index"));
  }
  waypoint_index_blackboard_id_ = node->get_parameter("waypoint_index_blackboard_id").as_string();
  blackboard->set<int>(waypoint_index_blackboard_id_, -1);

  if (!node->has_parameter("goal_pose_blackboard_id")) {
    node->declare_parameter("goal_pose_blackboard_id", std::string("goal_pose"));
  }
  goal_pose_blackboard_id_ = node->get_parameter("goal_pose_blackboard_id").as_string();
  blackboard->set<geometry_msgs::msg::PoseStamped>(goal_pose_blackboard_id_, geometry_msgs::msg::PoseStamped());

  if (!node->has_parameter("replanning_count_blackboard_id")) {
    node->declare_parameter("replanning_count_blackboard_id", std::string("replanning_count"));
  }
  replanning_count_blackboard_id_ = node->get_parameter("replanning_count_blackboard_id").as_string();
  blackboard->set<int>(replanning_count_blackboard_id_, 0);

  if (!node->has_parameter("farthest_obstacle_point_blackboard_id")) {
    node->declare_parameter("farthest_obstacle_point_blackboard_id", std::string("farthest_obstacle_point"));
  }
  farthest_obstacle_point_blackboard_id_ = node->get_parameter("farthest_obstacle_point_blackboard_id").as_string();

  if (!node->has_parameter("manual_goal_pose_blackboard_id")) {
    node->declare_parameter("manual_goal_pose_blackboard_id", std::string("manual_goal"));
  }
  manual_goal_pose_blackboard_id_ = node->get_parameter("manual_goal_pose_blackboard_id").as_string();
  geometry_msgs::msg::PoseStamped manual_goal_pose;
  manual_goal_pose.header.frame_id = "odom";
  manual_goal_pose.pose.position.x = 0.0;
  manual_goal_pose.pose.position.y = 0.1;
  blackboard->set<geometry_msgs::msg::PoseStamped>(manual_goal_pose_blackboard_id_, manual_goal_pose);

  // if (!node->has_parameter("command_blackboard_id")) {
  //   node->declare_parameter("command_blackboard_id", std::string("command"));
  // }
  // command_blackboard_id_ = node->get_parameter("command_blackboard_id").as_string();

  // if (!node->has_parameter("start_blackboard_id")) {
  //   node->declare_parameter("start_blackboard_id", std::string("start"));
  // }
  // start_blackboard_id_ = node->get_parameter("start_blackboard_id").as_string();
  // blackboard->set<std::string>(start_blackboard_id_, "false");

  if (!node->has_parameter("waypoints_blackboard_id")) {
    node->declare_parameter("waypoints_blackboard_id", std::string("waypoints"));
  }
  waypoints_blackboard_id_ = node->get_parameter("waypoints_blackboard_id").as_string();

  if (!node->has_parameter("waypoint_blackboard_id")) {
    node->declare_parameter("waypoint_blackboard_id", std::string("waypoint"));
  }
  waypoint_blackboard_id_ = node->get_parameter("waypoint_blackboard_id").as_string();

  if (!node->has_parameter("odometry_gps_blackboard_id")) {
    node->declare_parameter("odometry_gps_blackboard_id", std::string("odometry_gps"));
  }
  odometry_gps_blackboard_id_ = node->get_parameter("odometry_gps_blackboard_id").as_string();

  if (!node->has_parameter("curb_traction_point_blackboard_id")) {
    node->declare_parameter("curb_traction_point_blackboard_id", std::string("curb_traction_point"));
  }
  curb_traction_point_blackboard_id_ = node->get_parameter("curb_traction_point_blackboard_id").as_string();

  if (!node->has_parameter("curb_traction_point_extension_blackboard_id")) {
    node->declare_parameter("curb_traction_point_extension_blackboard_id", std::string("curb_traction_point_extension"));
  }
  curb_traction_point_extension_blackboard_id_ = node->get_parameter("curb_traction_point_extension_blackboard_id").as_string();

  if (!node->has_parameter("curb_path_extension_blackboard_id")) {
    node->declare_parameter("curb_path_extension_blackboard_id", std::string("curb_path_extension"));
  }
  curb_path_extension_blackboard_id_ = node->get_parameter("curb_path_extension_blackboard_id").as_string();

  if (!node->has_parameter("global_costmap_blackboard_id")) {
    node->declare_parameter("global_costmap_blackboard_id", std::string("global_costmap"));
  }
  global_costmap_blackboard_id_ = node->get_parameter("global_costmap_blackboard_id").as_string();

  if (!node->has_parameter("exception_blackboard_id")) {
    node->declare_parameter("exception_blackboard_id", std::string("exception"));
  }
  exception_blackboard_id_ = node->get_parameter("exception_blackboard_id").as_string();
  nav2_msgs::msg::Exception default_exception;
  blackboard->set<nav2_msgs::msg::Exception>(exception_blackboard_id_, default_exception);

  if (!node->has_parameter("goals_truncate_blackboard_id")) {
    node->declare_parameter("goals_truncate_blackboard_id", std::string("goals_truncated"));
  }
  goals_truncate_blackboard_id_ = node->get_parameter("goals_truncate_blackboard_id").as_string();

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  // command_sub_ = node->create_subscription<std_msgs::msg::String>(
  //   "command",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(&NavigateToPathNavigator::onCommandReceived, this, std::placeholders::_1));

  waypoints_sub_ = node->create_subscription<nav2_msgs::msg::WaypointArray>(
    "/waypoints",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onWaypointsReceived, this, std::placeholders::_1));

  odometry_gps_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/gps",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onOdometryGPSReceived, this, std::placeholders::_1));

  curb_traction_point_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/curb/traction_point",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onCurbTractionPointReceived, this, std::placeholders::_1));

  global_costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onGlobalCostmapReceived, this, std::placeholders::_1));

  // bt_navigator_start_sub_ = node->create_subscription<std_msgs::msg::String>(
  //   "/bt_navigator/start",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(&NavigateToPathNavigator::onBTNavigatorStartReceived, this, std::placeholders::_1));

  waypoints_service_ = node->create_service<nav2_msgs::srv::SetWaypoints>(
    "/waypoints",
    std::bind(&NavigateToPathNavigator::onWaypointsReceivedSrv, this, std::placeholders::_1, std::placeholders::_2));

  load_waypoints_service_ = node->create_service<nav2_msgs::srv::SetString>(
    "/command/load_waypoints",
    std::bind(&NavigateToPathNavigator::onLoadWaypointsSrv, this, std::placeholders::_1, std::placeholders::_2));

  bt_navigation_state_service_ = node->create_service<nav2_msgs::srv::SetString>(
    "/bt/navigation_state",
    std::bind(&NavigateToPathNavigator::onNavigationStateReceived, this, std::placeholders::_1, std::placeholders::_2));

  // bt_command_service_ = node->create_service<nav2_msgs::srv::SetString>(
  //   "/bt/command",
  //   std::bind(&NavigateToPathNavigator::onBTCommandReceived, this, std::placeholders::_1, std::placeholders::_2));

  // bt_start_service_ = node->create_service<nav2_msgs::srv::SetString>(
  //   "/bt/start",
  //   std::bind(&NavigateToPathNavigator::onBTStartReceived, this, std::placeholders::_1, std::placeholders::_2));

  last_loop_time_ = std::chrono::high_resolution_clock::now();

  return true;
}

std::string
NavigateToPathNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_nav_to_path_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    node->declare_parameter<std::string>(
      "default_nav_to_path_bt_xml",
      pkg_share_dir +
      "/behavior_trees/navigate_to_path_w_recovery.xml");
  }

  node->get_parameter("default_nav_to_path_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
NavigateToPathNavigator::cleanup()
{
  waypoints_sub_.reset();
  self_client_.reset();
  goal_sub_.reset();
  command_sub_.reset();
  return true;
}

bool
NavigateToPathNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPath(goal);

  return true;
}

void
NavigateToPathNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/,
  const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
}

void
NavigateToPathNavigator::onLoop()
{
  // std::chrono::high_resolution_clock::time_point now_time = std::chrono::high_resolution_clock::now();
  // std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(now_time - last_loop_time_);
  // last_loop_time_ = now_time;
  // RCLCPP_INFO(logger_, "onLoop: %ld ms", duration.count()/1000.0);  

  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  auto blackboard = bt_action_server_->getBlackboard();

  try {
    // Get current waypoints
    nav2_msgs::msg::WaypointArray current_waypoints;
    blackboard->get<nav2_msgs::msg::WaypointArray>(waypoints_blackboard_id_, current_waypoints);

    // Get current path points
    nav_msgs::msg::Path current_path;
    // blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);
    current_path.header = current_waypoints.header;
    for (size_t i = 0; i < current_waypoints.waypoints.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = current_waypoints.waypoints[i].header;
      pose.pose = current_waypoints.waypoints[i].pose;
      current_path.poses.push_back(pose);
    }

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

    // Calculate distance on the path
    double distance_remaining =
      nav2_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    // Get current speed
    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    // Calculate estimated time taken to goal if speed is higher than 1cm/s
    // and at least 10cm to go
    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  } catch (...) {
    // Ignore
  }

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = clock_->now() - start_time_;

  bt_action_server_->publishFeedback(feedback_msg);
}

void
NavigateToPathNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPath(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void
NavigateToPathNavigator::initializeGoalPath(ActionT::Goal::ConstSharedPtr goal)
{
  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);
  auto blackboard = bt_action_server_->getBlackboard();
  
  if(goal->manual_goal.header.frame_id != "") {
    RCLCPP_INFO(
      logger_, "Begin navigating from current location (%.2f, %.2f) to manual goal (%.2f, %.2f)", 
        current_pose.pose.position.x, current_pose.pose.position.y,
        goal->manual_goal.pose.position.x, goal->manual_goal.pose.position.y);

    // Reset state for new action feedback
    start_time_ = clock_->now();
    blackboard->set<int>("number_recoveries", 0);  // NOLINT

    // Update the goal path on the blackboard
    blackboard->set<geometry_msgs::msg::PoseStamped>(manual_goal_pose_blackboard_id_, goal->manual_goal);
  }

  if(!goal->waypoints.waypoints.empty()) {
    // Calculate total distance of the path
    double total_distance = 0.0;
    for (size_t i = 0; i < goal->waypoints.waypoints.size() - 1; ++i) {
      total_distance += nav2_util::geometry_utils::euclidean_distance(
        goal->waypoints.waypoints[i].pose, goal->waypoints.waypoints[i + 1].pose);
    }

    RCLCPP_INFO(
      logger_, "Begin navigating from current location (%.2f, %.2f) to path with %zu poses and total distance of %.2f meters", 
        current_pose.pose.position.x, current_pose.pose.position.y,
        goal->waypoints.waypoints.size(), total_distance);

    // Reset state for new action feedback
    start_time_ = clock_->now();
    blackboard->set<int>("number_recoveries", 0);  // NOLINT

    // Update the waypoints on the blackboard
    blackboard->set<nav2_msgs::msg::WaypointArray>(waypoints_blackboard_id_, goal->waypoints);
  }

  // if(goal->command.data != "") {
  //   RCLCPP_INFO(
  //     logger_, "Begin navigating from current location (%.2f, %.2f) to command %s", 
  //       current_pose.pose.position.x, current_pose.pose.position.y,
  //       goal->command.data.c_str());

  //   // Reset state for new action feedback
  //   start_time_ = clock_->now();

  //   // Update the goal path on the blackboard
  //   blackboard->set<std_msgs::msg::String>(command_blackboard_id_, goal->command);
  // }
}

void
NavigateToPathNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  ActionT::Goal goal;
  goal.manual_goal = *pose;
  self_client_->async_send_goal(goal);
}

// void
// NavigateToPathNavigator::onCommandReceived(const std_msgs::msg::String::SharedPtr command)
// {
//   ActionT::Goal goal;
//   goal.command = *command;
//   self_client_->async_send_goal(goal);
// }

// void
// NavigateToPathNavigator::onBTNavigatorStartReceived(const std_msgs::msg::String::SharedPtr msg)
// {
//   auto blackboard = bt_action_server_->getBlackboard();
//   blackboard->set<std::string>(start_blackboard_id_, msg->data);
// }

void
NavigateToPathNavigator::onWaypointsReceived(const nav2_msgs::msg::WaypointArray::SharedPtr msg)
{
  ActionT::Goal goal;
  goal.waypoints = *msg;
  self_client_->async_send_goal(goal);
}

void
NavigateToPathNavigator::onGlobalCostmapReceived(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav2_msgs::msg::Costmap>(global_costmap_blackboard_id_, *msg);
}

void 
NavigateToPathNavigator::onWaypointsReceivedSrv(
  const std::shared_ptr<nav2_msgs::srv::SetWaypoints::Request> request, 
  std::shared_ptr<nav2_msgs::srv::SetWaypoints::Response> response)
{
  RCLCPP_INFO(logger_, "Received waypoints request, The path has %ld waypoints.", request->data.waypoints.size());
  ActionT::Goal goal;
  goal.waypoints = request->data;
  self_client_->async_send_goal(goal);
  response->success = true;
}

void 
NavigateToPathNavigator::onLoadWaypointsSrv(
  const std::shared_ptr<nav2_msgs::srv::SetString::Request> request, 
  std::shared_ptr<nav2_msgs::srv::SetString::Response> response)
{
  RCLCPP_INFO(logger_, "Received waypoints request, The waypoints path is %s", request->data.c_str());
  auto waypoints = loadWaypoints(request->data);
  RCLCPP_INFO(logger_, "The path has %ld waypoints.", waypoints.waypoints.size());

  ActionT::Goal goal;
  goal.waypoints = waypoints;
  self_client_->async_send_goal(goal);
  response->success = true;

  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<std::string>(navigation_state_blackboard_id_, "path_following");
}

void
NavigateToPathNavigator::onOdometryGPSReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav_msgs::msg::Odometry>(odometry_gps_blackboard_id_, *msg);
}

void
NavigateToPathNavigator::onCurbTractionPointReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<geometry_msgs::msg::PoseStamped>(curb_traction_point_blackboard_id_, *msg);
}

void
NavigateToPathNavigator::onNavigationStateReceived(
    const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetString::Response> response)
{
  RCLCPP_INFO(logger_, "Received navigation state request: %s", request->data.c_str());
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<std::string>(navigation_state_blackboard_id_, request->data);
  response->success = true;
}

// void
// NavigateToPathNavigator::onBTCommandReceived(
//     const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
//     std::shared_ptr<nav2_msgs::srv::SetString::Response> response)
// {
//   RCLCPP_INFO(logger_, "Received command request: %s", request->data.c_str());
//   std_msgs::msg::String command;
//   command.data = request->data;
//   auto blackboard = bt_action_server_->getBlackboard();
//   blackboard->set<std_msgs::msg::String>(command_blackboard_id_, command);
//   response->success = true;
// }

// void
// NavigateToPathNavigator::onBTStartReceived(
//     const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
//     std::shared_ptr<nav2_msgs::srv::SetString::Response> response)
// {
//   RCLCPP_INFO(logger_, "Received start request: %s", request->data.c_str());
//   auto blackboard = bt_action_server_->getBlackboard();
//   blackboard->set<std::string>(start_blackboard_id_, request->data);
//   response->success = true;
// }

nav2_msgs::msg::WaypointArray NavigateToPathNavigator::loadWaypoints(const std::string& waypointsFile)
{
  auto blackboard = bt_action_server_->getBlackboard();
  auto node = blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::vector<nav2_msgs::msg::Waypoint> waypoints;

  try {
      YAML::Node yamlData = YAML::LoadFile(waypointsFile);
      const YAML::Node& waypointsNode = yamlData["waypoints"];

      for (const YAML::Node& transform : waypointsNode) {
          geometry_msgs::msg::Pose pose;
          pose.position.x = transform["position"]["x"].as<double>();
          pose.position.y = transform["position"]["y"].as<double>();
          pose.position.z = transform["position"]["z"].as<double>();
          pose.orientation.x = transform["orientation"]["x"].as<double>();
          pose.orientation.y = transform["orientation"]["y"].as<double>();
          pose.orientation.z = transform["orientation"]["z"].as<double>();
          pose.orientation.w = transform["orientation"]["w"].as<double>();

          nav2_msgs::msg::Waypoint waypoint;
          waypoint.header.frame_id = "map";
          waypoint.header.stamp = node->get_clock()->now();
          waypoint.pose = pose;
          waypoint.curb_distance = transform["curb"]["distance"].as<double>();
          waypoint.curb_direction = transform["curb"]["direction"].as<double>();
          waypoint.curb_residual = transform["curb"]["residual"].as<double>();
          waypoint.option_curb_direction_fix = transform["option"]["curb_direction_fix"].as<bool>();
          waypoint.option_curb_horizontal_fix = transform["option"]["curb_horizontal_fix"].as<bool>();
          waypoint.option_curb_traction_fix = transform["option"]["curb_traction"].as<bool>();
          waypoint.option_bypass_obstacle = transform["option"]["bypass_obstacle"].as<bool>();
          waypoint.option_stop_obstacle = transform["option"]["stop_obstacle"].as<bool>();
          waypoint.option_speed = transform["option"]["speed"].as<float>();
          waypoint.option_cleaning_mode = transform["option"]["cleaning_mode"].as<int>();

          waypoints.push_back(waypoint);
      }
      RCLCPP_INFO(logger_, "Waypoints loaded, total: %zu", waypoints.size());
  } catch (const std::exception& e) {
      RCLCPP_INFO(logger_, "Failed to load waypoints: %s", e.what());
  }

  nav2_msgs::msg::WaypointArray waypointsMsg;
  waypointsMsg.header.frame_id = "map";
  waypointsMsg.header.stamp = node->get_clock()->now();
  waypointsMsg.waypoints = waypoints;

  return waypointsMsg;
}


}  // namespace nav2_bt_navigator
