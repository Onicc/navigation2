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

  if (!node->has_parameter("obstacle_mode_blackboard_id")) {
    node->declare_parameter("obstacle_mode_blackboard_id", std::string("obstacle_mode"));
  }
  obstacle_mode_blackboard_id_ = node->get_parameter("obstacle_mode_blackboard_id").as_string();
  blackboard->set<std::string>(obstacle_mode_blackboard_id_, "auto");

  if (!node->has_parameter("detect_obstacle_distance_blackboard_id")) {
    node->declare_parameter("detect_obstacle_distance_blackboard_id", std::string("detect_obstacle_distance"));
  }
  detect_obstacle_distance_blackboard_id_ = node->get_parameter("detect_obstacle_distance_blackboard_id").as_string();
  blackboard->set<double>(detect_obstacle_distance_blackboard_id_, 4.0);

  if (!node->has_parameter("traffic_light_blackboard_id")) {
    node->declare_parameter("traffic_light_blackboard_id", std::string("traffic_light"));
  }
  traffic_light_blackboard_id_ = node->get_parameter("traffic_light_blackboard_id").as_string();
  blackboard->set<std::string>(traffic_light_blackboard_id_, "none");

  if (!node->has_parameter("robot_frame_blackboard_id")) {
    node->declare_parameter("robot_frame_blackboard_id", std::string("robot_frame"));
  }
  robot_frame_blackboard_id_ = node->get_parameter("robot_frame_blackboard_id").as_string();
  blackboard->set<std::string>(robot_frame_blackboard_id_, "base_link");

  if (!node->has_parameter("base_link_frame_id")) {
    node->declare_parameter("base_link_frame_id", std::string("base_link_frame"));
  }
  base_link_frame_id_ = node->get_parameter("base_link_frame_id").as_string();
  blackboard->set<std::string>(base_link_frame_id_, "front_base_link");

  if (!node->has_parameter("cmd_vel_frame_id")) {
    node->declare_parameter("cmd_vel_frame_id", std::string("cmd_vel"));
  }
  cmd_vel_frame_id_ = node->get_parameter("cmd_vel_frame_id").as_string();
  geometry_msgs::msg::Twist cmd_vel;
  blackboard->set<geometry_msgs::msg::Twist>(cmd_vel_frame_id_, cmd_vel);

  if (!node->has_parameter("front_odometry_blackboard_id")) {
    node->declare_parameter("front_odometry_blackboard_id", std::string("front_odometry"));
  }
  front_odometry_blackboard_id_ = node->get_parameter("front_odometry_blackboard_id").as_string();
  nav_msgs::msg::Odometry front_odometry;
  blackboard->set<nav_msgs::msg::Odometry>(front_odometry_blackboard_id_, front_odometry);

  if (!node->has_parameter("remaining_distance_blackboard_id")) {
    node->declare_parameter("remaining_distance_blackboard_id", std::string("remaining_distance"));
  }
  remaining_distance_blackboard_id_ = node->get_parameter("remaining_distance_blackboard_id").as_string();
  double remaining_distance=99999.9;
  blackboard->set<double>(remaining_distance_blackboard_id_, remaining_distance);

  // if (!node->has_parameter("manual_mode_blackboard_id")) {
  //   node->declare_parameter("manual_mode_blackboard_id", std::string("manual_mode"));
  // }
  // manual_mode_frame_id_ = node->get_parameter("manual_mode_blackboard_id").as_string();
  // blackboard->set<std::string>(manual_mode_frame_id_, "pose_mode");

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/foxglove/goal",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  // command_sub_ = node->create_subscription<std_msgs::msg::String>(
  //   "command",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(&NavigateToPathNavigator::onCommandReceived, this, std::placeholders::_1));

  waypoints_sub_ = node->create_subscription<nav2_msgs::msg::WaypointArray>(
    "/original_waypoints",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onWaypointsReceived, this, std::placeholders::_1));

  odometry_gps_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/gps/raw/var",
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

  detect_obstacle_distance_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    "/detect_obstacle_distance",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onDetectObstacleDistanceReceived, this, std::placeholders::_1));

  traffic_light_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "/traffic_light_recognition/result",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onTrafficLightReceived, this, std::placeholders::_1));

  robot_frame_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/robot_frame",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onRobotFrameReceived1, this, std::placeholders::_1));

  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onCmdVelReceived, this, std::placeholders::_1));

  front_odometry_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/articulation_controller/front_odometry",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPathNavigator::onFrontOdometryReceived, this, std::placeholders::_1));

  // teleop_cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
  //   "/manual/cmd_vel",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(&NavigateToPathNavigator::onTeleopCmdVelReceived, this, std::placeholders::_1));

  // bt_navigator_start_sub_ = node->create_subscription<std_msgs::msg::String>(
  //   "/bt_navigator/start",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(&NavigateToPathNavigator::onBTNavigatorStartReceived, this, std::placeholders::_1));

  waypoints_service_ = node->create_service<nav2_msgs::srv::SetWaypoints>(
    "/waypoints",
    std::bind(&NavigateToPathNavigator::onWaypointsReceivedSrv, this, std::placeholders::_1, std::placeholders::_2));

  // load_waypoints_service_ = node->create_service<nav2_msgs::srv::SetString>(
  //   "/command/load_waypoints",
  //   std::bind(&NavigateToPathNavigator::onLoadWaypointsSrv, this, std::placeholders::_1, std::placeholders::_2));

  start_auto_cleaning_service_ = node->create_service<nav2_msgs::srv::SetString>(
    "/command/start_auto_cleaning",
    std::bind(&NavigateToPathNavigator::onStartAutoCleaningSrv, this, std::placeholders::_1, std::placeholders::_2));

  bt_navigation_state_service_ = node->create_service<nav2_msgs::srv::SetString>(
    "/bt/navigation_state",
    std::bind(&NavigateToPathNavigator::onNavigationStateReceived, this, std::placeholders::_1, std::placeholders::_2));

  bt_obstacle_mode_service_ = node->create_service<nav2_msgs::srv::SetString>(
    "/bt/obstacle_mode",
    std::bind(&NavigateToPathNavigator::onObstacleModeReceived, this, std::placeholders::_1, std::placeholders::_2));

  bt_robot_frame_service_ = node->create_service<nav2_msgs::srv::SetString>(
    "/bt/robot_frame",
    std::bind(&NavigateToPathNavigator::onRobotFrameReceived, this, std::placeholders::_1, std::placeholders::_2));

  // bt_command_service_ = node->create_service<nav2_msgs::srv::SetString>(
  //   "/bt/command",
  //   std::bind(&NavigateToPathNavigator::onBTCommandReceived, this, std::placeholders::_1, std::placeholders::_2));

  // bt_start_service_ = node->create_service<nav2_msgs::srv::SetString>(
  //   "/bt/start",
  //   std::bind(&NavigateToPathNavigator::onBTStartReceived, this, std::placeholders::_1, std::placeholders::_2));

  beam_pub_ = node->create_publisher<std_msgs::msg::String>("vehicle/command/beam", 10);
  voice_pub_ = node->create_publisher<std_msgs::msg::String>("/voice", 10);
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/entire_path", 10);  
  optimized_waypoints_pub_ = node->create_publisher<nav2_msgs::msg::WaypointArray>("/optimized_waypoints", 10);

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
    if(std::fabs(goal->manual_goal.pose.position.x) < 0.0001 && std::fabs(goal->manual_goal.pose.position.y) < 0.0001) {
      blackboard->set<geometry_msgs::msg::PoseStamped>(manual_goal_pose_blackboard_id_, current_pose);
    } else {
      blackboard->set<geometry_msgs::msg::PoseStamped>(manual_goal_pose_blackboard_id_, goal->manual_goal);
    }
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
    blackboard->set<int>(waypoint_index_blackboard_id_, waypoint_index_blackboard_);
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

  auto blackboard = bt_action_server_->getBlackboard();
  // blackboard->set<std::string>(manual_mode_frame_id_, "pose_mode");
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
  // ActionT::Goal goal;
  // goal.waypoints = *msg;
  // self_client_->async_send_goal(goal);
  waypoints_ = *msg;
  RCLCPP_INFO(logger_, "Received waypoints msg");
}

void
NavigateToPathNavigator::onGlobalCostmapReceived(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav2_msgs::msg::Costmap>(global_costmap_blackboard_id_, *msg);
}

void
NavigateToPathNavigator::onDetectObstacleDistanceReceived(const std_msgs::msg::Float32::SharedPtr msg)
{
  double detect_obstacle_distance = msg->data;
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<double>(detect_obstacle_distance_blackboard_id_, detect_obstacle_distance);
}

void
NavigateToPathNavigator::onTrafficLightReceived(const std_msgs::msg::Int32::SharedPtr msg)
{
  int traffic_light = msg->data;
  auto blackboard = bt_action_server_->getBlackboard();
  if(traffic_light == 0) blackboard->set<std::string>(traffic_light_blackboard_id_, "none");
  if(traffic_light == 1) blackboard->set<std::string>(traffic_light_blackboard_id_, "red");
  if(traffic_light == 2) blackboard->set<std::string>(traffic_light_blackboard_id_, "yellow");
  if(traffic_light == 3) blackboard->set<std::string>(traffic_light_blackboard_id_, "green");
}

void
NavigateToPathNavigator::onRobotFrameReceived1(const std_msgs::msg::String::SharedPtr msg)
{
  robot_frame_ = msg->data;

  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<std::string>(base_link_frame_id_, robot_frame_);
}

void
NavigateToPathNavigator::onCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<geometry_msgs::msg::Twist>(cmd_vel_frame_id_, *msg);
}

void
NavigateToPathNavigator::onFrontOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav_msgs::msg::Odometry>(front_odometry_blackboard_id_, *msg);
}

// void
// NavigateToPathNavigator::onTeleopCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
// {
//   auto blackboard = bt_action_server_->getBlackboard();
//   blackboard->set<std::string>(manual_mode_frame_id_, "teleop_mode");
// }

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

// void 
// NavigateToPathNavigator::onLoadWaypointsSrv(
//   const std::shared_ptr<nav2_msgs::srv::SetString::Request> request, 
//   std::shared_ptr<nav2_msgs::srv::SetString::Response> response)
// {
//   RCLCPP_INFO(logger_, "Received waypoints request, The waypoints path is %s", request->data.c_str());
//   waypoints_path_ = request->data;

//   if(!rcpputils::fs::exists(waypoints_path_)) {
//     RCLCPP_ERROR(logger_, "The waypoints path is not exist.");
//     response->success = false;
//     return;
//   }

//   RCLCPP_INFO(logger_, "Successfully loaded path file %s", waypoints_path_.c_str());

//   response->success = true;
// }

void 
NavigateToPathNavigator::onStartAutoCleaningSrv(
  const std::shared_ptr<nav2_msgs::srv::SetString::Request> request, 
  std::shared_ptr<nav2_msgs::srv::SetString::Response> response)
{
  if(waypoints_.waypoints.size() == 0) {
    RCLCPP_INFO(logger_, "The waypoints path is empty.");
    response->success = false;
    return;
  }

  if(request->data == "start_point" || request->data == "middle_point") {
    if(request->data == "start_point") {
      waypoint_index_blackboard_ = 0;   // 从起点起步
      RCLCPP_INFO(logger_, "The command is not start_point.");
      voice_pub_->publish(std_msgs::msg::String().set__data("车辆准备运行，请注意避让"));
    }
    if(request->data == "middle_point") {
      waypoint_index_blackboard_ = -1;  // 从中途起步
      RCLCPP_INFO(logger_, "The command is not middle_point.");
      voice_pub_->publish(std_msgs::msg::String().set__data("车辆准备运行，请注意避让"));
    }

    RCLCPP_INFO(logger_, "Received waypoints request, The waypoints path is %s", request->data.c_str());
    auto waypoints = loadWaypoints(waypoints_path_);
    RCLCPP_INFO(logger_, "The path has %ld waypoints.", waypoints.waypoints.size());
    if(waypoints.waypoints.size() == 0) {
      response->success = false;
      return;
    }

    optimized_waypoints_pub_->publish(waypoints);

    // // Get current path points
    // nav_msgs::msg::Path entire_path;
    // entire_path.header = waypoints.header;
    // for (size_t i = 0; i < waypoints.waypoints.size(); ++i) {
    //   geometry_msgs::msg::PoseStamped pose;
    //   pose.header = waypoints.waypoints[i].header;
    //   pose.pose = waypoints.waypoints[i].pose;
    //   entire_path.poses.push_back(pose);
    // }
    // path_pub_->publish(entire_path);

    // auto beam_message = std_msgs::msg::String();
    // beam_message.data = "HAZARD_BEAM";
    // beam_pub_->publish(beam_message);
    // beam_message.data = "EMERGENCY_BEAM";
    // beam_pub_->publish(beam_message);

    ActionT::Goal goal;
    goal.waypoints = waypoints;
    self_client_->async_send_goal(goal);
    response->success = true;

    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::string>(navigation_state_blackboard_id_, "path_following");
  } else {
    RCLCPP_INFO(logger_, "The command is not start_point or middle_point.");
    response->success = false;
  }
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

  std::string navigation_state = request->data;
  // if(navigation_state == "stop") {
  //   auto beam_message = std_msgs::msg::String();
  //   beam_message.data = "OFF_HAZARD_BEAM";
  //   beam_pub_->publish(beam_message);
  //   beam_message.data = "OFF_EMERGENCY_BEAM";
  //   beam_pub_->publish(beam_message);
  // }

  response->success = true;
}

void
NavigateToPathNavigator::onObstacleModeReceived(
    const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetString::Response> response)
{
  RCLCPP_INFO(logger_, "Received obstacle mode request: %s", request->data.c_str());
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<std::string>(obstacle_mode_blackboard_id_, request->data);
  response->success = true;
}

void
NavigateToPathNavigator::onRobotFrameReceived(
    const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetString::Response> response)
{
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<std::string>(robot_frame_blackboard_id_, request->data);
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

  auto waypoints = waypoints_.waypoints;

  std::string robot_frame;
  blackboard->get<std::string>(robot_frame_blackboard_id_, robot_frame);
  
  // 查找最近点索引
  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, robot_frame,
    feedback_utils_.transform_tolerance);

  RCLCPP_INFO(logger_, "Current pose: (%.2f, %.2f)", current_pose.pose.position.x, current_pose.pose.position.y);
  RCLCPP_INFO(logger_, "robot_frame: %s", robot_frame.c_str());


  try {
    // Get current path points
    nav_msgs::msg::Path current_path;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = waypoints[i].header;
      pose.pose = waypoints[i].pose;
      current_path.poses.push_back(pose);
    }

    int max_find_index = current_path.poses.size();
    if(waypoint_index_blackboard_ == 0) {
      double max_find_distance = 3;
      double distance = 0.0;
      for (size_t curr_idx = 1; curr_idx < current_path.poses.size(); ++curr_idx) {
        distance += nav2_util::geometry_utils::euclidean_distance(
          current_path.poses[curr_idx-1], current_path.poses[curr_idx]);
        if (distance > max_find_distance) {
          max_find_index = curr_idx;
          break;
        }
      }
    }


    // Find the closest pose to current pose on global path
    size_t closest_pose_idx = 0;
    double curr_min_dist = std::numeric_limits<double>::max();
    for (size_t curr_idx = 0; curr_idx < max_find_index; ++curr_idx) {
      double curr_dist = nav2_util::geometry_utils::euclidean_distance(
        current_pose, current_path.poses[curr_idx]);
      if (curr_dist < curr_min_dist) {
        curr_min_dist = curr_dist;
        closest_pose_idx = curr_idx;
      }
    }

    RCLCPP_INFO(logger_, "Closest pose index: %ld", closest_pose_idx);

    double closest_pose_distance = nav2_util::geometry_utils::euclidean_distance(
      current_pose, current_path.poses[closest_pose_idx]);
    RCLCPP_INFO(logger_, "Closest pose distance: %.2f", closest_pose_distance);
    if(closest_pose_distance > 3.0) {
      RCLCPP_INFO(logger_, "Closest pose distance is too far, ignore waypoints.");
      nav2_msgs::msg::WaypointArray waypointsMsg;
      return waypointsMsg;
    }

    BezierCurve curve;
    std::array<double, 2> p0 = {current_pose.pose.position.x, current_pose.pose.position.y};
    double w = current_pose.pose.orientation.w;
    double x = current_pose.pose.orientation.x;
    double y = current_pose.pose.orientation.y;
    double z = current_pose.pose.orientation.z;
    double heading0 = atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
    if(waypoints[closest_pose_idx].option_speed < 0.0) {
      heading0 += M_PI;
    }

    // 在最近点后curve_max_find_distance米内查找最大曲率点
    int curve_max_find_index = current_path.poses.size();
    double curve_max_find_distance = 10;
    double curve_distance = 0.0;
    for (size_t curr_idx = (closest_pose_idx+1); curr_idx < current_path.poses.size(); ++curr_idx) {
      curve_distance += nav2_util::geometry_utils::euclidean_distance(
        current_path.poses[curr_idx-1], current_path.poses[curr_idx]);
      if (curve_distance > curve_max_find_distance) {
        curve_max_find_index = curr_idx;
        break;
      }
    }

    // 在最近点后查找最大曲率点
    for (size_t i = closest_pose_idx; i < curve_max_find_index; i += 1) {
        std::array<double, 2> p3 = {current_path.poses[i].pose.position.x, current_path.poses[i].pose.position.y};
        double heading3 = atan2(2*(current_path.poses[i].pose.orientation.w*current_path.poses[i].pose.orientation.z+current_path.poses[i].pose.orientation.x*current_path.poses[i].pose.orientation.y), 1-2*(current_path.poses[i].pose.orientation.y*current_path.poses[i].pose.orientation.y+current_path.poses[i].pose.orientation.z*current_path.poses[i].pose.orientation.z));
        std::tuple<bool, std::vector<std::array<double, 2>>, std::vector<double>> curvePointsOrientations = curve.bezierCurve(p0, p3, heading0, heading3, max_curvature, 100);
        if(std::get<0>(curvePointsOrientations) == true) {
          RCLCPP_INFO(logger_, "Search pose index: %ld", i);
          std::vector<std::array<double, 2>> curvePoints = std::get<1>(curvePointsOrientations);
          std::vector<double> curveOrientations = std::get<2>(curvePointsOrientations);
          std::vector<nav2_msgs::msg::Waypoint> curveWaypoints;
          for (size_t j = 0; j < curvePoints.size(); ++j) {
            nav2_msgs::msg::Waypoint waypoint;
            waypoint.header.frame_id = "map";
            waypoint.header.stamp = node->get_clock()->now();
            waypoint.pose.position.x = curvePoints[j][0];
            waypoint.pose.position.y = curvePoints[j][1];
            waypoint.pose.position.z = 0.0;
            waypoint.pose.orientation.x = 0.0;
            waypoint.pose.orientation.y = 0.0;
            waypoint.pose.orientation.z = sin(curveOrientations[j]/2);
            waypoint.pose.orientation.w = cos(curveOrientations[j]/2);
            waypoint.curb_distance = 0.0;
            waypoint.curb_direction = 0.0;
            waypoint.curb_residual = 0.0;
            waypoint.option_curb_direction_fix = waypoints[0].option_curb_direction_fix;
            waypoint.option_curb_horizontal_fix = waypoints[0].option_curb_horizontal_fix;
            waypoint.option_curb_traction_fix = waypoints[0].option_curb_traction_fix;
            waypoint.option_bypass_obstacle = waypoints[0].option_bypass_obstacle;
            waypoint.option_stop_obstacle = waypoints[0].option_stop_obstacle;
            waypoint.option_speed = waypoints[0].option_speed;
            waypoint.option_cleaning_mode = waypoints[0].option_cleaning_mode;
            waypoint.option_traffic_light = waypoints[0].option_traffic_light;
            waypoint.option_gps_poor_stop = waypoints[0].option_gps_poor_stop;
            waypoint.option_turn_signal = waypoints[0].option_turn_signal;
            curveWaypoints.push_back(waypoint);
          }
          curveWaypoints.insert(curveWaypoints.end(), waypoints.begin()+i, waypoints.end());
          RCLCPP_INFO(logger_, "Curve waypoints: %ld", curveWaypoints.size());
          nav2_msgs::msg::WaypointArray waypointsMsg;
          waypointsMsg.header.frame_id = "map";
          waypointsMsg.header.stamp = node->get_clock()->now();
          waypointsMsg.waypoints = curveWaypoints;
          return waypointsMsg;
        }
    }
  } catch (...) {
    // Ignore
  }

  RCLCPP_INFO(logger_, "No curvature line meets the criteria");

  nav2_msgs::msg::WaypointArray waypointsMsg;
  waypointsMsg.header.frame_id = "map";
  waypointsMsg.header.stamp = node->get_clock()->now();
  waypointsMsg.waypoints = waypoints;

  return waypointsMsg;
}


}  // namespace nav2_bt_navigator
