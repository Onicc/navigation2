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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_PATH_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_PATH_HPP_

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_bt_navigator/navigator.hpp"
#include "nav2_msgs/action/navigate_to_path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/srv/set_string.hpp"
#include "nav2_msgs/srv/set_waypoints.hpp"
#include "nav2_msgs/msg/waypoint.hpp"
#include "nav2_msgs/msg/waypoint_array.hpp"
#include "nav2_msgs/msg/costmap.hpp"

namespace nav2_bt_navigator
{

/**
 * @class NavigateToPathNavigator
 * @brief A navigator for navigating to a specified path
 */
class NavigateToPathNavigator
  : public nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateToPath>
{
public:
  using ActionT = nav2_msgs::action::NavigateToPath;

  /**
   * @brief A constructor for NavigateToPathNavigator
   */
  NavigateToPathNavigator()
  : Navigator() {}

  /**
   * @brief A configure state transition to configure navigator's state
   * @param node Weakptr to the lifecycle node
   * @param odom_smoother Object to get current smoothed robot's speed
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override;

  /**
   * @brief A subscription and callback to handle the topic-based goal published
   * from rviz
   * @param path Path received via atopic
   */
  void onGoalPathReceived(const nav_msgs::msg::Path::SharedPtr path);
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  void onCommandReceived(const std_msgs::msg::String::SharedPtr command);
  void onBTNavigatorStartReceived(const std_msgs::msg::String::SharedPtr msg);
  void onOdometryGPSReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onCurbTractionPointReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onWaypointsReceived(const nav2_msgs::msg::WaypointArray::SharedPtr msg);
  void onGlobalCostmapReceived(const nav2_msgs::msg::Costmap::SharedPtr msg);

  // ros service
  // void onBTCommandReceived(
  //     const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
  //     std::shared_ptr<nav2_msgs::srv::SetString::Response> response);
  // void onBTStartReceived(
  //     const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
  //     std::shared_ptr<nav2_msgs::srv::SetString::Response> response);
  void onWaypointsReceivedSrv(
    const std::shared_ptr<nav2_msgs::srv::SetWaypoints::Request> request, 
    std::shared_ptr<nav2_msgs::srv::SetWaypoints::Response> response);
  void onLoadWaypointsSrv(
    const std::shared_ptr<nav2_msgs::srv::SetString::Request> request, 
    std::shared_ptr<nav2_msgs::srv::SetString::Response> response);

  /**
   * @brief Get action name for this navigator
   * @return string Name of action server
   */
  std::string getName() {return std::string("navigate_to_path");}

  /**
   * @brief Get navigator's default BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to default XML
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   * @param goal Action template's goal message
   * @return bool if goal was received successfully to be processed
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * action result message or indicate that this action is done.
   * @param result Action template result message to populate
   * @param final_bt_status Resulting status of the behavior tree execution that may be
   * referenced while populating the result.
   */
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief Goal path initialization on the blackboard
   * @param goal Action template's goal message to process
   */
  void initializeGoalPath(ActionT::Goal::ConstSharedPtr goal);

  nav2_msgs::msg::WaypointArray loadWaypoints(const std::string& waypointsFile);

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bt_navigator_start_sub_;
  rclcpp::Subscription<nav2_msgs::msg::WaypointArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr curb_traction_point_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_sub_;

  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr bt_command_service_;
  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr bt_start_service_;
  rclcpp::Service<nav2_msgs::srv::SetWaypoints>::SharedPtr waypoints_service_;
  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr load_waypoints_service_;

  std::string goals_blackboard_id_;
  std::string path_blackboard_id_;
  std::string path_local_blackboard_id_;
  std::string navigation_state_blackboard_id_;
  std::string waypoint_index_blackboard_id_;
  std::string goal_pose_blackboard_id_;
  std::string replanning_count_blackboard_id_;
  std::string farthest_obstacle_point_blackboard_id_;
  std::string manual_goal_pose_blackboard_id_;
  std::string command_blackboard_id_;
  std::string start_blackboard_id_;
  std::string waypoints_blackboard_id_;
  std::string waypoint_blackboard_id_;
  std::string odometry_gps_blackboard_id_;
  std::string curb_traction_point_blackboard_id_;
  std::string curb_traction_point_extension_blackboard_id_;
  std::string curb_path_extension_blackboard_id_;
  std::string global_costmap_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  std::chrono::high_resolution_clock::time_point last_loop_time_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_PATH_HPP_
