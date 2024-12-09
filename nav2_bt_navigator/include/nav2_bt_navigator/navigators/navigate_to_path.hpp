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
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_bt_navigator/navigator.hpp"
#include "nav2_msgs/action/navigate_to_path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav2_msgs/srv/set_string.hpp"
#include "nav2_msgs/srv/set_waypoints.hpp"
#include "nav2_msgs/msg/waypoint.hpp"
#include "nav2_msgs/msg/waypoint_array.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/exception.hpp"

namespace nav2_bt_navigator
{

class BezierCurve {
public:
    std::tuple<std::array<double, 2>, std::array<double, 2>, double> cubicBezierCurves(
      std::array<double, 2> p0, std::array<double, 2> p1,
      std::array<double, 2> p2, std::array<double, 2> p3, double t) {
      
      // std::array<double, 2> P, P_prime;
      // double curvature;

      // P[0] = std::pow(1 - t, 3) * p0[0] + 3 * std::pow(1 - t, 2) * t * p1[0] +
      //         3 * (1 - t) * std::pow(t, 2) * p2[0] + std::pow(t, 3) * p3[0];
      // P[1] = std::pow(1 - t, 3) * p0[1] + 3 * std::pow(1 - t, 2) * t * p1[1] +
      //         3 * (1 - t) * std::pow(t, 2) * p2[1] + std::pow(t, 3) * p3[1];

      // P_prime[0] = 3 * std::pow(1 - t, 2) * (p1[0] - p0[0]) +
      //               6 * t * (1 - t) * (p2[0] - p1[0]) +
      //               3 * std::pow(t, 2) * (p3[0] - p2[0]);
      // P_prime[1] = 3 * std::pow(1 - t, 2) * (p1[1] - p0[1]) +
      //               6 * t * (1 - t) * (p2[1] - p1[1]) +
      //               3 * std::pow(t, 2) * (p3[1] - p2[1]);

      // double P_prime_norm = std::sqrt(std::pow(P_prime[0], 2) + std::pow(P_prime[1], 2));

      // curvature = std::sqrt(std::pow((P_prime[0] * (3 * (1 - t) * (p2[1] - 2 * p1[1] + p0[1]) +
      //                                             3 * t * (p3[1] - 2 * p2[1] + p1[1])) -
      //                                 P_prime[1] * (3 * (1 - t) * (p2[0] - 2 * p1[0] + p0[0]) +
      //                                               3 * t * (p3[0] - 2 * p2[0] + p1[0]))),
      //                         2) /
      //                       std::pow(P_prime_norm, 3));

      // return std::make_tuple(P, P_prime, curvature);

      // P(t) = (1-t)^3 * P0 + 3t(1-t)^2 * P1 + 3t^2(1-t) * P2 + t^3 * P3
      std::array<double, 2> P = {std::pow(1 - t, 3) * p0[0] + 3 * std::pow(1 - t, 2) * t * p1[0] + 3 * (1 - t) * std::pow(t, 2) * p2[0] + std::pow(t, 3) * p3[0],
                                  std::pow(1 - t, 3) * p0[1] + 3 * std::pow(1 - t, 2) * t * p1[1] + 3 * (1 - t) * std::pow(t, 2) * p2[1] + std::pow(t, 3) * p3[1]};
      
      // P'(t) = 3(1-t)^2 * (P1 - P0) + 6t(1-t) * (P2 - P1) + 3t^2 * (P3 - P2)
      std::array<double, 2> P_prime = {3 * std::pow(1 - t, 2) * (p1[0] - p0[0]) + 6 * t * (1 - t) * (p2[0] - p1[0]) + 3 * std::pow(t, 2) * (p3[0] - p2[0]),
                                        3 * std::pow(1 - t, 2) * (p1[1] - p0[1]) + 6 * t * (1 - t) * (p2[1] - p1[1]) + 3 * std::pow(t, 2) * (p3[1] - p2[1])};
      
      // P''(t) = 6(1-t) * (P2 - 2 * P1 + P0) + 6t * (P3 - 2 * P2 + P1)
      std::array<double, 2> P_double_prime = {6 * (1 - t) * (p2[0] - 2 * p1[0] + p0[0]) + 6 * t * (p3[0] - 2 * p2[0] + p1[0]),
                                              6 * (1 - t) * (p2[1] - 2 * p1[1] + p0[1]) + 6 * t * (p3[1] - 2 * p2[1] + p1[1])};
      
      // P'(t) x P''(t)
      double cross_product = P_prime[0] * P_double_prime[1] - P_prime[1] * P_double_prime[0];
      
      // |P'(t)|^3
      double norm_prime_cubed = std::pow(std::sqrt(std::pow(P_prime[0], 2) + std::pow(P_prime[1], 2)), 3);
      
      // K(t) = |P'(t) x P''(t)| / |P'(t)|^3
      double curvature = std::abs(cross_product) / norm_prime_cubed;

      return std::make_tuple(P, P_prime, curvature);
    }

    std::tuple<bool, std::vector<std::array<double, 2>>, std::vector<double>> bezierCurve(
        std::array<double, 2> p0, std::array<double, 2> p3, double heading0,
        double heading3, double max_curvature, int num_points = 100) {
        
        std::vector<double> t(num_points);
        for (int i = 0; i < num_points; ++i)
            t[i] = static_cast<double>(i) / (num_points - 1);

        std::array<double, 2> direction1 = {std::cos(heading0), std::sin(heading0)};
        std::array<double, 2> direction2 = {std::cos(heading3), std::sin(heading3)};

        std::array<double, 2> finally_p0_p1, finally_p2_p3;
        finally_p0_p1 = {0.0, 0.0};
        finally_p2_p3 = {0.0, 0.0};
        double finally_J = 0.0;

        int max_mn = static_cast<int>(std::sqrt(std::pow(p3[0] - p0[0], 2) + std::pow(p3[1] - p0[1], 2)));

        for (int j = 1; j < max_mn; ++j) {
          for (int k = 1; k < max_mn; ++k) {
            std::vector<std::array<double, 2>> curve_points;
            std::vector<double> curvatures;
            std::array<double, 2> p1, p2;
            p1[0] = p0[0] + j * direction1[0];
            p1[1] = p0[1] + j * direction1[1];
            p2[0] = p3[0] - k * direction2[0];
            p2[1] = p3[1] - k * direction2[1];
            double total_length = 0.0;
            std::array<double, 2> p_i_1;
            for (int i = 0; i < num_points; ++i) {
              std::array<double, 2> p_i, p_prime_i;
              double curvature;
              std::tie(p_i, p_prime_i, curvature) = cubicBezierCurves(p0, p1, p2, p3, t[i]);
              if (i > 0) {
                double segment_length = std::sqrt(std::pow(p_i[0] - p_i_1[0], 2) + std::pow(p_i[1] - p_i_1[1], 2));
                total_length += segment_length;
              }
              p_i_1 = p_i;
              curve_points.push_back(p_i);
              curvatures.push_back(curvature);
            }
            if (*std::max_element(curvatures.begin(), curvatures.end()) < max_curvature) {
                double J_ = (1 / total_length) * (1 / (*std::max_element(curvatures.begin(), curvatures.end())));
                if (J_ >= finally_J) {
                    finally_J = J_;
                    finally_p0_p1 = {p0[0] + j * direction1[0], p0[1] + j * direction1[1]};
                    finally_p2_p3 = {p3[0] - k * direction2[0], p3[1] - k * direction2[1]};
                }
            }
          }
        }

        if (std::fabs(finally_p0_p1[0]) < 0.000001 && std::fabs(finally_p0_p1[1]) < 0.000001)
            return std::make_tuple(false, std::vector<std::array<double, 2>>(), std::vector<double>());

        std::vector<std::array<double, 2>> curve_points;
        std::vector<double> orientations;
        std::vector<double> curvatures;
        for (int i = 0; i < num_points; ++i) {
            std::array<double, 2> p_i, p_prime_i;
            double curvature;
            std::tie(p_i, p_prime_i, curvature) = cubicBezierCurves(p0, finally_p0_p1, finally_p2_p3, p3, t[i]);
            curve_points.push_back(p_i);
            orientations.push_back(std::fmod(std::atan2(p_prime_i[1], p_prime_i[0]) + 2 * M_PI, 2 * M_PI));
            curvatures.push_back(curvature);
        }

        std::cout << "max curvature: " << *std::max_element(curvatures.begin(), curvatures.end()) << std::endl;

        return std::make_tuple(true, curve_points, orientations);
    }
};

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
  void onOdometryLidarReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onCurbTractionPointReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onWaypointsReceived(const nav2_msgs::msg::WaypointArray::SharedPtr msg);
  void onGlobalCostmapReceived(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void onDetectObstacleDistanceReceived(const std_msgs::msg::Float32::SharedPtr msg);
  void onTrafficLightReceived(const std_msgs::msg::Int32::SharedPtr msg);
  void onRobotFrameReceived1(const std_msgs::msg::String::SharedPtr msg);
  void onCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onTeleopCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onFrontOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ros service
  // void onBTCommandReceived(
  //     const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
  //     std::shared_ptr<nav2_msgs::srv::SetString::Response> response);
  // void onBTStartReceived(
  //     const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
  //     std::shared_ptr<nav2_msgs::srv::SetString::Response> response);
  void onNavigationStateReceived(
      const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
      std::shared_ptr<nav2_msgs::srv::SetString::Response> response);
  void onWaypointsReceivedSrv(
    const std::shared_ptr<nav2_msgs::srv::SetWaypoints::Request> request, 
    std::shared_ptr<nav2_msgs::srv::SetWaypoints::Response> response);
  void onLoadWaypointsSrv(
    const std::shared_ptr<nav2_msgs::srv::SetString::Request> request, 
    std::shared_ptr<nav2_msgs::srv::SetString::Response> response);
  void onStartAutoCleaningSrv(
    const std::shared_ptr<nav2_msgs::srv::SetString::Request> request, 
    std::shared_ptr<nav2_msgs::srv::SetString::Response> response);
  void onObstacleModeReceived(
      const std::shared_ptr<nav2_msgs::srv::SetString::Request> request,
      std::shared_ptr<nav2_msgs::srv::SetString::Response> response);
  void onRobotFrameReceived(
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr curb_traction_point_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr detect_obstacle_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr traffic_light_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_frame_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr front_odometry_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr beam_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav2_msgs::msg::WaypointArray>::SharedPtr optimized_waypoints_pub_;

  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr bt_navigation_state_service_;
  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr bt_command_service_;
  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr bt_start_service_;
  rclcpp::Service<nav2_msgs::srv::SetWaypoints>::SharedPtr waypoints_service_;
  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr load_waypoints_service_;
  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr start_auto_cleaning_service_;
  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr bt_obstacle_mode_service_;
  rclcpp::Service<nav2_msgs::srv::SetString>::SharedPtr bt_robot_frame_service_;

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
  std::string odometry_lidar_blackboard_id_;
  std::string curb_traction_point_blackboard_id_;
  std::string curb_traction_point_extension_blackboard_id_;
  std::string curb_path_extension_blackboard_id_;
  std::string global_costmap_blackboard_id_;
  std::string exception_blackboard_id_;
  std::string goals_truncate_blackboard_id_;
  std::string obstacle_mode_blackboard_id_;
  std::string detect_obstacle_distance_blackboard_id_;
  std::string traffic_light_blackboard_id_;
  std::string robot_frame_blackboard_id_;
  std::string base_link_frame_id_;
  std::string cmd_vel_frame_id_;
  std::string manual_mode_frame_id_;
  std::string front_odometry_blackboard_id_;
  std::string remaining_distance_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  std::chrono::high_resolution_clock::time_point last_loop_time_;

  double max_curvature = 0.2;
  std::string robot_frame_ = "front_base_link";

  std::string waypoints_path_ = "";
  int waypoint_index_blackboard_ = -1;

  nav2_msgs::msg::WaypointArray waypoints_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_PATH_HPP_
