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

#include "nav2_behavior_tree/plugins/action/find_farthest_obstacle_point.hpp"

namespace nav2_behavior_tree
{

FindFarthestObstaclePoint::FindFarthestObstaclePoint(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus FindFarthestObstaclePoint::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path path;
  int input_index;
  double resolution;
  int replanning_count;
  double find_distance;
  double safe_distance;
  double back_find_distance;

  getInput("input_path", path);
  getInput("input_index", input_index);
  getInput("resolution", resolution);
  getInput("replanning_count", replanning_count);
  getInput("find_distance", find_distance);
  getInput("safe_distance", safe_distance);
  getInput("back_find_distance", back_find_distance);

  int find_distance_index = static_cast<int>(path.poses.size()-1);
  double sum_dis = 0.0;
  for (size_t i = input_index; i < (path.poses.size()-1); i++) {
    double curr_dist = nav2_util::geometry_utils::euclidean_distance(
                path.poses[i], path.poses[i+1]);
    sum_dis += curr_dist;
    if (sum_dis > find_distance) {
      find_distance_index = i;
      break;
    }
  }

  int back_distance_index = 0;
  sum_dis = 0.0;
  for (int i = input_index; i > 0; i--) {
    double curr_dist = nav2_util::geometry_utils::euclidean_distance(
                path.poses[i], path.poses[i-1]);
    sum_dis += curr_dist;
    if (sum_dis > back_find_distance) {
      back_distance_index = i;
      break;
    }
  }

  double backoff_distance = resolution*replanning_count;
  if(backoff_distance >= (find_distance+back_find_distance)) backoff_distance=find_distance+back_find_distance;

  geometry_msgs::msg::PoseStamped farthest_obstacle_point;
  int farthest_obstacle_index = back_distance_index;
  sum_dis = 0.0;
  for (int i = find_distance_index; i > back_distance_index; i--) {
    double curr_dist = nav2_util::geometry_utils::euclidean_distance(
                path.poses[i], path.poses[i-1]);
    sum_dis += curr_dist;
    if (sum_dis > backoff_distance) {
      farthest_obstacle_index = i;
      break;
    }
  }
  if(farthest_obstacle_index == back_distance_index) {
    // 路径上没有障碍物或者障碍物消失
    return BT::NodeStatus::FAILURE;
  }
  farthest_obstacle_point = path.poses[farthest_obstacle_index];

  setOutput("farthest_obstacle_point", farthest_obstacle_point);
  // setOutput("farthest_obstacle_index", farthest_obstacle_index);

  sum_dis = 0;
  geometry_msgs::msg::PoseStamped recovery_point=path.poses.back();
  for (size_t i = farthest_obstacle_index; i < (path.poses.size()-1); i++) {
    double curr_dist = nav2_util::geometry_utils::euclidean_distance(
                path.poses[i], path.poses[i+1]);
    sum_dis += curr_dist;
    if (sum_dis > safe_distance) {
      std::cout << "recovery_point_index:" << i << std::endl;
      recovery_point = path.poses[i];
      break;
    }
  }
  
  setOutput("recovery_point", recovery_point);

  std::cout << "recovery_point:" << recovery_point.pose.position.x << "," << recovery_point.pose.position.y << std::endl;
  std::cout << "distance:" << nav2_util::geometry_utils::euclidean_distance(
                path.poses[input_index], recovery_point) << std::endl;
  std::cout << "input_index:" << input_index << std::endl;
  std::cout << "back_distance_index:" << back_distance_index << std::endl;
  std::cout << "find_distance_index:" << find_distance_index << std::endl;
  std::cout << "farthest_obstacle_index:" << farthest_obstacle_index << std::endl;
  std::cout << "backoff_distance:" << backoff_distance << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::FindFarthestObstaclePoint>("FindFarthestObstaclePoint");
}
