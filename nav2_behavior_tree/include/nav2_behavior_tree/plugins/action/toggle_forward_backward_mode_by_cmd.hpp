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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TOGGLE_FORWARD_BACKWARD_MODE_BY_CMD_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TOGGLE_FORWARD_BACKWARD_MODE_BY_CMD_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/msg/waypoint.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_behavior_tree
{

class ToggleForwardBackwardModeByCmd : public BT::ActionNodeBase
{
public:
  ToggleForwardBackwardModeByCmd(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Twist>("cmd_vel", "command velocity"),
      BT::InputPort<std::string>("base_link_frame", "Base link frame"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  std::string input_state_, output_state_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_robot_frame_pub_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TOGGLE_FORWARD_BACKWARD_MODE_HPP_
