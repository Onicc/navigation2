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

#include "nav2_behavior_tree/plugins/action/automatic_light.hpp"

namespace nav2_behavior_tree
{

AutomaticLight::AutomaticLight(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  turn_signal_pub_ = node_->create_publisher<std_msgs::msg::String>("vehicle/command/turn_signal", qos);
  beam_pub_ = node_->create_publisher<std_msgs::msg::String>("vehicle/command/beam", qos);
  voice_text_pub_ = node_->create_publisher<std_msgs::msg::String>("/voice", qos);
}

inline BT::NodeStatus AutomaticLight::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  static std::string last_navigation_state;

  nav2_msgs::msg::Waypoint waypoint;
  std::string navigation_state;
  getInput("waypoint", waypoint);
  getInput("navigation_state", navigation_state);

  std_msgs::msg::String turn_signal_msg;
  std_msgs::msg::String beam_msg;
  std_msgs::msg::String voice_msg;

  //   auto beam_msg = std_msgs::msg::String();
  //   beam_msg.data = "OFF_HAZARD_BEAM";
  //   beam_pub_->publish(beam_msg);
  //   beam_msg.data = "OFF_EMERGENCY_BEAM";
  //   beam_pub_->publish(beam_msg);

  if(navigation_state == "stop") {
    beam_msg.data = "OFF_ALL";
    beam_pub_->publish(beam_msg);
    turn_signal_msg.data = "off";
    turn_signal_pub_->publish(turn_signal_msg);
    if(last_navigation_state != navigation_state) {
      voice_msg.data = "停止自动运行";
      voice_text_pub_->publish(voice_msg);
    }
  }

  if(navigation_state == "manual") {
    beam_msg.data = "HAZARD_BEAM";
    beam_pub_->publish(beam_msg);
    beam_msg.data = "EMERGENCY_BEAM";
    beam_pub_->publish(beam_msg);
    turn_signal_msg.data = "off";
    turn_signal_pub_->publish(turn_signal_msg);
    if(last_navigation_state != navigation_state) {
      voice_msg.data = "进入远程人工模式";
      voice_text_pub_->publish(voice_msg);
    }
  }

  if(navigation_state == "path_following") {
    beam_msg.data = "EMERGENCY_BEAM";
    beam_pub_->publish(beam_msg);
    if(waypoint.option_turn_signal < 0) {
      turn_signal_msg.data = "right";
      turn_signal_pub_->publish(turn_signal_msg);
      beam_msg.data = "OFF_HAZARD_BEAM";
      beam_pub_->publish(beam_msg);
      voice_msg.data = "右转弯请注意";
      voice_text_pub_->publish(voice_msg);
    } else if(waypoint.option_turn_signal > 0) {
      turn_signal_msg.data = "left";
      turn_signal_pub_->publish(turn_signal_msg);
      beam_msg.data = "OFF_HAZARD_BEAM";
      beam_pub_->publish(beam_msg);
      voice_msg.data = "左转弯请注意";
      voice_text_pub_->publish(voice_msg);
    } else {
      turn_signal_msg.data = "off";
      turn_signal_pub_->publish(turn_signal_msg);
      beam_msg.data = "HAZARD_BEAM";
      beam_pub_->publish(beam_msg);
    }
  }

  last_navigation_state = navigation_state;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::AutomaticLight>("AutomaticLight");
}
