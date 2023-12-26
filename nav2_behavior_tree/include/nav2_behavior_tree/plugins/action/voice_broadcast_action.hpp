// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__VOICE_BROADCAST_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__VOICE_BROADCAST_ACTION_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class VoiceBroadcast : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::VoiceBroadcast constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  VoiceBroadcast(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("text", "欢迎使用星火科技智能清扫车", "voice text"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_text_pub_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
