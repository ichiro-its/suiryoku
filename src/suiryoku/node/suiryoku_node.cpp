// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <chrono>
#include <memory>
#include <string>

#include "suiryoku/node/suiryoku_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "suiryoku/config/node/config_node.hpp"
#include "suiryoku/locomotion/control/node/control_node.hpp"
#include "suiryoku/locomotion/node/locomotion_node.hpp"
#include "suiryoku/locomotion/process/locomotion.hpp"

using namespace std::chrono_literals;

namespace suiryoku
{

SuiryokuNode::SuiryokuNode(rclcpp::Node::SharedPtr node)
: node(node), config_node(nullptr), locomotion_node(nullptr),
  locomotion_control_node(nullptr)
{
  node_timer = node->create_wall_timer(
    8ms,
    [this]() {
      if (this->locomotion_control_node) {
        this->locomotion_control_node->update();
      }
    }
  );
}

void SuiryokuNode::run_locomotion_service(
  std::shared_ptr<Locomotion> locomotion, bool include_control)
{
  locomotion_node = std::make_shared<LocomotionNode>(node, locomotion);

  if (include_control) {
    locomotion_control_node = std::make_shared<control::ControlNode>(
      node, locomotion);
  }
}

void SuiryokuNode::run_config_service(const std::string & path)
{
  config_node = std::make_shared<ConfigNode>(node, path);
}

}  // namespace suiryoku
