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

#ifndef SUIRYOKU__NODE__SUIRYOKU_NODE_HPP_
#define SUIRYOKU__NODE__SUIRYOKU_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "suiryoku/config/node/config_node.hpp"
#include "suiryoku/locomotion/control/node/control_node.hpp"
#include "suiryoku/locomotion/node/locomotion_node.hpp"
#include "suiryoku/locomotion/process/locomotion.hpp"

namespace suiryoku
{

class SuiryokuNode
{
public:
  explicit SuiryokuNode(rclcpp::Node::SharedPtr node);

  void run_locomotion_service(
    std::shared_ptr<Locomotion> locomotion, bool include_control = false);

  void run_config_service(const std::string & path);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr node_timer;

  std::shared_ptr<ConfigNode> config_node;

  std::shared_ptr<LocomotionNode> locomotion_node;
  std::shared_ptr<control::ControlNode> locomotion_control_node;

  std::shared_ptr<Locomotion> locomotion;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__NODE__SUIRYOKU_NODE_HPP_
