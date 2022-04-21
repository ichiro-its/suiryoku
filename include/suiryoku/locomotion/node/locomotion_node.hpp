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

#ifndef SUIRYOKU__LOCOMOTION__NODE__LOCOMOTION_NODE_HPP_
#define SUIRYOKU__LOCOMOTION__NODE__LOCOMOTION_NODE_HPP_

#include <memory>
#include <string>

#include "aruku_interfaces/msg/set_walking.hpp"
#include "atama_interfaces/msg/head.hpp"
#include "kansei_interfaces/msg/axis.hpp"
#include "rclcpp/rclcpp.hpp"

namespace suiryoku
{

class LocomotionNode
{
public:
  using Axis = kansei_interfaces::msg::Axis;
  using Head = atama_interfaces::msg::Head;
  using SetWalking = aruku_interfaces::msg::SetWalking;

  explicit LocomotionNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<WalkingManager> walking_manager);

  void update();

private:
  std::string get_node_prefix() const;

  void publish_walking();

  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<SetWalking>::SharedPtr set_walking_publisher;

  rclcpp::Subscription<Axis>::SharedPtr orientation_subscriber;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION__NODE__LOCOMOTION_NODE_HPP_
