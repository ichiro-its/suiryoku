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

#ifndef SUIRYOKU__LOCOMOTION__CONTROL__NODE__CONTROL_NODE_HPP_
#define SUIRYOKU__LOCOMOTION__CONTROL__NODE__CONTROL_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "suiryoku_interfaces/msg/run_locomotion.hpp"
#include "suiryoku/locomotion/model/robot.hpp"
#include "suiryoku/locomotion/process/locomotion.hpp"

namespace suiryoku::control
{

class ControlNode
{
public:
  using RunLocomotion = suiryoku_interfaces::msg::RunLocomotion;
  using Bool = std_msgs::msg::Bool;

  static std::string get_node_prefix();
  static std::string run_locomotion_topic();
  static std::string status_topic();

  explicit ControlNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<suiryoku::Locomotion> locomotion);

  void update();

private:
  void run_locomotion_callback(const RunLocomotion::SharedPtr message);

  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<RunLocomotion>::SharedPtr run_locomotion_subscriber;
  rclcpp::Publisher<Bool>::SharedPtr status_publisher;

  std::shared_ptr<Locomotion> locomotion;

  std::function<bool()> process;
};

}  // namespace suiryoku::control

#endif  // SUIRYOKU__LOCOMOTION__CONTROL__NODE__CONTROL_NODE_HPP_
