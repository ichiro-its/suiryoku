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

#include "aruku_interfaces/msg/point2.hpp"
#include "aruku_interfaces/msg/set_walking.hpp"
#include "aruku_interfaces/msg/status.hpp"
#include "atama_interfaces/msg/head.hpp"
#include "kansei_interfaces/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/model/robot.hpp"
#include "suiryoku/locomotion/process/locomotion.hpp"

namespace suiryoku
{

class LocomotionNode
{
public:
  using Head = atama_interfaces::msg::Head;
  using MeasurementStatus = kansei_interfaces::msg::Status;
  using Point2 = aruku_interfaces::msg::Point2;
  using SetWalking = aruku_interfaces::msg::SetWalking;
  using WalkingStatus = aruku_interfaces::msg::Status;

  static std::string get_node_prefix();

  explicit LocomotionNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<Locomotion> locomotion);

  void update();

  bool set_odometry;

private:
  void publish_walking();
  void publish_odometry();

  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<SetWalking>::SharedPtr set_walking_publisher;

  rclcpp::Publisher<Point2>::SharedPtr set_odometry_publisher;
  rclcpp::Subscription<MeasurementStatus>::SharedPtr
    measurement_status_subscriber;
  rclcpp::Subscription<WalkingStatus>::SharedPtr walking_status_subscriber;

  rclcpp::Subscription<Head>::SharedPtr head_subscriber;

  std::shared_ptr<Locomotion> locomotion;
  std::shared_ptr<Robot> robot;

  // teporary for start/stop the walking
  bool walking_state;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION__NODE__LOCOMOTION_NODE_HPP_
