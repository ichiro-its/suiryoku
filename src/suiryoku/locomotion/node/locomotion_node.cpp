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

#include <memory>
#include <string>

#include "suiryoku/locomotion/node/locomotion_node.hpp"

#include "aruku/walking/walking.hpp"
#include "kansei/measurement/measurement.hpp"
#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/process/locomotion.hpp"

namespace suiryoku
{

std::string LocomotionNode::get_node_prefix()
{
  return "locomotion";
}

LocomotionNode::LocomotionNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<Locomotion> locomotion)
: locomotion(locomotion), robot(locomotion->get_robot()), walking_state(false), set_odometry(false)
{
  set_walking_publisher = node->create_publisher<SetWalking>(
    aruku::WalkingNode::set_walking_topic(), 10);

  measurement_status_subscriber = node->create_subscription<MeasurementStatus>(
    kansei::measurement::MeasurementNode::status_topic(), 10,
    [this](const MeasurementStatus::SharedPtr message) {
      this->robot->is_calibrated = message->is_calibrated;
      this->robot->orientation = keisan::make_degree(message->orientation.yaw);
    });

  set_odometry_publisher = node->create_publisher<Point2>(
    aruku::WalkingNode::set_odometry_topic(), 10);

  walking_status_subscriber = node->create_subscription<WalkingStatus>(
    aruku::WalkingNode::status_topic(), 10,
    [this](const WalkingStatus::SharedPtr message)
    {
      this->robot->is_walking = message->is_running;
      this->robot->x_amplitude = message->x_amplitude;
      this->robot->y_amplitude = message->y_amplitude;
      this->robot->a_amplitude = message->a_amplitude;
      this->robot->position.x = message->odometry.x;
      this->robot->position.y = message->odometry.y;
    });

  head_subscriber = node->create_subscription<Head>(
    "/head/set_head_data", 10,
    [this](const Head::SharedPtr message) {
      this->robot->pan = keisan::make_degree(message->pan_angle);
      this->robot->tilt = keisan::make_degree(message->tilt_angle);
    });

  locomotion->stop = [this]() {this->walking_state = false;};
  locomotion->start = [this]() {this->walking_state = true;};
}

void LocomotionNode::update()
{
  publish_walking();
  if (set_odometry) {
    publish_odometry();
  }
}

void LocomotionNode::publish_walking()
{
  auto walking_msg = SetWalking();

  walking_msg.run = walking_state;
  walking_msg.x_move = robot->x_speed;
  walking_msg.y_move = robot->y_speed;
  walking_msg.a_move = robot->a_speed;
  walking_msg.aim_on = robot->aim_on;

  set_walking_publisher->publish(walking_msg);
}

void LocomotionNode::publish_odometry()
{
  auto odometry_msg = Point2();

  odometry_msg.x = robot->position.x;
  odometry_msg.y = robot->position.y;

  set_odometry_publisher->publish(odometry_msg);
  set_odometry = false;
}

}  // namespace suiryoku
