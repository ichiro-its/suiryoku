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
: locomotion(locomotion), robot(locomotion->get_robot())
{
  set_walking_publisher = node->create_publisher<SetWalking>(
    aruku::WalkingNode::set_walking_topic(), 10);

  orientation_subscriber = node->create_subscription<Axis>(
    "/measurement/orientation", 10,
    [this](const Axis::SharedPtr message) {
      this->robot->orientation = keisan::make_degree(message->yaw);
    });

  odometry_subscriber = node->create_subscription<Odometry>(
    aruku::WalkingNode::odometry_topic(), 10,
    [this](const Odometry::SharedPtr message) {
      this->robot->position.x = message->position_x;
      this->robot->position.y = message->position_y;
    });

  walking_status_subscriber = node->create_subscription<Status>(
    aruku::WalkingNode::status_topic(), 10,
    [this](const Status::SharedPtr message) {
      this->robot->is_walking = message->is_running;
      this->locomotion->update_move_amplitude(
        message->x_amplitude, message->y_amplitude);
    });

  head_subscriber = node->create_subscription<Head>(
    "/head/set_head_data", 10,
    [this](const Head::SharedPtr message) {
      this->robot->pan = message->pan_angle;
      this->robot->tilt = message->tilt_angle;
    });

  locomotion->set_stop_walking_callback(
    [this]() {
      auto walking_msg = SetWalking();
      walking_msg.run = false;

      this->set_walking_publisher->publish(walking_msg);
    });
}

void LocomotionNode::update()
{
  if (robot->is_walking) {
    publish_walking();
  }
}

void LocomotionNode::publish_walking()
{
  auto walking_msg = SetWalking();

  walking_msg.run = robot->is_walking;
  walking_msg.x_move = robot->x_speed;
  walking_msg.y_move = robot->y_speed;
  walking_msg.a_move = robot->a_speed;
  walking_msg.aim_on = robot->aim_on;

  set_walking_publisher->publish(walking_msg);
}

}  // namespace suiryoku
