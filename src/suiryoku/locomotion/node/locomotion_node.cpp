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

#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/process/locomotion.hpp"

namespace suiryoku
{

LocomotionNode::LocomotionNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<Locomotion> locomotion)
: locomotion(locomotion)
{
  set_walking_publisher = node->create_publisher<SetWalking>(
    get_node_prefix() + "/set_walking", 10);

  orientation_subscriber = node->create_subscription<Axis>(
    "/measurement/orientation", 10,
    [this](const Axis::SharedPtr message) {
      this->locomotion_manager->update_orientation(
        keisan::make_degree(message->yaw));
    });

  odometry_publisher = node->create_publisher<Odometry>(
    get_node_prefix() + "/odometry", 10);
}

void LocomotionNode::update()
{
  publish_joints();
  publish_odometry();
}

std::string LocomotionNode::get_node_prefix() const
{
  return "locomotion";
}

void LocomotionNode::publish_joints()
{
  auto joints_msg = SetJoints();
  joints_msg.control_type = tachimawari::joint::Middleware::FOR_WALKING;

  const auto & joints = locomotion_manager->get_joints();
  auto & joint_msgs = joints_msg.joints;

  joint_msgs.resize(joints.size());
  for (size_t i = 0; i < joints.size() && i < joint_msgs.size(); ++i) {
    joint_msgs[i].id = joints[i].get_id();
    joint_msgs[i].position = joints[i].get_position();
  }

  set_joints_publisher->publish(joints_msg);
}

void LocomotionNode::publish_odometry()
{
  auto odometry_msg = Odometry();

  odometry_msg.position_x = locomotion_manager->get_position().x;
  odometry_msg.position_y = locomotion_manager->get_position().y;

  odometry_publisher->publish(odometry_msg);
}

}  // namespace suiryoku
