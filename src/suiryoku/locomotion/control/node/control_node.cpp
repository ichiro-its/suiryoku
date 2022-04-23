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

#ifndef SUIRYOKU__LOCOMOTION__CONTROL__NODE__LOCOMOTION_NODE_HPP_
#define SUIRYOKU__LOCOMOTION__CONTROL__NODE__LOCOMOTION_NODE_HPP_

#include <memory>
#include <string>

#include "suiryoku/locomotion/control/node/control_node.hpp"

#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/control/helper/command.hpp"
#include "suiryoku/locomotion/process/locomotion.hpp"

namespace suiryoku::control
{

ControlNode::ControlNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<suiryoku::Locomotion> locomotion)
: node(node), locomotion(locomotion), process([]() {return false;})
{
  run_locomotion_subscriber = node->create_subscription<RunLocomotion>(
    get_node_prefix() + "/run_locomotion", 10,
    [this](const RunLocomotion::SharedPtr message) {
      auto parameters = nlohmann::json::parse(message->parameters);

      switch (message->command)
      {
        case Command::WALK_IN_POSITION:
          {
            bool until_stop = false;

            for (auto &[key, val] : parameters.items()) {
              if (key == "until_stop") {
                until_stop = val.get<bool>();
              }
            }

            process = [this, until_stop]() {
              if (until_stop) {
                return this->locomotion->walk_in_position_until_stop();
              } else {
                return this->locomotion->walk_in_position();
              }
            };

            break;
          }

        case Command::BACKWARD:
          {
            for (auto &[key, val] : parameters.items()) {
              if (key == "direction") {
                auto direction = keisan::make_degree(val);

                process = [this, direction]() {
                  this->locomotion->move_backward(direction);

                  return false;
                };

                break;
              } else if (key == "target") {
                auto target_x = val["x"].get<double>();
                auto target_y = val["y"].get<double>();

                process = [this, target_x, target_y]() {
                  this->locomotion->move_backward_to(target_x, target_y);

                  return false;
                };

                break;
              }
            }

            break;
          }
      }
    });
}

void ControlNode::update()
{
  if (process()) {
    process = []() {return false;};
  }
}

std::string ControlNode::get_node_prefix() const
{
  return "locomotion/control";
}

}  // namespace suiryoku::control

#endif  // SUIRYOKU__LOCOMOTION__CONTROL__NODE__LOCOMOTION_NODE_HPP_
