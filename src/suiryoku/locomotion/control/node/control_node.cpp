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

#include "suiryoku/locomotion/control/node/control_node.hpp"

#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/control/helper/command.hpp"
#include "suiryoku/locomotion/locomotion.hpp"

using keisan::literals::operator""_deg;
using std::placeholders::_1;

namespace suiryoku::control
{

std::string ControlNode::get_node_prefix()
{
  return suiryoku::LocomotionNode::get_node_prefix() + "/control";
}

std::string ControlNode::run_locomotion_topic()
{
  return get_node_prefix() + "/run_locomotion";
}

std::string ControlNode::status_topic()
{
  return get_node_prefix() + "/status";
}

ControlNode::ControlNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<suiryoku::Locomotion> locomotion)
: node(node), locomotion(locomotion), process([]() {return false;})
{
  run_locomotion_subscriber = node->create_subscription<RunLocomotion>(
    run_locomotion_topic(), 10,
    std::bind(&ControlNode::run_locomotion_callback, this, _1));

  status_publisher = node->create_publisher<Bool>(
    status_topic(), 10);
}

void ControlNode::run_locomotion_callback(const RunLocomotion::SharedPtr message)
{
  auto parameters = nlohmann::json::parse(message->parameters);

  switch (message->command) {
    case Command::WALK_IN_POSITION:
      {
        bool until_stop = false;

        for (auto &[key, val] : parameters.items()) {
          if (key == "until_stop") {
            until_stop = val.get<bool>();
          }
        }

        if (until_stop) {
          process = [this]() {
              return this->locomotion->walk_in_position_until_stop();
            };
        } else {
          process = [this]() {
              return this->locomotion->walk_in_position();
            };
        }

        break;
      }

    case Command::BACKWARD:
      {
        for (auto &[key, val] : parameters.items()) {
          if (key == "direction") {
            auto direction = keisan::make_degree(val.get<double>());

            process = [this, direction]() {
                this->locomotion->move_backward(direction);

                return false;
              };

            break;
          } else if (key == "target") {
            keisan::Point2 target(
              val["x"].get<double>(), val["y"].get<double>());

            process = [this, target]() {
                return this->locomotion->move_backward_to(target);
              };

            break;
          }
        }

        break;
      }

    case Command::FORWARD:
      {
        for (auto &[key, val] : parameters.items()) {
          if (key == "direction") {
            auto direction = keisan::make_degree(val.get<double>());

            process = [this, direction]() {
                this->locomotion->move_forward(direction);

                return false;
              };

            break;
          } else if (key == "target") {
            keisan::Point2 target(
              val["x"].get<double>(), val["y"].get<double>());

            process = [this, target]() {
                return this->locomotion->move_forward_to(target);
              };
          }
        }

        break;
      }

    case Command::ROTATE:
      {
        keisan::Angle<double> direction(0_deg);
        bool a_move_only = false;

        for (auto &[key, val] : parameters.items()) {
          if (key == "direction") {
            direction = keisan::make_degree(val.get<double>());
          } else if (key == "a_move_only") {
            a_move_only = val.get<bool>();
          }
        }

        process = [this, direction, a_move_only]() {
            return this->locomotion->rotate_to(direction, a_move_only);
          };

        break;
      }

    case Command::FOLLOW_HEAD:
      {
        keisan::Angle<double> min_tilt = 0_deg;
        bool is_default = true;

        for (auto &[key, val] : parameters.items()) {
          if (key == "min_tilt") {
            min_tilt = keisan::make_degree(val.get<double>());
            is_default = false;
          }
        }

        if (is_default) {
          process = [this]() {
              return this->locomotion->move_follow_head();
            };
        } else {
          process = [this, min_tilt]() {
              return this->locomotion->move_follow_head(min_tilt);
            };
        }

        break;
      }

    case Command::SKEW:
      {
        for (auto &[key, val] : parameters.items()) {
          if (key == "direction") {
            auto direction = keisan::make_degree(val.get<double>());

            process = [this, direction]() {
                return !this->locomotion->move_skew(direction);
              };
          }
        }

        break;
      }

    case Command::DRIBBLE:
      {
        for (auto &[key, val] : parameters.items()) {
          if (key == "direction") {
            auto direction = keisan::make_degree(val.get<double>());

            process = [this, direction]() {
                return !this->locomotion->dribble(direction);
              };
          }
        }

        break;
      }

    case Command::PIVOT:
      {
        for (auto &[key, val] : parameters.items()) {
          if (key == "direction") {
            auto direction = keisan::make_degree(val.get<double>());

            process = [this, direction]() {
                return this->locomotion->pivot(direction);
              };
          }
        }

        break;
      }

    case Command::POSITION:
      {
        keisan::Angle<double> direction(0_deg);
        keisan::Angle<double> target_pan = 0_deg;
        keisan::Angle<double> target_tilt = 0_deg;
        bool is_left_kick = false;
        bool is_right_kick = false;

        for (auto &[key, val] : parameters.items()) {
          if (key == "direction") {
            direction = keisan::make_degree(val.get<double>());
          } else if (key == "target") {
            target_pan = keisan::make_degree(val["pan"].get<double>());
            target_tilt = keisan::make_degree(val["tilt"].get<double>());
          } else if (key == "is_left_kick") {
            is_left_kick = val.get<bool>();
          } else if (key == "is_right_kick") {
            is_right_kick = val.get<bool>();
          }
        }

        if (is_left_kick && is_right_kick) {
          process = [this, direction]() {
              return this->locomotion->position_kick_general(direction);
            };
        } else if (is_left_kick) {
          process = [this, direction]() {
              return this->locomotion->position_left_kick(direction);
            };
        } else if (is_right_kick) {
          process = [this, direction]() {
              return this->locomotion->position_right_kick(direction);
            };
        } else {
          process = [this, target_pan, target_tilt, direction]() {
              return this->locomotion->position_until(target_pan, target_tilt, direction);
            };
        }

        break;
      }
  }
}

void ControlNode::update()
{
  bool is_over = process();
  if (is_over) {
    process = []() {return false;};
  }

  auto status_msg = Bool();
  status_msg.data = is_over;

  status_publisher->publish(status_msg);
}

}  // namespace suiryoku::control
