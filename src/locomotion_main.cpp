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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/model/robot.hpp"
#include "suiryoku/locomotion/node/locomotion_node.hpp"
#include "suiryoku/locomotion/process/locomotion.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  char host_name[64];
  gethostname(host_name, 64);
  std::string path = "/home/ichiro/ros2-ws/configuration/";
  path += host_name;
  path += "/locomotion/";
  auto node = std::make_shared<rclcpp::Node>("suiryoku_node");

  auto robot = std::make_shared<suiryoku::Robot>();
  auto locomotion = std::make_shared<suiryoku::Locomotion>(robot);
  locomotion->load_config(path);

  suiryoku::LocomotionNode locomotion_node(node, locomotion);

  rclcpp::Rate rcl_rate(8ms);
  while (rclcpp::ok()) {
    rcl_rate.sleep();

    rclcpp::spin_some(node);

    locomotion->walk_in_position();
    locomotion_node.update();
  }

  rclcpp::shutdown();

  return 0;
}
