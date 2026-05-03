// Copyright (c) 2026 ICHIRO ITS
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/planner/node/davg_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string config_path = "";
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "-c" && i + 1 < args.size()) {
      config_path = args[i + 1];
      i++;
    } else if (args[i].front() != '-') {
      config_path = args[i];
    }
  }

  auto node = std::make_shared<suiryoku::DAVGPlannerNode>();
  
  if (!config_path.empty()) {
    node->load_config(config_path);
  } else {
    RCLCPP_WARN(node->get_logger(), "Config path not provided! Using default parameters.");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}