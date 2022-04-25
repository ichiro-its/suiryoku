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
#include "suiryoku/locomotion/process/locomotion.hpp"
#include "suiryoku/node/suiryoku_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Please specify the path!" << std::endl;
    return 0;
  }

  std::string path = argv[1];
  auto node = std::make_shared<rclcpp::Node>("suiryoku_node");
  auto suiryoku_node = std::make_shared<suiryoku::SuiryokuNode>(node);

  auto robot = std::make_shared<suiryoku::Robot>();
  auto locomotion = std::make_shared<suiryoku::Locomotion>(robot);
  locomotion->load_config(path);

  suiryoku_node->run_locomotion_service(locomotion, true);
  suiryoku_node->run_config_service(path);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
