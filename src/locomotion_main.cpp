// Copyright (c) 2021 ICHIRO ITS
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

#include <aruku/walking.hpp>
#include <atama/head.hpp>
#include <kansei/imu.hpp>
#include <suiryoku/locomotion.hpp>
#include <robocup_client/robocup_client.hpp>

#include <nlohmann/json.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <thread>

int main(int argc, char * argv[])
{
  if (argc < 4) {
    std::cerr << "Please specify the host, the port, and the path!" << std::endl;
    return 0;
  }

  std::string host = argv[1];
  int port = std::stoi(argv[2]);
  std::string path = argv[3];

  robocup_client::RobotClient client(host, port);
  if (!client.connect()) {
    std::cerr << "Failed to connect to server on port " <<
      client.get_port() << "!" << std::endl;

    return 1;
  }

  robocup_client::MessageHandler message;
  message.add_sensor_time_step("gyro", 8);
  message.add_sensor_time_step("accelerometer", 8);
  client.send(*message.get_actuator_request());

  auto imu = std::make_shared<kansei::Imu>();
  auto walking = std::make_shared<aruku::Walking>(imu);

  walking->initialize();
  walking->load_data(path + "walking/");
  walking->start();
  std::cout << "start now: x " << walking->POSITION_X << " y " << walking->POSITION_Y << std::endl; 

  auto head = std::make_shared<atama::Head>(walking, imu);
  head->initialize();
  head->load_data(path + "head/");
  
  auto locomotion = std::make_shared<suiryoku::Locomotion>(walking, head, imu);

  std::string cmds[3] = {};

  bool is_running = false;
  bool is_running_now = false;
  std::string current_mode = "";
  float target_x, target_y, target_direction;

  std::thread input_handler([&cmds, &is_running, &is_running_now] {
    while (true) {
      if (!is_running && !is_running_now) {
        std::cout << "> run : ";
        std::cin >> cmds[0];

        if (cmds[0].find("move_to_target") != std::string::npos || cmds[0].find("set_position") != std::string::npos ) {
          std::cout << "  target x : ";
          std::cin >> cmds[1];
          std::cout << "  target y : ";
          std::cin >> cmds[2];
        } else {
          std::cout << "  direction : ";
          std::cin >> cmds[1];
          cmds[2] = "empty";
        }

        is_running = true;
      }
    }
  });

  while (client.get_tcp_socket()->is_connected()) {
    try {
      message.clear_actuator_request();
      auto sensors = client.receive();

      float gy[3];
      if (sensors.get()->gyros_size() > 0) {
        auto gyro = sensors.get()->gyros(0);
        gy[0] = gyro.value().x();
        gy[1] = gyro.value().y();
        gy[2] = gyro.value().z();
      }

      float acc[3];
      if (sensors.get()->accelerometers_size() > 0) {
        auto accelerometer = sensors.get()->accelerometers(0);
        acc[0] = accelerometer.value().x();
        acc[1] = accelerometer.value().y();
        acc[2] = accelerometer.value().z();
      }

      auto seconds = (sensors.get()->time() + 0.0) / 1000;

      imu->compute_rpy(gy, acc, seconds);
      
      if (is_running && is_running_now) {
        std::cout << "masuk running mode " << current_mode << std::endl;
        if (current_mode == "pivot") {
          if(locomotion->pivot(target_direction)) { 
            is_running = false;
            is_running_now = false; 
          }
          else { std::cout << "still pivoting go to " << target_direction << std::endl; }
        } else if (current_mode == "move_follow_head") {
          if(locomotion->move_follow_head(target_direction)) { 
            is_running = false; 
            is_running_now = false; 
          }
          else { std::cout << "still move follow head go to " << target_direction << std::endl; }
        } else if (current_mode == "move_to_target") {
          if(locomotion->move_to_target(target_x, target_y)) { 
            is_running = false; 
            is_running_now = false; 
          }
          else { std::cout << "still move_to_target go to x " << target_x << " y " << target_y << std::endl; }
        } else if (current_mode == "set_position") {
          locomotion->set_position(target_x, target_y);
          is_running = false;
          is_running_now = false; 
        }
      } else if (!cmds[0].empty() && !cmds[1].empty() && !cmds[2].empty()) {
        if (cmds[0] == "q") { 
          break;
        }

        locomotion->load_data(path + "locomotion/");
        std::cout << "loaded data\n";

        current_mode = cmds[0];
        if ((cmds[0] == "pivot" ||  cmds[0] == "move_follow_head") && !cmds[1].empty()) {
          target_direction = std::stof(cmds[1]);
          std::cout << "will " << current_mode << " at " << target_direction << "\n";
        } else if ((cmds[0] == "move_to_target" ||  cmds[0] == "set_position") && !cmds[2].empty()) {
          target_x = std::stof(cmds[1]);
          target_y = std::stof(cmds[2]);
          std::cout << "will " << current_mode << " at x " << target_x << " - y " << target_y << "\n";
        } else {
          std::cout << "-ERR command was not valid\n" << std::endl;
          is_running = false;
        }

        is_running_now = true;

        cmds[0].clear();
        cmds[1].clear();
        cmds[2].clear();
      }

      head->process();
      imu->compute_rpy(gy, acc, seconds);
      walking->process();
      if(is_running_now) std::cout << "in main now: x " << walking->POSITION_X << " y " << walking->POSITION_Y << std::endl; 

      for (auto joint : walking->get_joints()) {
        std::string joint_name = joint.get_joint_name();
        float position = joint.get_goal_position();

        if (joint_name.find("shoulder_pitch") != std::string::npos) {
          joint_name += " [shoulder]";
        } else if (joint_name.find("hip_yaw") != std::string::npos) {
          joint_name += " [hip]";
        }

        message.add_motor_position_in_degree(joint_name, position);
      }
      client.send(*message.get_actuator_request());
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}