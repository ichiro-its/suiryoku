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
  if (argc < 3) {
    std::cerr << "Please specify the host, the port, and the path!" << std::endl;
    return 0;
  }

  std::string host = argv[1];
  int port = std::stoi(argv[2]);

  robocup_client::RobotClient client(host, port);
  if (!client.connect()) {
    std::cerr << "Failed to connect to server on port " <<
      client.get_port() << "!" << std::endl;

    return 1;
  }

  robocup_client::MessageHandler message;
  message.add_sensor_time_step("gyro", 8);
  message.add_sensor_time_step("accelerometer", 8);

  message.add_sensor_time_step("neck_yaw_s", 8);
  message.add_sensor_time_step("neck_pitch_s", 8);
  message.add_sensor_time_step("left_shoulder_pitch_s", 8);
  message.add_sensor_time_step("left_shoulder_roll_s", 8);
  message.add_sensor_time_step("left_elbow_s", 8);
  message.add_sensor_time_step("right_shoulder_pitch_s", 8);
  message.add_sensor_time_step("right_shoulder_roll_s", 8);
  message.add_sensor_time_step("right_elbow_s", 8);
  message.add_sensor_time_step("left_hip_yaw_s", 8);
  message.add_sensor_time_step("left_hip_roll_s", 8);
  message.add_sensor_time_step("left_hip_pitch_s", 8);
  message.add_sensor_time_step("left_knee_s", 8);
  message.add_sensor_time_step("left_ankle_roll_s", 8);
  message.add_sensor_time_step("left_ankle_pitch_s", 8);
  message.add_sensor_time_step("right_hip_yaw_s", 8);
  message.add_sensor_time_step("right_hip_roll_s", 8);
  message.add_sensor_time_step("right_hip_pitch_s", 8);
  message.add_sensor_time_step("right_knee_s", 8);
  message.add_sensor_time_step("right_ankle_roll_s", 8);
  message.add_sensor_time_step("right_ankle_pitch_s", 8);

  client.send(*message.get_actuator_request());

  auto imu = std::make_shared<kansei::Imu>();

  auto walking = std::make_shared<aruku::Walking>(imu);
  walking->initialize();
  walking->start();

  auto head = std::make_shared<atama::Head>(walking, imu);

  auto locomotion = std::make_shared<suiryoku::Locomotion>(walking, head, imu);

  std::string cmds[3] = {};

  bool is_running = false;
  std::thread input_handler([&cmds, &is_running] {
    while (true) {
      if (!is_running) {
        std::cout << "> run : ";
        std::cin >> cmds[0];

        if (cmds[0].find("move") != std::string::npos) {
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

      auto time = sensors.get()->time();

      imu->compute_rpy(gy, acc, time);
      head->process();
      walking->process();

      if (!cmds[0].empty() && !cmds[1].empty() && !cmds[2].empty()) {
        if (cmds[0] == "q") { 
          break;
        }

        locomotion->load_data();
        std::cout << "loaded data\n";

        if (cmds[0] == "pivot" && !cmds[1].empty()) {
          locomotion->pivot(std::stof(cmds[1]));
        } else if (cmds[0] == "move_follow_head" && !cmds[1].empty()) {
          locomotion->move_follow_head(std::stof(cmds[1]));
        } else if (cmds[0] == "move_to_target" && !cmds[2].empty()) {
          locomotion->move_to_target(std::stof(cmds[1]), std::stof(cmds[2]));
        } else if (cmds[0] == "set_position" && !cmds[2].empty()) {
          locomotion->set_position(std::stof(cmds[1]), std::stof(cmds[2]));
        } else {
          std::cout << "-ERR command was not valid\n" << std::endl;
          is_running = false;
        }
      }

      for (auto joint : walking->get_joints()) {
        if (joint.get_joint_name().find("shoulder_pitch") != std::string::npos) {
          message.add_motor_position_in_radian(
            joint.get_joint_name() + " [shouder]", joint.get_goal_position());
        } else if (joint.get_joint_name().find("hip_yaw") != std::string::npos) {
          message.add_motor_position_in_radian(
            joint.get_joint_name() + " [hip]", joint.get_goal_position());
        } else {
          message.add_motor_position_in_radian(joint.get_joint_name(), joint.get_goal_position());
        }
      }
      client.send(*message.get_actuator_request());

      cmds[0].clear();
      cmds[1].clear();
      cmds[2].clear();
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}
