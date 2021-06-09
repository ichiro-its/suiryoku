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
  auto head = std::make_shared<atama::Head>(walking, imu);
  auto locomotion = std::make_shared<suiryoku::Locomotion>(walking, head, imu);

  while (client.get_tcp_socket()->is_connected()) {
    try {
      message.clear_actuator_request();

      auto sensors = client.receive();

      auto time = sensors.get()->time();

    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}
