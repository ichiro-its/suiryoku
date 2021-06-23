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
#include <ninshiki_opencv/detector.hpp>
#include <nlohmann/json.hpp>
#include <robocup_client/robocup_client.hpp>
#include <suiryoku/locomotion.hpp>

#include <unistd.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "common/algebra.h"

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
  message.add_sensor_time_step("Camera", 16);
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
  imu->load_data(path);

  auto walking = std::make_shared<aruku::Walking>(imu);
  walking->initialize();
  walking->load_data(path);
  // walking->start();

  auto head = std::make_shared<atama::Head>(walking, imu);
  head->initialize();
  head->load_data(path);

  auto locomotion = std::make_shared<suiryoku::Locomotion>(walking, head, imu);
  locomotion->load_data(path);

  auto camera = std::make_shared<CameraMeasurement>();
  cv::Mat frame, frame_hsv, field_mask;
  keisan::Point2 ball_pos;
  float view_h_angle, view_v_angle;

  ninshiki_opencv::Detector detector;

  std::string cmds[3] = {};

  bool is_running = false;
  bool is_running_now = false;
  std::string current_mode = "";
  float target_x, target_y, target_direction;
  std::thread input_handler([&cmds, &is_running, &is_running_now, &imu] {
      while (true) {
        if (!is_running && !is_running_now) {
          std::cout << "> run : ";
          std::cin >> cmds[0];

          bool input[3] = {};
          input[0] = cmds[0].find("move_to_target") != std::string::npos || cmds[0].find(
            "set_position") != std::string::npos;
          input[1] = cmds[0].find("walk_in_position") != std::string::npos || cmds[0].find(
            "move_follow_head") != std::string::npos;
          if (input[0]) {
            std::cout << "  target x : ";
            std::cin >> cmds[1];
            std::cout << "  target y : ";
            std::cin >> cmds[2];
          } else if (input[1]) {
            cmds[1] = "empty";
            cmds[2] = "empty";
          } else {
            std::cout << "current orientation: " << imu->get_yaw() << std::endl;
            std::cout << "  direction : ";
            std::cin >> cmds[1];
            cmds[2] = "empty";
          }

          is_running = true;
        } else {
          usleep(100000);
        }
      }
    });

  while (client.get_tcp_socket()->is_connected()) {
    try {
      auto sensors = client.receive();

      auto seconds = (sensors.get()->time() + 0.0) / 1000;

      double gy[3];
      if (sensors.get()->gyros_size() > 0) {
        auto gyro = sensors.get()->gyros(0);
        gy[0] = gyro.value().x();
        gy[1] = gyro.value().y();
        gy[2] = gyro.value().z();
      }

      double acc[3];
      if (sensors.get()->accelerometers_size() > 0) {
        auto accelerometer = sensors.get()->accelerometers(0);
        acc[0] = accelerometer.value().x();
        acc[1] = accelerometer.value().y();
        acc[2] = accelerometer.value().z();
      }

      imu->compute_rpy(gy, acc, seconds);

      if (is_running && is_running_now) {
        if (current_mode == "pivot") {
          if (locomotion->pivot(target_direction)) {
            locomotion->walk_in_position();
            is_running = false;
            is_running_now = false;
          } else {
            std::cout << "pivot target_direction " << target_direction << std::endl;
          }
        } else if (current_mode == "dribble") {
          if (locomotion->dribble(target_direction)) {
            locomotion->walk_in_position();
            is_running = false;
            is_running_now = false;
          } else {
            std::cout << "dribble target_direction " << target_direction << std::endl;
          }
        } else if (current_mode == "move_to_position_left_kick") {
          if (locomotion->move_to_position_left_kick(target_direction)) {
            locomotion->walk_in_position();
            is_running = false;
            is_running_now = false;
          } else {
            std::cout << "move_to_position_left_kick target_direction " << target_direction <<
              std::endl;
          }
        } else if (current_mode == "move_to_position_right_kick") {
          if (locomotion->move_to_position_right_kick(target_direction)) {
            locomotion->walk_in_position();
            is_running = false;
            is_running_now = false;
          } else {
            std::cout << "move_to_position_right_kick target_direction " << target_direction <<
              std::endl;
          }
        } else if (current_mode == "move_follow_head") {
          if (locomotion->move_follow_head()) {
            locomotion->walk_in_position();
            is_running = false;
            is_running_now = false;
          } else {
            std::cout << "move_follow_head at " << head->get_tilt_angle() << std::endl;
          }
        } else if (current_mode == "move_to_target") {
          if (locomotion->move_to_target(target_x, target_y)) {
            locomotion->walk_in_position();
            is_running = false;
            is_running_now = false;
          } else {
            std::cout << "move_to_target target_x " << target_x << ", target_y " << target_y <<
              std::endl;
          }
        } else if (current_mode == "move_backward") {
          if (locomotion->move_backward(target_direction)) {
            locomotion->walk_in_position();
            is_running = false;
            is_running_now = false;
          } else {
            std::cout << "move_backward target_direction " << target_direction << std::endl;
          }
        } else if (current_mode == "rotate_to_target") {
          if (locomotion->rotate_to_target(target_direction)) {
            locomotion->walk_in_position();
            is_running = false;
            is_running_now = false;
          } else {
            std::cout << "rotato_to_target target_direction " << target_direction << std::endl;
          }
        } else if (current_mode == "set_position") {
          locomotion->set_position(target_x, target_y);
          is_running = false;
          is_running_now = false;
        }
      } else if (!cmds[0].empty() && !cmds[1].empty() && !cmds[2].empty()) {
        if (cmds[0] == "q") {
          break;
        }

        current_mode = cmds[0];
        if ((cmds[0] == "pivot" || cmds[0] == "dribble" ||
          cmds[0] == "move_to_position_right_kick" || cmds[0] == "move_to_position_left_kick" ||
          cmds[0] == "rotate_to_target" || cmds[0] == "move_backward") && !cmds[1].empty())
        {
          target_direction = std::stof(cmds[1]);
          std::cout << "curr orientation " << imu->get_yaw() << std::endl;
          std::cout << "will " << current_mode << " at " << target_direction << "\n";
        } else if ((cmds[0] == "move_to_target" || cmds[0] == "set_position") && !cmds[2].empty()) {
          target_x = std::stof(cmds[1]);
          target_y = std::stof(cmds[2]);
          std::cout << "will " << current_mode << " at x " << target_x << " - y " << target_y <<
            "\n";
        } else if ((cmds[0] == "walk_in_position" || cmds[0] == "move_follow_head")) {
          std::cout << "will " << current_mode << "\n";
        } else {
          std::cout << "-ERR command was not valid\n" << std::endl;
          is_running = false;
        }

        is_running_now = true;

        cmds[0].clear();
        cmds[1].clear();
        cmds[2].clear();
      }

      if (is_running_now) {
        std::cout << "pan " << head->get_pan_angle() << std::endl;
        std::cout << "tilt " << head->get_tilt_angle() << std::endl;
        std::cout << "orientation " << imu->get_yaw() << std::endl;
        std::cout << "pos_x " << walking->POSITION_X << ", pos_y " << walking->POSITION_Y <<
          std::endl;
        std::cout << "x_speed " << walking->X_MOVE_AMPLITUDE << ", y_speed " <<
          walking->Y_MOVE_AMPLITUDE <<
          ", a_speed " << walking->A_MOVE_AMPLITUDE << std::endl;
        std::cout << "============================================" << std::endl;
      }

      // Get Ball Position
      if (sensors.get()->cameras_size() > 0) {
        *camera = sensors.get()->cameras(0);
        cv::Mat temp = detector.get_image(sensors);

        frame = temp.clone();
        frame_hsv = temp.clone();
        cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

        detector.vision_process(frame_hsv, frame);

        int field_of_view = 78;

        float diagonal = sqrt(pow(camera->width(), 2) + pow(camera->height(), 2));
        float depth = (diagonal / 2) / tan(field_of_view * M_PI / 180.0 / 2);
        view_h_angle = 2 * atan2(camera->width() / 2, depth) * 180.0 / M_PI;
        view_v_angle = 2 * atan2(camera->height() / 2, depth) * 180.0 / M_PI;

        ball_pos = keisan::Point2(detector.get_ball_pos_x(), detector.get_ball_pos_y());
      }
      // std::cout <<"ball pos = "<< ball_pos.x << " " << ball_pos.y << std::endl;
      if (ball_pos.x == 0 && ball_pos.y == 0) {
        head->move_scan_ball_down();
      } else if (ball_pos.x != 0 || ball_pos.y != 0) {
        head->track_ball(camera, ball_pos, view_v_angle, view_h_angle);
        // locomoti
        // locomotion->follow_ball(ball_pos);
      }

      head->process();
      walking->process();

      message.clear_actuator_request();
      for (auto joint : walking->get_joints()) {
        std::string joint_name = joint.get_joint_name();
        if (joint_name != "neck_yaw" || joint_name != "neck_pitch") {
          float position = joint.get_goal_position();

          if (joint_name.find("shoulder_pitch") != std::string::npos) {
            joint_name += " [shoulder]";
          } else if (joint_name.find("hip_yaw") != std::string::npos) {
            joint_name += " [hip]";
          }

          message.add_motor_position_in_degree(joint_name, position);
        }
      }

      std::cout << "neck_yaw: " << head->get_pan_angle() << std::endl;
      std::cout << "neck_pitch: " << head->get_tilt_angle() << std::endl;
      message.add_motor_position_in_degree("neck_yaw", head->get_pan_angle());
      message.add_motor_position_in_degree("neck_pitch", head->get_tilt_angle());
      client.send(*message.get_actuator_request());

      if (cmds[0] == "empty") {
        usleep(100000);
      }
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}
