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

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "suiryoku/locomotion/process/locomotion.hpp"

#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"

#include "unistd.h"  // NOLINT

using namespace keisan::literals;  // NOLINT

namespace suiryoku
{

Locomotion::Locomotion(std::shared_ptr<Robot> robot)
: position_prev_delta_pan(0.0), position_prev_delta_tilt(0.0),
  position_in_belief(0.0), x_speed_amplitude(0.0), y_speed_amplitude(0.0),
  stop_walking([]() {}), robot(robot)
{
}

void Locomotion::load_config(const std::string & path)
{
  std::ifstream file(path + "locomotion.json");
  nlohmann::json data = nlohmann::json::parse(file);

  set_config(data);

  file.close();
}

void Locomotion::set_config(const nlohmann::json & json)
{
  for (auto &[key, val] : json.items()) {
    if (key == "move") {
      try {
        val.at("min_x").get_to(move_min_x);
        val.at("max_x").get_to(move_max_x);
        val.at("max_y").get_to(move_max_y);
        val.at("max_a").get_to(move_max_a);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "follow") {
      try {
        val.at("max_x").get_to(follow_max_x);
        val.at("max_a").get_to(follow_max_a);
        val.at("min_tilt_").get_to(follow_min_tilt);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "dribble") {
      try {
        val.at("min_x").get_to(dribble_min_x);
        val.at("max_x").get_to(dribble_max_x);
        val.at("min_ly").get_to(dribble_min_ly);
        val.at("max_ly").get_to(dribble_max_ly);
        val.at("min_ry").get_to(dribble_min_ry);
        val.at("max_ry").get_to(dribble_max_ry);
        val.at("max_a").get_to(dribble_max_a);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "pivot") {
      try {
        val.at("min_x").get_to(pivot_min_x);
        val.at("max_x").get_to(pivot_max_x);
        val.at("max_ly").get_to(pivot_max_ly);
        val.at("max_ry").get_to(pivot_max_ry);
        val.at("max_a").get_to(pivot_max_a);
        val.at("target_tilt").get_to(pivot_target_tilt);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "position") {
      try {
        val.at("min_x").get_to(position_min_x);
        val.at("max_x").get_to(position_max_x);
        val.at("min_ly").get_to(position_min_ly);
        val.at("max_ly").get_to(position_max_ly);
        val.at("min_ry").get_to(position_min_ry);
        val.at("max_ry").get_to(position_max_ry);
        val.at("max_a").get_to(position_max_a);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "left_kick") {
      try {
        val.at("target_pan").get_to(left_kick_target_pan);
        val.at("target_tilt").get_to(left_kick_target_tilt);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "right_kick") {
      try {
        val.at("target_pan").get_to(right_kick_target_pan);
        val.at("target_tilt").get_to(right_kick_target_tilt);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
  }
}

bool Locomotion::walk_in_position()
{
  robot->x_speed = 0;
  robot->y_speed = 0;
  robot->a_speed = 0;
  robot->aim_on = false;

  bool in_position = fabs(x_speed_amplitude) < 5.0;
  in_position &= fabs(y_speed_amplitude) < 5.0;

  return in_position;
}

bool Locomotion::walk_in_position_until_stop()
{
  position_in_belief = 0.0;

  if (robot->is_walking) {
    robot->x_speed = 0;
    robot->y_speed = 0;
    robot->a_speed = 0;
    robot->aim_on = false;

    bool in_position = fabs(x_speed_amplitude) < 5.0;
    in_position &= fabs(y_speed_amplitude) < 5.0;

    if (!in_position) {
      return false;
    }

    stop_walking();
  }

  return !robot->is_walking;
}

void Locomotion::move_backward(const keisan::Angle<double> & direction)
{
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  double x_speed = move_min_x;
  double a_speed = keisan::map(delta_direction, -15.0, 15.0, move_max_a, -move_max_a);

  if (fabs(delta_direction) > 15.0) {
    a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;
    x_speed = 0.0;
  }

  robot->x_speed = x_speed;
  robot->y_speed = 0.0;
  robot->a_speed = a_speed;
  robot->aim_on = false;
}

bool Locomotion::move_backward_to(double target_x, double target_y)
{
  double delta_x = (robot->position_x - target_x);
  double delta_y = (robot->position_y - target_y);

  double target_distance = std::hypot(delta_x, delta_y);

  if (target_distance < 40.0) {
    return true;
  }

  auto direction = keisan::make_radian(atan2(delta_x, delta_y)).normalize();
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  double x_speed = move_min_x;
  double a_speed = keisan::map(delta_direction, -15.0, 15.0, move_max_a, -move_max_a);

  if (fabs(delta_direction) > 15.0) {
    a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;
    x_speed = 0.0;
  }

  robot->x_speed = x_speed;
  robot->y_speed = 0.0;
  robot->a_speed = a_speed;
  robot->aim_on = false;

  return false;
}

bool Locomotion::move_forward_to(double target_x, double target_y)
{
  double delta_x = (target_x - robot->position_x);
  double delta_y = (target_y - robot->position_y);

  double target_distance = std::hypot(delta_x, delta_y);

  if (target_distance < 40.0) {
    return true;
  }

  auto direction = keisan::make_radian(atan2(delta_x, delta_y)).normalize();
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  double a_speed =
    keisan::map(delta_direction, -10.0, 10.0, move_max_a, -move_max_a);
  double x_speed = keisan::map(fabs(a_speed), 0.0, move_max_a, move_max_x, 0.);

  if (target_distance < 100.0) {
    x_speed = keisan::map(target_distance, 0.0, 100.0, move_max_x * 0.25, move_max_x);
  }

  if (fabs(delta_direction) > 15.0) {
    a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;
    x_speed = 0.0;
  }

  robot->x_speed = x_speed;
  robot->y_speed = 0.0;
  robot->a_speed = a_speed;
  robot->aim_on = false;

  return false;
}

bool Locomotion::rotate_to(const keisan::Angle<double> & direction, bool a_move_only)
{
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  if (fabs(delta_direction) < move_max_a * 0.75) {
    return true;
  }

  double y_speed = 0.0;
  if (!a_move_only) {
    y_speed = (delta_direction < 0.0) ? move_max_y : -move_max_y;
  }

  double a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;

  robot->x_speed = 0.0;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = false;

  return false;
}

bool Locomotion::move_follow_head()
{
  return move_follow_head(follow_min_tilt);
}

bool Locomotion::move_follow_head(double min_tilt)
{
  double a_speed = keisan::map(robot->pan, -10.0, 10.0, -follow_max_a, follow_max_a);

  double x_speed = keisan::map(fabs(a_speed), 0.0, follow_max_a, follow_max_x, 0.);
  x_speed = keisan::map(robot->tilt - min_tilt, 10.0, 0.0, x_speed, 0.0);

  robot->x_speed = x_speed;
  robot->y_speed = 0.0;
  robot->a_speed = a_speed;
  robot->aim_on = false;

  return robot->tilt < min_tilt;
}

bool Locomotion::dribble(const keisan::Angle<double> & direction)
{
  double pan = robot->get_pan();
  bool is_dribble = true;

  double x_speed = 0;
  if (fabs(pan) < 15.0) {
    x_speed = keisan::map(fabs(pan), 0.0, 15.0, dribble_max_x, 0.);
  } else {
    is_dribble = false;
    x_speed = keisan::map(fabs(pan), 15.0, 45.0, 0.0, dribble_min_x);
  }

  double y_speed = 0.0;
  if (pan < -6.0) {
    y_speed = keisan::map(pan, -25.0, -6.0, dribble_max_ry, dribble_min_ry);
  } else if (pan > 6.0) {
    y_speed = keisan::map(pan, 6.0, 25.0, dribble_min_ly, dribble_max_ly);
  }

  auto delta_direction = (direction - robot->orientation).normalize().degree();
  double a_speed = keisan::map(
    delta_direction, -15.0, 15.0, dribble_max_a, -dribble_max_a);

  robot->x_speed = x_speed;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = false;

  return is_dribble;
}

bool Locomotion::pivot(const keisan::Angle<double> & direction)
{
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  if (fabs(delta_direction) < 30.0) {
    return true;
  }

  double pan = robot->get_pan();
  double tilt = robot->get_tilt();
  double delta_tilt = pivot_target_tilt - tilt;

  double x_speed = 0;
  if (delta_tilt > 0.0) {
    x_speed = keisan::map(delta_tilt, 0.0, 20.0, 0.0, pivot_min_x);
  } else {
    x_speed = keisan::map(delta_tilt, -20.0, 0.0, pivot_max_x, 0.);
  }

  double y_speed = (delta_direction < 0) ? pivot_max_ry : pivot_max_ly;

  double a_speed = keisan::map(pan, -10.0, 10.0, pivot_max_a, -pivot_max_a);

  robot->x_speed = x_speed;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = true;

  return false;
}

bool Locomotion::position_until(
  double target_pan, double target_tilt,
  const keisan::Angle<double> & direction)
{
  double pan = robot->get_pan();
  double tilt = robot->get_tilt();
  double delta_pan = fabs(target_pan - pan);
  double delta_tilt = fabs(target_tilt - tilt);
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  double abs_delta_pan = fabs(delta_pan);
  double abs_delta_tilt = fabs(delta_tilt);

  if (fabs(delta_direction) < 10.0) {
    if (abs_delta_pan < (3.0 + (3.0 * position_in_belief))) {
      position_in_belief += pow((0.24 * (1.0 - (abs_delta_pan / 6.0))), 2.0);
    } else if (abs_delta_pan <= fabs(position_prev_delta_pan) && abs_delta_pan < 6.0) {
      position_in_belief += pow((0.12 * (1.0 - (abs_delta_pan / 6.0))), 2.0);
    } else {
      position_in_belief -= 0.09;
    }

    if (abs_delta_tilt < (3.0 + (3.0 * position_in_belief))) {
      position_in_belief += pow((0.18 * (1.0 - (abs_delta_tilt / 6.0))), 2.0);
    } else if (abs_delta_tilt <= fabs(position_prev_delta_tilt) && abs_delta_tilt < 6.0) {
      position_in_belief += pow((0.9 * (1.0 - (abs_delta_tilt / 6.0))), 2.0);
    } else {
      position_in_belief -= 0.06;
    }
  } else {
    position_in_belief -= 0.10;
  }

  position_in_belief = keisan::clamp(position_in_belief, 0.0, 1.0);

  position_prev_delta_pan = delta_pan;
  position_prev_delta_tilt = delta_tilt;

  double x_speed = 0.0;
  double delta_tilt_pan = delta_tilt + (abs_delta_pan * 0.5);

  if (delta_tilt_pan > 3.0) {
    x_speed = keisan::map(
      delta_tilt_pan, 3.0, 20.0, position_min_x * 0.5, position_min_x);
  } else if (delta_tilt_pan < -3.0) {
    x_speed = keisan::map(
      delta_tilt_pan, -20.0, -3.0, position_max_x, position_max_x * 0.);
  }

  double y_speed = 0.0;
  if (delta_pan < -3.0) {
    y_speed = keisan::map(delta_pan, -20.0, -3.0, position_max_ly, position_min_ly);
  } else if (delta_pan > 3.0) {
    y_speed = keisan::map(delta_pan, 3.0, 20.0, position_min_ry, position_max_ry);
  }

  double a_speed = keisan::map(
    delta_direction, -15.0, 15.0, position_max_a, -position_max_a);

  robot->x_speed = x_speed;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = false;

  if (position_in_belief >= 1.0) {
    return true;
  }

  return false;
}

bool Locomotion::position_left_kick(const keisan::Angle<double> & direction)
{
  return position_until(
    left_kick_target_pan, left_kick_target_tilt, direction);
}

bool Locomotion::position_right_kick(const keisan::Angle<double> & direction)
{
  return position_until(
    right_kick_target_pan, right_kick_target_tilt, direction);
}

std::shared_ptr<Robot> Locomotion::get_robot() const
{
  return robot;
}

void Locomotion::update_move_amplitude(double x_amplitude, double y_amplitude)
{
  x_speed_amplitude = x_amplitude;
  y_speed_amplitude = y_amplitude;
}

void Locomotion::set_stop_walking_callback(const std::function<void()> & stop_walking)
{
  this->stop_walking = stop_walking;
}

}  // namespace suiryoku
