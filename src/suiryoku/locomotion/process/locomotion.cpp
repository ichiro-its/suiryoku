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
  position_in_belief(0.0), stop([]() {}), start([]() {}), robot(robot)
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
        follow_min_tilt = keisan::make_degree(val.at("min_tilt_").get<double>());
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "skew") {
      try {
        val.at("max_x").get_to(skew_max_x);
        val.at("max_a").get_to(skew_max_a);
        val.at("tilt").get_to(skew_tilt);
        val.at("pan_comp").get_to(skew_pan_comp);
        val.at("delta_direction_comp").get_to(skew_delta_direction_comp);
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
        pivot_target_tilt = keisan::make_degree(val.at("target_tilt").get<double>());
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
        val.at("min_delta_tilt").get_to(position_min_delta_tilt);
        val.at("min_delta_pan").get_to(position_min_delta_pan);
        val.at("min_delta_pan_tilt").get_to(position_min_delta_pan_tilt);
        val.at("min_delta_direction").get_to(position_min_delta_direction);
      }
      catch (nlohmann::json::parse_error &ex)
      {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "left_kick") {
      try {
        left_kick_target_pan = keisan::make_degree(val.at("target_pan").get<double>());
        left_kick_target_tilt =
          keisan::make_degree(val.at("target_tilt").get<double>());
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "right_kick") {
      try {
        right_kick_target_pan = keisan::make_degree(val.at("target_pan").get<double>());
        right_kick_target_tilt =
          keisan::make_degree(val.at("target_tilt").get<double>());
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
  start();

  bool in_position = fabs(robot->x_amplitude) < 5.0;
  in_position &= fabs(robot->y_amplitude) < 5.0;

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
    stop();

    bool in_position = fabs(robot->x_amplitude) < 5.0;
    in_position &= fabs(robot->y_amplitude) < 5.0;

    if (!in_position) {
      return false;
    }
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
  start();
}

bool Locomotion::move_backward_to(const keisan::Point2 & target)
{
  double delta_x = (robot->position.x - target.x);
  double delta_y = (robot->position.y - target.y);

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
  start();

  return false;
}

bool Locomotion::move_forward_to(const keisan::Point2 & target)
{
  double delta_x = (target.x - robot->position.x);
  double delta_y = (target.y - robot->position.y);

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
  start();

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
  start();

  return false;
}

bool Locomotion::move_follow_head()
{
  return move_follow_head(follow_min_tilt);
}

bool Locomotion::move_follow_head(const keisan::Angle<double> & min_tilt)
{
  double a_speed = keisan::map(
    robot->pan.degree(), -10.0, 10.0, -follow_max_a, follow_max_a);

  double x_speed = keisan::map(fabs(a_speed), 0.0, follow_max_a, follow_max_x, 0.);
  x_speed = keisan::map((robot->tilt - min_tilt).degree(), 10.0, 0.0, x_speed, 0.0);

  robot->x_speed = x_speed;
  robot->y_speed = 0.0;
  robot->a_speed = a_speed;
  robot->aim_on = false;
  start();

  return robot->tilt < min_tilt;
}

bool Locomotion::move_skew(const keisan::Angle<double> & direction)
{
  auto current_direction = (robot->orientation - robot->pan).normalize();
  auto delta_direction = (direction - current_direction).normalize().degree();
  return move_skew(direction, delta_direction > 0);
}

bool Locomotion::move_skew(const keisan::Angle<double> & direction, bool skew_left)
{
  auto current_direction = (robot->orientation - robot->pan).normalize();
  double delta_direction = (direction - current_direction).normalize().degree();

  if (delta_direction < skew_delta_direction_comp && fabs((direction - robot->orientation).normalize().degree()) < 10.0)
  {
    return true;
  }
  double min_skew_tilt = skew_tilt + 10.0;
  double max_skew_tilt = std::min(skew_tilt - 15.0, -60.0);
  double pan_comp = keisan::map(robot->tilt.degree(), min_skew_tilt, max_skew_tilt, 0.0, skew_pan_comp);
  auto target_direction = current_direction.degree();
  if (skew_left) {
    target_direction -= pan_comp;
  } else {
    target_direction += pan_comp;
  }
  auto target_direction_deg = keisan::make_degree(target_direction).normalize();

  double delta_target_skew_direction = (target_direction_deg - robot->orientation).normalize().degree();

  if (pan_comp > 0.0) {

    double min_delta_target_skew_dir = 2.0;
    double max_delta_target_skew_dir = (skew_pan_comp * 0.3);

    double move_a = 0.0;
    if (delta_target_skew_direction > 0) {
      move_a = keisan::map(delta_target_skew_direction, min_delta_target_skew_dir, max_delta_target_skew_dir, 0.0, -skew_max_a);
    } else {
      move_a = keisan::map(delta_target_skew_direction, -max_delta_target_skew_dir, -min_delta_target_skew_dir, skew_max_a, 0.0);
    }

    double move_x = 0.0;
    if (delta_direction > 10.0) {
      move_x = keisan::map(fabs(move_a), 0.0, skew_max_a, skew_max_x, 0.0);
    }

    robot->x_speed = move_x;
    robot->y_speed = 0.0;
    robot->a_speed = move_a;
    robot->aim_on = false;
    start();
  } else {
    move_follow_head();
  }
}

bool Locomotion::dribble(const keisan::Angle<double> & direction)
{
  double pan = robot->get_pan().degree();
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
  start();

  return is_dribble;
}

bool Locomotion::pivot(const keisan::Angle<double> & direction)
{
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  if (fabs(delta_direction) < 30.0) {
    return true;
  }

  double delta_tilt = (pivot_target_tilt - robot->get_tilt()).degree();

  double x_speed = 0;
  if (delta_tilt > 0.0) {
    x_speed = keisan::map(delta_tilt, 0.0, 20.0, 0.0, pivot_min_x);
  } else {
    x_speed = keisan::map(delta_tilt, -20.0, 0.0, pivot_max_x, 0.);
  }

  double y_speed = (delta_direction < 0) ? pivot_max_ry : pivot_max_ly;

  double a_speed = keisan::map(
    robot->get_pan().degree(), -10.0, 10.0, pivot_max_a, -pivot_max_a);

  robot->x_speed = x_speed;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = true;
  start();

  return false;
}

bool Locomotion::position_until(
  const keisan::Angle<double> & target_pan,
  const keisan::Angle<double> & target_tilt,
  const keisan::Angle<double> & direction)
{
  double delta_pan = fabs((target_pan - robot->get_pan()).degree());
  double delta_tilt = fabs((target_tilt - robot->get_tilt()).degree());
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  double abs_delta_pan = fabs(delta_pan);
  double abs_delta_tilt = fabs(delta_tilt);

  double x_speed = 0.0;
  double delta_tilt_pan = delta_tilt + (abs_delta_pan * 0.3);

  if (delta_tilt_pan > 3.0) {
    x_speed = keisan::map(
      delta_tilt_pan, 3.0, 20.0, position_min_x * 0.5, position_min_x);
  } else if (delta_tilt_pan < -3.0) {
    x_speed = keisan::map(
      delta_tilt_pan, -20.0, -3.0, position_max_x, position_max_x * 0.5);
  }

  double y_speed = 0.0;
  if (delta_pan < -3.0) {
    y_speed = keisan::map(delta_pan, -20.0, -position_min_delta_pan, position_max_ly, position_min_ly);
  } else if (delta_pan > 3.0) {
    y_speed = keisan::map(delta_pan, position_min_delta_pan, 20.0, position_min_ry, position_max_ry);
  }

  double a_speed = keisan::map(
    delta_direction, -15.0, 15.0, position_max_a, -position_max_a);

  robot->x_speed = x_speed;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = false;
  start();

  if (abs_delta_pan < position_min_delta_pan && abs_delta_tilt < position_min_delta_tilt) {
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

bool Locomotion::position_kick_general(const keisan::Angle<double> & direction)
{
  double delta_pan_left = fabs((left_kick_target_pan - robot->get_pan()).degree());
  double delta_tilt_left = fabs((left_kick_target_tilt - robot->get_tilt()).degree());

  double delta_pan_right = fabs((right_kick_target_pan - robot->get_pan()).degree());
  double delta_tilt_right = fabs((right_kick_target_tilt - robot->get_tilt()).degree());

  auto delta_direction = (direction - robot->orientation).normalize().degree();

  bool left_kick_valid = delta_pan_left < position_min_delta_pan && delta_tilt_left < position_min_delta_tilt;
  bool right_kick_valid = delta_pan_right < position_min_delta_pan && delta_tilt_right < position_min_delta_tilt;

  if (left_kick_valid || right_kick_valid) {
    return true;
  }

  keisan::Angle<double> target_pan, target_tilt;
  target_pan = left_kick_target_pan;
  target_tilt = left_kick_target_tilt;

  if ((robot->get_pan() - right_kick_target_pan).degree() < 0) {
    target_pan = right_kick_target_pan;
    target_tilt = right_kick_target_tilt;
  }

  return position_until(target_pan, target_tilt, direction);
}

std::shared_ptr<Robot> Locomotion::get_robot() const
{
  return robot;
}

}  // namespace suiryoku
