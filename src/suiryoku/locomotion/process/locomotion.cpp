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
#include <algorithm>

#include "suiryoku/locomotion/process/locomotion.hpp"

#include "keisan/keisan.hpp"
#include "jitsuyo/config.hpp"
#include "nlohmann/json.hpp"

#include "unistd.h"  // NOLINT

using namespace keisan::literals;  // NOLINT

namespace suiryoku
{

Locomotion::Locomotion(std::shared_ptr<Robot> robot)
: config_name("locomotion.json"), position_prev_delta_pan(0.0_deg), position_prev_delta_tilt(0.0_deg),
  position_in_belief(0.0), stop([]() {}), start([]() {}), robot(robot)
{
}

void Locomotion::load_config(const std::string & path)
{
  nlohmann::json data;
  if (!jitsuyo::load_config(path, config_name, data)) {
    throw std::runtime_error("Failed to find config file `locomotion.json`");
  }

  set_config(data);
}

void Locomotion::set_config(const nlohmann::json & json)
{

  bool valid_config = true;

  nlohmann::json move_section;
  if (jitsuyo::assign_val(json, "move", move_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(move_section, "min_x", move_min_x);
    valid_section &= jitsuyo::assign_val(move_section, "max_x", move_max_x);
    valid_section &= jitsuyo::assign_val(move_section, "max_y", move_max_y);
    valid_section &= jitsuyo::assign_val(move_section, "max_a", move_max_a);
    if (!valid_section) {
      std::cout << "Error found at section `move`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json rotate_section;
  if (jitsuyo::assign_val(json, "rotate", rotate_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(rotate_section, "max_a", rotate_max_a);
    valid_section &= jitsuyo::assign_val(rotate_section, "max_delta_direction", rotate_max_delta_direction);
    if (!valid_section) {
      std::cout << "Error found at section `rotate`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json backward_section;
  if (jitsuyo::assign_val(json, "backward", backward_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(backward_section, "min_x", backward_min_x);
    valid_section &= jitsuyo::assign_val(backward_section, "max_x", backward_max_x);
    valid_section &= jitsuyo::assign_val(backward_section, "max_a", backward_max_a);
    if (!valid_section) {
      std::cout << "Error found at section `backward`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json dribble_section;
  if (jitsuyo::assign_val(json, "dribble", dribble_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(dribble_section, "max_x", dribble_max_x);
    valid_section &= jitsuyo::assign_val(dribble_section, "min_x", dribble_min_x);
    valid_section &= jitsuyo::assign_val(dribble_section, "max_ly", dribble_max_ly);
    valid_section &= jitsuyo::assign_val(dribble_section, "min_ly", dribble_min_ly);
    valid_section &= jitsuyo::assign_val(dribble_section, "max_ry", dribble_max_ry);
    valid_section &= jitsuyo::assign_val(dribble_section, "min_ry", dribble_min_ry);
    valid_section &= jitsuyo::assign_val(dribble_section, "max_a", dribble_max_a);
    valid_section &= jitsuyo::assign_val(dribble_section, "pan_comp", dribble_pan_comp);
    if (!valid_section) {
      std::cout << "Error found at section `dribble`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json follow_section;
  if (jitsuyo::assign_val(json, "follow", follow_section)) {
    bool valid_section = true;

    double follow_min_tilt_double;

    valid_section &= jitsuyo::assign_val(follow_section, "pan_ratio", follow_pan_ratio);
    valid_section &= jitsuyo::assign_val(follow_section, "max_x", follow_max_x);
    valid_section &= jitsuyo::assign_val(follow_section, "min_x", follow_min_x);
    valid_section &= jitsuyo::assign_val(follow_section, "max_a", follow_max_a);
    valid_section &= jitsuyo::assign_val(follow_section, "l_a_offset", follow_l_a_offset);
    valid_section &= jitsuyo::assign_val(follow_section, "r_a_offset", follow_r_a_offset);
    valid_section &= jitsuyo::assign_val(follow_section, "y_move", follow_y_move);
    valid_section &= jitsuyo::assign_val(follow_section, "max_ry", follow_max_ry);
    valid_section &= jitsuyo::assign_val(follow_section, "min_ry", follow_min_ry);
    valid_section &= jitsuyo::assign_val(follow_section, "max_ly", follow_max_ly);
    valid_section &= jitsuyo::assign_val(follow_section, "min_ly", follow_min_ly);
    valid_section &= jitsuyo::assign_val(follow_section, "min_tilt_", follow_min_tilt_double);

    follow_min_tilt = keisan::make_degree(follow_min_tilt_double);

    if (!valid_section) {
      std::cout << "Error found at section `follow`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json skew_section;
  if (jitsuyo::assign_val(json, "skew", skew_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(skew_section, "max_x", skew_max_x);
    valid_section &= jitsuyo::assign_val(skew_section, "max_a", skew_max_a);
    valid_section &= jitsuyo::assign_val(skew_section, "tilt", skew_tilt);
    valid_section &= jitsuyo::assign_val(skew_section, "pan_comp", skew_pan_comp);
    valid_section &= jitsuyo::assign_val(skew_section, "delta_direction_comp", skew_delta_direction_comp);
    if (!valid_section) {
      std::cout << "Error found at section `skew`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json pivot_section;
  if (jitsuyo::assign_val(json, "pivot", pivot_section)) {
    bool valid_section = true;

    double pivot_target_tilt_double;
    double pivot_stop_limit_double;

    valid_section &= jitsuyo::assign_val(pivot_section, "min_x", pivot_min_x);
    valid_section &= jitsuyo::assign_val(pivot_section, "max_x", pivot_max_x);
    valid_section &= jitsuyo::assign_val(pivot_section, "max_ly", pivot_max_ly);
    valid_section &= jitsuyo::assign_val(pivot_section, "max_ry", pivot_max_ry);
    valid_section &= jitsuyo::assign_val(pivot_section, "max_a", pivot_max_a);
    valid_section &= jitsuyo::assign_val(pivot_section, "max_delta_direction", pivot_max_delta_direction);
    valid_section &= jitsuyo::assign_val(pivot_section, "pan_range_a_speed", pivot_pan_range_a_speed);
    valid_section &= jitsuyo::assign_val(pivot_section, "target_tilt", pivot_target_tilt_double);
    valid_section &= jitsuyo::assign_val(pivot_section, "pivot_stop_limit", pivot_stop_limit_double);

    pivot_target_tilt = keisan::make_degree(pivot_target_tilt_double);
    pivot_stop_limit = keisan::make_degree(pivot_stop_limit_double);

    if (!valid_section) {
      std::cout << "Error found at section `pivot`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json position_section;
  if (jitsuyo::assign_val(json, "position", position_section)) {
    bool valid_section = true;

    double position_min_delta_tilt_double;
    double position_min_delta_pan_double;
    double position_min_delta_pan_tilt_double;
    double position_min_delta_direction_double;
    double position_min_range_tilt_double;
    double position_max_range_tilt_double;
    double position_min_range_pan_double;
    double position_max_range_pan_double;
    double position_center_right_range_pan_double;
    double position_center_left_range_pan_double;

    valid_section &= jitsuyo::assign_val(position_section, "min_x", position_min_x);
    valid_section &= jitsuyo::assign_val(position_section, "max_x", position_max_x);
    valid_section &= jitsuyo::assign_val(position_section, "min_ly", position_min_ly);
    valid_section &= jitsuyo::assign_val(position_section, "max_ly", position_max_ly);
    valid_section &= jitsuyo::assign_val(position_section, "min_ry", position_min_ry);
    valid_section &= jitsuyo::assign_val(position_section, "max_ry", position_max_ry);
    valid_section &= jitsuyo::assign_val(position_section, "max_a", position_max_a);
    valid_section &= jitsuyo::assign_val(position_section, "min_delta_tilt", position_min_delta_tilt_double);
    valid_section &= jitsuyo::assign_val(position_section, "min_delta_pan", position_min_delta_pan_double);
    valid_section &= jitsuyo::assign_val(position_section, "min_delta_pan_tilt", position_min_delta_pan_tilt_double);
    valid_section &= jitsuyo::assign_val(position_section, "min_delta_direction", position_min_delta_direction_double);
    valid_section &= jitsuyo::assign_val(position_section, "min_range_tilt", position_min_range_tilt_double);
    valid_section &= jitsuyo::assign_val(position_section, "max_range_tilt", position_max_range_tilt_double);
    valid_section &= jitsuyo::assign_val(position_section, "min_range_pan", position_min_range_pan_double);
    valid_section &= jitsuyo::assign_val(position_section, "max_range_pan", position_max_range_pan_double);
    valid_section &= jitsuyo::assign_val(position_section, "center_right_range_pan", position_center_right_range_pan_double);
    valid_section &= jitsuyo::assign_val(position_section, "center_left_range_pan", position_center_left_range_pan_double);

    position_min_delta_tilt = keisan::make_degree(position_min_delta_tilt_double);
    position_min_delta_pan = keisan::make_degree(position_min_delta_pan_double);
    position_min_delta_pan_tilt = keisan::make_degree(position_min_delta_pan_tilt_double);
    position_min_delta_direction = keisan::make_degree(position_min_delta_direction_double);
    position_min_range_tilt = keisan::make_degree(position_min_range_tilt_double);
    position_max_range_tilt = keisan::make_degree(position_max_range_tilt_double);
    position_min_range_pan = keisan::make_degree(position_min_range_pan_double);
    position_max_range_pan = keisan::make_degree(position_max_range_pan_double);
    position_center_right_range_pan = keisan::make_degree(position_center_right_range_pan_double);
    position_center_left_range_pan = keisan::make_degree(position_center_left_range_pan_double);

    if (!valid_section) {
      std::cout << "Error found at section `position`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json left_kick_section;
  if (jitsuyo::assign_val(json, "left_kick", left_kick_section)) {
    bool valid_section = true;

    double left_kick_target_pan_double;
    double left_kick_target_tilt_double;

    valid_section &= jitsuyo::assign_val(left_kick_section, "target_pan", left_kick_target_pan_double);
    valid_section &= jitsuyo::assign_val(left_kick_section, "target_tilt", left_kick_target_tilt_double);

    left_kick_target_pan = keisan::make_degree(left_kick_target_pan_double);
    left_kick_target_tilt = keisan::make_degree(left_kick_target_tilt_double);

    if (!valid_section) {
      std::cout << "Error found at section `left_kick`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json right_kick_section;
  if (jitsuyo::assign_val(json, "right_kick", right_kick_section)) {
    bool valid_section = true;

    double right_kick_target_pan_double;
    double right_kick_target_tilt_double;

    valid_section &= jitsuyo::assign_val(right_kick_section, "target_pan", right_kick_target_pan_double);
    valid_section &= jitsuyo::assign_val(right_kick_section, "target_tilt", right_kick_target_tilt_double);

    right_kick_target_pan = keisan::make_degree(right_kick_target_pan_double);
    right_kick_target_tilt = keisan::make_degree(right_kick_target_tilt_double);

    if (!valid_section) {
      std::cout << "Error found at section `right_kick`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json localization_section;
  if (jitsuyo::assign_val(json, "localization", localization_section)) {
    bool valid_section = true;

    valid_section &= jitsuyo::assign_val(localization_section, "enable", robot->use_localization);
    valid_section &= jitsuyo::assign_val(localization_section, "num_particles", robot->num_particles);
    valid_section &= jitsuyo::assign_val(localization_section, "min_centered_particles_ratio", robot->min_centered_particles_ratio);
    valid_section &= jitsuyo::assign_val(localization_section, "short_term_avg_ratio", robot->short_term_avg_ratio);
    valid_section &= jitsuyo::assign_val(localization_section, "long_term_avg_ratio", robot->long_term_avg_ratio);
    valid_section &= jitsuyo::assign_val(localization_section, "reset_particles_threshold", robot->reset_particles_threshold);
    valid_section &= jitsuyo::assign_val(localization_section, "sigma_x", robot->sigma_x);
    valid_section &= jitsuyo::assign_val(localization_section, "sigma_y", robot->sigma_y);
    valid_section &= jitsuyo::assign_val(localization_section, "max_distance_x", robot->max_object_distance.x);
    valid_section &= jitsuyo::assign_val(localization_section, "max_distance_y", robot->max_object_distance.y);

    if (!valid_section) {
      std::cout << "Error found at section `localization`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `locomotion.json`");
  }
}

bool Locomotion::walk_in_position()
{
  robot->x_speed = 0;
  robot->y_speed = 0;
  robot->a_speed = 0;
  robot->aim_on = false;
  start();

  bool in_position = std::abs(robot->x_amplitude) < 5.0;
  in_position &= std::abs(robot->y_amplitude) < 5.0;

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

    bool in_position = std::abs(robot->x_amplitude) < 5.0;
    in_position &= std::abs(robot->y_amplitude) < 5.0;

    if (!in_position) {
      return false;
    }
  }

  return !robot->is_walking;
}

void Locomotion::move_backward(const keisan::Angle<double> & direction)
{
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  #if ITHAARO || UMARU || MIRU
  double min_delta_direction = 15;
  #else
  double min_delta_direction = 10;
  #endif

  double x_speed = 0.0;
  double a_speed = keisan::map(delta_direction, -min_delta_direction, min_delta_direction, backward_max_a, -backward_max_a);
  if (std::abs(delta_direction) > 15.0) {
    a_speed = (delta_direction < 0.0) ? backward_max_a : -backward_max_a;
  } else {
    x_speed = keisan::map(std::abs(delta_direction), 0.0, 15.0, backward_max_x, backward_min_x);
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

  if (target_distance < 8.0) {
    return true;
  }

  auto direction = keisan::signed_arctan(delta_y, delta_x).normalize();
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  double x_speed = keisan::map(std::abs(delta_direction), 0.0, 15.0, backward_max_x, backward_min_x);

  double a_speed = keisan::map(delta_direction, -25.0, 25.0, backward_max_a, -backward_max_a);
  if (std::abs(delta_direction) > 15.0) {
    a_speed = (delta_direction < 0.0) ? backward_max_a : -backward_max_a;
    x_speed = keisan::map(std::abs(a_speed), 0.0, backward_max_a, backward_max_a, 0.0);
  }

  x_speed = keisan::map(target_distance, 0.0, 25.0, backward_min_x, x_speed);

  robot->x_speed = x_speed;
  robot->y_speed = 0.0;
  robot->a_speed = a_speed;
  robot->aim_on = false;
  start();

  return false;
}

void Locomotion::move_forward(const keisan::Angle<double> & direction)
{
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  double x_speed = move_max_x;
  double a_speed = keisan::map(delta_direction, -15.0, 15.0, move_max_a, -move_max_a);

  if (std::abs(delta_direction) > 15.0) {
    a_speed = keisan::sign(delta_direction) * -move_max_a;
    x_speed = 0.0;
  }

  robot->x_speed = x_speed;
  robot->y_speed = 0.0;
  robot->a_speed = a_speed;
  robot->aim_on = false;
  start();
}

bool Locomotion::move_forward_to(const keisan::Point2 & target)
{
  double delta_x = (target.x - robot->position.x);
  double delta_y = (target.y - robot->position.y);

  double target_distance = std::hypot(delta_x, delta_y);

  if (target_distance < 8.0) {
    return true;
  }

  auto direction = keisan::signed_arctan(delta_y, delta_x).normalize();
  double delta_direction = (direction - robot->orientation).normalize().degree();

  double x_speed = keisan::map(std::abs(delta_direction), 0.0, 15.0, move_max_x, move_min_x);
  if (target_distance < 100.0) {
    x_speed = keisan::map(target_distance, 0.0, 100.0, move_max_x * 0.25, move_max_x);
  }

  double a_speed = keisan::map(delta_direction, -25.0, 25.0, move_max_a, -move_max_a);
  if (std::abs(delta_direction) > 15.0) {
    a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;
    x_speed = keisan::map(std::abs(a_speed), 0.0, move_max_a, move_max_x, 0.0);
  }

  x_speed = keisan::smooth(robot->x_speed, x_speed, 0.4);

  robot->x_speed = x_speed;
  robot->y_speed = 0.0;
  robot->a_speed = a_speed;
  robot->aim_on = false;
  start();

  return false;
}

bool Locomotion::rotate_to_target(const keisan::Angle<double> & direction)
{
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  if (std::abs(delta_direction) < rotate_max_delta_direction) {
    walk_in_position();
    return true;
  }

  double y_speed = (delta_direction < 0.0) ? move_max_y : -move_max_y;
  double a_speed = (delta_direction < 0.0) ? rotate_max_a : -rotate_max_a;

  robot->x_speed = keisan::smooth(robot->x_speed, 0.0, 0.8);
  robot->y_speed = keisan::smooth(robot->y_speed, y_speed, 0.8);
  robot->a_speed = keisan::smooth(robot->a_speed, a_speed, 0.8);
  robot->aim_on = false;
  start();

  return false;
}

bool Locomotion::rotate_to(const keisan::Angle<double> & direction, bool a_move_only)
{
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  if (std::abs(delta_direction) < move_max_a * 0.75) {
    return true;
  }

  double y_speed = 0.0;
  if (!a_move_only) {
    y_speed = keisan::sign(delta_direction) * -move_max_y;
  }

  double a_speed = keisan::sign(delta_direction) * -move_max_a;

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
  double a_speed = 0.0;
  if (robot->get_pan().degree() < 0.0) {
    a_speed = keisan::map(robot->get_pan().degree(), -30.0, follow_pan_ratio * right_kick_target_pan.degree(), -follow_max_a, 0.0);
  } else {
    a_speed = keisan::map(robot->get_pan().degree(), follow_pan_ratio * left_kick_target_pan.degree(), 30.0, 0.0, follow_max_a);
  }

  double x_speed = 0.0;
  if (follow_max_a != 0) {
    x_speed = keisan::map(std::abs(a_speed), 0.0, follow_max_a, follow_max_x, 0.0);
    x_speed = keisan::map((robot->get_tilt() + robot->tilt_center - min_tilt).degree(), 10.0, 0.0, x_speed, follow_min_x);
  } else {
    x_speed = keisan::map((robot->get_tilt() + robot->tilt_center - min_tilt).degree(), 10.0, 0.0, follow_max_x, follow_min_x);
    const auto max_a_speed = (robot->pan.degree() > 3.0) ? follow_l_a_offset : follow_r_a_offset;
    a_speed = keisan::map(x_speed, follow_min_x, follow_max_x, 0.0, max_a_speed);
  }

  double y_speed = 0.0;
  if (follow_y_move){
    if (robot->get_pan().degree() < -3.0) {
      y_speed = keisan::map(robot->get_pan().degree(), -15.0, 0.0, follow_max_ry, follow_min_ry);
    } else if (robot->get_pan().degree() > 3.0) {
      y_speed = keisan::map(robot->get_pan().degree(), 0.0, 15.0, follow_min_ly, follow_max_ly);
    }
  }

  double smooth_ratio = 1.0;

  x_speed = keisan::smooth(robot->x_speed, x_speed, smooth_ratio);

  robot->x_speed = x_speed;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = false;
  start();

  return robot->get_tilt() + robot->tilt_center < min_tilt;
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
  double delta_direction = std::fabs((direction - current_direction).normalize().degree());

  printf("current direction in skew: %f\n", current_direction.degree());
  printf("delta direction in skew: %f\n", delta_direction);

  if (delta_direction < skew_delta_direction_comp &&
    std::abs((direction - robot->orientation).normalize().degree()) < 10.0)
  {
    return true;
  }
  double min_skew_tilt = skew_tilt + 10.0;
  double max_skew_tilt = std::max(skew_tilt - 15.0, -60.0);
  double pan_comp = keisan::map(
    robot->tilt.degree(), min_skew_tilt, max_skew_tilt, 0.0, skew_pan_comp);
  auto target_direction = current_direction.degree();
  if (skew_left) {
    target_direction -= pan_comp;
  } else {
    target_direction += pan_comp;
  }
  auto target_direction_deg = keisan::make_degree(target_direction).normalize();

  printf("target direction skew: %f\n", target_direction_deg.degree());

  double delta_target_skew_direction =
    (target_direction_deg - robot->orientation).normalize().degree();

  printf("delta target skew dir: %f\n", delta_target_skew_direction);

  if (pan_comp > 0.0) {
    double min_delta_target_skew_dir = 2.0;
    double max_delta_target_skew_dir = (skew_pan_comp * 0.3);

    double move_a = 0.0;
    if (delta_target_skew_direction > 0) {
      move_a = keisan::map(
        delta_target_skew_direction, min_delta_target_skew_dir,
        max_delta_target_skew_dir, 0.0, -skew_max_a);
    } else {
      move_a = keisan::map(
        delta_target_skew_direction, -max_delta_target_skew_dir,
        -min_delta_target_skew_dir, skew_max_a, 0.0);
    }

    double move_x = 0.0;
    if (delta_direction > 10.0) {
      move_x = keisan::map(std::abs(move_a), 0.0, skew_max_a, skew_max_x, 0.0);
    }

    robot->x_speed = move_x;
    robot->y_speed = 0.0;
    robot->a_speed = move_a;
    robot->aim_on = false;
    start();

    return false;
  } else {
    return move_follow_head();
  }
}

bool Locomotion::dribble(const keisan::Angle<double> & direction)
{
  double min_kick_tilt = std::min(left_kick_target_tilt.degree(), right_kick_target_tilt.degree());
  bool is_dribble = (robot->get_tilt() + robot->tilt_center).degree() <= min_kick_tilt;

  double pan = robot->get_pan().degree();
  double delta_direction = (direction - robot->orientation).normalize().degree();

  double pan_range = std::max(std::abs(right_kick_target_pan.degree()), left_kick_target_pan.degree()) + dribble_pan_comp;
  double max_tilt = std::max(right_kick_target_tilt.degree(), left_kick_target_tilt.degree());

  double tilt = (robot->get_tilt() + robot->tilt_center).degree();
  double pan_range_ratio = keisan::map(tilt, max_tilt + 10.0, max_tilt, 0.0, 0.4);

  double x_speed = 0;
  if (std::abs(pan) < pan_range) {
    x_speed = keisan::map(std::abs(pan), pan_range_ratio * pan_range, pan_range, dribble_max_x, 0.0);
  } else {
    is_dribble = false;
    x_speed = keisan::map(std::abs(pan), pan_range, 45.0, 0.0, dribble_min_x);
  }

  // y movement
  double y_speed = 0.0;
  if (pan < -(pan_range) + (pan_range_ratio * pan_range)) {
    y_speed = keisan::map(pan, -(pan_range), -6.0, dribble_max_ry, dribble_min_ry);
  } else if (pan > pan_range - (pan_range_ratio * pan_range)) {
    y_speed = keisan::map(pan, 6.0, pan_range, dribble_min_ly, dribble_max_ly);
  }

  // a movement
  double a_speed = 0.0;
  if (delta_direction < 0.0) {
    a_speed = keisan::map(delta_direction, -15.0, -5.0, dribble_max_a, 0.0);
  } else {
    a_speed = keisan::map(delta_direction, 5.0, 15.0, 0.0, -dribble_max_a);
  }

  #if ITHAARO || UMARU || MIRU
  double smooth_ratio = 1.0;
  #else
  double smooth_ratio = 0.8;
  #endif

  a_speed = keisan::smooth(robot->a_speed, a_speed, smooth_ratio);
  x_speed = keisan::smooth(robot->x_speed, x_speed, smooth_ratio);
  y_speed = keisan::smooth(robot->y_speed, y_speed, smooth_ratio);

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

  if (std::abs(delta_direction) < pivot_max_delta_direction) {
    walk_in_position();
    return true;
  }

  double delta_tilt = (pivot_target_tilt - robot->tilt + robot->tilt_center).degree();

  double x_speed = 0;
  if (delta_tilt > 0.0) {
    x_speed = keisan::map(delta_tilt, 0.0, 20.0, 0.0, pivot_min_x);
  } else {
    x_speed = keisan::map(delta_tilt, -20.0, 0.0, pivot_max_x, 0.);
  }

  double pan = (robot->pan + robot->pan_center).degree();

  double y_speed = 0.0;
  if (delta_direction < 0.0) {
    y_speed = keisan::map(pan, 30.0, 0.0, pivot_max_ry * 0.5, pivot_max_ry);
  } else {
    y_speed = keisan::map(pan, -30.0, 0.0, pivot_max_ly * 0.5, pivot_max_ly);
  }

  double a_speed = 0.0;
  if (fabs(pan) > pivot_pan_range_a_speed) {
    a_speed = keisan::map(pan, -pivot_pan_range_a_speed, pivot_pan_range_a_speed, pivot_max_a, -pivot_max_a);
  }

  robot->x_speed = x_speed;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = true;
  start();

  return false;
}

bool Locomotion::pivot_new(const keisan::Angle<double> & direction)
{
  if (initial_pivot) {
    initial_pivot = false;
    walk_in_position();
    return false;
  }

  auto delta_direction = (direction - robot->orientation).normalize().degree();

  if (std::abs(delta_direction) < pivot_max_delta_direction) {
    walk_in_position();
    return true;
  }

  auto tilt = robot->get_tilt();
  double delta_tilt = (pivot_target_tilt - tilt).degree();

  double x_speed = 0;
  if (delta_tilt > 0.0) {
    x_speed = keisan::map(delta_tilt, 0.0, 20.0, 0.0, pivot_min_x);
  } else {
    x_speed = keisan::map(delta_tilt, -20.0, 0.0, pivot_max_x, 0.);
  }

  double pan = (robot->pan + robot->pan_center).degree();

  double y_speed = 0.0;
  if (fabs(pan) < pivot_pan_range_a_speed) {
    if (delta_direction < 0.0) {
      y_speed = keisan::map(pan, 30.0, 0.0, pivot_max_ry * 0.5, pivot_max_ry);
    } else {
      y_speed = keisan::map(pan, -30.0, 0.0, pivot_max_ly * 0.5, pivot_max_ly);
    }
  }

  double a_speed = 0.0;
  if (fabs(pan) > pivot_pan_range_a_speed) {
    a_speed = keisan::map(pan, -pivot_pan_range_a_speed, pivot_pan_range_a_speed, pivot_max_a, -pivot_max_a);
  } else {
    a_speed = y_speed < 0.0
    ? keisan::map(delta_direction, -180.0, 0.0, -pivot_max_a * 0.9, -pivot_max_a)
    : keisan::map(delta_direction, 180.0, 0.0, pivot_max_a, pivot_max_a * 0.9);
  }

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
  auto pan = robot->get_pan() + robot->pan_center;
  auto tilt = robot->get_tilt() + robot->tilt_center;

  double delta_pan = std::abs((target_pan - pan).degree());
  double delta_tilt = std::abs((target_tilt - tilt).degree());
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  // x movement
  double delta_tilt_pan = delta_tilt + (std::abs(delta_pan) * 0.3);
  printf("delta tilt pan %.1f\n", delta_tilt_pan);

  double x_speed = 0.0;
  if (delta_tilt_pan > 3.0) {
    x_speed = keisan::map(delta_tilt_pan, 3.0, 20.0, position_min_x * 0.5, position_min_x);
  } else if (delta_tilt_pan < -3.0) {
    x_speed = keisan::map(delta_tilt_pan, -20.0, -3.0, position_max_x, position_max_x * 0.5);
  }

  // y movement
  double y_speed = 0.0;
  if (delta_pan < -position_min_delta_pan.degree()) {
    y_speed = keisan::map(delta_pan, -20.0, -position_min_delta_pan.degree(), position_max_ly, position_min_ly);
  } else if (delta_pan > position_min_delta_pan.degree()) {
    y_speed = keisan::map(delta_pan, position_min_delta_pan.degree(), 20.0, position_min_ry, position_max_ry);
  }

  // a movement
  double a_speed = keisan::map(delta_direction, -30.0, 30.0, position_max_a, -position_max_a);

  #if ITHAARO || UMARU || MIRU
  double smooth_ratio = 1.0;
  #else
  double smooth_ratio = 0.8;
  #endif

  a_speed = keisan::smooth(robot->a_speed, a_speed, smooth_ratio);
  x_speed = keisan::smooth(robot->x_speed, x_speed, smooth_ratio);
  y_speed = keisan::smooth(robot->y_speed, y_speed, smooth_ratio);

  robot->x_speed = x_speed;
  robot->y_speed = y_speed;
  robot->a_speed = a_speed;
  robot->aim_on = false;
  start();

  if (std::abs(delta_tilt) < position_min_delta_tilt.degree() && std::abs(delta_pan) < position_min_delta_pan.degree()) {
    printf("done by pan tilt\n");
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
  return position_kick_custom_pan_tilt(direction, right_kick_target_pan, left_kick_target_pan,
    (left_kick_target_tilt < right_kick_target_tilt ? left_kick_target_tilt : right_kick_target_tilt) - position_min_delta_tilt,
    (left_kick_target_tilt > right_kick_target_tilt ? left_kick_target_tilt : right_kick_target_tilt) + position_min_delta_tilt);
}

bool Locomotion::position_kick_custom_pan_tilt(const keisan::Angle<double> & direction, const keisan::Angle<double> & min_pan,
  const keisan::Angle<double> & max_pan, const keisan::Angle<double> & min_tilt, const keisan::Angle<double> & max_tilt)
{
  double pan = (robot->get_pan() + robot->pan_center).degree();
  double tilt = (robot->get_tilt() + robot->tilt_center).degree();
  double target_pan = (min_pan + max_pan).degree() / 2;
  double target_tilt = (min_tilt + max_tilt).degree() / 2;
  double delta_pan = target_pan - pan;
  double delta_tilt = target_tilt - tilt;

  double delta_direction = (direction - robot->orientation).normalize().degree();

  // x movement
  double delta_tilt_pan = delta_tilt + (std::abs(delta_pan) * 0.3);
  printf("delta tilt pan %.1f\n", delta_tilt_pan);

  double x_speed = 0.0;
  if (tilt != keisan::clamp(tilt, min_tilt.degree(), max_tilt.degree())) {
    if (delta_tilt_pan > 3.0) {
      x_speed = keisan::map(delta_tilt_pan, 3.0, 20.0, position_min_x * 0.5, position_min_x);
    } else if (delta_tilt_pan < -3.0) {
      x_speed = keisan::map(delta_tilt_pan, -20.0, -3.0, position_max_x, position_max_x * 0.5);
    }
  }

  // y movement
  double y_speed = 0.0;
  if (pan != keisan::clamp(pan, min_pan.degree(), max_pan.degree())) {
    if (delta_pan < -position_min_delta_pan.degree()) {
      y_speed = keisan::map(delta_pan, -20.0, -position_min_delta_pan.degree(), position_max_ly, position_min_ly);
    } else if (delta_pan > position_min_delta_pan.degree()) {
      y_speed = keisan::map(delta_pan, position_min_delta_pan.degree(), 20.0, position_min_ry, position_max_ry);
    }
  }

  // a movement
  double a_speed = keisan::map(delta_direction, -30.0, 30.0, position_max_a, -position_max_a);

  #if ITHAARO || UMARU || MIRU
  double smooth_ratio = 1.0;
  #else
  double smooth_ratio = 0.8;
  #endif

  a_speed = keisan::smooth(robot->a_speed, a_speed, smooth_ratio);
  x_speed = keisan::smooth(robot->x_speed, x_speed, smooth_ratio);
  y_speed = keisan::smooth(robot->y_speed, y_speed, smooth_ratio);

  printf("delta pan %.1f, delta tilt %.1f, delta direction %.1f\n", delta_pan, delta_tilt, delta_direction);

  if (tilt == keisan::clamp(tilt, min_tilt.degree(), max_tilt.degree()) && pan == keisan::clamp(pan, min_pan.degree(), max_pan.degree())) {
    printf("done by pan tilt\n");
    return true;
  }

  return false;
}

bool Locomotion::position_kick_range_pan_tilt(const keisan::Angle<double> & direction, bool precise_kick, bool left_kick, bool is_positioning_center)
{
  auto tilt = robot->get_tilt();
  auto pan = robot->get_pan();
  auto delta_direction = (direction - robot->orientation).normalize().degree();

  bool tilt_in_range = tilt > position_min_range_tilt && tilt < position_max_range_tilt;
  bool right_kick_in_range = pan > position_min_range_pan && pan < -position_center_right_range_pan;
  bool left_kick_in_range = pan > position_center_left_range_pan && pan < position_max_range_pan;
  bool pan_in_range = precise_kick ? (left_kick ? left_kick_in_range : right_kick_in_range) : (right_kick_in_range || left_kick_in_range);
  bool direction_in_range = std::fabs(delta_direction) < position_min_delta_direction.degree();

  if (tilt_in_range && pan_in_range && direction_in_range) {
    return true;
  }

    // y movement
  if (!precise_kick) left_kick = pan > 0.0_deg;
  auto target_pan = left_kick ? left_kick_target_pan : right_kick_target_pan;

  if (is_positioning_center) {
    target_pan = (left_kick) ? -position_center_right_range_pan : position_center_left_range_pan;
  }

  double delta_pan = (target_pan - pan).degree();
  double y_speed = 0.0;

  if (!pan_in_range) {
    if (delta_pan < -position_min_delta_pan.degree()) {
      y_speed = keisan::map(delta_pan, -20.0, -position_min_delta_pan.degree(), position_max_ly, position_min_ly);
    } else if (delta_pan > position_min_delta_pan.degree()) {
      y_speed = keisan::map(delta_pan, position_min_delta_pan.degree(), 20.0, position_min_ry, position_max_ry);
    }
  }

  auto target_tilt = left_kick ? left_kick_target_tilt : right_kick_target_tilt;
  double delta_tilt = (target_tilt - tilt).degree();

  bool pan_in_kick_range = pan > position_min_range_pan && pan < position_max_range_pan;
  double closest_delta_pan = pan_in_kick_range ? 0 : delta_pan;

  // x movement
  double delta_tilt_pan = delta_tilt + (closest_delta_pan * 0.3);
  printf("delta tilt pan %.1f\n", delta_tilt_pan);

  double x_speed = 0.0;
  if (!tilt_in_range) {
    if (delta_tilt_pan > 3.0) {
      x_speed = keisan::map(delta_tilt_pan, 3.0, 20.0, position_min_x * 0.5, position_min_x);
    } else if (delta_tilt_pan < -3.0) {
      x_speed = keisan::map(delta_tilt_pan, -20.0, -3.0, position_max_x, position_max_x * 0.5);
    }
  }

  // a movement
  double a_speed = 0;
  if (!direction_in_range) {
    a_speed = keisan::map(delta_direction, -30.0, 30.0, position_max_a, -position_max_a);
  }

  #if ITHAARO || UMARU || MIRU
  double smooth_ratio = 1.0;
  #else
  double smooth_ratio = 0.8;
  #endif

  robot->x_speed = keisan::smooth(robot->x_speed, x_speed, smooth_ratio);
  robot->y_speed = keisan::smooth(robot->y_speed, y_speed, smooth_ratio);
  robot->a_speed = keisan::smooth(robot->a_speed, a_speed, smooth_ratio);
  robot->aim_on = false;
  start();

  printf("delta pan %.1f, delta tilt %.1f, delta direction %.1f\n", delta_pan, delta_tilt, delta_direction);

  return false;
}

bool Locomotion::is_time_to_follow()
{
  return (robot->tilt - follow_min_tilt).degree() > 20.0;
}

bool Locomotion::pivot_fulfilled()
{
  return (robot->tilt - pivot_target_tilt).degree() < 0.0;
}

bool Locomotion::in_pan_kick_range()
{
  double pan = robot->pan.degree();
  return pan > right_kick_target_pan.degree() && pan < left_kick_target_pan.degree();
}

bool Locomotion::in_tilt_kick_range()
{
  double tilt = robot->tilt.degree();
  double min_target_tilt =
    std::min(left_kick_target_tilt.degree(), right_kick_target_tilt.degree());
  double max_target_tilt =
    std::max(left_kick_target_tilt.degree(), right_kick_target_tilt.degree());
  return tilt > min_target_tilt && tilt < max_target_tilt;
}

std::shared_ptr<Robot> Locomotion::get_robot() const
{
  return robot;
}

}  // namespace suiryoku
