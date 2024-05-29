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
  std::ifstream file(path + config_name);
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
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "rotate") {
      try {
        val.at("max_a").get_to(rotate_max_a);
        val.at("max_delta_direction").get_to(rotate_max_delta_direction);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "backward") {
      try {
        val.at("min_x").get_to(backward_min_x);
        val.at("max_x").get_to(backward_max_x);
        val.at("max_a").get_to(backward_max_a);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "dribble") {
      try {
        val.at("max_x").get_to(dribble_max_x);
        val.at("min_x").get_to(dribble_min_x);
        val.at("max_ly").get_to(dribble_max_ly);
        val.at("min_ly").get_to(dribble_min_ly);
        val.at("max_ry").get_to(dribble_max_ry);
        val.at("min_ry").get_to(dribble_min_ry);
        val.at("max_a").get_to(dribble_max_a);
        val.at("pan_comp").get_to(dribble_pan_comp);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "follow") {
      try {
        val.at("pan_ratio").get_to(follow_pan_ratio);
        val.at("max_x").get_to(follow_max_x);
        val.at("min_x").get_to(follow_min_x);
        val.at("max_a").get_to(follow_max_a);
        val.at("l_a_offset").get_to(follow_l_a_offset);
        val.at("r_a_offset").get_to(follow_r_a_offset);
        val.at("y_move").get_to(follow_y_move);
        val.at("max_ry").get_to(follow_max_ry);
        val.at("min_ry").get_to(follow_min_ry);
        val.at("max_ly").get_to(follow_max_ly);
        val.at("min_ly").get_to(follow_min_ly);
        follow_min_tilt = keisan::make_degree(val.at("min_tilt_").get<double>());
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "skew") {
      try {
        val.at("max_x").get_to(skew_max_x);
        val.at("max_a").get_to(skew_max_a);
        val.at("tilt").get_to(skew_tilt);
        val.at("pan_comp").get_to(skew_pan_comp);
        val.at("delta_direction_comp").get_to(skew_delta_direction_comp);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
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
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "pivot") {
      try {
        val.at("min_x").get_to(pivot_min_x);
        val.at("max_x").get_to(pivot_max_x);
        val.at("max_ly").get_to(pivot_max_ly);
        val.at("max_ry").get_to(pivot_max_ry);
        val.at("max_a").get_to(pivot_max_a);
        val.at("max_delta_direction").get_to(pivot_max_delta_direction);
        val.at("pan_range_ratio").get_to(pivot_pan_range_ratio);
        pivot_target_tilt = keisan::make_degree(val.at("target_tilt").get<double>());
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
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
        position_min_delta_tilt = keisan::make_degree(val.at("min_delta_tilt").get<double>());
        position_min_delta_pan = keisan::make_degree(val.at("min_delta_pan").get<double>());
        position_min_delta_pan_tilt = keisan::make_degree(val.at("min_delta_pan_tilt").get<double>());
        position_min_delta_direction = keisan::make_degree(val.at("min_delta_direction").get<double>());
        position_min_range_tilt = keisan::make_degree(val.at("min_range_tilt").get<double>());
        position_max_range_tilt = keisan::make_degree(val.at("max_range_tilt").get<double>());
        position_min_range_pan = keisan::make_degree(val.at("min_range_pan").get<double>());
        position_max_range_pan = keisan::make_degree(val.at("max_range_pan").get<double>());
        position_center_range_pan = keisan::make_degree(val.at("center_range_pan").get<double>());
        min_dynamic_range_pan = keisan::make_degree(val.at("min_dynamic_range_pan").get<double>());
        max_dynamic_range_pan = keisan::make_degree(val.at("max_dynamic_range_pan").get<double>());
        min_dynamic_range_tilt = keisan::make_degree(val.at("min_dynamic_range_tilt").get<double>());
        max_dynamic_range_tilt = keisan::make_degree(val.at("max_dynamic_range_tilt").get<double>());
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "left_kick") {
      try {
        left_kick_target_pan = keisan::make_degree(val.at("target_pan").get<double>());
        left_kick_target_tilt =
          keisan::make_degree(val.at("target_tilt").get<double>());
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "right_kick") {
      try {
        right_kick_target_pan = keisan::make_degree(val.at("target_pan").get<double>());
        right_kick_target_tilt =
          keisan::make_degree(val.at("target_tilt").get<double>());
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
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
    walk_in_position();
    return true;
  }

  auto direction = keisan::make_degree(atan2(delta_x, delta_y)).normalize() + 180.0_deg;
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

  auto direction = keisan::make_degree(atan2(delta_x, delta_y)).normalize();
  auto delta_direction = (direction - robot->orientation).normalize().degree();

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
  double delta_direction = (direction - current_direction).normalize().degree();

  if (delta_direction < skew_delta_direction_comp &&
    std::abs((direction - robot->orientation).normalize().degree()) < 10.0)
  {
    return true;
  }
  double min_skew_tilt = skew_tilt + 10.0;
  double max_skew_tilt = std::min(skew_tilt - 15.0, -60.0);
  double pan_comp = keisan::map(
    robot->tilt.degree(), min_skew_tilt, max_skew_tilt, 0.0, skew_pan_comp);
  auto target_direction = current_direction.degree();
  if (skew_left) {
    target_direction -= pan_comp;
  } else {
    target_direction += pan_comp;
  }
  auto target_direction_deg = keisan::make_degree(target_direction).normalize();

  double delta_target_skew_direction =
    (target_direction_deg - robot->orientation).normalize().degree();

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
  } else {
    move_follow_head();
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

  #if ITHAARO || UMARU || MIRU
  double pan_range_a_speed = 15.0;
  #else
  double pan_range_a_speed = 10.0;
  #endif

  double a_speed = 0.0;
  if (fabs(pan) > pan_range_a_speed * pivot_pan_range_ratio) {
    a_speed = keisan::map(pan, -pan_range_a_speed, pan_range_a_speed, pivot_max_a, -pivot_max_a);
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
  if (!tilt == keisan::clamp(tilt, min_tilt.degree(), max_tilt.degree())) {
    if (delta_tilt_pan > 3.0) {
      x_speed = keisan::map(delta_tilt_pan, 3.0, 20.0, position_min_x * 0.5, position_min_x);
    } else if (delta_tilt_pan < -3.0) {
      x_speed = keisan::map(delta_tilt_pan, -20.0, -3.0, position_max_x, position_max_x * 0.5);
    }
  }

  // y movement
  double y_speed = 0.0;
  if (!pan == keisan::clamp(pan, min_pan.degree(), max_pan.degree())) {
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

bool Locomotion::position_kick_range_pan_tilt(const keisan::Angle<double> & direction, bool precise_kick, bool left_kick, bool dynamic_kick)
{
  if (dynamic_kick) {
    position_max_range_pan = max_dynamic_range_pan;
    position_min_range_pan = min_dynamic_range_pan;
    position_min_range_tilt = min_dynamic_range_tilt;
    position_max_range_tilt = max_dynamic_range_tilt;
  }

  auto tilt = robot->get_tilt();
  auto pan = robot->get_pan();

  if (dynamic_kick && pan < keisan::make_degree(0.0))
    mapped_tilt = keisan::exponentialmap(pan.degree(), 0.0, position_min_range_pan.degree(), position_min_range_tilt.degree(), position_max_range_tilt.degree());
  else if (dynamic_kick && pan >= keisan::make_degree(0.0))
    mapped_tilt = keisan::exponentialmap(pan.degree(), 0.0, position_max_range_pan.degree(), position_min_range_tilt.degree(), position_max_range_tilt.degree());

  keisan::Angle<double> min_tilt = (dynamic_kick) ? keisan::make_degree(mapped_tilt) - position_min_range_tilt : position_min_range_tilt;
  keisan::Angle<double> max_tilt = (dynamic_kick) ? keisan::make_degree(mapped_tilt) + position_min_range_tilt : position_max_range_tilt;
  printf("tilt range: %.2f to %.2f\n", min_tilt.degree(), max_tilt.degree());
  printf("pan range: %.2f to %.2f\n", position_min_range_pan, position_max_range_pan);

  bool tilt_in_range = tilt > min_tilt && tilt < max_tilt;
  bool right_kick_in_range = pan > position_min_range_pan && pan < -position_center_range_pan;
  bool left_kick_in_range = pan > position_center_range_pan && pan < position_max_range_pan;
  bool pan_in_range = precise_kick ? (left_kick ? left_kick_in_range : right_kick_in_range) : (right_kick_in_range || left_kick_in_range);

  if (tilt_in_range && pan_in_range) {
    return true;
  }

  auto target_tilt = 0.5 * (position_min_range_tilt + position_max_range_tilt);
  double delta_tilt = (target_tilt - tilt).degree();

  bool pan_in_kick_range = pan > position_min_range_pan && pan < position_max_range_pan;
  double closest_delta_pan = pan_in_kick_range ? 0 : std::min(std::abs((left_kick_target_pan - pan).degree()), std::abs((right_kick_target_pan - pan).degree()));

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
  double delta_direction = (direction - robot->orientation).normalize().degree();
  double a_speed = 0;

  bool direction_in_range = std::fabs(delta_direction) < position_min_delta_direction.degree();
  if (!direction_in_range) {
    a_speed = keisan::map(delta_direction, -30.0, 30.0, position_max_a, -position_max_a);
  }

  // y movement
  auto target_pan = pan > 0.0_deg ? 0.5 * (position_center_range_pan + position_max_range_pan) : 0.5 * (-position_center_range_pan + position_min_range_pan);
  if (precise_kick) {
    target_pan = left_kick ? 0.5 * (position_center_range_pan + position_max_range_pan) : 0.5 * (-position_center_range_pan + position_min_range_pan);
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

  if (tilt_in_range && pan_in_range) {
    printf("done by range pan tilt\n");
    return true;
  }

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
