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

#include <unistd.h>

#include <aruku/walking.hpp>
#include <kansei/imu.hpp>
#include <suiryoku/locomotion.hpp>

#include <cmath>
#include <memory>

#include "common/algebra.h"

namespace suiryoku
{

Locomotion::Locomotion(
  std::shared_ptr<aruku::Walking> walking, std::shared_ptr<atama::Head> head,
  std::shared_ptr<kansei::Imu> imu)
{
  imu = imu;
  walking = walking;
  head = head;

  position_prev_delta_pan = 0.0;
  position_prev_delta_tilt = 0.0;
  position_in_position_belief = 0.0;

  move_finished = true;
  rotate_finished = true;
  pivot_finished = true;
}

bool Locomotion::walk_in_position()
{
  walking->X_MOVE_AMPLITUDE = 0;
  walking->Y_MOVE_AMPLITUDE = 0;
  walking->A_MOVE_AMPLITUDE = 0;
  walking->A_MOVE_AIM_ON = false;
  walking->start();

  bool in_position = true;
  in_position &= fabs(walking->get_mx_move_amplitude()) < 5;
  in_position &= fabs(walking->get_my_move_amplitude()) < 5;

  return in_position;
}

bool Locomotion::walk_in_position_until_stop()
{
  position_in_position_belief = 0.0;

  if (!walking->is_running()) {
    return true;
  }

  walking->X_MOVE_AMPLITUDE = 0;
  walking->Y_MOVE_AMPLITUDE = 0;
  walking->A_MOVE_AMPLITUDE = 0;
  walking->A_MOVE_AIM_ON = false;

  bool in_position = true;
  in_position &= (fabs(walking->get_mx_move_amplitude()) < 5.0);
  in_position &= (fabs(walking->get_my_move_amplitude()) < 5.0);

  if (!in_position) {
    return false;
  }

  walking->stop();
  while (walking->is_running()) {
    usleep(8000);
  }

  return !walking->is_running();
}

bool Locomotion::move_backward(float direction)
{
  float delta_direction = alg::deltaAngle(direction, imu->get_yaw());

  float x_speed = move_min_x;

  float a_speed = alg::mapValue(delta_direction, -15, 15, move_max_a, -move_max_a);
  if (fabs(delta_direction) > 15.0) {
    a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;
    x_speed = 0.0;
  }

  walking->X_MOVE_AMPLITUDE = x_speed;
  walking->Y_MOVE_AMPLITUDE = 0.0;
  walking->A_MOVE_AMPLITUDE = a_speed;
  walking->A_MOVE_AIM_ON = false;
  walking->start();

  return true;
}

bool Locomotion::move_to_target(float target_x, float target_y)
{
  float delta_x = (target_x - walking->POSITION_X);
  float delta_y = (target_y - walking->POSITION_Y);

  float target_distance = alg::distance(delta_x, delta_y);

  move_finished = (target_distance < ((move_finished) ? 40.0 : 30.0));
  if (move_finished) {
    return true;
  }

  float target_direction = alg::direction(delta_x, delta_y) * alg::rad2Deg();
  float delta_direction = alg::deltaAngle(target_direction, imu->get_yaw());

  float x_speed = alg::mapValue(fabs(move_max_a), 0, 15, 50, 40);
  if (target_distance < 100.0) {
    x_speed = alg::mapValue(target_distance, 0.0, 100.0, move_max_x * 0.25, move_max_x);
  }

  float a_speed = alg::mapValue(delta_direction, -10, 10, move_max_a, -move_max_a);
  float y_speed = 0;
  if (fabs(delta_direction) > 15.0) {
    y_speed = 0;
    a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;
    x_speed = 0.0;
  }

  walking->X_MOVE_AMPLITUDE = x_speed;
  walking->Y_MOVE_AMPLITUDE = y_speed;
  walking->A_MOVE_AMPLITUDE = a_speed;
  walking->A_MOVE_AIM_ON = false;
  walking->start();

  return move_finished;
}

bool Locomotion::rotate_to_target(float target_direction)
{
  float delta_direction = alg::deltaAngle(target_direction, imu->get_yaw());

  rotate_finished = (fabs(delta_direction) < ((rotate_finished) ? 20.0 : 15.0));
  if (rotate_finished) {
    return true;
  }

  if (rotate_finished) {
    if (fabs(delta_direction) < 10.0) {
      return true;
    } else {
      rotate_finished = false;
    }
  } else {
    if (fabs(delta_direction) < 5.0) {
      return rotate_finished = true;
    }
  }

  float y_speed = (delta_direction < 0.0) ? move_max_y : -move_max_y;
  float a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;

  walking->X_MOVE_AMPLITUDE = 0.0;
  walking->Y_MOVE_AMPLITUDE = y_speed;
  walking->A_MOVE_AMPLITUDE = a_speed;
  walking->A_MOVE_AIM_ON = false;
  walking->start();

  return false;
}

bool Locomotion::rotate_to_target(float target_direction, bool a_move_only)
{
  float delta_direction = alg::deltaAngle(target_direction, imu->get_yaw());

  rotate_finished = (fabs(delta_direction) < ((rotate_finished) ? 20.0 : 15.0));
  if (rotate_finished) {
    return true;
  }

  if (rotate_finished) {
    if (fabs(delta_direction) < move_max_a * 0.75) {
      return true;
    } else {
      rotate_finished = false;
    }
  } else {
    if (fabs(delta_direction) < move_max_a * 0.5) {
      return rotate_finished = true;
    }
  }

  float y_speed = 0.0;

  if (!(a_move_only)) {
    y_speed = (delta_direction < 0.0) ? move_max_y : -move_max_y;
  }
  float a_speed = (delta_direction < 0.0) ? move_max_a : -move_max_a;

  walking->X_MOVE_AMPLITUDE = 0.0;
  walking->Y_MOVE_AMPLITUDE = y_speed;
  walking->A_MOVE_AMPLITUDE = a_speed;
  walking->A_MOVE_AIM_ON = false;
  walking->start();

  return false;
}

bool Locomotion::move_follow_head(float min_tilt)
{
  float a_speed = alg::mapValue(head->get_pan_angle(), -10.0, 10.0, -follow_max_a, follow_max_a);

  float x_speed = alg::mapValue(fabs(a_speed), 0.0, follow_max_a, follow_max_x, 0.);
  x_speed = alg::mapValue(head->get_tilt_angle() - min_tilt, 10.0, 0.0, x_speed, 0.0);

  walking->X_MOVE_AMPLITUDE = x_speed;
  walking->Y_MOVE_AMPLITUDE = 0.0;
  walking->A_MOVE_AMPLITUDE = a_speed;
  walking->A_MOVE_AIM_ON = false;
  walking->start();

  return head->get_tilt_angle() < min_tilt;
}

bool Locomotion::dribble(float direction)
{
  bool is_dribble = true;

  float pan = head->get_pan_angle() + head->get_pan_center();
  float delta_direction = alg::deltaAngle(direction, imu->get_yaw());

  // x movement
  float x_speed = 0;
  if (fabs(pan) < 15.0) {
    x_speed = alg::mapValue(fabs(pan), 0.0, 15.0, dribble_max_x, 0.);
  } else {
    is_dribble = false;
    x_speed = alg::mapValue(fabs(pan), 15.0, 45.0, 0.0, dribble_min_x);
  }

  // y movement
  float y_speed = 0.0;
  if (pan < -6.0) {
    y_speed = alg::mapValue(pan, -25.0, -6.0, dribble_max_ry, dribble_min_ry);
  } else if (pan > 6.0) {
    y_speed = alg::mapValue(pan, 6.0, 25.0, dribble_min_ly, dribble_max_ly);
  }

  // a movement
  float a_speed = alg::mapValue(delta_direction, -15.0, 15.0, dribble_max_a, -dribble_max_a);

  walking->X_MOVE_AMPLITUDE = x_speed;
  walking->Y_MOVE_AMPLITUDE = y_speed;
  walking->A_MOVE_AMPLITUDE = a_speed;
  walking->A_MOVE_AIM_ON = false;
  walking->start();

  return is_dribble;
}

bool Locomotion::pivot(float direction)
{
  float delta_direction = alg::deltaAngle(direction, imu->get_yaw());

  pivot_finished = (fabs(delta_direction) < ((pivot_finished) ? 30.0 : 20.0));
  if (pivot_finished) {
    return true;
  }

  float pan = head->get_pan_angle() + head->get_pan_center();
  float tilt = head->get_tilt_angle() + head->get_tilt_center();

  float delta_tilt = pivot_target_tilt - tilt;

  // x_movement
  float x_speed = 0;
  if (delta_tilt > 0.0) {
    x_speed = alg::mapValue(delta_tilt, 0.0, 20.0, 0.0, pivot_min_x);
  } else {
    x_speed = alg::mapValue(delta_tilt, -20.0, 0.0, pivot_max_x, 0.);
  }

  // y movement
  float y_speed = (delta_direction < 0) ? pivot_max_ry : pivot_max_ly;

  // a movement
  float a_speed = alg::mapValue(pan, -10.0, 10.0, pivot_max_a, -pivot_max_a);

  walking->X_MOVE_AMPLITUDE = x_speed;
  walking->Y_MOVE_AMPLITUDE = y_speed;
  walking->A_MOVE_AMPLITUDE = a_speed;
  walking->A_MOVE_AIM_ON = true;
  walking->start();

  return false;
}

bool Locomotion::move_to_position_until_pan_tilt(
  float target_pan, float target_tilt,
  float direction)
{
  float pan = head->get_pan_angle() + head->get_pan_center() + head->get_pan_error() * 0.5;
  float tilt = head->get_tilt_angle() + head->get_tilt_center() + head->get_tilt_error() * 0.5;

  float delta_pan = target_pan - pan;
  float delta_tilt = target_tilt - tilt;

  float delta_direction = alg::deltaAngle(direction, walking->ORIENTATION);

  printf("pan err %.1f tilt err %.1f\n", head->get_pan_error(), head->get_tilt_error());
  printf("positioning %.2f %.2f (%.2f)\n", delta_pan, delta_tilt, position_in_position_belief);

  if (fabs(delta_direction) < 10.0) {
    if (fabs(delta_pan) < (3.0 + (3.0 * position_in_position_belief))) {
      position_in_position_belief += pow((0.24 * (1.0 - (fabs(delta_pan) / 6.0))), 2.0);
    } else if (fabs(delta_pan) <= fabs(position_prev_delta_pan) && fabs(delta_pan) < 6.0) {
      position_in_position_belief += pow((0.12 * (1.0 - (fabs(delta_pan) / 6.0))), 2.0);
    } else {
      position_in_position_belief -= 0.09;
    }

    if (fabs(delta_tilt) < (3.0 + (3.0 * position_in_position_belief))) {
      position_in_position_belief += pow((0.18 * (1.0 - (fabs(delta_tilt) / 6.0))), 2.0);
    } else if (fabs(delta_tilt) <= fabs(position_prev_delta_tilt) && fabs(delta_tilt) < 6.0) {
      position_in_position_belief += pow((0.9 * (1.0 - (fabs(delta_tilt) / 6.0))), 2.0);
    } else {
      position_in_position_belief -= 0.06;
    }
  } else {
    position_in_position_belief -= 0.10;
  }

  position_in_position_belief = alg::clampValue(position_in_position_belief, 0.0, 1.);

  position_prev_delta_pan = delta_pan;
  position_prev_delta_tilt = delta_tilt;

  // x movement
  float x_speed = 0.0;
  float delta_tilt_pan = delta_tilt + (fabs(delta_pan) * 0.5);
  printf("delta tilt pan %.1f\n", delta_tilt_pan);
  if (delta_tilt_pan > 3.0) {
    x_speed = alg::mapValue(delta_tilt_pan, 3.0, 20.0, position_min_x * 0.5, position_min_x);
  } else if (delta_tilt_pan < -3.0) {
    x_speed = alg::mapValue(delta_tilt_pan, -20.0, -3.0, position_max_x, position_max_x * 0.);
  }

  // y movement
  float y_speed = 0.0;
  if (delta_pan < -3.0) {
    y_speed = alg::mapValue(delta_pan, -20.0, -3.0, position_max_ly, position_min_ly);
  } else if (delta_pan > 3.0) {
    y_speed = alg::mapValue(delta_pan, 3.0, 20.0, position_min_ry, position_max_ry);
  }

  // a movement
  float a_speed = alg::mapValue(delta_direction, -15.0, 15.0, position_max_a, -position_max_a);

  walking->X_MOVE_AMPLITUDE = x_speed;
  walking->Y_MOVE_AMPLITUDE = y_speed;
  walking->A_MOVE_AMPLITUDE = a_speed;
  walking->A_MOVE_AIM_ON = false;
  walking->start();

  if (position_in_position_belief >= 1.0) {
    printf("done by in position belief\n");
    return true;
  }

  return false;
}

bool Locomotion::move_to_position_left_kick(float direction)
{
  return move_to_position_until_pan_tilt(left_kick_target_pan, left_kick_target_tilt, direction);
}

bool Locomotion::move_to_position_left_right(float direction)
{
  return move_to_position_until_pan_tilt(right_kick_target_pan, right_kick_target_tilt, direction);
}

}  // namespace suiryoku
