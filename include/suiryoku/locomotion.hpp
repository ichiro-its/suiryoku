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

#ifndef SUIRYOKU__LOCOMOTION_HPP_
#define SUIRYOKU__LOCOMOTION_HPP_

#include <aruku/walking.hpp>
#include <atama/head.hpp>
#include <kansei/imu.hpp>

#include <memory>

namespace suiryoku
{

class Locomotion
{
public:
  Locomotion(
    std::shared_ptr<aruku::Walking> walking, std::shared_ptr<atama::Head> head,
    std::shared_ptr<kansei::Imu> imu);

  float get_position_x() {return walking->POSITION_X;}
  float get_position_y() {return walking->POSITION_Y;}

  void set_position(float x, float y) {walking->POSITION_X = x; walking->POSITION_Y = y;}

  bool walk_in_position();
  bool walk_in_position_until_stop();

  bool move_backward(float direction);

  bool move_to_target(float target_x, float target_y);
  bool rotate_to_target(float target_direction);
  bool rotate_to_target(float target_direction, bool a_move_only);

  bool move_follow_head() {return move_follow_head(follow_min_tilt);}
  bool move_follow_head(float min_tilt);

  bool back_sprint(float target_x, float target_y);

  bool dribble(float direction);
  bool pivot(float direction);

  bool move_to_position_until_pan_tilt(float target_pan, float target_tilt, float direction);

  bool move_to_position_left_kick(float direction);
  bool move_to_position_left_right(float direction);

  void stop() {move_finished = rotate_finished = true;}
  bool is_finished() {return move_finished && rotate_finished;}

  void load_data(const std::string & path);

private:
  std::shared_ptr<kansei::Imu> imu;
  std::shared_ptr<aruku::Walking> walking;
  std::shared_ptr<atama::Head> head;

  float move_min_x;
  float move_max_x;
  float move_max_y;
  float move_max_a;

  float follow_max_x;
  float follow_max_a;
  float follow_min_tilt;

  float dribble_min_x;
  float dribble_max_x;
  float dribble_min_ly;
  float dribble_max_ly;
  float dribble_min_ry;
  float dribble_max_ry;
  float dribble_max_a;

  float pivot_target_tilt;
  float pivot_min_x;
  float pivot_max_x;
  float pivot_max_ly;
  float pivot_max_ry;
  float pivot_max_a;

  float position_min_x;
  float position_max_x;
  float position_min_ly;
  float position_max_ly;
  float position_min_ry;
  float position_max_ry;
  float position_max_a;
  float position_prev_delta_pan;
  float position_prev_delta_tilt;
  float position_in_position_belief;

  float left_kick_target_pan;
  float left_kick_target_tilt;

  float right_kick_target_pan;
  float right_kick_target_tilt;

  bool move_finished;
  bool rotate_finished;
  bool pivot_finished;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION_HPP_
