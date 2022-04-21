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

#ifndef SUIRYOKU__LOCOMOTION__PROCESS__LOCOMOTION_HPP_
#define SUIRYOKU__LOCOMOTION__PROCESS__LOCOMOTION_HPP_

#include <memory>
#include <string>

#include "keisan/keisan.hpp"
#include "suiryoku/locomotion/model/robot.hpp"

namespace suiryoku
{

class Locomotion
{
public:
  explicit Locomotion(std::shared_ptr<Robot> robot);

  void load_config(const std::string & path);

  bool walk_in_position();
  bool walk_in_position_until_stop();

  void move_backward(const keisan::Angle<double> & direction);
  bool move_backward_to(double target_x, double target_y);

  bool move_to(double target_x, double target_y);
  bool rotate_to(const keisan::Angle<double> & direction);
  bool rotate_to(const keisan::Angle<double> & direction, bool a_move_only);

  bool move_follow_head();
  bool move_follow_head(double min_tilt);

  bool back_sprint(double target_x, double target_y);

  bool dribble(const keisan::Angle<double> & direction);
  bool pivot(const keisan::Angle<double> & direction);

  bool move_to_position_until_pan_tilt(
    double target_pan, double target_tilt,
    const keisan::Angle<double> & direction);
  bool move_to_position_left_kick(const keisan::Angle<double> & direction);
  bool move_to_position_right_kick(const keisan::Angle<double> & direction);

private:
  double move_min_x;
  double move_max_x;
  double move_max_y;
  double move_max_a;

  double follow_max_x;
  double follow_max_a;
  double follow_min_tilt;

  double dribble_min_x;
  double dribble_max_x;
  double dribble_min_ly;
  double dribble_max_ly;
  double dribble_min_ry;
  double dribble_max_ry;
  double dribble_max_a;

  double pivot_target_tilt;
  double pivot_min_x;
  double pivot_max_x;
  double pivot_max_ly;
  double pivot_max_ry;
  double pivot_max_a;

  double position_min_x;
  double position_max_x;
  double position_min_ly;
  double position_max_ly;
  double position_min_ry;
  double position_max_ry;
  double position_max_a;
  double position_prev_delta_pan;
  double position_prev_delta_tilt;
  double position_in_position_belief;

  double left_kick_target_pan;
  double left_kick_target_tilt;

  double right_kick_target_pan;
  double right_kick_target_tilt;

  std::shared_ptr<Robot> robot;
  double x_speed_amplitude;
  double y_speed_amplitude;

  std::function<void()> stop_walking;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION__PROCESS__LOCOMOTION_HPP_
