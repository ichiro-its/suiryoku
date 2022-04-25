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

#ifndef SUIRYOKU__LOCOMOTION__CONTROL__HELPER__PARAMETER_HPP_
#define SUIRYOKU__LOCOMOTION__CONTROL__HELPER__PARAMETER_HPP_

#include <string>

#include "keisan/keisan.hpp"
#include "suiryoku/locomotion/model/robot.hpp"

namespace suiryoku::control
{

class Parameter
{
public:
  static std::string walk_in_position();
  static std::string walk_in_position_until_stop();

  static std::string move_backward(const keisan::Angle<double> & direction);
  static std::string move_backward_to(double target_x, double target_y);

  static std::string move_forward_to(double target_x, double target_y);
  static std::string rotate_to(const keisan::Angle<double> & direction, bool a_move_only);

  static std::string move_follow_head();
  static std::string move_follow_head(double min_tilt);

  static std::string back_sprint(double target_x, double target_y);

  static std::string dribble(const keisan::Angle<double> & direction);
  static std::string pivot(const keisan::Angle<double> & direction);

  static std::string position_until(
    double target_pan, double target_tilt,
    const keisan::Angle<double> & direction);
  static std::string position_left_kick(const keisan::Angle<double> & direction);
  static std::string position_right_kick(const keisan::Angle<double> & direction);
};

}  // namespace suiryoku::control

#endif  // SUIRYOKU__LOCOMOTION__CONTROL__HELPER__PARAMETER_HPP_
