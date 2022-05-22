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

#ifndef SUIRYOKU__LOCOMOTION__MODEL__ROBOT_HPP_
#define SUIRYOKU__LOCOMOTION__MODEL__ROBOT_HPP_

#include <string>

#include "keisan/keisan.hpp"

namespace suiryoku
{

class Robot
{
public:
  Robot();

  const keisan::Angle<double> & get_pan() const;
  const keisan::Angle<double> & get_tilt() const;

  // member for getting
  keisan::Angle<double> orientation;
  keisan::Point2 position;

  bool is_walking;

  keisan::Angle<double> pan;
  keisan::Angle<double> pan_center;
  keisan::Angle<double> tilt;
  keisan::Angle<double> tilt_center;

  // member for setting
  double x_speed;
  double y_speed;
  double a_speed;
  bool aim_on;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION__MODEL__ROBOT_HPP_
