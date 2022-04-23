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

#include <string>

#include "suiryoku/locomotion/control/helper/parameter.hpp"

#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"

namespace suiryoku::control
{

std::string Parameter::walk_in_position()
{
  nlohmann::json param = {
    {"until_stop", false},
  };

  return param.dump();
}

std::string Parameter::walk_in_position_until_stop()
{
  nlohmann::json param = {
    {"until_stop", true},
  };

  return param.dump();
}

std::string Parameter::move_backward(const keisan::Angle<double> & direction)
{
  nlohmann::json param = {
    {"direction", direction.degree()},
  };

  return param.dump();
}

std::string Parameter::move_backward_to(double target_x, double target_y)
{
  nlohmann::json param = {
    {
      "target", {
        {"x", target_x},
        {"y", target_y},
      }
    }
  };

  return param.dump();
}

std::string Parameter::move_forward_to(double target_x, double target_y)
{
  nlohmann::json param = {
    {
      "target", {
        {"x", target_x},
        {"y", target_y},
      }
    }
  };

  return param.dump();
}

std::string Parameter::rotate_to(const keisan::Angle<double> & direction, bool a_move_only)
{
  nlohmann::json param = {
    {"direction", direction.degree()},
    {"a_move_only", a_move_only},
  };

  return param.dump();
}

std::string Parameter::move_follow_head(double min_tilt)
{
  nlohmann::json param = {
    {"min_tilt", min_tilt},
  };

  return param.dump();
}

std::string Parameter::dribble(const keisan::Angle<double> & direction)
{
  nlohmann::json param = {
    {"direction", direction.degree()},
  };

  return param.dump();
}

std::string Parameter::pivot(const keisan::Angle<double> & direction)
{
  nlohmann::json param = {
    {"direction", direction.degree()},
  };

  return param.dump();
}

std::string Parameter::position_until(double target_pan, double target_tilt,
  const keisan::Angle<double> & direction)
{
  nlohmann::json param = {
    {"direction", direction.degree()},
    {
      "target", {
        {"pan", target_pan},
        {"tilt", target_tilt},
      }
    }
  };

  return param.dump();
}

std::string Parameter::position_left_kick(const keisan::Angle<double> & direction)
{
  nlohmann::json param = {
    {"direction", direction.degree()},
    {"is_left_kick", true},
  };

  return param.dump();
}

std::string Parameter::position_right_kick(const keisan::Angle<double> & direction)
{
  nlohmann::json param = {
    {"direction", direction.degree()},
    {"is_right_kick", true},
  };

  return param.dump();
}

}  // namespace suiryoku::control
