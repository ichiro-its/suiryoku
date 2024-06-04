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
#include "nlohmann/json.hpp"
#include "suiryoku/locomotion/model/robot.hpp"

namespace suiryoku
{

class Locomotion
{
public:
  explicit Locomotion(std::shared_ptr<Robot> robot);

  void load_config(const std::string & path);
  void set_config(const nlohmann::json & json);

  bool walk_in_position();
  bool walk_in_position_until_stop();

  void move_backward(const keisan::Angle<double> & direction);
  bool move_backward_to(const keisan::Point2 & target);

  void move_forward(const keisan::Angle<double> & direction);
  bool move_forward_to(const keisan::Point2 & target);

  bool move_to_left_and_right(const keisan::Point2 & target);

  bool rotate_to_target(const keisan::Angle<double> & direction);
  bool rotate_to(const keisan::Angle<double> & direction, bool a_move_only);

  bool move_follow_head();
  bool move_follow_head(const keisan::Angle<double> & min_tilt);

  bool move_skew(const keisan::Angle<double> & direction);
  bool move_skew(const keisan::Angle<double> & direction, bool skew_left);

  bool dribble(const keisan::Angle<double> & direction);
  bool pivot(const keisan::Angle<double> & direction);
  bool pivot_inverse_a_move(const keisan::Angle<double> & direction);

  bool position_until(
    const keisan::Angle<double> & target_pan,
    const keisan::Angle<double> & target_tilt,
    const keisan::Angle<double> & direction);
  bool position_left_kick(const keisan::Angle<double> & direction);
  bool position_right_kick(const keisan::Angle<double> & direction);
  bool position_kick_custom_pan_tilt(const keisan::Angle<double> & direction, const keisan::Angle<double> & min_pan, 
                                     const keisan::Angle<double> & max_pan, const keisan::Angle<double> & min_tilt, 
                                     const keisan::Angle<double> & max_tilt);
  bool position_kick_general(const keisan::Angle<double> & direction);
  bool position_kick_range_pan_tilt(const keisan::Angle<double> & direction, bool precise_kick, bool left_kick);

  bool is_time_to_follow();
  bool pivot_fulfilled();
  bool in_pan_kick_range();
  bool in_tilt_kick_range();

  std::shared_ptr<Robot> get_robot() const;
  void update_move_amplitude(double x_amplitude, double y_amplitude);

  std::function<void()> stop;
  std::function<void()> start;

  std::string config_name;

  bool initial_pivot;

private:

  double move_min_x;
  double move_max_x;
  double move_max_y;
  double move_max_a;

  double backward_max_x;
  double backward_min_x;
  double backward_max_a;

  double rotate_max_a;
  double rotate_max_delta_direction;

  double follow_pan_ratio;
  double follow_max_x;
  double follow_min_x;
  double follow_max_a;
  double follow_l_a_offset;
  double follow_r_a_offset;
  bool follow_y_move;
  double follow_max_ry;
  double follow_min_ry;
  double follow_max_ly;
  double follow_min_ly;
  keisan::Angle<double> follow_min_tilt;

  double dribble_min_x;
  double dribble_max_x;
  double dribble_min_ly;
  double dribble_max_ly;
  double dribble_min_ry;
  double dribble_max_ry;
  double dribble_max_a;
  double dribble_pan_comp;

  keisan::Angle<double> pivot_target_tilt;
  double pivot_max_delta_direction;
  double pivot_pan_range_ratio;
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
  double position_in_belief;
  keisan::Angle<double> position_min_delta_tilt;
  keisan::Angle<double> position_min_delta_pan;
  keisan::Angle<double> position_min_delta_pan_tilt;
  keisan::Angle<double> position_min_delta_direction;
  keisan::Angle<double> position_prev_delta_pan;
  keisan::Angle<double> position_prev_delta_tilt;
  keisan::Angle<double> position_min_range_tilt;
  keisan::Angle<double> position_max_range_tilt;
  keisan::Angle<double> position_min_range_pan;
  keisan::Angle<double> position_center_range_pan;
  keisan::Angle<double> position_max_range_pan;

  double skew_max_x;
  double skew_max_a;
  double skew_tilt;
  double skew_pan_comp;
  double skew_delta_direction_comp;

  keisan::Angle<double> left_kick_target_pan;
  keisan::Angle<double> left_kick_target_tilt;

  keisan::Angle<double> right_kick_target_pan;
  keisan::Angle<double> right_kick_target_tilt;

  double right_min_ry;
  double right_max_ry;
  double right_max_a;
  double left_min_ly;
  double left_max_ly;
  double left_max_a;


  std::shared_ptr<Robot> robot;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION__PROCESS__LOCOMOTION_HPP_
