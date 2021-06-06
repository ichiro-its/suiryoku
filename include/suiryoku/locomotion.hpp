#ifndef SUIRYOKU__LOCOMOTION_HPP_
#define SUIRYOKU__LOCOMOTION_HPP_

#include "common/config.h"

#include <aruku/walking.hpp>

#include <memory>

namespace suiryoku
{

class Locomotion
{

public:
  Locomotion();
  ~Locomotion();

  float get_position_x() { return walking->POSITION_X; }
  float get_position_y() { return walking->POSITION_Y; }

  void set_position(float x, float y) { walking->POSITION_X = x; walking->POSITION_Y = y; }

  bool walk_in_position();
  bool walk_in_position_until_stop();

  bool move_backward(float direction);

  bool move_to_target(float target_x, float target_y);
  bool rotate_to_target(float target_direction);
  bool rotate_to_target(float target_direction, bool a_move_only);

  bool move_follow_head() { return move_follow_head(follow_min_tilt); }
  bool move_follow_head(float min_tilt);

  bool back_sprint(float target_x, float target_y);

  bool dribble(float direction);
  bool pivot(float direction);

  bool move_to_position_until_pan_tilt(float target_pan, float target_tilt, float direction);

  bool move_to_position_left_kick(float direction);
  bool move_to_position_left_right(float direction);

  void stop() { move_finished = rotate_finished = true; }
  bool is_finished() { return move_finished && rotate_finished; }

private:
  // MPU *mpu;
  std::shared_ptr<aruku::Walking> walking;
  // Robot::Head *head;

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
