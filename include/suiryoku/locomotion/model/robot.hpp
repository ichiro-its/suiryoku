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

#include <random>
#include <string>

#include "keisan/keisan.hpp"
#include "keisan/hungarian.hpp"
#include "suiryoku/locomotion/model/field.hpp"

namespace suiryoku
{

struct ProjectedObject
{
  std::string label;
  keisan::Point3 position;
};

struct Particle
{
  keisan::Point2 position;
  keisan::Angle<double> orientation;
  double weight;
};

struct LandmarkGroup {
  std::vector<ProjectedObject>* projected;
  std::vector<keisan::Point2>* landmarks;
};

class Robot
{
public:
  Robot();

  keisan::Angle<double> get_pan() const;
  keisan::Angle<double> get_tilt() const;

  // localizations
  void localize();
  void reset_localization();
  void clear_projected_objects();
  void init_particles(const keisan::Point2 init_position);
  void resample_particles();
  void update_motion();
  void calculate_weight();
  void estimate_position();
  void print_particles();
  void print_estimate_position();
  void set_initial_localization(bool initial) { initial_localization = initial; }

  double calculate_landmark_cost(const keisan::Point2 & landmark, const keisan::Point2 & projected_object);
  double calculate_total_likelihood(const Particle & particle);
  double calculate_object_likelihood(const ProjectedObject & measurement,
                                     const keisan::Point2 & landmark,
                                     const Particle & particle);

  double get_sum_weight();
  keisan::Matrix<6, 6> calculate_cost_matrix(const Particle & particle,
                                             const std::vector<ProjectedObject> & projected_objects,
                                             const std::vector<keisan::Point2> & landmarks);

  std::vector<Particle> particles;
  keisan::Point2 estimated_position;

  int num_particles;
  double min_centered_particles_ratio;
  double reset_particles_threshold;
  double short_term_avg_ratio;
  double long_term_avg_ratio;
  double sigma_x;
  double sigma_y;

  bool use_localization;
  bool apply_localization;
  bool print_debug;

  // IPM
  std::vector<ProjectedObject> projected_X;
  std::vector<ProjectedObject> projected_L;
  std::vector<ProjectedObject> projected_T;
  std::vector<ProjectedObject> projected_goalpost;
  keisan::Point2 max_object_distance;
  int num_projected_objects;

  // member for getting
  bool is_calibrated;
  keisan::Angle<double> orientation;
  keisan::Angle<double> orientation_roll;
  keisan::Angle<double> orientation_pitch;
  keisan::Point2 position;
  keisan::Point2 delta_position;

  bool is_walking;

  keisan::Angle<double> pan;
  keisan::Angle<double> pan_center;
  keisan::Angle<double> tilt;
  keisan::Angle<double> tilt_center;

  double x_amplitude;
  double y_amplitude;
  double a_amplitude;

  // member for setting
  double x_speed;
  double y_speed;
  double a_speed;
  bool aim_on;

private:
  Field field;
  Particle* best_particle;
  std::vector<Particle*> center_particles;

  double xvar;
  double yvar;

  std::mt19937 rand_gen;
  keisan::Hungarian<6> hungarian;

  double weight_avg;
  double short_term_avg;
  double long_term_avg;
  double last_weight_avg;
  double prob;

  bool initial_localization;
  bool reset_particles;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION__MODEL__ROBOT_HPP_
