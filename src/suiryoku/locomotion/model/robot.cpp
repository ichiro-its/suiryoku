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
#include <string>

#include "suiryoku/locomotion/model/robot.hpp"

#include "keisan/keisan.hpp"

using keisan::literals::operator""_deg;

namespace suiryoku
{

Robot::Robot()
: pan(0_deg), tilt(0_deg), pan_center(0_deg), tilt_center(0_deg), x_speed(0.0),
  y_speed(0.0), a_speed(0.0), aim_on(false), is_walking(false), position(0.0, 0.0),
  orientation(0_deg), orientation_roll(0_deg), orientation_pitch(0_deg),
  x_amplitude(0.0), y_amplitude(0.0), a_amplitude(0.0), is_calibrated(false),
  use_localization(false), apply_localization(false), print_debug(false),
  num_particles(500), min_centered_particles_ratio(0.3), reset_particles_threshold(0.3),
  short_term_avg_ratio(0.1), long_term_avg_ratio(0.001), sigma_x(5.0), sigma_y(5.0),
  xvar(10.0), yvar(10.0), best_particle(nullptr), rand_gen(std::random_device{}()),
  weight_avg(0.0), short_term_avg(0.0), long_term_avg(0.0), last_weight_avg(0.0),
  estimated_position(-1.0, -1.0), initial_localization(true), reset_particles(false),
  num_projected_objects(0), max_object_distance(300.0, 300.0), pending_accept_count(0)
{
}

keisan::Angle<double> Robot::get_pan() const
{
  return pan + pan_center;
}

keisan::Angle<double> Robot::get_tilt() const
{
  return tilt + tilt_center;
}

void Robot::localize()
{
  if (!use_localization) {
    return;
  }

  if (initial_localization) {
    init_particles(keisan::Point2(-1, -1));
  } else {
    update_motion();
  }

  bool evaluate_particles = num_projected_objects > 0;
  evaluate_particles &= fabs(orientation_roll.degree()) <= 30;
  evaluate_particles &= fabs(orientation_pitch.degree()) <= 30;
  evaluate_particles &= fabs(get_pan().degree()) <= 90;
  evaluate_particles &= get_tilt().degree() > -50;

  if (evaluate_particles) {
    weight_avg = 0.0;
    calculate_weight();
    if (weight_avg <= 0 || std::isnan(weight_avg)) {
      weight_avg = last_weight_avg;
    } else {
      if (initial_localization) {
        short_term_avg = weight_avg;
        long_term_avg = weight_avg;
      }
      last_weight_avg = weight_avg;
    }

    if (!std::isnan(weight_avg)) {
      short_term_avg += short_term_avg_ratio * (weight_avg - short_term_avg);
      long_term_avg += long_term_avg_ratio * (weight_avg - long_term_avg);
    }

    resample_particles();
  }
  estimate_position();

  clear_projected_objects();
}

void Robot::reset_localization() {
  apply_localization = true;
  initial_localization = true;
  particles.clear();
  center_particles.clear();
  clear_projected_objects();
  estimated_position = keisan::Point2(-1.0, -1.0);
  best_particle = nullptr;
  weight_avg = 0.0;
  short_term_avg = 0.0;
  long_term_avg = 0.0;
  last_weight_avg = 0.0;
  reset_particles = false;
  prob = 0.0;
  pending_accept_count = 0;
  pending_estimated_position = keisan::Point2(-1.0, -1.0);

  if (use_localization) {
    localize();
  }
}

void Robot::clear_projected_objects()
{
  projected_X.clear();
  projected_L.clear();
  projected_T.clear();
  projected_goalpost.clear();
  num_projected_objects = 0;
}

void Robot::init_particles(const keisan::Point2 init_position)
{
  double uniform_weight = 1.0 / num_particles;
  particles.clear();
  particles.resize(num_particles);
  initial_localization = true;
  estimated_position = init_position;

  if (init_position.x == -1 && init_position.y == -1) {
    std::uniform_int_distribution<int> xrg(0, 900);
    std::uniform_int_distribution<int> yrg(0, 600);

    for (int i = 0; i < num_particles; ++i) {
      particles[i].position = keisan::Point2(xrg(rand_gen), yrg(rand_gen));
      particles[i].orientation = orientation;
      particles[i].weight = uniform_weight;
    }
  } else {
    std::normal_distribution<double> xrg(init_position.x, 50);
    std::normal_distribution<double> yrg(init_position.y, 50);

    for (int i = 0; i < num_particles; ++i) {
      particles[i].position = keisan::Point2(xrg(rand_gen), yrg(rand_gen));
      particles[i].orientation = orientation;
      particles[i].weight = uniform_weight;
    }
  }
}

void Robot::resample_particles()
{
  std::vector<Particle> new_particles(num_particles);
  std::uniform_int_distribution<int> rand_index(0, num_particles - 1);
  int index = rand_index(rand_gen);

  double beta = 0.0;
  double max_weight = 0.0;
  double sum_weight = get_sum_weight();
  prob = std::max(0.0, 1.0 - short_term_avg / long_term_avg);
  // reset_particles = prob > reset_particles_threshold;

  // find the best particle
  if (sum_weight > 0) {
    for (auto & p : particles) {
      if (p.weight > max_weight) {
        max_weight = p.weight;
        best_particle = &p;
      }
    }
  }

  // determine resample interval area
  int interval_x[2] = {10, 890};
  int interval_y[2] = {10, 590};

  bool resample_nearby = num_projected_objects == 1;
  resample_nearby |= fabs(get_pan().degree()) > 80;
  resample_nearby |= get_tilt().degree() < -40;
  resample_nearby |= reset_particles;

  if (resample_nearby) {
    interval_x[0] = position.x - 50;
    interval_x[1] = position.x + 50;
    interval_y[0] = position.y - 50;
    interval_y[1] = position.y + 50;
    reset_particles = false;
    prob = std::min(0.05, prob);
  }

  // resample particles
  std::uniform_real_distribution<double> rand_prob(0.0, 1.0);
  std::uniform_real_distribution<double> rand_beta(0.0, 2.0 * max_weight);
  std::uniform_int_distribution<int> xrg(interval_x[0], interval_x[1]);
  std::uniform_int_distribution<int> yrg(interval_y[0], interval_y[1]);

  // adaptive noise
  double max_sigma = 3.0;
  double min_sigma = 0.1;

  double sigma_factor = 1.0 - prob;
  double resample_sigma_x = min_sigma + sigma_factor * (max_sigma - min_sigma);
  double resample_sigma_y = min_sigma + sigma_factor * (max_sigma - min_sigma);

  std::normal_distribution<double> noise_x(0.0, resample_sigma_x);
  std::normal_distribution<double> noise_y(0.0, resample_sigma_y);

  for (int i = 0; i < num_particles; ++i) {
    if (rand_prob(rand_gen) < prob) {
      new_particles[i].position = keisan::Point2(xrg(rand_gen), yrg(rand_gen));
      new_particles[i].orientation = orientation;
      new_particles[i].weight = 0.0;
    } else {
      beta += rand_beta(rand_gen);

      while (beta > particles[index].weight) {
        beta -= particles[index].weight;
        index = (index + 1) % num_particles;
      }
      new_particles[i] = particles[index];

      new_particles[i].position.x += noise_x(rand_gen);
      new_particles[i].position.y += noise_y(rand_gen);
    }

    // check is particle out of bound
    if (new_particles[i].position.x < 0.0 || new_particles[i].position.x > 900.0 ||
        new_particles[i].position.y < 0.0 || new_particles[i].position.y > 600.0) {
      new_particles[i].position = keisan::Point2(xrg(rand_gen), yrg(rand_gen));
      new_particles[i].orientation = orientation;
      new_particles[i].weight = 0.0;
    }
  }

  particles = new_particles;
}

void Robot::update_motion()
{
  static std::normal_distribution<double> xgen(0.0, xvar), ygen(0.0, yvar);
  bool update_with_noise = num_projected_objects > 0;
  update_with_noise &= fabs(orientation_roll.degree()) <= 30;
  update_with_noise &= fabs(orientation_pitch.degree()) <= 30;
  update_with_noise &= fabs(get_pan().degree()) <= 90;
  update_with_noise &= get_tilt().degree() > -50;

  if (update_with_noise || true) {
    for (auto & p : particles) {
      double static_noise_x = xgen(rand_gen) / 10.0;
      double static_noise_y = ygen(rand_gen) / 10.0;
      double dynamic_noise_x = fabs(delta_position.x) * xgen(rand_gen) / 5.0;
      double dynamic_noise_y = fabs(delta_position.y) * ygen(rand_gen) / 5.0;
      double x_yterm = fabs(delta_position.y) * xgen(rand_gen) / 30.0;
      double y_xterm = fabs(delta_position.x) * ygen(rand_gen) / 30.0;

      p.position.x += delta_position.x + static_noise_x + dynamic_noise_x + x_yterm;
      p.position.y += delta_position.y + static_noise_y + dynamic_noise_y + y_xterm;
      p.orientation = orientation;

      p.position.x = keisan::clamp(p.position.x, 0.0, 900.0);
      p.position.y = keisan::clamp(p.position.y, 0.0, 600.0);
    }
  } else {
    for (auto & p : particles) {
      p.position.x += delta_position.x;
      p.position.y += delta_position.y;
      p.orientation = orientation;

      p.position.x = keisan::clamp(p.position.x, 0.0, 900.0);
      p.position.y = keisan::clamp(p.position.y, 0.0, 600.0);
    }
  }

  estimated_position += delta_position;
}

double Robot::get_sum_weight()
{
  double sum_weight = 0.0;
  for (const auto & p : particles) {
    sum_weight += p.weight;
  }
  return sum_weight;
}

void Robot::calculate_weight()
{
  double sum_weight = 0.0;

  for (auto & p : particles) {
    p.weight = calculate_total_likelihood(p);
    sum_weight += p.weight;
  }

  if (!std::isnan(sum_weight) && sum_weight > 0) {
    weight_avg = sum_weight / num_particles;
    for (auto & p : particles) {
      p.weight /= sum_weight;
    }
  } else {
    weight_avg = 0.0;
  }
}

double Robot::calculate_distance(const keisan::Point2 & point_1, const keisan::Point2 & point_2) {
  double dx = point_1.x - point_2.x;
  double dy = point_1.y - point_2.y;

  return sqrt(dx * dx + dy * dy);
}

keisan::Matrix<6, 6> Robot::calculate_cost_matrix(
  const Particle & particle, const std::vector<ProjectedObject> & projected_objects,
  const std::vector<keisan::Point2> & landmarks) {
  auto cost_matrix = keisan::Matrix<6, 6>::infinite();
  int num_projected_objects = projected_objects.size();
  int num_landmarks = landmarks.size();

  for (int i = 0; i < num_projected_objects; ++i) {
    for (int j = 0; j < num_landmarks; ++j) {
      double dx = projected_objects[i].position.x * 100;
      double dy = projected_objects[i].position.y * 100;

      double x_rot = dx * particle.orientation.cos() + dy * particle.orientation.sin();
      double y_rot = dx * particle.orientation.sin() - dy * particle.orientation.cos();

      double relative_position_x = particle.position.x + x_rot;
      double relative_position_y = particle.position.y + y_rot;

      keisan::Point2 projected_object(relative_position_x, relative_position_y);

      cost_matrix[i][j] = calculate_distance(landmarks[j], projected_object);
    }
  }

  return cost_matrix;
}

double Robot::calculate_total_likelihood(const Particle & particle) {
  double total_likelihood = 1.0;

  std::vector<LandmarkGroup> landmark_groups = {
    {&projected_X, &field.landmarks_X},
    {&projected_L, &field.landmarks_L},
    {&projected_T, &field.landmarks_T},
    {&projected_goalpost, &field.landmarks_goalpost}
  };

  for (const auto & group : landmark_groups) {
    if (group.projected->empty() || group.landmarks->empty()) {
      continue;
    }

    auto cost_matrix = calculate_cost_matrix(particle, *group.projected, *group.landmarks);
    auto result = hungarian.solve(cost_matrix, group.landmarks->size());

    for (int i = 0; i < group.projected->size(); ++i) {
      for (int j = 0; j < group.landmarks->size(); ++j) {
        if (result[i][j] == 1) {
          total_likelihood *= calculate_object_likelihood((*group.projected)[i], (*group.landmarks)[j], particle);
        }
      }
    }
  }

  return total_likelihood;
}

double Robot::calculate_object_likelihood(
  const ProjectedObject & measurement, const keisan::Point2 & landmark, const Particle & particle) {

  double dx = measurement.position.x * 100;
  double dy = measurement.position.y * 100;

  double x_rot = dx * particle.orientation.cos() + dy * particle.orientation.sin();
  double y_rot = dx * particle.orientation.sin() - dy * particle.orientation.cos();

  double relative_position_x = particle.position.x + x_rot;
  double relative_position_y = particle.position.y + y_rot;

  double dist = sqrt(dx * dx + dy * dy);
  double dist_factor = keisan::map(dist, 150.0, 500.0, 1.0, 0.9);

  double exponent =
    -0.5 *
    (pow((landmark.x - relative_position_x), 2) / pow(sigma_x, 2) +
    pow((landmark.y - relative_position_y), 2) / pow(sigma_y, 2));

  return exp(exponent) / (2 * M_PI * sigma_x * sigma_y) * dist_factor;
}

void Robot::estimate_position() {
  if (best_particle == nullptr || num_projected_objects == 0) {
    return;
  }

  keisan::Point2 sum_position = {0.0, 0.0};
  int centered_particles = 0;
  center_particles.clear();

  // Find numbers of particles closed to the best particle
  for (int i = 0; i < num_particles; ++i) {
    double distance = calculate_distance(particles[i].position, best_particle->position);

    if (distance < 25.0) {
      centered_particles++;
      sum_position.x += particles[i].position.x;
      sum_position.y += particles[i].position.y;

      center_particles.push_back(&particles[i]);
    }
  }

  // Use mean of centered particles position if more than threshold are centered
  if (centered_particles > min_centered_particles_ratio * num_particles) {
    auto last_estimated_position = estimated_position;
    estimated_position.x = sum_position.x / centered_particles;
    estimated_position.y = sum_position.y / centered_particles;

    estimated_position.x = keisan::clamp(estimated_position.x, 0.0, 900.0);
    estimated_position.y = keisan::clamp(estimated_position.y, 0.0, 600.0);

    double jump_distance = calculate_distance(estimated_position, position);

    if (jump_distance <= 100.0 || initial_localization) {
      position = estimated_position;
      apply_localization = true;
      if (initial_localization) {
        initial_localization = false;
      }
    } else {
      double delta_estimate = calculate_distance(pending_estimated_position, estimated_position);
      if (delta_estimate < 20.0) {
        pending_accept_count++;
      } else {
        pending_accept_count = 0;
      }
      pending_estimated_position = estimated_position;

      if (pending_accept_count >= 10) {
        position = estimated_position;
        apply_localization = true;
        pending_accept_count = 0;
      } else {
        estimated_position = last_estimated_position;
        apply_localization = false;
        reset_particles = true;
      }
    }
  }
}

void Robot::print_particles() {
  if (!print_debug) {
    return;
  }

  printf("Particles num: %d\n", particles.size());
  printf("Prob: %.2f | Threshold: %.2f | Above threshold: %s\n",
          prob,
          reset_particles_threshold,
          (prob > reset_particles_threshold) ? "true" : "false");
  printf("Reset particles: %s\n", reset_particles ? "true" : "false");
  printf("Short term avg: %.8f\n", short_term_avg);
  printf("Long term avg: %.8f\n", long_term_avg);
  printf("Last weight avg: %.8f\n", last_weight_avg);
  printf("Weight avg: %.8f\n", weight_avg);
  printf("Sum weights: %.8f\n", get_sum_weight());

  printf("========================================\n");
  printf("Minimal Centered Particles: %.0f\n", min_centered_particles_ratio * num_particles);

  printf("Centered particles: %d\n", center_particles.size());
  for (int i = 0; i < center_particles.size(); ++i) {
    printf("Centered Particle %d  weight: %.5f  [%.2f, %.2f, %.2f]\n",
            i,
            center_particles[i]->weight,
            center_particles[i]->position.x,
            center_particles[i]->position.y,
            center_particles[i]->orientation.degree());
    if (i >= 10) {
      printf("... and %d more\n", center_particles.size() - 10);
      break;
    }
  }

  if (best_particle == nullptr) {
    printf("Best particle: NULL\n");
  } else {
    printf("Best particle: %.5f  [%.2f, %.2f, %.2f]\n",
            best_particle->weight,
            best_particle->position.x,
            best_particle->position.y,
            best_particle->orientation.degree());
  }

  printf("Num objects: %d\n", num_projected_objects);
  printf("Num particles: %d\n", num_particles);
  printf("Projected objects: %d\n", num_projected_objects);
  printf("Initial Localization %d\n", initial_localization);
  print_estimate_position();
}

void Robot::print_estimate_position() {
  printf("Pose estimation: [%.2f, %.2f])\n", estimated_position.x, estimated_position.y);
}

}  // namespace suiryoku
