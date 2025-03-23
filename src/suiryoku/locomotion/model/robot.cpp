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
  y_speed(0.0), a_speed(0.0), aim_on(false), is_walking(false), orientation(0_deg),
  position(0.0, 0.0), x_amplitude(0.0), y_amplitude(0.0), a_amplitude(0.0),
  is_calibrated(false), use_localization(false), apply_localization(false),
  num_particles(500), xvar(10.0), yvar(10.0), kidnap_counter(0), best_particle(nullptr),
  weight_avg(0.0), rand_gen(std::random_device{}()), short_term_avg(0.0), long_term_avg(0.0),
  last_weight_avg(0.0), estimated_position(0.0, 0.0), initial_localization(true),
  reset_particles(false), num_projected_objects(0), sigma_x(1.0), sigma_y(1.0),
  short_term_avg_ratio(0.1), long_term_avg_ratio(0.001)
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
    init_particles();
  } else {
    update_motion();
  }

  if (!projected_objects.empty()){
    weight_avg = 0.0;
    calculate_weight();
    if (weight_avg <= 0 || std::isnan(weight_avg)) {
      weight_avg = last_weight_avg;
    } else {
      if (initial_localization) {
        short_term_avg = weight_avg;
        long_term_avg = weight_avg;
        initial_localization = false;
      }
      last_weight_avg = weight_avg;
    }

    short_term_avg += short_term_avg_ratio * (weight_avg - short_term_avg);
    long_term_avg += long_term_avg_ratio * (weight_avg - long_term_avg);

    resample_particles();
    estimate_position();
  }

  projected_objects.clear();
}

void Robot::reset_localization() {
  apply_localization = true;
  initial_localization = true;
  particles.clear();
  center_particles.clear();
  projected_objects.clear();
  estimated_position = keisan::Point2(0.0, 0.0);
  best_particle = nullptr;
  weight_avg = 0.0;
  short_term_avg = 0.0;
  long_term_avg = 0.0;
  last_weight_avg = 0.0;
  current_resample_interval = ResampleInterval::FIELD_INIT;
  reset_particles = false;
  prob = 0.0;

  if (use_localization) {
    localize();
  }
}

void Robot::init_particles()
{
  double uniform_weight = 1.0 / num_particles;
  particles.clear();
  particles.resize(num_particles);
  std::uniform_int_distribution<int> xrg(0, 900);
  std::uniform_int_distribution<int> yrg(0, 600);
  current_resample_interval = ResampleInterval::FIELD_INIT;

  for (int i = 0; i < num_particles; ++i) {
    particles[i].position = keisan::Point2(xrg(rand_gen), yrg(rand_gen));
    particles[i].orientation = orientation;
    particles[i].weight = uniform_weight;
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
  reset_particles = prob > reset_particles_threshold;

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
  int interval_x[2] = {0, 900};
  int interval_y[2] = {0, 600};

  if (!projected_objects.empty() && !reset_particles && sum_weight > 0) {
    interval_x[0] = position.x - 75;
    interval_x[1] = position.x + 75;
    interval_y[0] = position.y - 75;
    interval_y[1] = position.y + 75;
    prob = std::min(0.05, prob);
    current_resample_interval = ResampleInterval::CENTER;
  } else {
    if (projected_objects.empty()) {
      current_resample_interval = ResampleInterval::FIELD_EMPTY_PROJECTED_OBJECTS;
    } else if (reset_particles) {
      current_resample_interval = ResampleInterval::FIELD_RESET;
    } else if (sum_weight <= 0) {
      current_resample_interval = ResampleInterval::FIELD_ZERO_WEIGHT;
    }
  }

  // resample particles
  std::uniform_real_distribution<double> rand_prob(0.0, 1.0);
  std::uniform_real_distribution<double> rand_prob2(0.0, 1.0);
  std::uniform_real_distribution<double> rand_beta(0.0, 2.0 * max_weight);
  std::uniform_int_distribution<int> xrg(interval_x[0], interval_x[1]);
  std::uniform_int_distribution<int> yrg(interval_y[0], interval_y[1]);
  std::uniform_int_distribution<int> large_xrg(interval_x[0] - 25, interval_x[1] + 25);
  std::uniform_int_distribution<int> large_yrg(interval_y[0] - 25, interval_y[1] + 25);
  double uniform_weight = 1.0 / num_particles;

  // adaptive noise
  double max_sigma = 30.0;
  double min_sigma = 1.0;

  double sigma_factor = 1.0 - prob;
  double resample_sigma_x = min_sigma + sigma_factor * (max_sigma - min_sigma);
  double resample_sigma_y = min_sigma + sigma_factor * (max_sigma - min_sigma);

  std::normal_distribution<double> noise_x(0.0, resample_sigma_x);
  std::normal_distribution<double> noise_y(0.0, resample_sigma_y);

  for (int i = 0; i < num_particles; ++i) {
    if (rand_prob2(rand_gen) < 0.97) {
      if (rand_prob(rand_gen) < prob) {
        new_particles[i].position = keisan::Point2(xrg(rand_gen), yrg(rand_gen));
        new_particles[i].orientation = orientation;
        new_particles[i].weight = uniform_weight;
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
    } else {
      new_particles[i].position = keisan::Point2(large_xrg(rand_gen), large_yrg(rand_gen));
      new_particles[i].orientation = orientation;
      new_particles[i].weight = 0.0;
    }
  }

  particles = new_particles;

  reset_particles = false;
}

void Robot::update_motion()
{
  static std::normal_distribution<double> xgen(0.0, xvar), ygen(0.0, yvar);

  if (!projected_objects.empty()) {
    for (auto & p : particles) {
      double static_noise_x = xgen(rand_gen) / 5.0;
      double static_noise_y = ygen(rand_gen) / 5.0;
      double dynamic_noise_x = fabs(delta_position.x) * xgen(rand_gen) / 5.0;
      double dynamic_noise_y = fabs(delta_position.y) * ygen(rand_gen) / 5.0;
      double x_yterm = fabs(delta_position.y) * xgen(rand_gen) / 30.0;
      double y_xterm = fabs(delta_position.x) * ygen(rand_gen) / 30.0;

      p.position.x += delta_position.x + static_noise_x + dynamic_noise_x + x_yterm;
      p.position.y += delta_position.y + static_noise_y + dynamic_noise_y + y_xterm;
      p.orientation = orientation;
    }
    update_motion_state = UpdateMotionState::WITH_NOISE;
  } else {
    for (auto & p : particles) {
      p.position.x += delta_position.x;
      p.position.y += delta_position.y;
      p.orientation = orientation;
    }
    update_motion_state = UpdateMotionState::WITHOUT_NOISE;
  }
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
  } else {
    weight_avg = 0.0;
  }
}

double Robot::calculate_total_likelihood(const Particle & particle) {
  double total_likelihood = 0.0;
  for (const auto & object_measurement : projected_objects) {
    total_likelihood +=
      calculate_object_likelihood(object_measurement, particle);
  }

  return total_likelihood;
}

double Robot::calculate_object_likelihood(
  const ProjectedObject & measurement, const Particle & particle) {
  std::vector<keisan::Point2> landmarks;
  double relative_position_x, relative_position_y;
  double dx, dy, x_rot, y_rot, exponent, likelihood;
  double current_likelihood = 0.0;

  if (measurement.label == "L-Intersection") {
    landmarks = field.landmarks_L;
  } else if (measurement.label == "T-Intersection") {
    landmarks = field.landmarks_T;
  } else if (measurement.label == "X-Intersection") {
    landmarks = field.landmarks_X;
  } else if (measurement.label == "goalpost") {
    landmarks = field.landmarks_goalpost;
  }

  for (int i = 0; i < landmarks.size(); i++) {
    dx = measurement.position.x * 100;
    dy = measurement.position.y * 100;

    x_rot = dx * particle.orientation.cos() + dy * particle.orientation.sin();
    y_rot = dx * particle.orientation.sin() - dy * particle.orientation.cos();

    relative_position_x = particle.position.x + x_rot;
    relative_position_y = particle.position.y + y_rot;

    exponent =
    -0.5 *
    (pow((landmarks[i].x - relative_position_x), 2) / pow(sigma_x, 2) +
    pow((landmarks[i].y - relative_position_y), 2) / pow(sigma_y, 2));

    likelihood = exp(exponent) / (2 * M_PI * sigma_x * sigma_y) * 100;

    current_likelihood += likelihood;
  }

  return current_likelihood;
}

void Robot::estimate_position() {
  if (best_particle == nullptr || projected_objects.empty()) {
    return;
  }

  keisan::Point2 sum_position = {0.0, 0.0};
  int centered_particles = 0;
  center_particles.clear();

  // Find numbers of particles closed to the best particle
  for (int i = 0; i < num_particles; ++i) {
    double distance = sqrt(pow(particles[i].position.x - best_particle->position.x, 2) +
                           pow(particles[i].position.y - best_particle->position.y, 2));

    if (distance < 50.0) {
      centered_particles++;
      sum_position.x += particles[i].position.x;
      sum_position.y += particles[i].position.y;

      center_particles.push_back(&particles[i]);
    }
  }

  // Use mean of centered particles position if more than 30% of particles are centered
  if (centered_particles > min_centered_particles_ratio * num_particles) {
    estimated_position.x = sum_position.x / centered_particles;
    estimated_position.y = sum_position.y / centered_particles;

    if (estimated_position.x >= 0.0 && estimated_position.x <= 900.0 &&
      estimated_position.y >= 0.0 && estimated_position.y <= 600.0) {
      position = estimated_position;
      apply_localization = true;
    }
  }
}

void Robot::print_particles() {
  std::string resample_interval;
  switch (current_resample_interval) {
    case ResampleInterval::CENTER:
      resample_interval = "CENTER";
      break;
    case ResampleInterval::FIELD_EMPTY_PROJECTED_OBJECTS:
      resample_interval = "FIELD_EMPTY_PROJECTED_OBJECTS";
      break;
    case ResampleInterval::FIELD_INIT:
      resample_interval = "FIELD_INIT";
      break;
    case ResampleInterval::FIELD_RESET:
      resample_interval = "FIELD_RESET";
      break;
    case ResampleInterval::FIELD_ZERO_WEIGHT:
      resample_interval = "FIELD_ZERO_WEIGHT";
      break;
    default:
      resample_interval = "UNKNOWN";
      break;
  }

  std::string update_motion;
  switch (update_motion_state) {
    case UpdateMotionState::NOT_UPDATED:
      update_motion = "NOT_UPDATED";
      break;
    case UpdateMotionState::WITH_NOISE:
      update_motion = "WITH_NOISE";
      break;
    case UpdateMotionState::WITHOUT_NOISE:
      update_motion = "WITHOUT_NOISE";
      break;
    default:
      update_motion = "UNKNOWN";
      break;
  }

  printf("Particles num: %d\n", particles.size());
  printf("Resample interval: %s\n", resample_interval.c_str());
  printf("Update motion: %s\n", update_motion.c_str());
  printf("Prob: %.2f | is more than %.2f: %s\n", prob, (prob > reset_particles_threshold) ? "true" : "false", reset_particles_threshold);
  printf("Reset particles: %s\n", reset_particles ? "true" : "false");
  printf("Short term avg: %.5f\n", short_term_avg);
  printf("Long term avg: %.5f\n", long_term_avg);
  printf("Last weight avg: %.5f\n", last_weight_avg);
  printf("Weight avg: %.5f\n", weight_avg);

  printf("========================================\n");
  printf("Minimal Centered Particles: %.0f\n", min_centered_particles_ratio * num_particles);

  printf("Centered particles: %d\n", center_particles.size());
  for (int i = 0; i < center_particles.size(); ++i) {
    std::cout << "Centered Particle " << std::setw(5) << i
              << "  weight: " << std::fixed << std::setprecision(5)
              << center_particles[i]->weight << std::setw(5) << " ["
              << std::fixed << std::setprecision(2) << center_particles[i]->position.x
              << ", " << std::fixed << std::setprecision(2)
              << center_particles[i]->position.y << ", " << std::fixed
              << std::setprecision(2) << center_particles[i]->orientation.degree() << "]"
              << std::endl;
    if (i >= 10) {
      printf("... and %d more\n", center_particles.size() - 10);
      break;
    }
  }

  if (best_particle == nullptr) {
    std::cout << "Best particle: NULL" << std::endl;
  } else {
    std::cout << "Best particle: " << std::fixed << std::setprecision(5)
      << best_particle->weight << std::setw(5) << " ["
      << std::fixed << std::setprecision(2) << best_particle->position.x
      << ", " << std::fixed << std::setprecision(2)
      << best_particle->position.y << ", " << std::fixed
      << std::setprecision(2) << best_particle->orientation.degree() << "]"
      << std::endl;
  }

  std::cout << "Num particles: " << num_particles << std::endl;
  std::cout << "Sum weights: " << get_sum_weight() << std::endl;
  std::cout << "Projected objects: " << num_projected_objects << std::endl;
  print_estimate_position();
}

void Robot::print_estimate_position() {
  std::cout << "Pose estimation: "
            << " [" << std::fixed << std::setprecision(2)
            << estimated_position.x << ", " << std::fixed
            << std::setprecision(2) << estimated_position.y
            << "])" << std::endl;
}

}  // namespace suiryoku
