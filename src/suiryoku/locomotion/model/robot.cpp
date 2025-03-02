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
  num_particles(500), xvar(10.0), yvar(10.0), kidnap_counter(0), weight_avg(0.0),
  rand_gen(std::random_device{}()), short_term_avg(0.0), long_term_avg(0.0),
  last_weight_avg(0.0), estimated_position(0.0, 0.0),
  initial_localization(true), reset_particles(false)
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
  if (initial_localization) {
    init_particles();
  } else {
    update_motion();
  }

  if (!projected_objects.empty()) {
    printf("calculate weight start\n");
    calculate_weight();
    printf("calculate weight done\n");
    if (weight_avg <= 0 || std::isnan(weight_avg)) {
      weight_avg = last_weight_avg;
    } else {
      if (initial_localization) {
        short_term_avg = weight_avg;
        long_term_avg = weight_avg;
        initial_localization = false;
      }
      last_weight_avg = weight_avg;

      short_term_avg = 0.1 * (weight_avg - short_term_avg);
      long_term_avg = 0.001 * (weight_avg - long_term_avg);
    }
    resample_particles();
    estimate_position();
    projected_objects.clear();
  }
}

void Robot::init_particles()
{
  double uniform_weight = 1.0 / num_particles;
  particles.clear();
  particles.resize(num_particles);
  // std::uniform_int_distribution<int> xrg(position.x - 100, position.x + 100); // use if initial position is known
  // std::uniform_int_distribution<int> yrg(position.y - 100, position.y + 100);
  std::uniform_int_distribution<int> xrg(0, 900);
  std::uniform_int_distribution<int> yrg(0, 600);

  for (int i = 0; i < num_particles; ++i) {
    particles[i].position = keisan::Point2(xrg(rand_gen), yrg(rand_gen));
    particles[i].orientation = orientation;
    particles[i].weight = uniform_weight;
  }
}

void Robot::resample_particles()
{
  std::vector<Particle>& old_particles = particles;
  std::vector<Particle> new_particles(num_particles);
  Particle current_best_particle;

  std::uniform_int_distribution<int> rand_index(0, num_particles - 1);
  int index = rand_index(rand_gen);

  double beta = 0.0;
  double max_weight = 0.0;
  double prob = std::max(0.0, 1.0 - short_term_avg/long_term_avg);
  reset_particles = prob > 0.25;

  // find the best particle
  for (const auto & p : old_particles) {
    if (p.weight > max_weight) {
      max_weight = p.weight;
      current_best_particle = p;
    }
  }

  best_particle = current_best_particle;

  // determine resample interval area
  double interval_x[2] = {0.0, 900.0};
  double interval_y[2] = {0.0, 600.0};

  if (!projected_objects.empty()) {
    interval_x[0] = position.x - 100;
    interval_x[1] = position.x + 100;
    interval_y[0] = position.y - 100;
    interval_y[1] = position.y + 100;
  }

  // resample particles
  std::uniform_real_distribution<double> rand_prob(0.0, 1.0);
  double uniform_weight = 1.0 / num_particles;
  for (int i = 0; i < num_particles; ++i) {
    if (rand_prob(rand_gen) < prob) {
      std::uniform_int_distribution<int> xrg(interval_x[0], interval_x[1]);
      std::uniform_int_distribution<int> yrg(interval_y[0], interval_y[1]);
      new_particles[i].position = keisan::Point2(xrg(rand_gen), yrg(rand_gen));
      new_particles[i].orientation = orientation;
      new_particles[i].weight = uniform_weight;
    } else {
      std::uniform_real_distribution<double> rand_beta(0.0, 2.0 * max_weight);
      beta += rand_beta(rand_gen);

      while (beta > old_particles[index].weight) {
        beta -= old_particles[index].weight;
        index = (index + 1) % num_particles;
      }

      new_particles[i] = old_particles[index];
    }
  }
}

void Robot::update_motion()
{
  static std::random_device xrd, yrd, wrd;
  static std::normal_distribution<> xgen(0.0, xvar), ygen(0.0, yvar);

  for (auto & p : particles) {
    double static_noise_x = xgen(xrd) / 5.0;
    double static_noise_y = ygen(yrd) / 5.0;
    double dynamic_noise_x = fabs(delta_position.x) * xgen(xrd) / 5.0;
    double dynamic_noise_y = fabs(delta_position.y) * ygen(yrd) / 5.0;
    double x_yterm = fabs(delta_position.y)*xgen(xrd) / 30.0;
    double y_xterm = fabs(delta_position.x)*ygen(yrd) / 30.0;
    p.position.x += delta_position.x + static_noise_x + dynamic_noise_x + x_yterm;
    p.position.y += delta_position.y + static_noise_y + dynamic_noise_y + y_xterm;
    p.orientation = orientation;
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
  for (auto & p : particles) {
    p.weight = calculate_total_likelihood(p);
  }

  // normalize weights
  double sum_weight = get_sum_weight();
  for (auto & p : particles) {
    p.weight /= sum_weight;
  }

  weight_avg = sum_weight / num_particles;
}

double Robot::calculate_total_likelihood(const Particle & particle) {
  double total_likelihood = 1.0;
  for (const auto & object_measurement : projected_objects) {
    total_likelihood *=
      calculate_object_likelihood(object_measurement, particle);
  }

  return total_likelihood;
}

double Robot::calculate_object_likelihood(
  const ProjectedObject & measurement, const Particle & particle) {
  std::vector<keisan::Point2> landmarks;
  double sigma_x = 1.0, sigma_y = 1.0;
  double relative_position_x, relative_position_y;
  double dx, dy, x_rot, y_rot, exponent, likelihood;
  double current_likelihood = 0.0;
  // keisan::Angle<double> fov = 78.0_deg;
  // keisan::Angle<double> camera_orientation = orientation - get_pan();

  if (measurement.label == "L-Intersection") {
    landmarks = field.landmarks_L;
  } else if (measurement.label == "T-Intersection") {
    landmarks = field.landmarks_T;
  } else if (measurement.label == "X-Intersection") {
    landmarks = field.landmarks_X;
  } else if (measurement.label == "goalpost") {
    landmarks = field.landmarks_X;
  }

  for (int i = 0; i < landmarks.size(); i++) {
    // check if the landmark is in the camera field of view
    // keisan::Angle<double> angle_to_landmark = keisan::signed_arctan(
      // landmarks[i].y - particle.position.y, landmarks[i].x - particle.position.x);
    // keisan::Angle<double> angle_diff = angle_to_landmark - camera_orientation;

    // if (std::fabs(angle_diff.degree()) <= fov.degree() / 2) {
    dx = measurement.position.x * 100;
    dy = measurement.position.y * 100;

    x_rot = dx * particle.orientation.cos() - dy * particle.orientation.sin();
    y_rot = dx * particle.orientation.sin() + dy * particle.orientation.cos();

    relative_position_x = particle.position.x + x_rot;
    relative_position_y = particle.position.y + y_rot;

    exponent =
    -0.5 *
    (pow((landmarks[i].x - relative_position_x), 2) / pow(sigma_x, 2) +
    pow((landmarks[i].y - relative_position_y), 2) / pow(sigma_y, 2));

    likelihood = exp(exponent) / (2 * M_PI * sigma_x * sigma_y) * 100000;

    if (likelihood > current_likelihood) {
      current_likelihood = likelihood;
    }
    // }
  }

  return current_likelihood;
}

void Robot::estimate_position() {
  keisan::Point2 sum_position = {0.0, 0.0};
  int centered_particles = 0;
  center_particles.clear();

  // Find numbers of particles closed to the best particle
  for (int i = 0; i < num_particles; ++i) {
    double distance = sqrt(pow(particles[i].position.x - best_particle.position.x, 2) +
                           pow(particles[i].position.y - best_particle.position.y, 2));

    if (distance < 50.0) {
      centered_particles++;
      sum_position.x += particles[i].position.x;
      sum_position.y += particles[i].position.y;

      center_particles.push_back(particles[i]);
    }
  }

  // Use mean of centered particles position if more than 30% of particles are centered
  if (reset_particles ||  centered_particles < 3) {
    estimated_position = position + delta_position;
  } else{
    estimated_position.x = sum_position.x / centered_particles;
    estimated_position.y = sum_position.y / centered_particles;

    estimated_position.x = std::max(0.0, std::min(900.0, estimated_position.x));
    estimated_position.y = std::max(0.0, std::min(600.0, estimated_position.y));

    position = estimated_position;
    apply_localization = true;
  }
}

void Robot::print_particles() {
  double sum_samples = 0.0;

  for (int i = 0; i < num_particles; ++i) {
    if (particles[i].weight > 0.0001) {
      std::cout << "Particle " << std::setw(5) << i
                << "  weight: " << std::fixed << std::setprecision(5)
                << particles[i].weight << std::setw(5) << " ["
                << std::fixed << std::setprecision(2) << particles[i].position.x
                << ", " << std::fixed << std::setprecision(2)
                << particles[i].position.y << ", " << std::fixed
                << std::setprecision(2) << particles[i].orientation.degree() << "]"
                << std::endl;

      sum_samples += particles[i].weight;
    }
  }

  printf("========================================\n");

  printf("Centered particles: %d\n", center_particles.size());
  for (int i = 0; i < center_particles.size(); ++i) {
    std::cout << "Centered Particle " << std::setw(5) << i
              << "  weight: " << std::fixed << std::setprecision(5)
              << center_particles[i].weight << std::setw(5) << " ["
              << std::fixed << std::setprecision(2) << center_particles[i].position.x
              << ", " << std::fixed << std::setprecision(2)
              << center_particles[i].position.y << ", " << std::fixed
              << std::setprecision(2) << center_particles[i].orientation.degree() << "]"
              << std::endl;
  }

  std::cout << "Best particle: " << std::fixed << std::setprecision(5)
            << best_particle.weight << std::setw(5) << " ["
            << std::fixed << std::setprecision(2) << best_particle.position.x
            << ", " << std::fixed << std::setprecision(2)
            << best_particle.position.y << ", " << std::fixed
            << std::setprecision(2) << best_particle.orientation.degree() << "]"
            << std::endl;

  std::cout << "Num particles: " << num_particles << std::endl;
  std::cout << "Sum weights: " << sum_samples << std::endl;
  std::cout << "Projected objects: " << projected_objects.size() << std::endl;
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
