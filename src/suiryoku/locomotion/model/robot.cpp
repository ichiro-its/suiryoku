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
#include <random>
#include <string>

#include "suiryoku/locomotion/model/robot.hpp"

#include "keisan/keisan.hpp"

using keisan::literals::operator""_deg;

namespace suiryoku
{

Robot::Robot()
: pan(0_deg), tilt(0_deg), pan_center(0_deg), tilt_center(0_deg), x_speed(0.0),
  y_speed(0.0), a_speed(0.0), aim_on(false), is_walking(false),
  orientation(0_deg), position(0.0, 0.0), x_amplitude(0.0), y_amplitude(0.0),
  a_amplitude(0.0), is_calibrated(false), kidnapped(false), num_particles(0)
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
  if (kidnapped) {
    init_particles();
    kidnapped = false;
  } else {
    update_particles();
  }

  if (projected_objects.empty()) {
    return;
  }

  calculate_weight();
  resample_particles();
  estimate_position();
}

void Robot::init_particles()
{
  particles.clear();
  if (!kidnapped) {
    const double var_x = 10.0, var_y = 10.0, var_w = 0.05;
    num_particles = 1000;
    std::random_device xrd, yrd, wrd;
    std::normal_distribution<double> xrg(position.x, var_x),
      yrg(position.y, var_y), wrg(orientation.degree(), var_w);

    for (int i = 0; i < num_particles; ++i) {
      Particle new_particle;
      new_particle.position = keisan::Point2(xrg(xrd), yrg(yrd));
      new_particle.orientation = keisan::make_degree(wrg(wrd)).normalize();
      new_particle.weight = 1.0 / num_particles;

      particles.push_back(new_particle);
    }
  } else { // if kidnapped, generate particles all over the field
    const int x_gap = 5, y_gap = 5;
    num_particles = field.width * field.length / (x_gap * y_gap);

    for (int i = -field.width / 2; i < field.width / 2; i += x_gap) {
      for (int j = -field.length / 2; j < field.length / 2; j += y_gap) {
        Particle new_particle;
        new_particle.position = keisan::Point2(i, j);
        new_particle.orientation = orientation;
        new_particle.weight = 1.0 / num_particles;

        particles.push_back(new_particle);
      }
    }
  }
}

void Robot::resample_particles()
{
  std::vector<Particle> new_particles;
  std::random_device xrd, yrd, wrd;
  const double var_x = 5.0, var_y = 5.0, var_w = 0.01;

  for (auto & particle : particles) {
    if (particle.weight >= 1.0 / (particles.size() * 10.0)) {
      new_particles.push_back(particle);
      std::normal_distribution<double> xrg(particle.position.x, var_x),
        yrg(particle.position.y, var_y), wrg(particle.orientation.degree(), var_w);

      int n = particle.weight * 100;

      for (int i = 0; i < n; ++i) {
        Particle new_particle;
        new_particle.position = keisan::Point2(xrg(xrd), yrg(yrd));
        new_particle.orientation = keisan::make_degree(wrg(wrd)).normalize();
        new_particle.weight = 1.0 / n;

        new_particles.push_back(new_particle);
      }
    }
  }
  particles = new_particles;
  num_particles = particles.size();
}

void Robot::update_particles()
{
  for (int i = 0; i < num_particles; ++i) {
    particles[i].position.x += delta_position.x;
    particles[i].position.y += delta_position.y;
    particles[i].orientation = orientation;
  }
}

double Robot::get_sum_weight()
{
  double sum_weight = 0.0;
  for (int i = 0; i < num_particles; ++i) {
    sum_weight += particles[i].weight;
  }
  return sum_weight;
}

void Robot::calculate_weight()
{
  for (int i = 0; i < num_particles; ++i) {
    double likelihood = calculate_total_likelihood(particles[i]);
    particles[i].weight = likelihood;
  }

  double sum_weight = get_sum_weight();
  if (sum_weight > 0.0) {
    for (int i = 0; i < num_particles; ++i) {
      particles[i].weight /= sum_weight;
    }
  } else {
    for (int i = 0; i < num_particles; ++i) {
      particles[i].weight = 1 / num_particles;
    }
  }
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
  const double sigma_x = 1.0, sigma_y = 1.0;
  double relative_position_x, relative_position_y;
  double dx, dy, x_rot, y_rot, exponent, likelihood;
  double current_likelihood = 0.0;

  for (int i = 0; i < field.num_landmarks; i++) {
    dx = measurement.center.x * 100;
    dy = measurement.center.y * 100;

    x_rot = dx * cos(particle.orientation.degree()) - dy * sin(particle.orientation.degree());
    y_rot = dx * sin(particle.orientation.degree()) + dy * cos(particle.orientation.degree());

    relative_position_x = particle.position.x + x_rot;
    relative_position_y = particle.position.y + y_rot;

    exponent =
      -0.5 *
      (pow((field.landmarks[i].x - relative_position_x), 2) / pow(sigma_x, 2) +
       pow((field.landmarks[i].y - relative_position_y), 2) / pow(sigma_y, 2));

    likelihood = exp(exponent) / (2 * M_PI * sigma_x * sigma_y);

    if (likelihood > current_likelihood) {
      current_likelihood = likelihood;
    }
  }

  return current_likelihood;
}

void Robot::estimate_position() {
  double x_mean = 0.0;
  double y_mean = 0.0;
  for (auto p : particles) {
    x_mean += (1.0 / num_particles) * p.position.x;
    y_mean += (1.0 / num_particles) * p.position.y;
  }
  estimated_position.x = x_mean;
  estimated_position.y = y_mean;
}

}  // namespace suiryoku
