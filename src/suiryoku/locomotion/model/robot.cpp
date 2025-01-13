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
  y_speed(0.0), a_speed(0.0), aim_on(false), is_walking(false), orientation(0_deg),
  position(0.0, 0.0), x_amplitude(0.0), y_amplitude(0.0), a_amplitude(0.0),
  is_calibrated(false), use_localization(false), apply_localization(false),
  num_particles(0), xvar(10.0), yvar(10.0), kidnap_counter(0)
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

void Robot::localize(bool initial_localization)
{
  if (num_particles == 0 || initial_localization) {
    init_particles(initial_localization);
  } else {
    update_motion();
  }

  if (projected_objects.empty()) {
    return;
  }

  calculate_weight();
  resample_particles();

  estimate_position();
  print_estimate_position();
}

void Robot::init_particles(bool initial_localization)
{
  particles.clear();
  if (initial_localization) {
    std::cout << "INIT PARTICLES BASED ON LAST POSE" << std::endl;

    num_particles = 1000;
    std::random_device xrd, yrd;
    std::normal_distribution<double> xrg(position.x, xvar), yrg(position.y, yvar);

    for (int i = 0; i < num_particles; ++i) {
      Particle new_particle;
      new_particle.position = keisan::Point2(xrg(xrd), yrg(yrd));
      new_particle.orientation = orientation;
      new_particle.weight = 1.0 / num_particles;

      particles.push_back(new_particle);
    }
  } else { // if not initial, generate particles all over the field
    std::cout << "INIT PARTICLES ALL OVER FIELD" << std::endl;

    const int x_gap = 10, y_gap = 10;
    num_particles = field.width * field.length / (x_gap * y_gap);

    for (int i = 0; i <= field.length; i += x_gap) {
      for (int j = 0; j <= field.width; j += y_gap) {
        Particle new_particle;
        new_particle.position = keisan::Point2(i, j);
        new_particle.orientation = orientation;
        new_particle.weight = 1.0 / num_particles;

        particles.push_back(new_particle);
      }
    }
    kidnap_counter = 0;
  }
}

void Robot::resample_particles()
{
  std::vector<Particle> new_particles;
  std::random_device xrd, yrd, wrd;

  for (const auto & p : particles) {
    if (p.weight >= 1.0 / (particles.size() * 10.0)) {
      new_particles.push_back(p);
      std::normal_distribution<double> xrg(p.position.x, xvar), yrg(p.position.y, yvar);

      int n = p.weight * 100;
      for (int i = 0; i < n; ++i) {
        Particle new_particle;
        new_particle.position = keisan::Point2(xrg(xrd), yrg(yrd));
        new_particle.orientation = orientation;
        new_particle.weight = 1.0 / n;

        new_particles.push_back(new_particle);
      }
    }
  }
  particles = new_particles;
  num_particles = particles.size();
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

  double sum_weight = get_sum_weight();
  if (sum_weight > 0.0) {
    for (auto & p : particles) {
      p.weight /= sum_weight;
    }
    kidnap_counter = 0;
  } else {
    if (kidnap_counter++ < 1) {
      init_particles(true);
    } else {
      init_particles(false);
    }
  }

  projected_objects.clear();
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
    dx = measurement.position.x * 100;
    dy = measurement.position.y * 100;

    x_rot = dx * cos(particle.orientation.radian()) - dy * sin(particle.orientation.radian());
    y_rot = dx * sin(particle.orientation.radian()) + dy * cos(particle.orientation.radian());

    relative_position_x = particle.position.x + x_rot;
    relative_position_y = particle.position.y + y_rot;

    exponent =
      -0.5 *
      (pow((landmarks[i].x - relative_position_x), 2) / pow(sigma_x, 2) +
       pow((landmarks[i].y - relative_position_y), 2) / pow(sigma_y, 2));

    likelihood = exp(exponent) / (2 * M_PI * sigma_x * sigma_y);

    if (likelihood > current_likelihood) {
      current_likelihood = likelihood;
    }
  }

  return current_likelihood;
}

void Robot::estimate_position() {
  if (num_particles == 0 || get_sum_weight() == 0) {
    return;
  }

  double x_mean = 0.0;
  double y_mean = 0.0;
  for (const auto & p : particles) {
    x_mean += (1.0 / num_particles) * p.position.x;
    y_mean += (1.0 / num_particles) * p.position.y;
  }
  estimated_position.x = (x_mean < 0.0) ? 0.0 : (x_mean > 900.0 ? 900.0 : x_mean);
  estimated_position.y = (y_mean < 0.0) ? 0.0 : (y_mean > 600.0 ? 600.0 : y_mean);

  if (num_particles < 1000) {
    position = estimated_position;
    apply_localization = true;
  }
}

void Robot::print_particles() {
  double sum_samples = 0.0;

  for (int i = 0; i < num_particles; i++) {
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

  std::cout << "Num particles: " << num_particles << std::endl;
  std::cout << "Sum weights: " << sum_samples << std::endl;
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
