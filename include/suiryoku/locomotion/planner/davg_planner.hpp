// Copyright (c) 2026 ICHIRO ITS
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

#ifndef SUIRYOKU__LOCOMOTION__PLANNER__DAVG_PLANNER_HPP_
#define SUIRYOKU__LOCOMOTION__PLANNER__DAVG_PLANNER_HPP_

#include <vector>
#include <utility>

#include "keisan/keisan.hpp"

namespace suiryoku
{
    
struct Obstacle
{
  keisan::Point2 position;
  double radius;
};

class DAVGPlanner
{
public:
    explicit DAVGPlanner(
        double turning_penalty = 1.0,
        int polygon_edges = 8,
        double inflation_radius = 10.0
    );

    // calculate shortest path using augmented a*
    std::vector<keisan::Point2> calculate_path(
        const keisan::Point2 &start_pos,
        double start_theta,
        const keisan::Point2 &goal_pos,
        const std::vector<Obstacle> &obstacles
    );

    // check if line segment intersect with obstacle / inflation zone
    bool is_line_colliding(
        const keisan::Point2 &p_a,
        const keisan::Point2 &p_b,
        const std::vector<Obstacle> &active_obstacles,
        double inflation_radius
    ) const;

    double get_inflation_radius();

    void set_inflation_radius(double new_radius);
    void set_polygon_edges(double new_edges);
    void set_turning_penalty(double new_value);

private:
    using Graph = std::vector<std::vector<std::pair<int, double>>>;

    // check if obstacle in active area
    bool calculate_distance_to_line(
        const keisan::Point2 &start,
        const keisan::Point2 &goal,
        const Obstacle &obstacle,
        double left_width,
        double right_width,
        double &signed_dist
    ) const;

    // escape from inflation zone
    void force_escape(
        int node_idx,
        const std::vector<keisan::Point2> &all_nodes,
        int total_nodes,
        const std::vector<Obstacle> &active_obstacles,
        Graph &graph
    ) const;

    double turning_penalty_;
    int polygon_edges_;
    double inflation_radius_;
};

} // namespace suiryoku

#endif // SUIRYOKU__LOCOMOTION__PLANNER__DAVG_PLANNER_HPP_