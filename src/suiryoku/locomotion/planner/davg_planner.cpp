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

#include "suiryoku/locomotion/planner/davg_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

namespace suiryoku
{
DAVGPlanner::DAVGPlanner(double turning_penalty, int polygon_edges, double inflation_radius)
{
  turning_penalty_ = turning_penalty;
  polygon_edges_ = polygon_edges;
  inflation_radius_ = inflation_radius;
}

double DAVGPlanner::get_inflation_radius() const { return inflation_radius_; }

void DAVGPlanner::set_inflation_radius(double new_radius)
{
  if (new_radius < 0.0) {
    std::cerr << "inflation radius must be >= 0\n";
    return;
  }
  inflation_radius_ = new_radius;
}

void DAVGPlanner::set_polygon_edges(int new_edges)
{
  if (new_edges < 3) {
    std::cerr << "polygon edges must be >= 3\n";
    return;
  }
  polygon_edges_ = new_edges;
}

void DAVGPlanner::set_turning_penalty(double new_value) { turning_penalty_ = new_value; }

bool DAVGPlanner::is_obstacle_in_corridor(
  const keisan::Point2 & start,
  const keisan::Point2 & goal,
  const Obstacle & obstacle,
  double left_width,
  double right_width,
  double & signed_dist) const
{
  // sg = start-goal vector
  const double sg_x = goal.x - start.x;
  const double sg_y = goal.y - start.y;
  const double sg_sq = (sg_x * sg_x) + (sg_y * sg_y);
  if (sg_sq == 0.0) {return false;}
  const double sg_len = std::sqrt(sg_sq);

  // so = start-obstacle vector
  const double so_x = obstacle.position.x - start.x;
  const double so_y = obstacle.position.y - start.y;

  const double t = (so_x * sg_x + so_y * sg_y) / sg_sq;
  const double radius_border = obstacle.radius + inflation_radius_;
  const double proj_length = t * sg_len;

  // reject if the obstacle projection outside the segment
  if (proj_length < -radius_border || proj_length > sg_len + radius_border) {
    return false;
  }

  // signed lateral distance (positive = left of sg direction)
  const double cross = (sg_x * so_y) - (sg_y * so_x);
  signed_dist = cross / sg_len;

  const double obs_min = signed_dist - radius_border;
  const double obs_max = signed_dist + radius_border;

  return (obs_min <= left_width && obs_max >= -right_width);
}

bool DAVGPlanner::is_segment_colliding(
  const keisan::Point2 & p_a,
  const keisan::Point2 & p_b,
  const std::vector<Obstacle> & active_obstacles,
  double inflation_radius) const
{
  const double ab_x = p_b.x - p_a.x;
  const double ab_y = p_b.y - p_a.y;
  const double dist_sq = (ab_x * ab_x) + (ab_y * ab_y);
  if (dist_sq == 0.0) {return false;}

  for (const auto & obs : active_obstacles) {
    const double ao_x = obs.position.x - p_a.x;
    const double ao_y = obs.position.y - p_a.y;

    // clamp parameter to get the closest point on the segment.
    const double t = std::max(0.0, std::min(1.0, (ao_x * ab_x + ao_y * ab_y) / dist_sq));
    const double px = p_a.x + t * ab_x;
    const double py = p_a.y + t * ab_y;

    const double closest = std::hypot(obs.position.x - px, obs.position.y - py);
    if (closest < obs.radius + inflation_radius - 0.1) {
      return true;
    }
  }
  return false;
}

void DAVGPlanner::connect_trapped_node(
  int node_idx,
  const std::vector<keisan::Point2> & all_nodes,
  int total_nodes,
  const std::vector<Obstacle> & active_obstacles,
  Graph & graph) const
{
  const double nx = all_nodes[node_idx].x;
  const double ny = all_nodes[node_idx].y;

  for (const auto & obs : active_obstacles) {
    const double trap_threshold = obs.radius + inflation_radius_ * 0.5;
    if (std::hypot(obs.position.x - nx, obs.position.y - ny) >= trap_threshold) {
      continue; // node is not trapped by this obstacle
    }

    // build the list of all obstacles
    std::vector<Obstacle> other_obs;
    other_obs.reserve(active_obstacles.size() - 1);
    for (const auto & o : active_obstacles) {
      if (std::abs(o.position.x - obs.position.x) > 0.01 ||
        std::abs(o.position.y - obs.position.y) > 0.01)
      {
        other_obs.push_back(o);
      }
    }

    // connect node to every polygon vertex
    const std::vector<Obstacle> current_obs = {obs};
    for (int v_idx = 1; v_idx < total_nodes - 1; ++v_idx) {
      // check against the trapping obstacle only
      if (is_segment_colliding(all_nodes[node_idx], all_nodes[v_idx], current_obs, 0.0)) {
        continue;
      }
      // check against remaining obstacles with inflation
      if (is_segment_colliding(all_nodes[node_idx], all_nodes[v_idx], other_obs, inflation_radius_)) {
        continue;
      }
      const double dist = std::hypot(all_nodes[v_idx].x - nx, all_nodes[v_idx].y - ny);
      graph[node_idx].push_back({v_idx, dist});
      graph[v_idx].push_back({node_idx, dist});
    }
    break;
  }
}

std::vector<keisan::Point2> DAVGPlanner::calculate_path(
  const keisan::Point2 & start_pos,
  double start_theta,
  const keisan::Point2 & goal_pos,
  const std::vector<Obstacle> &obstacles)
{
  auto total_start = std::chrono::high_resolution_clock::now();

  // search for active obstacles
  auto active_search_start = std::chrono::high_resolution_clock::now();
  std::vector<Obstacle> active_obstacles;
  active_obstacles.reserve(obstacles.size());

  double active_region_left = 0.0;
  double active_region_right = 0.0;

  while (true) {
    bool found_new = false;
    for (const auto &obs : obstacles) {
      bool already_active = false;
      for (const auto &a_obs : active_obstacles) {
        if (std::abs(obs.position.x - a_obs.position.x) < 0.1 &&
          std::abs(obs.position.y - a_obs.position.y) < 0.1)
        {
          already_active = true;
          break;
        }
      }
      if (already_active) continue;

      double signed_dist = 0.0;
      if (!is_obstacle_in_corridor(
          start_pos, goal_pos, obs,
          active_region_left, active_region_right, signed_dist))
      {
        continue;
      }

      active_obstacles.push_back(obs);
      found_new = true;

      // expand corridor
      const double border = obs.radius + inflation_radius_;
      const double obs_max = signed_dist + border;
      const double obs_min = signed_dist - border;
      if (obs_max > active_region_left) {active_region_left = obs_max;}
      if (-obs_min > active_region_right) {active_region_right = -obs_min;}
    }
    if (!found_new) break;
  }
  auto active_search_end = std::chrono::high_resolution_clock::now();
  auto active_search_duration = std::chrono::duration_cast<std::chrono::milliseconds>(active_search_end - active_search_start);
  std::cout << "[DAVGPlanner] Active obstacle search: " << active_search_duration.count() << "ms\n";

  // generate vertices for each active obstacle
  auto graph_const_start = std::chrono::high_resolution_clock::now();
  const int num_active = static_cast<int>(active_obstacles.size());
  std::vector<keisan::Point2> vertices;
  vertices.reserve(num_active * polygon_edges_);

  const double edge_angle = 2.0 * M_PI / polygon_edges_;
  const double cos_factor = std::cos(M_PI / polygon_edges_);

  for (const auto & obs : active_obstacles) {
    const double safe_r = (obs.radius + inflation_radius_) / cos_factor;
    for (int i = 0; i < polygon_edges_; ++i) {
      const double angle = i * edge_angle;
      vertices.push_back({obs.position.x + safe_r * std::cos(angle),
        obs.position.y + safe_r * std::sin(angle)});
    }
  }

  // create edges for each node that is not collide with each other
  std::vector<keisan::Point2> all_nodes;
  all_nodes.reserve(vertices.size() + 2);
  all_nodes.push_back(start_pos);
  for (const auto & v : vertices) {all_nodes.push_back(v);}
  all_nodes.push_back(goal_pos);

  const int total_nodes = static_cast<int>(all_nodes.size());
  const int start_id = 0;
  const int goal_id = total_nodes - 1;

  // build visibility graph
  Graph graph(total_nodes);
  for (int i = 0; i < total_nodes; ++i) {
    graph[i].reserve(total_nodes / 2);
  }

  for (int i = 0; i < total_nodes; ++i) {
    for (int j = i + 1; j < total_nodes; ++j) {
      if (is_segment_colliding(all_nodes[i], all_nodes[j], active_obstacles, inflation_radius_)) {
        continue;
      }
      const double dist = std::hypot(
        all_nodes[j].x - all_nodes[i].x,
        all_nodes[j].y - all_nodes[i].y);
      graph[i].push_back({j, dist});
      graph[j].push_back({i, dist});
    }
  }

  // handle start/goal nodes that are inside an inflation zone.
  connect_trapped_node(start_id, all_nodes, total_nodes, active_obstacles, graph);
  connect_trapped_node(goal_id, all_nodes, total_nodes, active_obstacles, graph);
  auto graph_const_end = std::chrono::high_resolution_clock::now();
  auto graph_const_duration = std::chrono::duration_cast<std::chrono::milliseconds>(graph_const_end - graph_const_start);
  std::cout << "[DAVGPlanner] Visibility graph construction: " << graph_const_duration.count() << "ms\n";

  // augmented A* with turning penalty
  auto astar_start = std::chrono::high_resolution_clock::now();
  struct AStarNode
  {
    double approx_cost;
    double actual_cost;
    int node_id;
    double theta;
    std::vector<int> path;

    bool operator>(const AStarNode & other) const
    {
      return approx_cost > other.approx_cost;
    }
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

  {
    AStarNode seed;
    seed.approx_cost = 0.0;
    seed.actual_cost = 0.0;
    seed.node_id = start_id;
    seed.theta = start_theta;
    seed.path.reserve(total_nodes);
    seed.path.push_back(start_id);
    open_set.push(std::move(seed));
  }

  std::vector<double> costs(total_nodes, std::numeric_limits<double>::infinity());
  costs[start_id] = 0.0;

  std::vector<int> path_ids;

  while (!open_set.empty()) {
    AStarNode curr = open_set.top();
    open_set.pop();

    if (curr.node_id == goal_id) {
      path_ids = std::move(curr.path);
      break;
    }

    if (curr.actual_cost > costs[curr.node_id]) {continue;}

    const double cx = all_nodes[curr.node_id].x;
    const double cy = all_nodes[curr.node_id].y;

    for (const auto & [neighbor_id, dist] : graph[curr.node_id]) {
      const double nx = all_nodes[neighbor_id].x;
      const double ny = all_nodes[neighbor_id].y;

      const double neighbor_theta = std::atan2(ny - cy, nx - cx);
      double d_theta = neighbor_theta - curr.theta;

      while (d_theta > M_PI) {d_theta -= 2.0 * M_PI;}
      while (d_theta < -M_PI) {d_theta += 2.0 * M_PI;}

      const double new_g = curr.actual_cost + dist + turning_penalty_ * std::abs(d_theta);

      if (new_g >= costs[neighbor_id]) continue;

      costs[neighbor_id] = new_g;

      const double h = std::hypot(goal_pos.x - nx, goal_pos.y - ny);

      AStarNode next;
      next.approx_cost = new_g + h;
      next.actual_cost = new_g;
      next.node_id = neighbor_id;
      next.theta = neighbor_theta;
      next.path = curr.path;
      next.path.push_back(neighbor_id);
      open_set.push(std::move(next));
    }
  }
  auto astar_end = std::chrono::high_resolution_clock::now();
  auto astar_duration = std::chrono::duration_cast<std::chrono::milliseconds>(astar_end - astar_start);
  std::cout << "[DAVGPlanner] A* search loop: " << astar_duration.count() << "ms\n";

  // output route
  std::vector<keisan::Point2> route;
  route.reserve(path_ids.size());
  for (const int id : path_ids) {
    route.push_back(all_nodes[id]);
  }

  auto total_end = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start);
  std::cout << "[DAVGPlanner] Total execution time: " << total_duration.count() << "ms\n";

  return route;
}

} // namespace suiryoku
