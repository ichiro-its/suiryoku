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

#include "jitsuyo/config.hpp"

using namespace std::chrono;

namespace suiryoku
{
DAVGPlanner::DAVGPlanner(double turning_penalty, int polygon_edges, double inflation_radius)
{
  turning_penalty_ = turning_penalty;
  polygon_edges_ = polygon_edges;
  inflation_radius_ = inflation_radius;
  path_blocked_multiplier_ = 1.5;
  goal_clear_multiplier_ = 1.5;
  side_penalty_ = 50.0;
  sticky_threshold_ = 25.0;
}

bool DAVGPlanner::load_config(const std::string & path)
{
  nlohmann::json json;
  if (!jitsuyo::load_config(path, "locomotion.json", json)) {
    return false;
  }

  return set_config(json);
}

bool DAVGPlanner::set_config(const nlohmann::json & json)
{
  nlohmann::json planner_section;
  if (jitsuyo::assign_val(json, "davg_planner", planner_section)) {
    bool valid_section = true;
    double turning_penalty;
    int polygon_edges;
    double inflation_radius;
    double path_blocked_multiplier;
    double goal_clear_multiplier;
    double obstacle_ema_alpha;

    valid_section = jitsuyo::assign_val(planner_section, "turning_penalty", turning_penalty);
    valid_section &= jitsuyo::assign_val(planner_section, "polygon_edges", polygon_edges);
    valid_section &= jitsuyo::assign_val(planner_section, "inflation_radius", inflation_radius);
    valid_section &= jitsuyo::assign_val(planner_section, "path_blocked_multiplier", path_blocked_multiplier);
    valid_section &= jitsuyo::assign_val(planner_section, "goal_clear_multiplier", goal_clear_multiplier);
    valid_section &= jitsuyo::assign_val(planner_section, "side_penalty", side_penalty_);
    valid_section &= jitsuyo::assign_val(planner_section, "obstacle_merge_radius", obstacle_merge_radius_);
    valid_section &= jitsuyo::assign_val(planner_section, "obstacle_ema_alpha", obstacle_ema_alpha);
    valid_section &= jitsuyo::assign_val(planner_section, "enable_localization", enable_localization_);


    if (valid_section) {
      set_turning_penalty(turning_penalty);
      set_polygon_edges(polygon_edges);
      set_inflation_radius(inflation_radius);
      set_path_blocked_multiplier(path_blocked_multiplier);
      set_goal_clear_multiplier(goal_clear_multiplier);
      set_obstacle_ema_alpha(obstacle_ema_alpha);
    }
    
    return valid_section;
  }

  return false;
}

double DAVGPlanner::get_inflation_radius() const { return inflation_radius_; }

double DAVGPlanner::get_path_blocked_multiplier() const { return path_blocked_multiplier_; }

double DAVGPlanner::get_goal_clear_multiplier() const { return goal_clear_multiplier_; }

double DAVGPlanner::get_side_penalty() const { return side_penalty_; }

double DAVGPlanner::get_sticky_threshold() const { return sticky_threshold_; }

double DAVGPlanner::get_obstacle_merge_radius() const { return obstacle_merge_radius_; }

double DAVGPlanner::get_obstacle_ema_alpha() const { return obstacle_ema_alpha_; }

bool DAVGPlanner::is_localization_enabled() const { return enable_localization_; }

bool DAVGPlanner::set_inflation_radius(double new_radius)
{
  if (new_radius < 0.0) {
    std::cerr << "inflation radius must be >= 0\n";
    return false;
  }
  inflation_radius_ = new_radius;
  return true;
}

bool DAVGPlanner::set_path_blocked_multiplier(double new_multiplier)
{
  if (new_multiplier < 0.0) {
    std::cerr << "path blocked multiplier must be >= 0\n";
    return false;
  }
  path_blocked_multiplier_ = new_multiplier;
  return true;
}

bool DAVGPlanner::set_goal_clear_multiplier(double new_multiplier)
{
  if (new_multiplier < 0.0) {
    std::cerr << "goal clear multiplier must be >= 0\n";
    return false;
  }
  goal_clear_multiplier_ = new_multiplier;
  return true;
}

bool DAVGPlanner::set_polygon_edges(int new_edges)
{
  if (new_edges < 3) {
    std::cerr << "polygon edges must be >= 3\n";
    return false;
  }
  polygon_edges_ = new_edges;
  return true;
}

bool DAVGPlanner::set_turning_penalty(double new_value)
{
  turning_penalty_ = new_value;
  return true;
}

bool DAVGPlanner::set_obstacle_ema_alpha(double new_alpha)
{
  if (new_alpha < 0.0 || new_alpha > 1.0) {
    std::cerr << "obstacle EMA alpha must be between 0 and 1\n";
    return false;
  }
  obstacle_ema_alpha_ = new_alpha;
  return true;
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
    const double t = keisan::clamp((ao_x * ab_x + ao_y * ab_y) / dist_sq, 0.0, 1.0);
    const double px = p_a.x + t * ab_x;
    const double py = p_a.y + t * ab_y;

    const double closest = std::hypot(obs.position.x - px, obs.position.y - py);
    const double threshold = obs.radius + inflation_radius - 0.1;
    if (closest < threshold) {
      std::cout << "path to goal is blocked, distance obstacle to safe line: " << closest << " < " << threshold << "\n";
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
  const std::vector<Obstacle> & obstacles,
  const std::optional<keisan::Point2> & previous_target)
{
  auto total_start = high_resolution_clock::now();

  const double SG_x = goal_pos.x - start_pos.x;
  const double SG_y = goal_pos.y - start_pos.y;
  const double SG_dist_sq = SG_x * SG_x + SG_y * SG_y;

  double prev_side = 0.0;
  if (previous_target.has_value()) {
    if (SG_dist_sq > 100.0) {
      prev_side = (SG_x * (previous_target->y - start_pos.y) - SG_y * (previous_target->x - start_pos.x));

      if (std::abs(prev_side) < 8.0 * std::sqrt(SG_dist_sq)) {
        prev_side = 0.0;
      }
    }
  }

  // search for active obstacles  
  std::vector<Obstacle> active_obstacles = obstacles;

  auto active_search_end = high_resolution_clock::now();

  // generate vertices for each active obstacle
  auto graph_const_start = high_resolution_clock::now();
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
  auto vertex_gen_end = high_resolution_clock::now();

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

  auto vis_loop_start = high_resolution_clock::now();
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
  auto vis_loop_end = high_resolution_clock::now();

  // handle start/goal nodes that are inside an inflation zone.
  auto trap_start = high_resolution_clock::now();
  connect_trapped_node(start_id, all_nodes, total_nodes, active_obstacles, graph);
  connect_trapped_node(goal_id, all_nodes, total_nodes, active_obstacles, graph);
  auto trap_end = high_resolution_clock::now();

  auto graph_const_end = high_resolution_clock::now();

  // augmented A* with turning penalty
  auto astar_start = high_resolution_clock::now();
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

      double current_side_penalty = 0.0;
      if (prev_side != 0.0) {
        double node_side = (SG_x * (ny - start_pos.y) - SG_y * (nx - start_pos.x));
        if (prev_side * node_side < 0.0) {
          const double SG_dist = std::sqrt(SG_dist_sq);
          const double perp_dist = SG_dist > 0.0 ? std::abs(node_side) / SG_dist : 0.0;
          current_side_penalty = side_penalty_ + perp_dist * 1.5;
        }
      }

      const double new_g = curr.actual_cost + dist + turning_penalty_ * std::abs(d_theta) + current_side_penalty;

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
  auto astar_end = high_resolution_clock::now();

  if (path_ids.empty()) {
    std::cout << "A* failed to find a path\n";
  }

  // output route
  std::vector<keisan::Point2> route;
  route.reserve(path_ids.size());
  for (const int id : path_ids) {
    route.push_back(all_nodes[id]);
  }

  auto total_end = high_resolution_clock::now();
  auto total_duration = duration_cast<milliseconds>(total_end - total_start);
  if (!route.empty()) {
    std::cout << "path found with " << route.size() << " points.\n";
  }
  std::cout << "[DAVGPlanner] Total execution time: " << total_duration.count() << "ms\n";

  return route;
}

} // namespace suiryoku
