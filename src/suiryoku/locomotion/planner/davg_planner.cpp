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
#include <cmath>
#include <limits>
#include <queue>

namespace suiryoku
{
DAVGPlanner::DAVGPlanner(double turning_penalty, int polygon_edges, double inflation_radius)
{
    turning_penalty_ = turning_penalty;
    polygon_edges_ = polygon_edges;
    inflation_radius_ = inflation_radius;
}

double DAVGPlanner::get_inflation_radius() { return inflation_radius_; }

void DAVGPlanner::set_inflation_radius(double new_radius)
{
    if (new_radius < 0.0) {
        std::cerr << "radius must greater than 0\n";
        return;
    }
    inflation_radius_ = new_radius;
}

void DAVGPlanner::set_polygon_edges(double new_edges)
{
    if (new_edges < 3) {
        std::cerr << "edges must greater or equal 3\n";
        return;
    }
    polygon_edges_ = static_cast<int>(new_edges);
}

void DAVGPlanner::set_turning_penalty(double new_value) { turning_penalty_ = new_value; }

bool DAVGPlanner::calculate_distance_to_line(
    const keisan::Point2 &start,
    const keisan::Point2 &goal,
    const Obstacle &obstacle,
    double left_width,
    double right_width,
    double &signed_dist) const
{
    // SG = start-goal
    double SG_x = goal.x - start.x;
    double SG_y = goal.y - start.y;
    double SG_distance = (SG_x * SG_x) + (SG_y * SG_y);

    if (SG_distance == 0.0) return false;

    double SG_len = std::sqrt(SG_distance);

    // SO = start-obstacle
    double SO_x = obstacle.position.x - start.x;
    double SO_y = obstacle.position.y - start.y;

    double t = (SO_x * SG_x + SO_y * SG_y) / SG_distance;
    double radius_border = obstacle.radius + inflation_radius_;
    double proj_length = t * SG_len;

    // if obstacle projection exceed end of start/ goal line
    if (proj_length < -radius_border || proj_length > SG_len + radius_border) {
        return false;
    }

    double cross_prod = (SG_x * SO_y) - (SG_y * SO_x);
    signed_dist = cross_prod / SG_len;

    double obs_min = signed_dist - radius_border;
    double obs_max = signed_dist + radius_border;

    if (obs_min <= left_width && obs_max >= -right_width) {
        return true;
    }

    return false;
}

bool DAVGPlanner::is_line_colliding(
    const keisan::Point2 &p_a,
    const keisan::Point2 &p_b,
    const std::vector<Obstacle> &active_obstacles,
    double inflation_radius) const
{
    double AB_x = p_b.x - p_a.x;
    double AB_y = p_b.y - p_a.y;
    double dist_square = (AB_x * AB_x) + (AB_y * AB_y);

    if (dist_square == 0.0) {
        return false;
    }

    for (const auto &obs : active_obstacles) {
        double AO_x = obs.position.x - p_a.x;
        double AO_y = obs.position.y - p_a.y;
        
        double t = (AO_x * AB_x + AO_y * AB_y) / dist_square;
        double safe_t = std::max(0.0, std::min(1.0, t));

        double P_x = p_a.x + (safe_t * AB_x);
        double P_y = p_a.y + (safe_t * AB_y);

        double dist_x = obs.position.x - P_x;
        double dist_y = obs.position.y - P_y;
        double closest_dist = std::hypot(dist_x, dist_y);

        double radius_border = obs.radius + inflation_radius - 0.1;

        if (closest_dist < radius_border) {
            return true;
        }
    }
    return false;
}

void DAVGPlanner::force_escape(
    int node_idx,
    const std::vector<keisan::Point2> &all_nodes,
    int total_nodes,
    const std::vector<Obstacle> &active_obstacles,
    Graph &graph) const
{
    double nx = all_nodes[node_idx].x;
    double ny = all_nodes[node_idx].y;

    for (const auto &obs : active_obstacles) {
        double dist_to_obs = std::hypot(obs.position.x - nx, obs.position.y - ny);
        
        // trap threshold
        double trap_threshold = obs.radius + inflation_radius_ * 0.5;

        if (dist_to_obs < trap_threshold) {
            for (int v_idx = 1; v_idx < total_nodes - 1; ++v_idx) {
                double vx = all_nodes[v_idx].x;
                double vy = all_nodes[v_idx].y;

                // check physical collision
                std::vector<Obstacle> current_obs = {obs};
                bool is_hit_physical = is_line_colliding(all_nodes[node_idx], all_nodes[v_idx], current_obs, 0.0);

                if (!is_hit_physical) {
                    // check other obstacle
                    std::vector<Obstacle> other_obs;
                    for (const auto &o : active_obstacles) {
                        if (std::abs(o.position.x - obs.position.x) > 0.01 || 
                            std::abs(o.position.y - obs.position.y) > 0.01) {
                            other_obs.push_back(o);
                        }
                    }

                    bool is_hit_others = is_line_colliding(all_nodes[node_idx], all_nodes[v_idx], other_obs, inflation_radius_);

                    if (!is_hit_others) {
                        double dist_to_v = std::hypot(vx - nx, vy - ny);
                        graph[node_idx].push_back({v_idx, dist_to_v});
                        graph[v_idx].push_back({node_idx, dist_to_v});
                    }
                }
            }

            break;
        }
    }
}

std::vector<keisan::Point2> DAVGPlanner::calculate_path(
    const keisan::Point2 &start_pos,
    double start_theta,
    const keisan::Point2 &goal_pos,
    const std::vector<Obstacle> &obstacles)
{
    std::vector<Obstacle> active_obstacles;
    double active_region_left = 0.0;
    double active_region_right = 0.0;

    // search for active obstacles
    while (true) {
        bool is_new_obstacle = false;

        for (const auto &obs : obstacles) {
            // skip if already in active_obstacles
            bool already_active = false;
            for (const auto &active_obs : active_obstacles) {
                if (std::abs(obs.position.x - active_obs.position.x) < 0.01 && std::abs(obs.position.y - active_obs.position.y) < 0.01) {
                    already_active = true;
                    break;
                }
            }
            if (already_active) continue;

            double signed_dist = 0.0;
            bool is_inside = calculate_distance_to_line(
                start_pos, goal_pos, obs, 
                active_region_left, active_region_right, signed_dist);

            if (is_inside) {
                active_obstacles.push_back(obs);
                is_new_obstacle = true;

                double radius_border = obs.radius + inflation_radius_;
                double obs_max = signed_dist + radius_border;
                double obs_min = signed_dist - radius_border;

                if (obs_max > active_region_left) active_region_left = obs_max;
                if (-obs_min > active_region_right) active_region_right = -obs_min;
            }
        }
        if (!is_new_obstacle) break;
    }

    // make vertices for active obstacles
    std::vector<keisan::Point2> vertices;
    double edge_dist = 2.0 * M_PI / polygon_edges_;
    double cos_factor = std::cos(M_PI / polygon_edges_);

    for (const auto &obs : active_obstacles) {
        double safe_radius = (obs.radius + inflation_radius_) / cos_factor;

        for (int i = 0; i < polygon_edges_; ++i) {
            double edge_now = i * edge_dist;
            double point_x = obs.position.x + safe_radius * std::cos(edge_now);
            double point_y = obs.position.y + safe_radius * std::sin(edge_now);
            vertices.push_back({point_x, point_y});
        }
    }

    // create edges for each node that is not collide with each other
    std::vector<keisan::Point2> all_nodes;
    all_nodes.push_back(start_pos);
    for (const auto &v : vertices) all_nodes.push_back(v);
    all_nodes.push_back(goal_pos);

    int total_nodes = all_nodes.size();
    Graph graph(total_nodes);

    for (int i = 0; i < total_nodes; ++i) {
        for (int j = i + 1; j < total_nodes; ++j) {
            if (!is_line_colliding(all_nodes[i], all_nodes[j], active_obstacles, inflation_radius_)) {
                double dist = std::hypot(all_nodes[i].x - all_nodes[j].x, all_nodes[i].y - all_nodes[j].y);
                graph[i].push_back({j, dist});
                graph[j].push_back({i, dist});
            }
        }
    }

    int start_id = 0;
    int goal_id = total_nodes - 1;

    force_escape(start_id, all_nodes, total_nodes, active_obstacles, graph);
    force_escape(goal_id, all_nodes, total_nodes, active_obstacles, graph);

    // augmented A* search
    struct AStarNode {
        double approx_cost;
        double actual_cost;
        int node_id;
        double theta;
        std::vector<int> path;

        // min heap priority queue
        bool operator>(const AStarNode &other) const {
            return approx_cost > other.approx_cost;
        }
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> queue;
    queue.push({0.0, 0.0, start_id, start_theta, {start_id}});

    std::vector<double> costs(total_nodes, std::numeric_limits<double>::infinity());
    costs[start_id] = 0.0;
    std::vector<int> path_ids;

    while (!queue.empty()) {
        auto curr = queue.top();
        queue.pop();

        if (curr.node_id == goal_id) {
            path_ids = curr.path;
            break;
        }

        // ignore if there is cheaper route
        if (curr.actual_cost > costs[curr.node_id]) continue;

        double curr_x = all_nodes[curr.node_id].x;
        double curr_y = all_nodes[curr.node_id].y;

        for (const auto &neighbor : graph[curr.node_id]) {
            int neighbor_id = neighbor.first;
            double dist = neighbor.second;

            double neighbor_x = all_nodes[neighbor_id].x;
            double neighbor_y = all_nodes[neighbor_id].y;

            // calculate turning penalty
            double neighbor_theta = std::atan2(neighbor_y - curr_y, neighbor_x - curr_x);
            double d_theta = neighbor_theta - curr.theta;
            
            // normalize angle
            while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
            while (d_theta < -M_PI) d_theta += 2.0 * M_PI;

            double penalty = turning_penalty_ * std::abs(d_theta);
            double new_actual_cost = curr.actual_cost + dist + penalty;

            if (new_actual_cost < costs[neighbor_id]) {
                costs[neighbor_id] = new_actual_cost;

                // calculate heuristic cost
                double dx = goal_pos.x - neighbor_x;
                double dy = goal_pos.y - neighbor_y;
                double h_cost = std::hypot(dx, dy);

                double new_approx_cost = new_actual_cost + h_cost;

                std::vector<int> neighbor_path = curr.path;
                neighbor_path.push_back(neighbor_id);

                queue.push({new_approx_cost, new_actual_cost, neighbor_id, neighbor_theta, neighbor_path});
            }
        }
    }

    std::vector<keisan::Point2> route;
    for (int id : path_ids) {
        route.push_back(all_nodes[id]);
    }

    return route;
}

} // namespace suiryoku