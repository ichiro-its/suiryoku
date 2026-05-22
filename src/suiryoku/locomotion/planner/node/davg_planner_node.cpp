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

#include "suiryoku/locomotion/planner/node/davg_planner_node.hpp"

#include <cmath>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

#include "jitsuyo/config.hpp"
#include "nlohmann/json.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

namespace suiryoku
{
DAVGPlannerNode::DAVGPlannerNode(const rclcpp::NodeOptions &options)
: Node("davg_planner_node", options),
  planner(
    declare_parameter<double>("turning_penalty", 1.0),
    declare_parameter<int>("polygon_edges", 8),
    declare_parameter<double>("inflation_radius", 10.0))
{
    walking_status_subscriber = create_subscription<WalkingStatus>(
      "walking/status", 10, std::bind(&DAVGPlannerNode::on_odometry_pose, this, std::placeholders::_1));
    fused_position_subscriber = create_subscription<Point2>(
      "/localization/fused_pose", 10, std::bind(&DAVGPlannerNode::on_fused_pose, this, std::placeholders::_1));
    goal_pose_subscriber = create_subscription<aruku_interfaces::msg::Point2>(
      "planner/goal_pose", 10, std::bind(&DAVGPlannerNode::on_goal_pose, this, std::placeholders::_1));
    obstacles_subscriber = create_subscription<suiryoku_interfaces::msg::Obstacles>(
      "planner/active_obstacles", 10, std::bind(&DAVGPlannerNode::on_obstacle, this, std::placeholders::_1));

    route_publisher = create_publisher<suiryoku_interfaces::msg::Route>("planner/route", 10);
    visual_route_publisher = create_publisher<nav_msgs::msg::Path>("planner/path_visualization", 10);

    // run planner every 150 ms
    timer = create_wall_timer(150ms, std::bind(&DAVGPlannerNode::timer_callback, this));
}

void DAVGPlannerNode::load_config(const std::string &path)
{
  if (!planner.load_config(path)) {
    RCLCPP_WARN(this->get_logger(), "Failed to load config from %s", path.c_str());
  }
}

void DAVGPlannerNode::on_fused_pose(const Point2::SharedPtr msg)
{ 
  latest_fused_pose = std::make_shared<Point2>();
  latest_fused_pose->x = msg->x;
  latest_fused_pose->y = msg->y;
}

void DAVGPlannerNode::on_odometry_pose(const WalkingStatus::SharedPtr msg)
{ 
  latest_odometry_pose = std::make_shared<Point2>();
  latest_odometry_pose->x = msg->odometry.x;
  latest_odometry_pose->y = msg->odometry.y;
}

void DAVGPlannerNode::on_goal_pose(const Point2::SharedPtr msg) 
{ 
  RCLCPP_INFO(this->get_logger(), "received goal pose: x=%.2f, y=%.2f", msg->x, msg->y);
  latest_goal_pose = msg; 
}

void DAVGPlannerNode::on_obstacle(const Obstacles::SharedPtr msg)
{
    latest_obstacles.clear();
    latest_obstacles.reserve(msg->obstacles.size());
    
    for (const auto &obstacle : msg->obstacles) {
        suiryoku::Obstacle obs;
        obs.position = keisan::Point2(obstacle.position.x, obstacle.position.y);
        obs.radius = obstacle.radius;
        latest_obstacles.push_back(obs);
    }
}

void DAVGPlannerNode::timer_callback()
{
  bool enable_localization = planner.is_localization_enabled();

  auto current_pose = enable_localization ? latest_fused_pose : latest_odometry_pose;
  if (!current_pose || !latest_goal_pose) {
    if (!current_pose && !latest_goal_pose) {
      RCLCPP_DEBUG(this->get_logger(), "waiting for start pose and goal pose");
    } else {
      RCLCPP_DEBUG(this->get_logger(), "waiting for %s", !current_pose ? "start pose" : "goal pose");
    }
    return;
  }
  run_planner(current_pose);
}

void DAVGPlannerNode::run_planner(const Point2::SharedPtr current_pose)
{
    const keisan::Point2 start_pos(current_pose->x, current_pose->y);
    const keisan::Point2 goal_pos(latest_goal_pose->x, latest_goal_pose->y);

    const double start_theta = std::atan2(goal_pos.y - start_pos.y, goal_pos.x - start_pos.x);
    const auto route = planner.calculate_path(start_pos, start_theta, goal_pos, latest_obstacles);

    if (route.empty()) {
        RCLCPP_WARN(this->get_logger(), "planner returned an empty path");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "planner found a route with %zu points", route.size());

    suiryoku_interfaces::msg::Route route_msg;
    route_msg.points.reserve(route.size());
    for (const auto & point : route) {
        suiryoku_interfaces::msg::Point2 pt;
        pt.x = point.x;
        pt.y = point.y;
        route_msg.points.push_back(pt);
    }
    route_publisher->publish(route_msg);

    // for rviz visualization
    nav_msgs::msg::Path viz_msg;
    viz_msg.header.stamp = this->now();
    viz_msg.header.frame_id = "map";
    viz_msg.poses.reserve(route.size());
    for (const auto & pt : route) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = viz_msg.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        viz_msg.poses.push_back(pose);
    }
    visual_route_publisher->publish(viz_msg);
}

} // namespace suiryoku
