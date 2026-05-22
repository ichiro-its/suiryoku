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

#ifndef SUIRYOKU__LOCOMOTION__PLANNER__NODE__DAVG_PLANNER_NODE_HPP_
#define SUIRYOKU__LOCOMOTION__PLANNER__NODE__DAVG_PLANNER_NODE_HPP_

#include <memory>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "suiryoku_interfaces/msg/obstacles.hpp"
#include "suiryoku_interfaces/msg/route.hpp"
#include "aruku_interfaces/msg/point2.hpp"
#include "aruku_interfaces/msg/status.hpp"

 
#include "suiryoku/locomotion/planner/davg_planner.hpp"

namespace suiryoku
{

class DAVGPlannerNode : public rclcpp::Node
{
public:
    explicit DAVGPlannerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    void load_config(const std::string & path);

private:
    using Point2 = aruku_interfaces::msg::Point2;
    using Obstacles = suiryoku_interfaces::msg::Obstacles;
    using Route = suiryoku_interfaces::msg::Route;
    using WalkingStatus = aruku_interfaces::msg::Status;

    void on_odometry_pose(const WalkingStatus::SharedPtr msg);
    void on_fused_pose(const Point2::SharedPtr msg);
    void on_goal_pose(const aruku_interfaces::msg::Point2::SharedPtr msg);
    void on_obstacle(const Obstacles::SharedPtr msg);
    void timer_callback();

    void run_planner(const Point2::SharedPtr current_pose);

    rclcpp::Subscription<WalkingStatus>::SharedPtr walking_status_subscriber;
    rclcpp::Subscription<Point2>::SharedPtr fused_position_subscriber;
    rclcpp::Subscription<Point2>::SharedPtr goal_pose_subscriber;
    rclcpp::Subscription<Obstacles>::SharedPtr obstacles_subscriber;

    rclcpp::Publisher<Route>::SharedPtr route_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visual_route_publisher; // for rviz visualization

    rclcpp::TimerBase::SharedPtr timer;
    
    Point2::SharedPtr latest_fused_pose;
    Point2::SharedPtr latest_odometry_pose;
    Point2::SharedPtr latest_goal_pose;
    std::vector<suiryoku::Obstacle> latest_obstacles;
    
    suiryoku::DAVGPlanner planner;
};

} // namespace suiryoku


#endif // SUIRYOKU__LOCOMOTION__PLANNER__NODE__DAVG_PLANNER_NODE_HPP_