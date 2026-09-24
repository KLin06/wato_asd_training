#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <optional>

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void subscribeToMap();
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    void subscribeToGoal();
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);

    void subscribeToOdom();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Timer-driven: goal reached, timed out, or replan from the current position
    void timerCallback();

  private:
    // Publish whatever the core decided to send, if anything
    void publishIfAny(const std::optional<nav_msgs::msg::Path>& path);

    const int QUEUE_SIZE = 10;
    // 500 ms: fast enough that "goal reached" is noticed within ~0.3 m at
    // 0.6 m/s, while A* (a few ms here) stays a negligible load
    const int TIMER_PERIOD_MS = 500;

    robot::PlannerCore planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
