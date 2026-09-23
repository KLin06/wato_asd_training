#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

    // Timer-driven: have we reached the goal, or timed out and need to replan?
    void timerCallback();

  private:
    const int QUEUE_SIZE = 10;
    const int TIMER_PERIOD_SECONDS = 1;

    robot::PlannerCore planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
