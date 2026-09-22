#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <string>

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void subscribeToCostmap();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);

    void subscribeToOdom();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Timer-driven: publishes an updated /map whenever the core says one is due
    void updateMap();

  private:
    const int QUEUE_SIZE = 10;
    const int TIMER_PERIOD_SECONDS = 1;

    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // The world frame the global map is expressed in — captured from the
    // odometry we track position/orientation in, same frame the transform
    // math in integrateCostmap() actually operates in
    std::string world_frame_id_;
};

#endif
