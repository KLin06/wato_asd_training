#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Store the most recently received local costmap
    void updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr& costmap);

    // Track the robot's position; marks the map "due for an update" once
    // it has moved UPDATE_DISTANCE meters since the last recorded position
    void updatePosition(double x, double y);

    // True once both a new costmap has arrived AND the robot has moved far enough
    bool shouldUpdateMap() const;

    // Merge the latest costmap into the persistent global map
    void integrateCostmap();

    nav_msgs::msg::OccupancyGrid getGlobalMap() const;

    // Clear the "due for an update" flags after a publish
    void resetUpdateFlags();

  private:
    const double UPDATE_DISTANCE = 5.0;

    double calculateDistance(double x1, double y1, double x2, double y2);

    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    double last_x_ = 0.0;
    double last_y_ = 0.0;
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
};

}

#endif
