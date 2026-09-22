#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <optional>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Store the most recently received local costmap
    void updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr& costmap);

    // Track the robot's position; marks the map "due for an update" once
    // it has moved UPDATE_DISTANCE meters since the last recorded position.
    // orientation is just recorded as-is, not part of the distance check —
    // integrateCostmap will need it to rotate the local costmap into the
    // global frame.
    void updatePosition(double x, double y, const geometry_msgs::msg::Quaternion& orientation);

    // True once both a new costmap has arrived AND the robot has moved far enough
    bool shouldUpdateMap() const;

    // Merge the latest costmap into the persistent global map
    void integrateCostmap();

    nav_msgs::msg::OccupancyGrid getGlobalMap() const;

    // Clear the "due for an update" flags after a publish
    void resetUpdateFlags();

  private:
    // Yaw (rotation about the world's vertical axis) extracted from
    // orientation_, via tf2 — the only part of a 3D orientation a 2D
    // occupancy grid actually needs
    double getYaw() const;

    const double UPDATE_DISTANCE = 1.5;
    const double RESOLUTION = 0.1;
    const double MAP_SIZE_METERS = 40.0;

    double calculateDistance(double x1, double y1, double x2, double y2);

    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    // No value until the first odometry reading arrives — updatePosition
    // uses this to record an initial position instead of comparing against
    // a made-up starting point
    std::optional<double> last_x_;
    std::optional<double> last_y_;
    geometry_msgs::msg::Quaternion orientation_;
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
};

}

#endif
