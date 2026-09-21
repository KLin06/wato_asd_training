#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>
#include <cstdint>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Step 1: reset the grid to all-free before processing a new scan
    void initializeCostmap();

    // Step 2: loop over scan->ranges, convert each valid reading to a grid
    // cell via convertToGrid, and mark it via markObstacle
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Convert a polar (range, angle) reading into grid cell indices
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);

    // Mark a single grid cell as occupied
    void markObstacle(int x_grid, int y_grid);

    // Step 3: spread decaying cost around occupied cells
    void inflateObstacles();

  private:
    rclcpp::Logger logger_;
    // grid_[y][x] — 2D for easy indexing during markObstacle/inflateObstacles;
    // flattened into OccupancyGrid.data only when publishing later
    std::vector<std::vector<int8_t>> grid_;

};

}

#endif