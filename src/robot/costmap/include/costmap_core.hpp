#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>
#include <cstdint>
#include <utility>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Step 1: reset the grid to all-free before processing a new scan
    void initializeCostmap(double maxDistance);

    // Step 2: loop over scan->ranges, convert each valid reading to a grid
    // cell via convertToGrid, and mark it via markObstacle
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Step 4: flatten occupancy_grid_ into an OccupancyGrid's info + data.
    // header (frame_id/stamp) is left default — CostmapNode fills that in,
    // since it needs the node's clock and the scan's frame, not CostmapCore.
    nav_msgs::msg::OccupancyGrid getOccupancyGrid();

  private:
    const double RESOLUTION = 0.1;
    // Inflate obstacle costs out to 2 m. The planner rejects cells with
    // cost >= 50, which corresponds to the inner 1 m of this radius.
    const double INFLATION_RADIUS = 2.0;
    const int8_t MAX_COST = 100;

    // Convert a polar (range, angle) reading into grid cell indices
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);

    // Mark a single grid cell as occupied
    void markObstacle(int x_grid, int y_grid);

    // Step 3: spread decaying cost around occupied cells
    void inflateObstacles();

    // Euclidean distance, in meters, between two grid cells
    double calculateDistance(int x1, int y1, int x2, int y2);

    // cost = max_cost * (1 - distance / inflation_radius)
    int8_t calculateCost(double distance);

    // Whether (x, y) falls inside the current grid's bounds
    bool inBounds(int x, int y) const;

    rclcpp::Logger logger_;
    // occupancy_grid_[y][x] — 2D for easy indexing during markObstacle/inflateObstacles;
    // flattened into OccupancyGrid.data only when publishing later
    std::vector<std::vector<int8_t>> occupancy_grid_;
    // Cached dimensions of occupancy_grid_, set by initializeCostmap so
    // convertToGrid/markObstacle don't each re-derive them from .size()
    int32_t grid_width_ = 0;
    int32_t grid_height_ = 0;
    // (x_grid, y_grid) of every cell markObstacle has set this scan,
    // so inflateObstacles doesn't have to rescan the whole grid to find them
    std::vector<std::pair<int32_t, int32_t>> obstacles_;

};

}

#endif
