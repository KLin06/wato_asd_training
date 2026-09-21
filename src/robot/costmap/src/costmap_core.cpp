#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap() {
  // TODO: resize/clear grid_ to all-free
}

void CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range > scan->range_min && range < scan->range_max) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  (void)range;
  (void)angle;
  x_grid = 0;
  y_grid = 0;
  // TODO: convert polar (range, angle) to grid indices
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  (void)x_grid;
  (void)y_grid;
  // TODO: set grid_ cell to occupied
}

void CostmapCore::inflateObstacles() {
  // TODO: spread decaying cost around occupied cells within inflation radius
}

}