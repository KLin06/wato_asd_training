#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap(double maxDistance) {
  int32_t size = 2 * static_cast<int32_t>(maxDistance / RESOLUTION);
  occupancy_grid_.assign(size, std::vector<int8_t>(size, 0));
  grid_height_ = size;
  grid_width_ = size;
  obstacles_.clear();
}

void CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap(scan->range_max);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range > scan->range_min && range < scan->range_max) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  // Cartesian coordinates of the detected point, relative to the robot
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  // Origin: the robot sits at the center of the grid, so cell (origin_x, origin_y)
  // corresponds to real-world (0, 0)
  int origin_x = grid_width_ / 2;
  int origin_y = grid_height_ / 2;

  x_grid = origin_x + static_cast<int>(x / RESOLUTION);
  y_grid = origin_y + static_cast<int>(y / RESOLUTION);
}

bool CostmapCore::inBounds(int x, int y) const {
  return x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_;
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (!inBounds(x_grid, y_grid)) {
    return;
  }
  occupancy_grid_[y_grid][x_grid] = MAX_COST;
  obstacles_.emplace_back(x_grid, y_grid);
}

void CostmapCore::inflateObstacles() {
  int32_t radius_cells = static_cast<int32_t>(std::ceil(INFLATION_RADIUS / RESOLUTION));

  for (const auto& obstacle : obstacles_) {
    int32_t ox = obstacle.first;
    int32_t oy = obstacle.second;

    for (int32_t dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int32_t dx = -radius_cells; dx <= radius_cells; ++dx) {
        int32_t nx = ox + dx;
        int32_t ny = oy + dy;

        if (!inBounds(nx, ny)) {
          continue;
        }

        double distance = calculateDistance(ox, oy, nx, ny);
        if (distance > INFLATION_RADIUS) {
          continue;
        }

        int8_t cost = calculateCost(distance);
        if (cost > occupancy_grid_[ny][nx]) {
          occupancy_grid_[ny][nx] = cost;
        }
      }
    }
  }
}

double CostmapCore::calculateDistance(int x1, int y1, int x2, int y2) {
  double dx = (x2 - x1) * RESOLUTION;
  double dy = (y2 - y1) * RESOLUTION;
  return std::sqrt(dx * dx + dy * dy);
}

int8_t CostmapCore::calculateCost(double distance) {
  return static_cast<int8_t>(MAX_COST * (1 - distance / INFLATION_RADIUS));
}

nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid() {
  nav_msgs::msg::OccupancyGrid grid_msg;

  grid_msg.info.resolution = RESOLUTION;
  grid_msg.info.width = static_cast<uint32_t>(grid_width_);
  grid_msg.info.height = static_cast<uint32_t>(grid_height_);

  // occupancy_grid_[y][x] is centered on the robot; the origin is the
  // real-world pose of cell (0, 0) — the grid's bottom-left corner
  grid_msg.info.origin.position.x = -(grid_width_ / 2.0) * RESOLUTION;
  grid_msg.info.origin.position.y = -(grid_height_ / 2.0) * RESOLUTION;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  grid_msg.data.reserve(grid_width_ * grid_height_);
  for (const auto& row : occupancy_grid_) {
    grid_msg.data.insert(grid_msg.data.end(), row.begin(), row.end());
  }

  return grid_msg;
}

}
