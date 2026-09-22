#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

double MapMemoryCore::calculateDistance(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr& costmap) {
  latest_costmap_ = *costmap;
  costmap_updated_ = true;
}

void MapMemoryCore::updatePosition(double x, double y) {
  double distance = calculateDistance(last_x_, last_y_, x, y);
  if (distance >= UPDATE_DISTANCE) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

bool MapMemoryCore::shouldUpdateMap() const {
  return should_update_map_ && costmap_updated_;
}

void MapMemoryCore::integrateCostmap() {
  // TODO: transform and merge latest_costmap_ into global_map_
  // (grid alignment/merging logic)
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

void MapMemoryCore::resetUpdateFlags() {
  should_update_map_ = false;
}

}
