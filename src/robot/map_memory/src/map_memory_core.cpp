#include "map_memory_core.hpp"
#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger)
{
  int32_t size = static_cast<int32_t>(MAP_SIZE_METERS / RESOLUTION);

  global_map_.info.resolution = RESOLUTION;
  global_map_.info.width = static_cast<uint32_t>(size);
  global_map_.info.height = static_cast<uint32_t>(size);

  // Fixed at world (0, 0) — the room's own center, per the sim's wall
  // placement — not the robot's arbitrary starting position
  global_map_.info.origin.position.x = -(size / 2.0) * RESOLUTION;
  global_map_.info.origin.position.y = -(size / 2.0) * RESOLUTION;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.assign(size * size, 0);
}

double MapMemoryCore::calculateDistance(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr& costmap) {
  latest_costmap_ = *costmap;
  costmap_updated_ = true;
}

void MapMemoryCore::updatePosition(double x, double y, const geometry_msgs::msg::Quaternion& orientation) {
  // Always track the latest pose for fusion. last_x_/last_y_ only move when
  // the distance gate fires, so fusing with them pairs a fresh costmap and
  // fresh yaw with a position up to one timer period stale (~0.6 m at
  // 0.6 m/s) — measured as walls jumping ~0.9 m between /map messages.
  current_x_ = x;
  current_y_ = y;
  orientation_ = orientation;

  if (!last_x_.has_value() || !last_y_.has_value()) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
    return;
  }

  double distance = calculateDistance(*last_x_, *last_y_, x, y);
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
  double yaw = getYaw();
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  int32_t local_width = static_cast<int32_t>(latest_costmap_.info.width);
  int32_t local_height = static_cast<int32_t>(latest_costmap_.info.height);
  int32_t global_width = static_cast<int32_t>(global_map_.info.width);
  int32_t global_height = static_cast<int32_t>(global_map_.info.height);

  for (int32_t ly = 0; ly < local_height; ++ly) {
    for (int32_t lx = 0; lx < local_width; ++lx) {
      int8_t value = latest_costmap_.data[ly * local_width + lx];
      // Skip 0 as well as -1: the costmap never raycasts, so 0 means "no
      // return landed here", not "observed free". Copying it in would erase
      // obstacles mapped earlier that are merely occluded or out of range now.
      if (value <= 0) {
        continue;
      }

      // Local costmap cell -> real-world offset, relative to the robot.
      // +0.5: an index names the cell's bottom-left corner; its centre is
      // half a cell further in. Without it every point is biased half a cell
      // down-left in the LIDAR frame, which after rotation becomes a
      // yaw-dependent smear in the world.
      double local_x = latest_costmap_.info.origin.position.x + (lx + 0.5) * latest_costmap_.info.resolution;
      double local_y = latest_costmap_.info.origin.position.y + (ly + 0.5) * latest_costmap_.info.resolution;

      // Rotate into the world frame by the robot's current heading, then
      // translate by the robot's current world position
      double global_x = *current_x_ + (local_x * cos_yaw - local_y * sin_yaw);
      double global_y = *current_y_ + (local_x * sin_yaw + local_y * cos_yaw);

      // World position -> global_map_ grid indices. floor, for the same
      // reason as the costmap: truncation would fold (-0.1, 0) m past the
      // map edge into cell 0 instead of dropping it.
      int32_t gx = static_cast<int32_t>(std::floor((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution));
      int32_t gy = static_cast<int32_t>(std::floor((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution));

      if (gx < 0 || gx >= global_width || gy < 0 || gy >= global_height) {
        continue;
      }

      global_map_.data[gy * global_width + gx] = value;
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

void MapMemoryCore::resetUpdateFlags() {
  should_update_map_ = false;
  // Also require a fresh costmap before the next fusion, so we never
  // re-fuse an old scan against a newer pose
  costmap_updated_ = false;
}

double MapMemoryCore::getYaw() const {
  tf2::Quaternion tf_quat;
  tf2::fromMsg(orientation_, tf_quat);
  return tf2::getYaw(tf_quat);
}

}
