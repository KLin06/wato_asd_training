#include <chrono>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  subscribeToCostmap();
  subscribeToOdom();

  // TRANSIENT_LOCAL ("latched"): /map only publishes when the robot has
  // moved UPDATE_DISTANCE, so a late subscriber (the planner, Foxglove)
  // would otherwise see nothing while the robot sits still. Depth 1 — only
  // the newest map is worth replaying. Subscribers must request
  // transient_local too; a volatile subscriber still works, but without
  // the replay.
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable());
  timer_ = this->create_wall_timer(std::chrono::seconds(TIMER_PERIOD_SECONDS), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::subscribeToCostmap() {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", QUEUE_SIZE, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  map_memory_.updateCostmap(costmap);
}

void MapMemoryNode::subscribeToOdom() {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", QUEUE_SIZE, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  world_frame_id_ = odom->header.frame_id;
  map_memory_.updatePosition(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.orientation);
}

void MapMemoryNode::updateMap() {
  if (map_memory_.shouldUpdateMap()) {
    map_memory_.integrateCostmap();

    auto grid_msg = map_memory_.getGlobalMap();
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = world_frame_id_;
    map_pub_->publish(grid_msg);

    map_memory_.resetUpdateFlags();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
