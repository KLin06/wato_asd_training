#include <chrono>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  subscribeToCostmap();
  subscribeToOdom();

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", QUEUE_SIZE);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
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
  map_memory_.updatePosition(odom->pose.pose.position.x, odom->pose.pose.position.y);
}

void MapMemoryNode::updateMap() {
  if (map_memory_.shouldUpdateMap()) {
    map_memory_.integrateCostmap();
    map_pub_->publish(map_memory_.getGlobalMap());
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
