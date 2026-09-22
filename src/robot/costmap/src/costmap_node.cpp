#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  subscribeToLidar();
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", QUEUE_SIZE);
}

void CostmapNode::subscribeToLidar() {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", QUEUE_SIZE, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_.processScan(scan);

  auto grid_msg = costmap_.getOccupancyGrid();
  grid_msg.header.stamp = this->now();
  grid_msg.header.frame_id = scan->header.frame_id;
  costmap_pub_->publish(grid_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
