#include <chrono>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  subscribeToMap();
  subscribeToGoal();
  subscribeToOdom();

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", QUEUE_SIZE);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD_MS), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::subscribeToMap() {
  // Must match map_memory's TRANSIENT_LOCAL publisher to get the last map
  // replayed on startup; /map is only republished after the robot moves
  // 1.5 m, so a volatile subscription would sit with no map while the
  // robot is still. Depth 1: only the newest map is ever useful.
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  publishIfAny(planner_.updateMap(map, this->now()));
}

void PlannerNode::subscribeToGoal() {
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", QUEUE_SIZE, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  publishIfAny(planner_.updateGoal(goal->point, goal->header.frame_id, this->now()));
}

void PlannerNode::subscribeToOdom() {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", QUEUE_SIZE, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  // This pose is the lidar's pose in sim_world (odometry_spoof publishes it
  // that way), the same frame the map is in, so no transform is needed.
  planner_.updatePosition(odom->pose.pose.position.x, odom->pose.pose.position.y);
}

void PlannerNode::timerCallback() {
  publishIfAny(planner_.checkGoalStatus(this->now()));
}

void PlannerNode::publishIfAny(const std::optional<nav_msgs::msg::Path>& path) {
  if (path) {
    path_pub_->publish(*path);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
