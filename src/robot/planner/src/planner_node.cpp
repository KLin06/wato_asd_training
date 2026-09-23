#include <chrono>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  subscribeToMap();
  subscribeToGoal();
  subscribeToOdom();

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", QUEUE_SIZE);
  timer_ = this->create_wall_timer(std::chrono::seconds(TIMER_PERIOD_SECONDS), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::subscribeToMap() {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", QUEUE_SIZE, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  planner_.updateMap(map);
}

void PlannerNode::subscribeToGoal() {
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", QUEUE_SIZE, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  planner_.updateGoal(goal->point);
}

void PlannerNode::subscribeToOdom() {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", QUEUE_SIZE, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  planner_.updatePosition(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.orientation);
}

void PlannerNode::timerCallback() {
  planner_.checkGoalStatus();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
