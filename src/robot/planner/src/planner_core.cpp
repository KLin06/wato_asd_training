#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
  latest_map_ = *map;
  // TODO: trigger a replan if a goal is active and the map changed enough
}

void PlannerCore::updateGoal(const geometry_msgs::msg::Point& goal) {
  goal_ = goal;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  // TODO: kick off the initial A* plan to this goal
}

void PlannerCore::updatePosition(double x, double y, const geometry_msgs::msg::Quaternion& orientation) {
  current_x_ = x;
  current_y_ = y;
  orientation_ = orientation;
}

void PlannerCore::checkGoalStatus() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  // TODO: check if the robot has reached goal_ -> state_ = State::WAITING_FOR_GOAL
  // TODO: check if we've timed out waiting -> replan (A*) toward goal_
}

}
