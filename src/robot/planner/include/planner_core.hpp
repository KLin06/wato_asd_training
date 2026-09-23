#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <optional>

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    enum class State {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };

    // Store the most recently received map
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map);

    // Store the new goal and transition into WAITING_FOR_ROBOT_TO_REACH_GOAL
    void updateGoal(const geometry_msgs::msg::Point& goal);

    // Track the robot's current position/orientation
    void updatePosition(double x, double y, const geometry_msgs::msg::Quaternion& orientation);

    // Timer-driven: check whether the goal has been reached, or whether
    // it's time to replan. TODO: implement once A* is in place.
    void checkGoalStatus();

  private:
    rclcpp::Logger logger_;

    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid latest_map_;
    std::optional<geometry_msgs::msg::Point> goal_;

    std::optional<double> current_x_;
    std::optional<double> current_y_;
    geometry_msgs::msg::Quaternion orientation_;
};

}

#endif
