#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

namespace robot
{

  // A grid cell, x is the column, y the row,
  // so the flat OccupancyGrid index is y * width + x.
  struct CellIndex
  {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const { return x == other.x && y == other.y; }
    bool operator!=(const CellIndex &other) const { return !(*this == other); }
  };

  // Lets CellIndex be an unordered_map key. The << 1 matters: a plain
  // hash(x) ^ hash(y) gives (3,5) and (5,3) the same hash, and every cell on
  // the diagonal x == y hashes to 0, so the diagonal buckets pile up.
  // Shifting y first breaks that symmetry.
  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex &idx) const
    {
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };

  class PlannerCore
  {
  public:
    explicit PlannerCore(const rclcpp::Logger &logger);

    enum class State
    {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };

    // Each input returns a Path when the node should publish one, and
    // nullopt when there is nothing new to send. The core never touches ROS
    // publishers or clocks itself; the node passes `now` in, so this class
    // stays testable without a running node.

    // Store the map; replan if we're currently driving to a goal
    std::optional<nav_msgs::msg::Path> updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
                                                 const rclcpp::Time &now);

    // Accept a new goal (replacing any current one) and plan to it. A goal in
    // the wrong frame, off the map, or in the no-go band is ignored, and
    // any current goal keeps running.
    std::optional<nav_msgs::msg::Path> updateGoal(const geometry_msgs::msg::Point &goal,
                                                  const std::string &frame_id,
                                                  const rclcpp::Time &now);

    // Track the robot's position. Orientation doesn't matter: the goal is a
    // point, and A* plans positions only.
    void updatePosition(double x, double y);

    // Timer: goal reached? timed out? otherwise replan from where the robot
    // is now, since the start of the last path is behind us.
    std::optional<nav_msgs::msg::Path> checkGoalStatus(const rclcpp::Time &now);

    State getState() const { return state_; }

  private:
    // Cells with inflation cost >= this are refused. Costmap cost is
    // 100 * (1 - d / 1.0 m), so cost 50 <=> 0.5 m from an obstacle point.
    // The chassis is 1.0 m wide, so with its centreline on a cell of cost
    // >= 50 a side is already touching something. The lidar (which the path
    // is traced by) sits on that centreline.
    static constexpr int8_t OBSTACLE_THRESHOLD = 50;

    // Cells below the threshold are passable but cost: traversal cost
    // is resolution * (1 + COST_WEIGHT * cost / 100). Without this, A*
    // hugs walls at cost 49, and a controller that cuts corners
    // then clips them. It also keeps h (plain distance) admissible,
    // because every step costs at least its distance.
    static constexpr double COST_WEIGHT = 4.0;

    static constexpr double GOAL_TOLERANCE = 0.5; // m
    static constexpr double GOAL_TIMEOUT = 90.0;  // s, then give up on the goal

    // A* from start to goal (grid cells). Returns cells start..goal, or an
    // empty vector if there's no path.
    std::vector<CellIndex> aStar(const CellIndex &start, const CellIndex &goal) const;

    // Build the path for the current robot position and goal, handling the
    // "not ready" and "no path" cases. Always returns a Path (could be empty).
    nav_msgs::msg::Path plan(const rclcpp::Time &now);

    nav_msgs::msg::Path emptyPath(const rclcpp::Time &now) const;

    // Grid helpers. worldToGrid returns false for points outside the map.
    bool worldToGrid(double wx, double wy, CellIndex &cell) const;
    geometry_msgs::msg::Point gridToWorld(const CellIndex &cell) const;
    bool inBounds(const CellIndex &cell) const;
    int8_t cost(const CellIndex &cell) const;
    bool traversable(const CellIndex &cell) const;

    rclcpp::Logger logger_;

    State state_ = State::WAITING_FOR_GOAL;

    std::optional<nav_msgs::msg::OccupancyGrid> map_;
    std::optional<geometry_msgs::msg::Point> goal_;
    std::optional<geometry_msgs::msg::Point> position_;
    rclcpp::Time goal_start_time_;
  };

}

#endif
