#include "planner_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace robot
{

namespace
{

// One entry in the A* open list. The same cell can be pushed several times
// with improving g; stale copies are skipped when popped (lazy deletion),
// because std::priority_queue has no decrease-key.
struct AStarNode
{
  CellIndex cell;
  double f;  // g + h
  double g;
};

// std::priority_queue is a MAX-heap: it pops whatever compares "largest".
// Returning a.f > b.f makes the smallest f come out first. Ties go to the
// larger g, i.e. the node that is further along, which is nearer the goal;
// that saves many expansions across wide open areas where f ties are common.
struct CompareF
{
  bool operator()(const AStarNode& a, const AStarNode& b) const
  {
    if (a.f == b.f) {
      return a.g < b.g;
    }
    return a.f > b.f;
  }
};

// Octile distance: the exact shortest distance on an EMPTY 8-connected grid
// (diagonal steps first, then straight). It never overestimates, since every
// real step costs at least its length (the cost penalty only adds), so A*
// still returns an optimal path. Euclidean would also be admissible but is
// looser, so A* would expand more cells; Manhattan OVERestimates diagonal
// moves and would make the paths suboptimal.
double octile(const CellIndex& a, const CellIndex& b, double resolution)
{
  double dx = std::abs(a.x - b.x);
  double dy = std::abs(a.y - b.y);
  return resolution * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
}

}  // namespace

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

std::optional<nav_msgs::msg::Path> PlannerCore::updateMap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr& map, const rclcpp::Time& now)
{
  map_ = *map;
  // "Replan whenever the map updates": new obstacles may block the current
  // path. When no goal is active there is nothing to replan.
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return std::nullopt;
  }
  return plan(now);
}

std::optional<nav_msgs::msg::Path> PlannerCore::updateGoal(
  const geometry_msgs::msg::Point& goal, const std::string& frame_id, const rclcpp::Time& now)
{
  // We never transform points, so a goal must already be in the map's frame.
  // Foxglove stamps a clicked point with the panel's display frame: a click
  // in robot/chassis/lidar would be read as world coordinates and send the
  // robot somewhere else entirely. Refuse it loudly instead.
  if (map_ && frame_id != map_->header.frame_id) {
    RCLCPP_ERROR(logger_, "Goal is in frame '%s' but the map is in '%s'; ignoring it. "
                 "Set Foxglove's display frame to '%s'.",
                 frame_id.c_str(), map_->header.frame_id.c_str(), map_->header.frame_id.c_str());
    return std::nullopt;
  }

  // Reject goals that can't be planned to NOW rather than accepting them and
  // failing every 500 ms until the timeout: off the map never becomes valid,
  // and a goal inside the no-go band is almost always a mis-click. Like a
  // wrong-frame goal, it leaves any current goal running. plan() still
  // re-checks, because a map update can later cover an accepted goal.
  if (map_) {
    CellIndex cell;
    if (!worldToGrid(goal.x, goal.y, cell)) {
      RCLCPP_ERROR(logger_, "Goal (%.2f, %.2f) is outside the map; ignoring it", goal.x, goal.y);
      return std::nullopt;
    }
    if (!traversable(cell)) {
      RCLCPP_ERROR(logger_, "Goal (%.2f, %.2f) is in or too close to an obstacle (cost %d); ignoring it",
                   goal.x, goal.y, cost(cell));
      return std::nullopt;
    }
  }

  goal_ = goal;
  goal_start_time_ = now;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(logger_, "New goal (%.2f, %.2f)", goal.x, goal.y);
  return plan(now);
}

void PlannerCore::updatePosition(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  position_ = p;
}

std::optional<nav_msgs::msg::Path> PlannerCore::checkGoalStatus(const rclcpp::Time& now)
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return std::nullopt;
  }

  // Check "reached" before "timed out": a robot that arrives on the same
  // tick the timer expires has succeeded, not failed.
  if (position_) {
    double d = std::hypot(goal_->x - position_->x, goal_->y - position_->y);
    if (d < GOAL_TOLERANCE) {
      RCLCPP_INFO(logger_, "Goal reached (%.2f m away)", d);
      state_ = State::WAITING_FOR_GOAL;
      goal_.reset();
      // Publish an EMPTY path rather than nothing: the last published path
      // is otherwise still the latest one control has, and it would keep
      // chasing it. Empty tells control to stop.
      return emptyPath(now);
    }
  }

  // Both times come from the node's clock, so they have the same clock type.
  // Subtracting rclcpp::Times with different clock types throws.
  if ((now - goal_start_time_).seconds() > GOAL_TIMEOUT) {
    RCLCPP_WARN(logger_, "Goal not reached within %.0f s; giving up", GOAL_TIMEOUT);
    state_ = State::WAITING_FOR_GOAL;
    goal_.reset();
    return emptyPath(now);
  }

  // Still driving: replan from where the robot is now. The map only updates
  // every 1.5 m, so without this the path's start would lag behind the
  // robot between map updates.
  return plan(now);
}

nav_msgs::msg::Path PlannerCore::plan(const rclcpp::Time& now)
{
  if (!map_ || !position_) {
    RCLCPP_WARN(logger_, "Can't plan yet: %s", !map_ ? "no map received" : "no odometry received");
    return emptyPath(now);
  }

  CellIndex start, goal;
  if (!worldToGrid(position_->x, position_->y, start)) {
    RCLCPP_WARN(logger_, "Robot (%.2f, %.2f) is outside the map", position_->x, position_->y);
    return emptyPath(now);
  }
  if (!worldToGrid(goal_->x, goal_->y, goal)) {
    RCLCPP_WARN(logger_, "Goal (%.2f, %.2f) is outside the map", goal_->x, goal_->y);
    return emptyPath(now);
  }
  if (!traversable(goal)) {
    RCLCPP_WARN(logger_, "Goal (%.2f, %.2f) is in an obstacle or too close to one (cost %d)",
                goal_->x, goal_->y, cost(goal));
    return emptyPath(now);
  }

  auto t0 = std::chrono::steady_clock::now();
  std::vector<CellIndex> cells = aStar(start, goal);
  double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  if (cells.empty()) {
    // Keep the goal active: the next map update may open a route. The
    // timeout is what eventually gives up.
    RCLCPP_WARN(logger_, "No path to goal (A* took %.1f ms)", ms);
    return emptyPath(now);
  }

  nav_msgs::msg::Path path = emptyPath(now);
  path.poses.reserve(cells.size());
  for (const auto& c : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = gridToWorld(c);
    pose.pose.orientation.w = 1.0;  // heading unused; control follows positions
    path.poses.push_back(pose);
  }
  // End exactly on the clicked point, not the centre of its cell, so the
  // controller's last target is what the user asked for (up to 7 cm apart).
  path.poses.back().pose.position.x = goal_->x;
  path.poses.back().pose.position.y = goal_->y;

  RCLCPP_INFO(logger_, "Path: %zu poses, A* %.1f ms", path.poses.size(), ms);
  return path;
}

std::vector<CellIndex> PlannerCore::aStar(const CellIndex& start, const CellIndex& goal) const
{
  const double res = map_->info.resolution;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  g_score[start] = 0.0;
  open.push({start, octile(start, goal, res), 0.0});

  // 8-connected. Diagonals give paths ~8% shorter than 4-connected on
  // average and fewer "staircase" corners for control to follow.
  static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    AStarNode current = open.top();
    open.pop();

    // Lazy deletion: a better copy of this cell was pushed after this one,
    // and has already been expanded.
    if (current.g > g_score[current.cell]) {
      continue;
    }

    if (current.cell == goal) {
      // Walk parents back to the start, then reverse into start -> goal
      std::vector<CellIndex> cells{goal};
      CellIndex c = goal;
      while (c != start) {
        c = came_from.at(c);
        cells.push_back(c);
      }
      std::reverse(cells.begin(), cells.end());
      return cells;
    }

    // The robot can end up inside the no-go band: it drifts, or a map update
    // moves a wall by a cell. Treating the start like any other cell would
    // then give "no path" forever, stranding it. So from a non-traversable
    // cell we allow steps that don't go UPHILL in cost (and never into an
    // actual obstacle), letting the robot back out of the band. Only cells
    // reached this way are ever non-traversable, so this can't open a
    // shortcut through the band anywhere else.
    const bool escaping = !traversable(current.cell);

    for (int k = 0; k < 8; ++k) {
      CellIndex next(current.cell.x + DX[k], current.cell.y + DY[k]);
      if (!inBounds(next)) {
        continue;
      }

      const int8_t next_cost = cost(next);
      bool allowed = traversable(next) ||
                     (escaping && next_cost < 100 && next_cost <= cost(current.cell));
      if (!allowed) {
        continue;
      }

      const bool diagonal = DX[k] != 0 && DY[k] != 0;
      // No corner cutting: a diagonal step between two blocked cells would
      // pass through the point where their corners meet, which is blocked.
      if (diagonal && !escaping &&
          (!traversable({current.cell.x + DX[k], current.cell.y}) ||
           !traversable({current.cell.x, current.cell.y + DY[k]}))) {
        continue;
      }

      const double step = res * (diagonal ? std::sqrt(2.0) : 1.0);
      // std::max(0, ...): the map could contain -1 (unknown); treat it like
      // free, the same as the 0s this map actually uses for "never seen".
      const double penalty = 1.0 + COST_WEIGHT * std::max<int>(0, next_cost) / 100.0;
      const double tentative_g = current.g + step * penalty;

      auto it = g_score.find(next);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[next] = tentative_g;
        came_from[next] = current.cell;
        open.push({next, tentative_g + octile(next, goal, res), tentative_g});
      }
    }
  }

  return {};  // open list exhausted: the goal is unreachable
}

nav_msgs::msg::Path PlannerCore::emptyPath(const rclcpp::Time& now) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = now;
  // Same frame as the map the path was planned on (sim_world)
  path.header.frame_id = map_ ? map_->header.frame_id : "sim_world";
  return path;
}

bool PlannerCore::worldToGrid(double wx, double wy, CellIndex& cell) const
{
  // Assumes the map's origin has no rotation (map_memory publishes w = 1).
  // floor, not a cast: truncation would put points just below the origin
  // into cell 0 instead of rejecting them.
  const auto& info = map_->info;
  cell.x = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
  cell.y = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
  return inBounds(cell);
}

geometry_msgs::msg::Point PlannerCore::gridToWorld(const CellIndex& cell) const
{
  // +0.5: the cell's centre, not its bottom-left corner
  const auto& info = map_->info;
  geometry_msgs::msg::Point p;
  p.x = info.origin.position.x + (cell.x + 0.5) * info.resolution;
  p.y = info.origin.position.y + (cell.y + 0.5) * info.resolution;
  return p;
}

bool PlannerCore::inBounds(const CellIndex& cell) const
{
  return cell.x >= 0 && cell.y >= 0 &&
         cell.x < static_cast<int>(map_->info.width) &&
         cell.y < static_cast<int>(map_->info.height);
}

int8_t PlannerCore::cost(const CellIndex& cell) const
{
  return map_->data[cell.y * map_->info.width + cell.x];
}

bool PlannerCore::traversable(const CellIndex& cell) const
{
  // Unknown (-1) and never-observed (0) both pass: this map writes 0 for
  // unobserved cells, so they can't be told apart from free ones, and
  // refusing them would make every goal outside the lidar's past view
  // unreachable. The path gets corrected by the replan when the map updates.
  return cost(cell) < OBSTACLE_THRESHOLD;
}

}
