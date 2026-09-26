#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

namespace
{

constexpr double MAX_ANGULAR_SPEED = 1.0;  // rad/s

}  // namespace

ControlCore::ControlCore(rclcpp::Node& node)
  : logger_(node.get_logger()),
    lookahead_distance_(1.0),
    goal_tolerance_(0.1),
    linear_speed_(0.5) {
  path_sub_ = node.create_subscription<nav_msgs::msg::Path>(
    "/path",
    10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      current_path_ = msg;
    });

  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      robot_odom_ = msg;
    });

  cmd_vel_pub_ = node.create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = node.create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      controlLoop();
    });
}

void ControlCore::controlLoop() {
  if (!current_path_ || !robot_odom_) {
    return;
  }

  // The planner publishes an empty path when the goal is reached or no path
  // exists. The simulator keeps executing the last /cmd_vel it received, so
  // publishing nothing would leave the robot driving; send one zero command.
  // Only once: repeating it would override the Foxglove teleop panel.
  const bool at_goal = !current_path_->poses.empty() &&
    computeDistance(robot_odom_->pose.pose.position,
                    current_path_->poses.back().pose.position) < goal_tolerance_;
  if (current_path_->poses.empty() || at_goal) {
    if (!stopped_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      stopped_ = true;
    }
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
  stopped_ = false;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() {
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
    return std::nullopt;
  }

  const auto& robot_position = robot_odom_->pose.pose.position;

  // Start at the closest path pose. This prevents the controller from
  // selecting a point behind the robot when the path is replanned.
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < current_path_->poses.size(); ++i) {
    const double distance = computeDistance(
      robot_position, current_path_->poses[i].pose.position);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  // Choose the first pose at least lookahead_distance away. If the remaining
  // path is shorter than that distance, use its final pose so the robot can
  // still converge to the goal.
  for (std::size_t i = closest_index; i < current_path_->poses.size(); ++i) {
    if (computeDistance(robot_position, current_path_->poses[i].pose.position) >=
        lookahead_distance_) {
      return current_path_->poses[i];
    }
  }

  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::PoseStamped& target) {
  geometry_msgs::msg::Twist cmd_vel;

  if (!robot_odom_) {
    return cmd_vel;
  }

  const auto& robot_pose = robot_odom_->pose.pose;
  const double robot_yaw = extractYaw(robot_pose.orientation);

  // Difference between the robot and target in the world frame.
  const double dx = target.pose.position.x - robot_pose.position.x;
  const double dy = target.pose.position.y - robot_pose.position.y;

  // Transform the target into the robot frame. In this frame, target_y is
  // the signed lateral distance used by pure pursuit.
  const double target_x =
    std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double target_y =
    -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  const double lookahead_distance = computeDistance(
    robot_pose.position, target.pose.position);
  const double lookahead_distance_squared =
    lookahead_distance * lookahead_distance;

  if (lookahead_distance_squared <= std::numeric_limits<double>::epsilon()) {
    return cmd_vel;
  }

  // Pure-pursuit curvature: kappa = 2 * lateral_error / L^2.
  const double curvature =
    2.0 * target_y / lookahead_distance_squared;

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = std::clamp(
    linear_speed_ * curvature,
    -MAX_ANGULAR_SPEED,
    MAX_ANGULAR_SPEED);
  return cmd_vel;
}

double ControlCore::computeDistance(
  const geometry_msgs::msg::Point& a,
  const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  return std::atan2(
    2.0 * (quat.w * quat.z + quat.x * quat.y),
    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

}  
