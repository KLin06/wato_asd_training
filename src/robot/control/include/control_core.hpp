#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(rclcpp::Node& node);
  
  private:
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(
      const geometry_msgs::msg::PoseStamped& target);
    double computeDistance(
      const geometry_msgs::msg::Point& a,
      const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);

    rclcpp::Logger logger_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};

} 

#endif 
