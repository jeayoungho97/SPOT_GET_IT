#ifndef SPOT_NAVIGATION__PATH_FOLLOWER_NODE_HPP_
#define SPOT_NAVIGATION__PATH_FOLLOWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <robot_interfaces/msg/localized_robot_pose.hpp>
#include <robot_interfaces/msg/navigation_state.hpp>

namespace spot_navigation
{

class PathFollowerNode : public rclcpp::Node
{
public:
  explicit PathFollowerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ============================================================
  // Subscribers
  // ============================================================
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LocalizedRobotPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<robot_interfaces::msg::NavigationState>::SharedPtr nav_state_sub_;

  // ============================================================
  // Publisher
  // ============================================================
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_raw_pub_;

  // ============================================================
  // Timer
  // ============================================================
  rclcpp::TimerBase::SharedPtr timer_;

  // ============================================================
  // Latest received data
  // ============================================================
  nav_msgs::msg::Path::SharedPtr latest_local_path_;
  robot_interfaces::msg::LocalizedRobotPose::SharedPtr latest_pose_;
  robot_interfaces::msg::NavigationState::SharedPtr latest_nav_state_;

  // ============================================================
  // Parameters
  // ============================================================
  std::string robot_id_;

  double max_linear_x_mps_;
  double min_linear_x_mps_;
  double max_angular_z_radps_;
  double k_yaw_;
  double heading_tolerance_rad_;
  double turn_in_place_threshold_rad_;
  double slow_down_angle_rad_;
  double slowdown_distance_m_;
  double path_lookahead_distance_m_;
  double timer_period_sec_;

  // ============================================================
  // Callbacks
  // ============================================================
  void on_local_path(nav_msgs::msg::Path::SharedPtr msg);
  void on_pose(robot_interfaces::msg::LocalizedRobotPose::SharedPtr msg);
  void on_nav_state(robot_interfaces::msg::NavigationState::SharedPtr msg);
  void on_timer();

  // ============================================================
  // Control logic
  // ============================================================

  // local path에서 현재 pose와 가장 가까운 waypoint index 반환
  size_t find_nearest_index();

  // nearest index 기준 lookahead target index 반환
  size_t find_lookahead_index(size_t nearest_index);

  // heading error 계산 (-π ~ π wrap)
  double compute_heading_error(double target_x, double target_y);

  // heading error, distance_to_goal 기반 v 계산
  double compute_linear_velocity(double heading_error, float distance_to_goal_m);

  // heading error 기반 w 계산
  double compute_angular_velocity(double heading_error);
};

}  // namespace spot_navigation

#endif  // SPOT_NAVIGATION__PATH_FOLLOWER_NODE_HPP_
