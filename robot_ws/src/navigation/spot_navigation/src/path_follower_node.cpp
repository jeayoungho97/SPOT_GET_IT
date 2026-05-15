#include "spot_navigation/path_follower_node.hpp"

#include <cmath>
#include <limits>

namespace spot_navigation
{

PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions & options)
: Node("path_follower_node", options)
{
  // ============================================================
  // Parameters
  // ============================================================
  robot_id_                    = declare_parameter<std::string>("robot_id", "spot_01");
  max_linear_x_mps_            = declare_parameter<double>("max_linear_x_mps", 0.40);
  min_linear_x_mps_            = declare_parameter<double>("min_linear_x_mps", 0.05);
  max_angular_z_radps_         = declare_parameter<double>("max_angular_z_radps", 0.40);
  k_yaw_                       = declare_parameter<double>("k_yaw", 1.0);
  heading_tolerance_rad_       = declare_parameter<double>("heading_tolerance_rad", 0.15);
  turn_in_place_threshold_rad_ = declare_parameter<double>("turn_in_place_threshold_rad", 0.5);
  slow_down_angle_rad_         = declare_parameter<double>("slow_down_angle_rad", 0.7);
  slowdown_distance_m_         = declare_parameter<double>("slowdown_distance_m", 0.5);
  path_lookahead_distance_m_   = declare_parameter<double>("path_lookahead_distance_m", 0.3);
  timer_period_sec_            = declare_parameter<double>("timer_period_sec", 0.1);

  // ============================================================
  // Subscribers
  // ============================================================
  local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/navigation/local_path/" + robot_id_, 10,
    std::bind(&PathFollowerNode::on_local_path, this, std::placeholders::_1));

  pose_sub_ = create_subscription<robot_interfaces::msg::LocalizedRobotPose>(
    "/localization/pose", 10,
    std::bind(&PathFollowerNode::on_pose, this, std::placeholders::_1));

  nav_state_sub_ = create_subscription<robot_interfaces::msg::NavigationState>(
    "/navigation/state/" + robot_id_, 10,
    std::bind(&PathFollowerNode::on_nav_state, this, std::placeholders::_1));

  // ============================================================
  // Publisher
  // ============================================================
  cmd_vel_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/control/cmd_vel_raw/" + robot_id_, 10);

  // ============================================================
  // Timer
  // ============================================================
  timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period_sec_),
    std::bind(&PathFollowerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "path_follower_node 시작 [robot_id: %s]", robot_id_.c_str());
}

// ============================================================
// Callbacks
// ============================================================

void PathFollowerNode::on_local_path(nav_msgs::msg::Path::SharedPtr msg)
{
  latest_local_path_ = msg;
}

void PathFollowerNode::on_pose(robot_interfaces::msg::LocalizedRobotPose::SharedPtr msg)
{
  latest_pose_ = msg;
}

void PathFollowerNode::on_nav_state(robot_interfaces::msg::NavigationState::SharedPtr msg)
{
  latest_nav_state_ = msg;
}

// ============================================================
// Timer callback
// ============================================================

void PathFollowerNode::on_timer()
{
  using NS = robot_interfaces::msg::NavigationState;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp    = now();
  cmd.header.frame_id = "base_link";

  // ----------------------------------------------------------
  // 데이터 유효성 검사
  // ----------------------------------------------------------
  if (!latest_nav_state_ || !latest_pose_ || !latest_local_path_) {
    cmd_vel_raw_pub_->publish(cmd);  // v=0, w=0
    return;
  }

  // ----------------------------------------------------------
  // FSM state 확인
  // TRACKING_GLOBAL_PATH 또는 FOLLOWING_LOCAL_PATH 아니면 정지
  // ----------------------------------------------------------
  const uint8_t nav_state = latest_nav_state_->nav_state;
  if (nav_state != NS::NAV_TRACKING_GLOBAL_PATH &&
      nav_state != NS::NAV_FOLLOWING_LOCAL_PATH)
  {
    cmd_vel_raw_pub_->publish(cmd);  // v=0, w=0
    return;
  }

  // ----------------------------------------------------------
  // local path 유효성 검사
  // ----------------------------------------------------------
  if (latest_local_path_->poses.empty()) {
    cmd_vel_raw_pub_->publish(cmd);  // v=0, w=0
    return;
  }

  // ----------------------------------------------------------
  // nearest point 탐색
  // ----------------------------------------------------------
  const size_t nearest_idx = find_nearest_index();

  // ----------------------------------------------------------
  // lookahead target 선택
  // ----------------------------------------------------------
  const size_t target_idx = find_lookahead_index(nearest_idx);
  const auto & target_pose = latest_local_path_->poses[target_idx];
  const double target_x = target_pose.pose.position.x;
  const double target_y = target_pose.pose.position.y;

  // ----------------------------------------------------------
  // heading error 계산
  // ----------------------------------------------------------
  const double heading_error = compute_heading_error(target_x, target_y);

  // ----------------------------------------------------------
  // v, w 계산
  // ----------------------------------------------------------
  const float distance_to_goal = latest_nav_state_->distance_to_goal_m;
  const double v_cmd = compute_linear_velocity(heading_error, distance_to_goal);
  const double w_cmd = compute_angular_velocity(heading_error);

  cmd.twist.linear.x  = v_cmd;
  cmd.twist.linear.y  = 0.0;
  cmd.twist.angular.z = w_cmd;

  cmd_vel_raw_pub_->publish(cmd);
}

// ============================================================
// nearest point 탐색
// ============================================================

size_t PathFollowerNode::find_nearest_index()
{
  const double cx = latest_pose_->x_m;
  const double cy = latest_pose_->y_m;

  size_t nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < latest_local_path_->poses.size(); ++i) {
    const double dx = latest_local_path_->poses[i].pose.position.x - cx;
    const double dy = latest_local_path_->poses[i].pose.position.y - cy;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist    = dist;
      nearest_idx = i;
    }
  }

  return nearest_idx;
}

// ============================================================
// lookahead target 선택
// ============================================================

size_t PathFollowerNode::find_lookahead_index(size_t nearest_index)
{
  const double cx = latest_pose_->x_m;
  const double cy = latest_pose_->y_m;
  const size_t path_size = latest_local_path_->poses.size();

  // nearest_index 이후 waypoint 중 lookahead_distance_m 이상 떨어진 첫 번째 선택
  for (size_t i = nearest_index; i < path_size; ++i) {
    const double dx = latest_local_path_->poses[i].pose.position.x - cx;
    const double dy = latest_local_path_->poses[i].pose.position.y - cy;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist >= path_lookahead_distance_m_) {
      return i;
    }
  }

  // lookahead 거리 내에 없으면 마지막 waypoint
  return path_size - 1;
}

// ============================================================
// heading error 계산 (-π ~ π wrap)
// ============================================================

double PathFollowerNode::compute_heading_error(double target_x, double target_y)
{
  const double cx = latest_pose_->x_m;
  const double cy = latest_pose_->y_m;
  const double current_yaw = latest_pose_->yaw_rad;

  const double target_heading = std::atan2(target_y - cy, target_x - cx);
  double error = target_heading - current_yaw;

  // -π ~ π wrap
  while (error >  M_PI) { error -= 2.0 * M_PI; }
  while (error < -M_PI) { error += 2.0 * M_PI; }

  return error;
}

// ============================================================
// v 계산
// heading_error 기반 감속 + 거리 기반 감속 중 더 작은 값 사용
// ============================================================

double PathFollowerNode::compute_linear_velocity(
  double heading_error, float distance_to_goal_m)
{
  // 제자리 회전 구간
  if (std::abs(heading_error) > turn_in_place_threshold_rad_) {
    return 0.0;
  }

  // heading_error 기반 감속
  // heading_error가 slow_down_angle_rad에 가까울수록 v 감소
  double v_angle = max_linear_x_mps_;
  if (std::abs(heading_error) > heading_tolerance_rad_) {
    const double ratio = 1.0 - std::abs(heading_error) / slow_down_angle_rad_;
    v_angle = max_linear_x_mps_ * std::max(ratio, 0.0);
  }

  // 거리 기반 감속
  double v_distance = max_linear_x_mps_;
  if (distance_to_goal_m < static_cast<float>(slowdown_distance_m_)) {
    v_distance = max_linear_x_mps_ *
      (static_cast<double>(distance_to_goal_m) / slowdown_distance_m_);
  }

  // 둘 중 더 작은 값
  double v_cmd = std::min(v_angle, v_distance);

  // min_v clamp (너무 느리면 못 걸으니까)
  if (v_cmd > 0.0) {
    v_cmd = std::max(v_cmd, min_linear_x_mps_);
  }

  return v_cmd;
}

// ============================================================
// w 계산
// ============================================================

double PathFollowerNode::compute_angular_velocity(double heading_error)
{
  double w_cmd = k_yaw_ * heading_error;

  // clamp
  if (w_cmd >  max_angular_z_radps_) { w_cmd =  max_angular_z_radps_; }
  if (w_cmd < -max_angular_z_radps_) { w_cmd = -max_angular_z_radps_; }

  return w_cmd;
}

}  // namespace spot_navigation

// ============================================================
// main
// ============================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spot_navigation::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
