#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "robot_interfaces/msg/joint_feedback.hpp"
#include "robot_interfaces/msg/joint_target.hpp"
#include "robot_interfaces/msg/robot_status.hpp"

using namespace std::chrono_literals;

class ActuatorBridgeNode : public rclcpp::Node
{
public:
  ActuatorBridgeNode()
  : Node("actuator_bridge_node")
  {
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 50.0);
    simulated_transport_ = this->declare_parameter<bool>("simulated_transport", true);
    feedback_alpha_ = this->declare_parameter<double>("feedback_alpha", 1.0);
    max_target_age_ms_ = this->declare_parameter<double>("max_target_age_ms", 200.0);
    bus_voltage_ = this->declare_parameter<double>("bus_voltage", 7.4);

    const std::vector<double> default_joint_angles =
      this->declare_parameter<std::vector<double>>(
        "default_joint_angles",
        {
          0.0, -0.6, 1.1,
          0.0, -0.6, 1.1,
          0.0, -0.6, 1.1,
          0.0, -0.6, 1.1
        });

    if (default_joint_angles.size() != 12) {
      throw std::runtime_error("default_joint_angles must have 12 elements");
    }

    for (size_t i = 0; i < 12; ++i) {
      joint_position_[i] = static_cast<float>(default_joint_angles[i]);
      prev_joint_position_[i] = joint_position_[i];
      target_rad_[i] = joint_position_[i];
    }

    target_sub_ = this->create_subscription<robot_interfaces::msg::JointTarget>(
      "/control/rl/joint_target",
      rclcpp::QoS(10),
      std::bind(&ActuatorBridgeNode::targetCallback, this, std::placeholders::_1));

    joint_feedback_pub_ = this->create_publisher<robot_interfaces::msg::JointFeedback>(
      "/control/actuator/joint_feedback",
      rclcpp::QoS(10));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/control/actuator/imu",
      rclcpp::QoS(10));

    status_pub_ = this->create_publisher<robot_interfaces::msg::RobotStatus>(
      "/control/actuator/status",
      rclcpp::QoS(10));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));

    timer_ = this->create_wall_timer(
      period,
      std::bind(&ActuatorBridgeNode::controlLoop, this));

    last_target_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "actuator_bridge_node started. rate=%.1f Hz, simulated_transport=%s",
      control_rate_hz_,
      simulated_transport_ ? "true" : "false");
  }

private:
  void targetCallback(const robot_interfaces::msg::JointTarget::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    latest_seq_ = msg->seq;
    latest_mode_ = msg->mode;
    latest_flags_ = msg->flags;
    last_target_time_ = this->now();
    have_target_ = true;

    for (size_t i = 0; i < 12; ++i) {
      target_rad_[i] = msg->target_rad[i];
    }
  }

  void controlLoop()
  {
    const auto loop_start = std::chrono::steady_clock::now();

    robot_interfaces::msg::JointTarget::_mode_type mode;
    uint16_t seq_echo;
    bool have_target;
    double target_age_ms;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      const auto now = this->now();
      target_age_ms = (now - last_target_time_).seconds() * 1000.0;
      have_target = have_target_;

      mode = latest_mode_;
      seq_echo = latest_seq_;

      for (size_t i = 0; i < 12; ++i) {
        prev_joint_position_[i] = joint_position_[i];

        if (have_target_) {
          const float error = target_rad_[i] - joint_position_[i];
          joint_position_[i] += static_cast<float>(feedback_alpha_) * error;
        }

        joint_velocity_[i] =
          static_cast<float>((joint_position_[i] - prev_joint_position_[i]) * control_rate_hz_);
      }
    }

    const bool stale = (!have_target) || (target_age_ms > max_target_age_ms_);
    const bool e_stop = (mode == robot_interfaces::msg::JointTarget::MODE_E_STOP);
    const bool disabled = (mode == robot_interfaces::msg::JointTarget::MODE_DISABLE);

    const bool torque_enabled = !stale && !e_stop && !disabled;

    publishJointFeedback(seq_echo);
    publishUprightImu();
    publishStatus(seq_echo, stale, torque_enabled, loop_start);
  }

  void publishJointFeedback(uint16_t seq_echo)
  {
    robot_interfaces::msg::JointFeedback msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.seq_echo = seq_echo;

    for (size_t i = 0; i < 12; ++i) {
      msg.position_rad[i] = joint_position_[i];
      msg.velocity_rad_s[i] = joint_velocity_[i];
      msg.load[i] = 0.0F;
      msg.temperature[i] = 25.0F;
    }

    joint_feedback_pub_->publish(msg);
  }

  void publishUprightImu()
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;

    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = 0.0;

    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = -9.81;

    imu_pub_->publish(msg);
  }

  void publishStatus(
    uint16_t seq_echo,
    bool stale,
    bool torque_enabled,
    const std::chrono::steady_clock::time_point & loop_start)
  {
    const auto loop_end = std::chrono::steady_clock::now();
    const double loop_time_ms =
      std::chrono::duration<double, std::milli>(loop_end - loop_start).count();

    robot_interfaces::msg::RobotStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.seq_echo = seq_echo;
    msg.status = stale
      ? robot_interfaces::msg::RobotStatus::STATUS_WARN
      : robot_interfaces::msg::RobotStatus::STATUS_OK;

    msg.fault_code = stale ? 1 : 0;
    msg.bus_voltage = static_cast<float>(bus_voltage_);
    msg.loop_time_ms = static_cast<float>(loop_time_ms);
    msg.spi_latency_ms = 0.0F;

    msg.packet_drop_count = packet_drop_count_;
    msg.crc_error_count = crc_error_count_;
    msg.missed_deadline_count = missed_deadline_count_;

    msg.torque_enabled = torque_enabled;
    msg.spi_connected = simulated_transport_;
    msg.servo_connected = simulated_transport_;

    status_pub_->publish(msg);
  }

private:
  double control_rate_hz_{50.0};
  bool simulated_transport_{true};
  double feedback_alpha_{1.0};
  double max_target_age_ms_{200.0};
  double bus_voltage_{7.4};

  std::mutex mutex_;

  bool have_target_{false};
  uint16_t latest_seq_{0};
  uint8_t latest_mode_{robot_interfaces::msg::JointTarget::MODE_STAND};
  uint8_t latest_flags_{0};

  rclcpp::Time last_target_time_;

  std::array<float, 12> target_rad_{};
  std::array<float, 12> joint_position_{};
  std::array<float, 12> prev_joint_position_{};
  std::array<float, 12> joint_velocity_{};

  uint32_t packet_drop_count_{0};
  uint32_t crc_error_count_{0};
  uint32_t missed_deadline_count_{0};

  rclcpp::Subscription<robot_interfaces::msg::JointTarget>::SharedPtr target_sub_;
  rclcpp::Publisher<robot_interfaces::msg::JointFeedback>::SharedPtr joint_feedback_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorBridgeNode>());
  rclcpp::shutdown();
  return 0;
}