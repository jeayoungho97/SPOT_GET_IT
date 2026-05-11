#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "robot_interfaces/msg/joint_feedback.hpp"
#include "robot_interfaces/msg/joint_target.hpp"
#include "robot_interfaces/msg/robot_status.hpp"

#include "actuator_bridge/packet_codec.hpp"
#include "actuator_bridge/spi_transport.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr uint8_t MODE_DISABLE = 0;
constexpr uint8_t STATUS_OK = 0;
constexpr uint8_t STATUS_WARN = 1;
constexpr uint8_t STATUS_FAULT = 2;

constexpr uint8_t FAULT_NONE = 0;
constexpr uint8_t FAULT_STALE_TARGET = 1;
constexpr uint8_t FAULT_SPI_OPEN_FAILED = 10;
constexpr uint8_t FAULT_SPI_TRANSFER_FAILED = 11;
constexpr uint8_t FAULT_FEEDBACK_DECODE_FAILED = 12;
constexpr uint8_t FAULT_SEQ_MISMATCH = 13;

}  // namespace

class ActuatorBridgeNode : public rclcpp::Node
{
public:
  ActuatorBridgeNode()
  : Node("actuator_bridge_node")
  {
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 50.0);
    max_target_age_ms_ = this->declare_parameter<double>("max_target_age_ms", 200.0);

    spi_device_ = this->declare_parameter<std::string>("spi_device", "/dev/spidev0.0");
    spi_speed_hz_ = this->declare_parameter<int>("spi_speed_hz", 1000000);
    spi_mode_ = this->declare_parameter<int>("spi_mode", 0);
    spi_bits_per_word_ = this->declare_parameter<int>("spi_bits_per_word", 8);

    const std::vector<double> default_joint_angles =
      this->declare_parameter<std::vector<double>>(
        "default_joint_angles",
        {
          0.0, -0.6, 1.1,
          0.0, -0.6, 1.1,
          0.0, -0.6, 1.1,
          0.0, -0.6, 1.1
        });

    const std::vector<double> default_max_delta_rad =
      this->declare_parameter<std::vector<double>>(
        "default_max_delta_rad",
        {
          0.03, 0.03, 0.03,
          0.03, 0.03, 0.03,
          0.03, 0.03, 0.03,
          0.03, 0.03, 0.03
        });

    if (default_joint_angles.size() != actuator_bridge::NUM_JOINTS) {
      throw std::runtime_error("default_joint_angles must have 12 elements");
    }

    if (default_max_delta_rad.size() != actuator_bridge::NUM_JOINTS) {
      throw std::runtime_error("default_max_delta_rad must have 12 elements");
    }

    for (std::size_t i = 0; i < actuator_bridge::NUM_JOINTS; ++i) {
      latest_target_rad_[i] = static_cast<float>(default_joint_angles[i]);
      latest_max_delta_rad_[i] = static_cast<float>(default_max_delta_rad[i]);
    }

    target_sub_ = this->create_subscription<robot_interfaces::msg::JointTarget>(
      "/control/rl/joint_target",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&ActuatorBridgeNode::targetCallback, this, std::placeholders::_1));

    joint_feedback_pub_ = this->create_publisher<robot_interfaces::msg::JointFeedback>(
      "/control/actuator/joint_feedback",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/control/actuator/imu",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    status_pub_ = this->create_publisher<robot_interfaces::msg::RobotStatus>(
      "/control/actuator/status",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    const bool spi_ok = spi_.open_device(
      spi_device_,
      static_cast<uint32_t>(spi_speed_hz_),
      static_cast<uint8_t>(spi_mode_),
      static_cast<uint8_t>(spi_bits_per_word_));

    if (!spi_ok) {
      spi_open_failed_ = true;
      RCLCPP_ERROR(
        this->get_logger(),
        "SPI open failed: %s",
        spi_.last_error().c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "SPI opened: device=%s, speed=%d Hz, mode=%d, bits=%d, frame=%zu bytes",
        spi_device_.c_str(),
        spi_speed_hz_,
        spi_mode_,
        spi_bits_per_word_,
        actuator_bridge::SPI_FRAME_SIZE);
    }

    last_target_time_ = this->now();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));

    timer_ = this->create_wall_timer(
      period,
      std::bind(&ActuatorBridgeNode::controlLoop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "actuator_bridge_node started in REAL SPI ONLY mode, rate=%.1f Hz",
      control_rate_hz_);
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

    for (std::size_t i = 0; i < actuator_bridge::NUM_JOINTS; ++i) {
      latest_target_rad_[i] = msg->target_rad[i];
      latest_max_delta_rad_[i] = msg->max_delta_rad[i];
    }
  }

  void controlLoop()
  {
    const auto loop_start = std::chrono::steady_clock::now();

    if (spi_open_failed_ || !spi_.is_open()) {
      publishBridgeFaultStatus(
        latest_seq_,
        FAULT_SPI_OPEN_FAILED,
        0.0F,
        loop_start);
      return;
    }

    actuator_bridge::CommandPacket command;
    const bool stale = fillCommandPacket(command);

    const std::vector<uint8_t> tx = actuator_bridge::make_spi_tx_frame(command);
    std::vector<uint8_t> rx;

    const auto spi_start = std::chrono::steady_clock::now();
    const bool transfer_ok = spi_.transfer(tx, rx);
    const auto spi_end = std::chrono::steady_clock::now();

    const float spi_latency_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(spi_end - spi_start).count());

    if (!transfer_ok) {
      packet_drop_count_++;
      publishBridgeFaultStatus(
        command.seq,
        FAULT_SPI_TRANSFER_FAILED,
        spi_latency_ms,
        loop_start);
      return;
    }

    actuator_bridge::FeedbackPacket feedback;
    const auto decode_result = actuator_bridge::decode_feedback_packet(rx, feedback);

    if (decode_result != actuator_bridge::DecodeResult::OK) {
      crc_error_count_++;

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "feedback decode failed: %s",
        actuator_bridge::to_string(decode_result));

      publishBridgeFaultStatus(
        command.seq,
        FAULT_FEEDBACK_DECODE_FAILED,
        spi_latency_ms,
        loop_start);
      return;
    }

    checkSequence(command.seq, feedback.seq_echo);

    publishJointFeedbackFromPacket(feedback);
    publishImuFromPacket(feedback);
    publishStatusFromPacket(feedback, stale, loop_start, spi_latency_ms);
  }

  bool fillCommandPacket(actuator_bridge::CommandPacket & command)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto now = this->now();
    const double target_age_ms = (now - last_target_time_).seconds() * 1000.0;
    const bool stale = (!have_target_) || (target_age_ms > max_target_age_ms_);

    command.seq = stale ? latest_seq_ : latest_seq_;
    command.timestamp_us = static_cast<uint32_t>(
      static_cast<uint64_t>(now.nanoseconds()) / 1000ULL);

    if (stale) {
      command.mode = MODE_DISABLE;
      command.flags = 0;
    } else {
      command.mode = latest_mode_;
      command.flags = latest_flags_;
    }

    for (std::size_t i = 0; i < actuator_bridge::NUM_JOINTS; ++i) {
      command.target_rad[i] = latest_target_rad_[i];
      command.max_delta_rad[i] = latest_max_delta_rad_[i];
    }

    return stale;
  }

  void checkSequence(uint16_t sent_seq, uint16_t seq_echo)
  {
    // STM32 SPI slave는 보통 이전 프레임에서 준비한 feedback을 내보내므로
    // seq_echo가 현재 seq 또는 직전 seq일 수 있다.
    const uint16_t prev_seq = static_cast<uint16_t>(sent_seq - 1U);

    if (seq_echo != sent_seq && seq_echo != prev_seq) {
      packet_drop_count_++;

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "seq mismatch: sent=%u, echo=%u",
        sent_seq,
        seq_echo);
    }
  }

  void publishJointFeedbackFromPacket(const actuator_bridge::FeedbackPacket & feedback)
  {
    robot_interfaces::msg::JointFeedback msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.seq_echo = feedback.seq_echo;

    for (std::size_t i = 0; i < actuator_bridge::NUM_JOINTS; ++i) {
      msg.position_rad[i] = feedback.position_rad[i];
      msg.velocity_rad_s[i] = feedback.velocity_rad_s[i];
      msg.load[i] = feedback.load_or_current[i];
      msg.temperature[i] = feedback.temperature[i];
    }

    joint_feedback_pub_->publish(msg);
  }

  void publishImuFromPacket(const actuator_bridge::FeedbackPacket & feedback)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    // FeedbackPacket은 wxyz, ROS sensor_msgs/Imu는 xyzw
    msg.orientation.w = feedback.quat_wxyz[0];
    msg.orientation.x = feedback.quat_wxyz[1];
    msg.orientation.y = feedback.quat_wxyz[2];
    msg.orientation.z = feedback.quat_wxyz[3];

    msg.angular_velocity.x = feedback.gyro_rad_s[0];
    msg.angular_velocity.y = feedback.gyro_rad_s[1];
    msg.angular_velocity.z = feedback.gyro_rad_s[2];

    // 현재 FeedbackPacket에는 linear acceleration이 없으므로 unknown으로 표시.
    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 0.0;
    msg.linear_acceleration_covariance[0] = -1.0;

    imu_pub_->publish(msg);
  }

  void publishStatusFromPacket(
    const actuator_bridge::FeedbackPacket & feedback,
    bool stale,
    const std::chrono::steady_clock::time_point & loop_start,
    float spi_latency_ms)
  {
    const auto loop_end = std::chrono::steady_clock::now();
    const float loop_time_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(loop_end - loop_start).count());

    robot_interfaces::msg::RobotStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.seq_echo = feedback.seq_echo;

    if (stale) {
      msg.status = STATUS_WARN;
      msg.fault_code = FAULT_STALE_TARGET;
      msg.torque_enabled = false;
    } else {
      msg.status = feedback.status;
      msg.fault_code = feedback.fault_code;
      msg.torque_enabled = (feedback.status == STATUS_OK && feedback.fault_code == FAULT_NONE);
    }

    msg.bus_voltage = feedback.bus_voltage;
    msg.loop_time_ms = loop_time_ms;
    msg.spi_latency_ms = spi_latency_ms;

    msg.packet_drop_count = packet_drop_count_;
    msg.crc_error_count = crc_error_count_;
    msg.missed_deadline_count = missed_deadline_count_;

    msg.spi_connected = spi_.is_open();
    msg.servo_connected = (feedback.status == STATUS_OK && feedback.fault_code == FAULT_NONE);

    if (loop_time_ms > (1000.0 / control_rate_hz_)) {
      missed_deadline_count_++;
    }

    status_pub_->publish(msg);
  }

  void publishBridgeFaultStatus(
    uint16_t seq_echo,
    uint8_t fault_code,
    float spi_latency_ms,
    const std::chrono::steady_clock::time_point & loop_start)
  {
    const auto loop_end = std::chrono::steady_clock::now();
    const float loop_time_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(loop_end - loop_start).count());

    robot_interfaces::msg::RobotStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.seq_echo = seq_echo;
    msg.status = STATUS_FAULT;
    msg.fault_code = fault_code;
    msg.bus_voltage = 0.0F;
    msg.loop_time_ms = loop_time_ms;
    msg.spi_latency_ms = spi_latency_ms;

    msg.packet_drop_count = packet_drop_count_;
    msg.crc_error_count = crc_error_count_;
    msg.missed_deadline_count = missed_deadline_count_;

    msg.torque_enabled = false;
    msg.spi_connected = spi_.is_open();
    msg.servo_connected = false;

    if (loop_time_ms > (1000.0 / control_rate_hz_)) {
      missed_deadline_count_++;
    }

    status_pub_->publish(msg);
  }

private:
  double control_rate_hz_{50.0};
  double max_target_age_ms_{200.0};

  std::string spi_device_{"/dev/spidev0.0"};
  int spi_speed_hz_{1000000};
  int spi_mode_{0};
  int spi_bits_per_word_{8};

  actuator_bridge::SpiTransport spi_;
  bool spi_open_failed_{false};

  std::mutex mutex_;

  bool have_target_{false};
  uint16_t latest_seq_{0};
  uint8_t latest_mode_{MODE_DISABLE};
  uint8_t latest_flags_{0};

  rclcpp::Time last_target_time_;

  std::array<float, actuator_bridge::NUM_JOINTS> latest_target_rad_{};
  std::array<float, actuator_bridge::NUM_JOINTS> latest_max_delta_rad_{};

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