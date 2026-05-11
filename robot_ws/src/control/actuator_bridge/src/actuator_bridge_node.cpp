#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdexcept>
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

rclcpp::QoS control_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace

class ActuatorBridgeNode : public rclcpp::Node
{
public:
  ActuatorBridgeNode()
  : Node("actuator_bridge_node")
  {
    control_rate_hz_ = this->declare_parameter("control_rate_hz", 50.0);
    max_target_age_ms_ = this->declare_parameter("max_target_age_ms", 200.0);

    spi_device_ = this->declare_parameter("spi_device", "/dev/spidev0.0");
    spi_speed_hz_ = this->declare_parameter("spi_speed_hz", 5000000);
    spi_mode_ = this->declare_parameter("spi_mode", 0);
    spi_bits_per_word_ = this->declare_parameter("spi_bits_per_word", 8);

    freeze_seq_when_stale_ = this->declare_parameter("freeze_seq_when_stale", true);

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
      default_target_rad_[i] = static_cast<float>(default_joint_angles[i]);
      latest_target_rad_[i] = default_target_rad_[i];

      default_max_delta_rad_[i] = static_cast<float>(default_max_delta_rad[i]);
      latest_max_delta_rad_[i] = default_max_delta_rad_[i];
    }

    target_topic_ = this->declare_parameter(
      "target_topic",
      "/control/selected/joint_target");

    target_sub_ = this->create_subscription<robot_interfaces::msg::JointTarget>(
      target_topic_,
      control_qos(),
      std::bind(&ActuatorBridgeNode::targetCallback, this, std::placeholders::_1));

    joint_feedback_pub_ =
      this->create_publisher<robot_interfaces::msg::JointFeedback>(
      "/control/actuator/joint_feedback",
      control_qos());

    imu_pub_ =
      this->create_publisher<sensor_msgs::msg::Imu>(
      "/control/actuator/imu",
      control_qos());

    status_pub_ =
      this->create_publisher<robot_interfaces::msg::RobotStatus>(
      "/control/actuator/status",
      control_qos());

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
      "actuator_bridge_node started: rate=%.1f Hz, target_topic=%s",
      control_rate_hz_,
      target_topic_.c_str());
  }

private:
  void targetCallback(const robot_interfaces::msg::JointTarget::SharedPtr msg)
  {
    if (msg->target_rad.size() != actuator_bridge::NUM_JOINTS ||
      msg->max_delta_rad.size() != actuator_bridge::NUM_JOINTS)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "invalid JointTarget array size");
      return;
    }

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
        last_sent_seq_,
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

    if (stale) {
      command.seq = freeze_seq_when_stale_ ? last_sent_seq_ : latest_seq_;
      command.mode = MODE_DISABLE;
      command.flags = 0;

      for (std::size_t i = 0; i < actuator_bridge::NUM_JOINTS; ++i) {
        command.target_rad[i] = default_target_rad_[i];
        command.max_delta_rad[i] = default_max_delta_rad_[i];
      }
    } else {
      command.seq = latest_seq_;
      command.mode = latest_mode_;
      command.flags = latest_flags_;

      for (std::size_t i = 0; i < actuator_bridge::NUM_JOINTS; ++i) {
        command.target_rad[i] = latest_target_rad_[i];
        command.max_delta_rad[i] = latest_max_delta_rad_[i];
      }
    }

    command.timestamp_us = static_cast<uint32_t>(
      static_cast<uint64_t>(now.nanoseconds()) / 1000ULL);

    last_sent_seq_ = command.seq;

    return stale;
  }

  void checkSequence(uint16_t sent_seq, uint16_t seq_echo)
  {
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

    // FeedbackPacket: wxyz
    // ROS sensor_msgs/Imu: xyzw
    msg.orientation.w = feedback.quat_wxyz[0];
    msg.orientation.x = feedback.quat_wxyz[1];
    msg.orientation.y = feedback.quat_wxyz[2];
    msg.orientation.z = feedback.quat_wxyz[3];

    msg.angular_velocity.x = feedback.gyro_rad_s[0];
    msg.angular_velocity.y = feedback.gyro_rad_s[1];
    msg.angular_velocity.z = feedback.gyro_rad_s[2];

    msg.linear_acceleration.x = feedback.accel_m_s2[0];
    msg.linear_acceleration.y = feedback.accel_m_s2[1];
    msg.linear_acceleration.z = feedback.accel_m_s2[2];

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
      msg.torque_enabled =
        (feedback.status == STATUS_OK && feedback.fault_code == FAULT_NONE);
    }

    msg.bus_voltage = feedback.bus_voltage;
    msg.loop_time_ms = loop_time_ms;
    msg.spi_latency_ms = spi_latency_ms;
    msg.packet_drop_count = packet_drop_count_;
    msg.crc_error_count = crc_error_count_;
    msg.missed_deadline_count = missed_deadline_count_;
    msg.spi_connected = spi_.is_open();
    msg.servo_connected =
      (feedback.status == STATUS_OK && feedback.fault_code == FAULT_NONE);

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
  int spi_speed_hz_{5000000};
  int spi_mode_{0};
  int spi_bits_per_word_{8};

  bool freeze_seq_when_stale_{true};

  actuator_bridge::SpiTransport spi_;
  bool spi_open_failed_{false};

  std::mutex mutex_;

  bool have_target_{false};
  uint16_t latest_seq_{0};
  uint16_t last_sent_seq_{0};
  uint8_t latest_mode_{MODE_DISABLE};
  uint8_t latest_flags_{0};
  rclcpp::Time last_target_time_;

  std::string target_topic_{"/control/selected/joint_target"};

  std::array<float, actuator_bridge::NUM_JOINTS> default_target_rad_{};
  std::array<float, actuator_bridge::NUM_JOINTS> default_max_delta_rad_{};
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