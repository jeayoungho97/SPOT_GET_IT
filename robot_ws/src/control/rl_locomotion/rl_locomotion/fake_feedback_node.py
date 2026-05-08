#!/usr/bin/env python3

import time
from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from robot_interfaces.msg import JointTarget, JointFeedback, RobotStatus


class FakeFeedbackNode(Node):
    """
    STM32/STS3215가 없는 상태에서 feedback loop를 검증하기 위한 fake actuator node.

    동작:
      - /control/rl/joint_target subscribe
      - 내부 joint_position이 target_rad를 1차 지연으로 따라감
      - /control/actuator/joint_feedback publish
      - /control/actuator/imu publish
      - /control/actuator/status publish
    """

    def __init__(self):
        super().__init__('fake_feedback_node')

        self.declare_parameter('feedback_rate_hz', 50.0)
        self.declare_parameter('tracking_alpha', 0.35)
        self.declare_parameter('default_joint_angles', [
            0.0, -0.6, 1.1,
            0.0, -0.6, 1.1,
            0.0, -0.6, 1.1,
            0.0, -0.6, 1.1,
        ])

        self.feedback_rate_hz = float(self.get_parameter('feedback_rate_hz').value)
        self.dt = 1.0 / self.feedback_rate_hz
        self.tracking_alpha = float(self.get_parameter('tracking_alpha').value)

        self.position = list(self.get_parameter('default_joint_angles').value)
        if len(self.position) != 12:
            raise RuntimeError('default_joint_angles must have 12 elements')

        self.velocity = [0.0] * 12
        self.target = list(self.position)

        self.seq_echo = 0
        self.last_update_time = time.perf_counter()
        self.last_target_time = None

        self.target_sub = self.create_subscription(
            JointTarget,
            '/control/rl/joint_target',
            self.target_callback,
            10
        )

        self.feedback_pub = self.create_publisher(
            JointFeedback,
            '/control/actuator/joint_feedback',
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/control/actuator/imu',
            10
        )

        self.status_pub = self.create_publisher(
            RobotStatus,
            '/control/actuator/status',
            10
        )

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'fake_feedback_node started: {self.feedback_rate_hz:.1f} Hz, '
            f'tracking_alpha={self.tracking_alpha:.2f}'
        )

    def target_callback(self, msg: JointTarget):
        if len(msg.target_rad) != 12:
            self.get_logger().warn('Received JointTarget with invalid target_rad length')
            return

        self.target = list(msg.target_rad)
        self.seq_echo = int(msg.seq)
        self.last_target_time = time.perf_counter()

    def timer_callback(self):
        now = time.perf_counter()
        dt = max(1e-6, now - self.last_update_time)
        self.last_update_time = now

        old_position = list(self.position)

        # 1차 지연으로 target 추종
        for i in range(12):
            self.position[i] = (
                (1.0 - self.tracking_alpha) * self.position[i]
                + self.tracking_alpha * self.target[i]
            )
            self.velocity[i] = (self.position[i] - old_position[i]) / dt

        stamp = self.get_clock().now().to_msg()

        fb = JointFeedback()
        fb.header.stamp = stamp
        fb.header.frame_id = 'base_link'
        fb.seq_echo = self.seq_echo
        fb.position_rad = self.position
        fb.velocity_rad_s = self.velocity
        fb.load = [0.0] * 12
        fb.temperature = [30.0] * 12
        self.feedback_pub.publish(fb)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'

        # upright identity orientation, ROS quaternion xyzw
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = 1.0

        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0

        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81

        self.imu_pub.publish(imu)

        status = RobotStatus()
        status.header.stamp = stamp
        status.header.frame_id = 'base_link'
        status.seq_echo = self.seq_echo
        status.status = RobotStatus.STATUS_OK
        status.fault_code = 0
        status.bus_voltage = 8.0
        status.loop_time_ms = 0.0
        status.spi_latency_ms = 0.0
        status.packet_drop_count = 0
        status.crc_error_count = 0
        status.missed_deadline_count = 0
        status.torque_enabled = True
        status.spi_connected = True
        status.servo_connected = True
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = FakeFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
