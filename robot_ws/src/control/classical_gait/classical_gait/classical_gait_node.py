#!/usr/bin/env python3

"""
Classical (deterministic) trot gait controller node.

Subscribes:
    /cmd_vel (geometry_msgs/Twist)
        linear.x  : forward velocity (m/s)
        linear.y  : strafe (현재 IK 무관, future-proof)
        angular.z : yaw rate (rad/s)

Publishes:
    /control/classical/joint_target (robot_interfaces/JointTarget) @ 50Hz
        mode=MODE_CLASSICAL(3), target_rad[12] = IK 기반 trot pattern

Hybrid 모드 구조:
    classical_gait_node ─→ /control/classical/joint_target ┐
    rl_locomotion_node  ─→ /control/rl/joint_target        │
    stand_motion_node   ─→ /control/stand/joint_target     │
                                                            ▼
                                          [joint_target_mux_node]
                                                            │
                                                            ▼
                                          /control/selected/joint_target

Mux 는 behavior_mode 파라미터 (CLASSICAL / RL / STAND) 로 어느 입력을 forward 할지 결정.
이 노드는 mux 와 무관하게 매 cycle 자기 출력만 publish — 항상 살아있는 상태.
"""

import math
import threading
import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from robot_interfaces.msg import JointTarget

from classical_gait.gait_phase import GaitPhaseGenerator
from classical_gait.ik_reference import TrotIkReference


# JointTarget mode 값 (robot_interfaces/JointTarget.msg 와 일치)
# 향후 msg 에 MODE_CLASSICAL=3 enum 추가하면 그쪽으로 교체.
MODE_DISABLE = 0
MODE_STAND = 1
MODE_RL = 2
MODE_CLASSICAL = 3


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ClassicalGaitNode(Node):
    """IK 기반 trot 보행 생성. ONNX inference 없음, open-loop."""

    def __init__(self):
        super().__init__('classical_gait_node')

        self.control_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # ============================================================
        # Parameters
        # ============================================================
        self.declare_parameter('rate_hz', 50.0)

        # IK link params (실제 robot 측정값 default)
        self.declare_parameter('l1_x_m', 0.03)
        self.declare_parameter('l1_z_m', 0.13)
        self.declare_parameter('l2_m', 0.110)
        self.declare_parameter('body_height_m', 0.20)
        self.declare_parameter('robot_width_m', 0.15)

        # Gait params
        self.declare_parameter('gait_period_s', 0.6)
        self.declare_parameter('duty_factor', 0.5)
        self.declare_parameter('step_height_m', 0.03)
        self.declare_parameter('phase_cmd_norm', 0.1)
        self.declare_parameter('blend_cmd_norm', 0.1)

        # cmd_vel limits
        self.declare_parameter('vx_min', 0.0)
        self.declare_parameter('vx_max', 0.40)
        self.declare_parameter('vy_min', 0.0)
        self.declare_parameter('vy_max', 0.0)
        self.declare_parameter('wz_min', -0.40)
        self.declare_parameter('wz_max', 0.40)

        # Joint default (STM stand_only 자세 일치 — 0, -0.9269, +1.5314 × 4)
        self.declare_parameter(
            'default_joint_angles',
            [
                0.0, -0.9269, 1.5314,
                0.0, -0.9269, 1.5314,
                0.0, -0.9269, 1.5314,
                0.0, -0.9269, 1.5314,
            ],
        )

        # Joint limits
        self.declare_parameter(
            'joint_min_rad',
            [
                -0.548, -2.666, -0.100,
                -0.548, -2.666, -0.100,
                -0.548, -2.666, -0.100,
                -0.548, -2.666, -0.100,
            ],
        )
        self.declare_parameter(
            'joint_max_rad',
            [
                0.548, 1.548, 2.590,
                0.548, 1.548, 2.590,
                0.548, 1.548, 2.590,
                0.548, 1.548, 2.590,
            ],
        )

        # Rate limit (rad/s) — max_delta_rad = max_joint_speed / rate_hz
        self.declare_parameter('max_joint_speed_rad_s', 1.5)

        # ============================================================
        # Read parameters
        # ============================================================
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        if self.rate_hz <= 0.0:
            raise RuntimeError('rate_hz must be positive')
        self.dt = 1.0 / self.rate_hz

        self.l1_x = float(self.get_parameter('l1_x_m').value)
        self.l1_z = float(self.get_parameter('l1_z_m').value)
        self.l2 = float(self.get_parameter('l2_m').value)
        self.body_height = float(self.get_parameter('body_height_m').value)
        self.robot_width = float(self.get_parameter('robot_width_m').value)

        self.gait_period = float(self.get_parameter('gait_period_s').value)
        self.duty_factor = float(self.get_parameter('duty_factor').value)
        self.step_height = float(self.get_parameter('step_height_m').value)
        self.phase_cmd_norm = float(self.get_parameter('phase_cmd_norm').value)
        self.blend_cmd_norm = float(self.get_parameter('blend_cmd_norm').value)

        self.vx_min = float(self.get_parameter('vx_min').value)
        self.vx_max = float(self.get_parameter('vx_max').value)
        self.vy_min = float(self.get_parameter('vy_min').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.wz_min = float(self.get_parameter('wz_min').value)
        self.wz_max = float(self.get_parameter('wz_max').value)

        self.default_joint_angles = [
            float(v) for v in self.get_parameter('default_joint_angles').value
        ]
        self.joint_min_rad = [float(v) for v in self.get_parameter('joint_min_rad').value]
        self.joint_max_rad = [float(v) for v in self.get_parameter('joint_max_rad').value]

        self.max_joint_speed_rad_s = float(
            self.get_parameter('max_joint_speed_rad_s').value
        )
        self.max_delta_rad = self.max_joint_speed_rad_s * self.dt

        self._validate_config()

        # ============================================================
        # Modules
        # ============================================================
        self.gait_phase_gen = GaitPhaseGenerator(
            gait_period=self.gait_period,
            phase_cmd_norm=self.phase_cmd_norm,
        )

        self.ik_reference = TrotIkReference(
            default_joint_angles=self.default_joint_angles,
            gait_period=self.gait_period,
            duty_factor=self.duty_factor,
            step_height=self.step_height,
            body_height=self.body_height,
            robot_width=self.robot_width,
            blend_cmd_norm=self.blend_cmd_norm,
            l1_x=self.l1_x,
            l1_z=self.l1_z,
            l2=self.l2,
        )

        # ============================================================
        # State
        # ============================================================
        self.state_lock = threading.Lock()

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        self.prev_target_rad = list(self.default_joint_angles)
        self.gait_cycle_count = 0
        self.seq = 0

        # ============================================================
        # ROS IO
        # ============================================================
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            self.control_qos,
        )

        self.target_pub = self.create_publisher(
            JointTarget,
            '/control/classical/joint_target',
            self.control_qos,
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'classical_gait_node started: {self.rate_hz:.1f} Hz, '
            f'L1=({self.l1_x*1000:.0f},{self.l1_z*1000:.0f})mm, '
            f'L2={self.l2*1000:.0f}mm, body_h={self.body_height*1000:.0f}mm, '
            f'gait_period={self.gait_period}s, duty={self.duty_factor}, '
            f'step_h={self.step_height*1000:.0f}mm'
        )

    # ----------------------------------------------------------------
    def _validate_config(self):
        for name, arr in [
            ('default_joint_angles', self.default_joint_angles),
            ('joint_min_rad', self.joint_min_rad),
            ('joint_max_rad', self.joint_max_rad),
        ]:
            if len(arr) != 12:
                raise RuntimeError(f'{name} must have 12 elements, got {len(arr)}')

        for i in range(12):
            if self.joint_min_rad[i] >= self.joint_max_rad[i]:
                raise RuntimeError(
                    f'invalid joint limit index {i}: '
                    f'{self.joint_min_rad[i]} >= {self.joint_max_rad[i]}'
                )
            if not (
                self.joint_min_rad[i]
                <= self.default_joint_angles[i]
                <= self.joint_max_rad[i]
            ):
                raise RuntimeError(
                    f'default angle out of limit index {i}: '
                    f'default={self.default_joint_angles[i]}, '
                    f'min={self.joint_min_rad[i]}, '
                    f'max={self.joint_max_rad[i]}'
                )

    # ----------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        with self.state_lock:
            self.cmd_vx = clamp(float(msg.linear.x), self.vx_min, self.vx_max)
            self.cmd_vy = clamp(float(msg.linear.y), self.vy_min, self.vy_max)
            self.cmd_wz = clamp(float(msg.angular.z), self.wz_min, self.wz_max)

    # ----------------------------------------------------------------
    def control_loop(self):
        # 1. Snapshot cmd_vel
        with self.state_lock:
            cmd_vx = self.cmd_vx
            cmd_vy = self.cmd_vy
            cmd_wz = self.cmd_wz

        # 2. Phase 진행 (cmd_vel norm 따라)
        prev_phase = self.gait_phase_gen.phase
        phase = self.gait_phase_gen.update(self.dt, cmd_vx, cmd_vy, cmd_wz)
        if phase < prev_phase:
            self.gait_cycle_count += 1

        # 3. IK reference 계산
        ik_ref = self.ik_reference.get_reference(phase, cmd_vx, cmd_vy, cmd_wz)

        # 4. Joint limits clamp
        target = [
            clamp(ik_ref[i], self.joint_min_rad[i], self.joint_max_rad[i])
            for i in range(12)
        ]

        # 5. Rate limit (per-cycle delta cap)
        limited = []
        for i in range(12):
            delta = target[i] - self.prev_target_rad[i]
            delta = clamp(delta, -self.max_delta_rad, self.max_delta_rad)
            limited.append(self.prev_target_rad[i] + delta)

        # 6. Publish JointTarget
        msg = JointTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.seq = self.seq
        msg.mode = MODE_CLASSICAL
        msg.flags = 0
        msg.target_rad = list(limited)
        msg.max_delta_rad = [self.max_delta_rad] * 12
        msg.gait_phase = float(phase)
        msg.gait_cycle_count = int(self.gait_cycle_count)

        self.target_pub.publish(msg)

        # 7. State update
        self.prev_target_rad = list(limited)
        self.seq = (self.seq + 1) & 0xFFFF


def main(args=None):
    rclpy.init(args=args)
    node = ClassicalGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
