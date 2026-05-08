#!/usr/bin/env python3

import os
import time
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_share_directory

from robot_interfaces.msg import JointTarget, RlDebug, JointFeedback, RobotStatus

from rl_locomotion.gait_phase import GaitPhaseGenerator
from rl_locomotion.ik_reference import TrotIkReference
from rl_locomotion.obs_builder import ObsBuilder
from rl_locomotion.imu_utils import projected_gravity_from_ros_quat_xyzw
from rl_locomotion.policy_runner import PolicyRunner

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class RlLocomotionNode(Node):
    def __init__(self):
        super().__init__('rl_locomotion_node')

        # ---- parameters ----
        self.declare_parameter('policy_rate_hz', 50.0)
        self.declare_parameter('obs_dim', 47)
        self.declare_parameter('action_dim', 12)
        self.declare_parameter('action_scale', 0.25)
        self.declare_parameter('action_clip', 0.5)

        self.declare_parameter('policy_backend', 'dummy')
        self.declare_parameter('model_path', '')
        self.declare_parameter('require_model', False)
        self.declare_parameter('obs_clip', 100.0)

        self.declare_parameter('gait_period', 0.6)
        self.declare_parameter('duty_factor', 0.5)
        self.declare_parameter('step_height', 0.03)
        self.declare_parameter('body_height', 0.206)
        self.declare_parameter('robot_width', 0.15)
        self.declare_parameter('phase_cmd_norm', 0.1)
        self.declare_parameter('blend_cmd_norm', 0.1)
        self.declare_parameter('debug_publish_rate_hz', 10.0)

        self.declare_parameter('vx_min', 0.0)
        self.declare_parameter('vx_max', 0.10)
        self.declare_parameter('vy_min', 0.0)
        self.declare_parameter('vy_max', 0.0)
        self.declare_parameter('wz_min', -0.05)
        self.declare_parameter('wz_max', 0.05)

        self.declare_parameter('joint_names', [
            'front_left_shoulder', 'front_left_leg', 'front_left_foot',
            'front_right_shoulder', 'front_right_leg', 'front_right_foot',
            'rear_left_shoulder', 'rear_left_leg', 'rear_left_foot',
            'rear_right_shoulder', 'rear_right_leg', 'rear_right_foot',
        ])

        self.declare_parameter('default_joint_angles', [
            0.0, -0.6, 1.1,
            0.0, -0.6, 1.1,
            0.0, -0.6, 1.1,
            0.0, -0.6, 1.1,
        ])

        self.declare_parameter('joint_min_rad', [
            -0.548, -2.666, -0.100,
            -0.548, -2.666, -0.100,
            -0.548, -2.666, -0.100,
            -0.548, -2.666, -0.100,
        ])

        self.declare_parameter('joint_max_rad', [
            0.548, 1.548, 2.590,
            0.548, 1.548, 2.590,
            0.548, 1.548, 2.590,
            0.548, 1.548, 2.590,
        ])

        self.declare_parameter('max_joint_speed_rad_s', 1.5)

        self.policy_rate_hz = float(self.get_parameter('policy_rate_hz').value)
        self.dt = 1.0 / self.policy_rate_hz

        self.obs_dim = int(self.get_parameter('obs_dim').value)
        self.action_dim = int(self.get_parameter('action_dim').value)
        self.action_scale = float(self.get_parameter('action_scale').value)
        self.action_clip = float(self.get_parameter('action_clip').value)

        self.policy_backend = str(self.get_parameter('policy_backend').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.require_model = bool(self.get_parameter('require_model').value)
        self.obs_clip = float(self.get_parameter('obs_clip').value)

        self.gait_period = float(self.get_parameter('gait_period').value)
        self.duty_factor = float(self.get_parameter('duty_factor').value)
        self.step_height = float(self.get_parameter('step_height').value)
        self.body_height = float(self.get_parameter('body_height').value)
        self.robot_width = float(self.get_parameter('robot_width').value)
        self.phase_cmd_norm = float(self.get_parameter('phase_cmd_norm').value)
        self.blend_cmd_norm = float(self.get_parameter('blend_cmd_norm').value)
        self.debug_publish_rate_hz = float(self.get_parameter('debug_publish_rate_hz').value)

        self.vx_min = float(self.get_parameter('vx_min').value)
        self.vx_max = float(self.get_parameter('vx_max').value)
        self.vy_min = float(self.get_parameter('vy_min').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.wz_min = float(self.get_parameter('wz_min').value)
        self.wz_max = float(self.get_parameter('wz_max').value)

        self.default_joint_angles = list(self.get_parameter('default_joint_angles').value)
        self.joint_min_rad = list(self.get_parameter('joint_min_rad').value)
        self.joint_max_rad = list(self.get_parameter('joint_max_rad').value)

        self.max_joint_speed_rad_s = float(self.get_parameter('max_joint_speed_rad_s').value)
        self.max_delta_rad = [self.max_joint_speed_rad_s * self.dt] * 12

        if len(self.default_joint_angles) != 12:
            raise RuntimeError('default_joint_angles must have 12 elements')
        if len(self.joint_min_rad) != 12 or len(self.joint_max_rad) != 12:
            raise RuntimeError('joint limits must have 12 elements')

        # ---- helper modules ----
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
        )

        # ---- state ----
        self.seq = 0
        self.gait_phase = 0.0

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        self.prev_actions = [0.0] * 12
        self.prev_target_rad = list(self.default_joint_angles)
        
        self.joint_position = list(self.default_joint_angles)
        self.joint_velocity = [0.0] * 12

        self.base_ang_vel = [0.0, 0.0, 0.0]
        self.projected_gravity = [0.0, 0.0, -1.0]

        self.last_feedback_time = None
        self.last_status_ok = True

        self.obs_builder = ObsBuilder(obs_dim=self.obs_dim)

        self.missed_deadline_count = 0
        self.last_loop_time = time.perf_counter()
        self.last_debug_pub_time = 0.0

        resolved_model_path = self.resolve_model_path(self.model_path)

        self.policy_runner = PolicyRunner(
            backend=self.policy_backend,
            model_path=resolved_model_path,
            obs_dim=self.obs_dim,
            action_dim=self.action_dim,
            obs_clip=self.obs_clip,
            require_model=self.require_model,
        )

        # ---- ROS IO ----
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.joint_feedback_sub = self.create_subscription(
            JointFeedback,
            '/control/actuator/joint_feedback',
            self.joint_feedback_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/control/actuator/imu',
            self.imu_callback,
            10
        )

        self.status_sub = self.create_subscription(
            RobotStatus,
            '/control/actuator/status',
            self.status_callback,
            10
        )

        self.target_pub = self.create_publisher(
            JointTarget,
            '/control/rl/joint_target',
            10
        )

        self.debug_pub = self.create_publisher(
            RlDebug,
            '/control/rl/debug',
            10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'rl_locomotion_node started: {self.policy_rate_hz:.1f} Hz, '
            f'obs_dim={self.obs_dim}, action_dim={self.action_dim}, '
            f'gait_period={self.gait_period:.3f}s'
            f'policy runner: backend={self.policy_runner.backend_name()}, '
            f'model_path="{resolved_model_path}"'
        )

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = clamp(msg.linear.x, self.vx_min, self.vx_max)
        self.cmd_vy = clamp(msg.linear.y, self.vy_min, self.vy_max)
        self.cmd_wz = clamp(msg.angular.z, self.wz_min, self.wz_max)

    def joint_feedback_callback(self, msg: JointFeedback):
        if len(msg.position_rad) == 12:
            self.joint_position = list(msg.position_rad)
        else:
            self.get_logger().warn('JointFeedback.position_rad length is not 12')

        if len(msg.velocity_rad_s) == 12:
            self.joint_velocity = list(msg.velocity_rad_s)
        else:
            self.get_logger().warn('JointFeedback.velocity_rad_s length is not 12')

        self.last_feedback_time = time.perf_counter()

    def imu_callback(self, msg: Imu):
        self.base_ang_vel = [
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        ]

        self.projected_gravity = list(
            projected_gravity_from_ros_quat_xyzw(
                qx=float(msg.orientation.x),
                qy=float(msg.orientation.y),
                qz=float(msg.orientation.z),
                qw=float(msg.orientation.w),
            )
        )

    def status_callback(self, msg: RobotStatus):
        self.last_status_ok = (
            msg.status == RobotStatus.STATUS_OK
            and msg.torque_enabled
            and msg.servo_connected
        )

    def update_gait_phase(self):
        self.gait_phase = self.gait_phase_gen.update(
            dt=self.dt,
            cmd_vx=self.cmd_vx,
            cmd_vy=self.cmd_vy,
            cmd_wz=self.cmd_wz,
        )

    def build_observation(self, ik_ref: List[float]) -> List[float]:
        return self.obs_builder.build(
            base_ang_vel=self.base_ang_vel,
            projected_gravity=self.projected_gravity,
            cmd_vx=self.cmd_vx,
            cmd_vy=self.cmd_vy,
            cmd_wz=self.cmd_wz,
            dof_pos=self.joint_position,
            dof_vel=self.joint_velocity,
            ik_ref=ik_ref,
            prev_actions=self.prev_actions,
            gait_phase=self.gait_phase,
        )

    def get_feedback_age_ms(self) -> float:
        if self.last_feedback_time is None:
            return 9999.0
        return (time.perf_counter() - self.last_feedback_time) * 1000.0

    def resolve_model_path(self, model_path: str) -> str:
        """
        model_path가 절대경로면 그대로 사용.
        상대경로면 rl_locomotion package share 기준으로 해석.
        예:
          models/policy.onnx
          /home/jetson/robot_ws/src/control/rl_locomotion/models/policy.onnx
        """
        if not model_path:
            return ""

        expanded = os.path.expanduser(model_path)

        if os.path.isabs(expanded):
            return expanded

        pkg_share = get_package_share_directory('rl_locomotion')
        return os.path.join(pkg_share, expanded)

    def run_policy(self, obs: List[float]) -> List[float]:
        return self.policy_runner.infer(obs)

    def postprocess_action(self, raw_action: List[float], ik_ref: List[float]) -> List[float]:
        clipped_action = [
            clamp(a, -self.action_clip, self.action_clip)
            for a in raw_action
        ]

        target = [
            ik_ref[i] + clipped_action[i] * self.action_scale
            for i in range(12)
        ]

        # joint limit clamp
        target = [
            clamp(target[i], self.joint_min_rad[i], self.joint_max_rad[i])
            for i in range(12)
        ]

        # slew rate limit
        limited = []
        for i in range(12):
            delta = target[i] - self.prev_target_rad[i]
            delta = clamp(delta, -self.max_delta_rad[i], self.max_delta_rad[i])
            limited.append(self.prev_target_rad[i] + delta)

        self.prev_target_rad = limited
        self.prev_actions = clipped_action

        return limited

    def control_loop(self):
        loop_start = time.perf_counter()
        elapsed = loop_start - self.last_loop_time
        self.last_loop_time = loop_start
        timer_elapsed_ms = elapsed * 1000.0
        timer_jitter_ms = (elapsed - self.dt) * 1000.0

        if elapsed > self.dt * 1.5:
            self.missed_deadline_count += 1

        self.update_gait_phase()

        t0 = time.perf_counter()
        ik_ref = self.ik_reference.get_reference(
            phase=self.gait_phase,
            cmd_vx=self.cmd_vx,
            cmd_vy=self.cmd_vy,
            cmd_wz=self.cmd_wz,
        )
        obs = self.build_observation(ik_ref)
        t1 = time.perf_counter()

        raw_action = self.run_policy(obs)
        t2 = time.perf_counter()

        target_rad = self.postprocess_action(raw_action, ik_ref)
        t3 = time.perf_counter()

        now_msg = self.get_clock().now().to_msg()

        target_msg = JointTarget()
        target_msg.header.stamp = now_msg
        target_msg.header.frame_id = 'base_link'
        target_msg.seq = self.seq

        # Step 3은 아직 실제 policy mode가 아니라 IK dry-run에 가깝다.
        # 그래도 actuator 쪽에서는 position target으로 처리할 수 있으므로 STAND로 둔다.
        target_msg.mode = JointTarget.MODE_STAND
        target_msg.flags = 0
        target_msg.target_rad = target_rad
        target_msg.max_delta_rad = self.max_delta_rad

        self.target_pub.publish(target_msg)

        now = time.perf_counter()
        if now - self.last_debug_pub_time >= 1.0 / self.debug_publish_rate_hz:
            dbg = RlDebug()
            dbg.header.stamp = now_msg
            dbg.header.frame_id = 'base_link'
            dbg.observation = obs
            dbg.raw_action = raw_action
            dbg.ik_ref_rad = ik_ref
            dbg.target_rad = target_rad
            dbg.gait_phase = float(self.gait_phase)
            dbg.cmd_vx = float(self.cmd_vx)
            dbg.cmd_vy = float(self.cmd_vy)
            dbg.cmd_wz = float(self.cmd_wz)
            dbg.feedback_age_ms = float(self.get_feedback_age_ms())
            dbg.obs_build_ms = float((t1 - t0) * 1000.0)
            dbg.inference_ms = float((t2 - t1) * 1000.0)
            dbg.postprocess_ms = float((t3 - t2) * 1000.0)
            dbg.total_loop_ms = float((time.perf_counter() - loop_start) * 1000.0)
            dbg.timer_elapsed_ms = float(timer_elapsed_ms)
            dbg.timer_jitter_ms = float(timer_jitter_ms)
            dbg.missed_deadline_count = self.missed_deadline_count
            dbg.emergency_stop = not self.last_status_ok
            self.debug_pub.publish(dbg)
            self.last_debug_pub_time = now

        self.seq = (self.seq + 1) & 0xFFFF


def main(args=None):
    rclpy.init(args=args)
    node = RlLocomotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
