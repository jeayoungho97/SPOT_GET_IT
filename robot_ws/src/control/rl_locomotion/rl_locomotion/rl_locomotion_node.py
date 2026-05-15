#!/usr/bin/env python3

import math
import os
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_share_directory

from robot_interfaces.msg import JointTarget, RlDebug, JointFeedback, RobotStatus

from rl_locomotion.gait_phase import GaitPhaseGenerator
from rl_locomotion.ik_reference import make_ik_reference
from rl_locomotion.obs_builder import ObsBuilder
from rl_locomotion.imu_utils import projected_gravity_from_ros_quat_xyzw
from rl_locomotion.policy_runner import PolicyRunner


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def finite_list(values: Sequence[float], expected_len: int) -> bool:
    if len(values) != expected_len:
        return False
    return all(math.isfinite(float(v)) for v in values)


class RlLocomotionNode(Node):
    """
    Final RL locomotion node.

    Runtime flow:
      /cmd_vel
      /control/actuator/joint_feedback
      /control/actuator/imu
      /control/actuator/status
          -> observation[47]
          -> ONNX policy
          -> /control/rl/joint_target, mode=MODE_RL

    Threading note:
      This node is safe for future MultiThreadedExecutor use because shared
      callback state is protected by self.state_lock.
    """

    def __init__(self):
        super().__init__("rl_locomotion_node")

        self.control_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # ---------------- Parameters ----------------
        self.declare_parameter("policy_rate_hz", 50.0)
        self.declare_parameter("obs_dim", 47)
        self.declare_parameter("action_dim", 12)
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("action_clip", 0.5)

        self.declare_parameter("policy_backend", "onnx")
        self.declare_parameter("model_path", "models/exp043_policy.onnx")
        self.declare_parameter("require_model", True)
        self.declare_parameter("obs_clip", 100.0)

        self.declare_parameter("gait_period", 0.6)
        self.declare_parameter("duty_factor", 0.5)
        self.declare_parameter("step_height", 0.03)
        self.declare_parameter("body_height", 0.206)
        self.declare_parameter("robot_width", 0.15)
        self.declare_parameter("phase_cmd_norm", 0.1)
        self.declare_parameter("blend_cmd_norm", 0.1)

        self.declare_parameter("ik_profile", "exp043")

        self.declare_parameter("max_stride_x", 0.12)
        self.declare_parameter("max_stride_y", 0.03)
        self.declare_parameter("shoulder_y_gain", 1.0)
        self.declare_parameter("shoulder_ref_limit", 0.1)

        self.declare_parameter("leg_origin_x", [0.093, 0.093, -0.093, -0.093])
        self.declare_parameter("leg_origin_y", [0.036, -0.036, 0.036, -0.036])
        self.declare_parameter("shoulder_sign", [1.0, -1.0, 1.0, -1.0])

        self.declare_parameter("debug_publish_rate_hz", 2.0)

        self.declare_parameter("feedback_timeout_ms", 150.0)
        self.declare_parameter("imu_timeout_ms", 150.0)
        self.declare_parameter("status_timeout_ms", 150.0)

        # 0 MODE_DISABLE, 1 MODE_STAND, 3 MODE_CROUCH, 4 MODE_E_STOP
        self.declare_parameter("safe_mode", int(JointTarget.MODE_DISABLE))

        self.declare_parameter("vx_min", 0.0)
        self.declare_parameter("vx_max", 0.40)
        self.declare_parameter("vy_min", 0.0)
        self.declare_parameter("vy_max", 0.0)
        self.declare_parameter("wz_min", -0.40)
        self.declare_parameter("wz_max", 0.40)

        self.declare_parameter(
            "joint_names",
            [
                "front_left_shoulder", "front_left_leg", "front_left_foot",
                "front_right_shoulder", "front_right_leg", "front_right_foot",
                "rear_left_shoulder", "rear_left_leg", "rear_left_foot",
                "rear_right_shoulder", "rear_right_leg", "rear_right_foot",
            ],
        )

        self.declare_parameter(
            "default_joint_angles",
            [
                0.0, -0.6, 1.1,
                0.0, -0.6, 1.1,
                0.0, -0.6, 1.1,
                0.0, -0.6, 1.1,
            ],
        )

        self.declare_parameter(
            "joint_min_rad",
            [
                -0.548, -2.666, -0.100,
                -0.548, -2.666, -0.100,
                -0.548, -2.666, -0.100,
                -0.548, -2.666, -0.100,
            ],
        )

        self.declare_parameter(
            "joint_max_rad",
            [
                0.548, 1.548, 2.590,
                0.548, 1.548, 2.590,
                0.548, 1.548, 2.590,
                0.548, 1.548, 2.590,
            ],
        )

        self.declare_parameter("max_joint_speed_rad_s", 1.5)

        # ---------------- Read parameters ----------------
        self.policy_rate_hz = float(self.get_parameter("policy_rate_hz").value)
        if self.policy_rate_hz <= 0.0:
            raise RuntimeError("policy_rate_hz must be positive")

        self.dt = 1.0 / self.policy_rate_hz
        self.obs_dim = int(self.get_parameter("obs_dim").value)
        self.action_dim = int(self.get_parameter("action_dim").value)
        self.action_scale = float(self.get_parameter("action_scale").value)
        self.action_clip = float(self.get_parameter("action_clip").value)

        self.policy_backend = str(self.get_parameter("policy_backend").value).lower().strip()
        self.model_path = str(self.get_parameter("model_path").value)
        self.require_model = bool(self.get_parameter("require_model").value)
        self.obs_clip = float(self.get_parameter("obs_clip").value)

        self.gait_period = float(self.get_parameter("gait_period").value)
        self.duty_factor = float(self.get_parameter("duty_factor").value)
        self.step_height = float(self.get_parameter("step_height").value)
        self.body_height = float(self.get_parameter("body_height").value)
        self.robot_width = float(self.get_parameter("robot_width").value)
        self.phase_cmd_norm = float(self.get_parameter("phase_cmd_norm").value)
        self.blend_cmd_norm = float(self.get_parameter("blend_cmd_norm").value)

        self.ik_profile = str(self.get_parameter("ik_profile").value).lower().strip()

        self.max_stride_x = float(self.get_parameter("max_stride_x").value)
        self.max_stride_y = float(self.get_parameter("max_stride_y").value)
        self.shoulder_y_gain = float(self.get_parameter("shoulder_y_gain").value)
        self.shoulder_ref_limit = float(self.get_parameter("shoulder_ref_limit").value)

        self.leg_origin_x = [float(x) for x in self.get_parameter("leg_origin_x").value]
        self.leg_origin_y = [float(x) for x in self.get_parameter("leg_origin_y").value]
        self.shoulder_sign = [float(x) for x in self.get_parameter("shoulder_sign").value]

        self.debug_publish_rate_hz = float(self.get_parameter("debug_publish_rate_hz").value)

        self.feedback_timeout_ms = float(self.get_parameter("feedback_timeout_ms").value)
        self.imu_timeout_ms = float(self.get_parameter("imu_timeout_ms").value)
        self.status_timeout_ms = float(self.get_parameter("status_timeout_ms").value)
        self.safe_mode = int(self.get_parameter("safe_mode").value)

        self.vx_min = float(self.get_parameter("vx_min").value)
        self.vx_max = float(self.get_parameter("vx_max").value)
        self.vy_min = float(self.get_parameter("vy_min").value)
        self.vy_max = float(self.get_parameter("vy_max").value)
        self.wz_min = float(self.get_parameter("wz_min").value)
        self.wz_max = float(self.get_parameter("wz_max").value)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.default_joint_angles = [float(x) for x in self.get_parameter("default_joint_angles").value]
        self.joint_min_rad = [float(x) for x in self.get_parameter("joint_min_rad").value]
        self.joint_max_rad = [float(x) for x in self.get_parameter("joint_max_rad").value]
        self.max_joint_speed_rad_s = float(self.get_parameter("max_joint_speed_rad_s").value)

        self._validate_config()

        self.max_delta_rad = [self.max_joint_speed_rad_s * self.dt] * 12

        # ---------------- Helper modules ----------------
        self.gait_phase_gen = GaitPhaseGenerator(
            gait_period=self.gait_period,
            phase_cmd_norm=self.phase_cmd_norm,
        )

        self.ik_reference = make_ik_reference(
            profile=self.ik_profile,
            default_joint_angles=self.default_joint_angles,
            gait_period=self.gait_period,
            duty_factor=self.duty_factor,
            step_height=self.step_height,
            body_height=self.body_height,
            robot_width=self.robot_width,
            blend_cmd_norm=self.blend_cmd_norm,
            leg_origin_x=self.leg_origin_x,
            leg_origin_y=self.leg_origin_y,
            shoulder_sign=self.shoulder_sign,
            max_stride_x=self.max_stride_x,
            max_stride_y=self.max_stride_y,
            shoulder_y_gain=self.shoulder_y_gain,
            shoulder_ref_limit=self.shoulder_ref_limit,
        )

        self.obs_builder = ObsBuilder(obs_dim=self.obs_dim)

        resolved_model_path = self.resolve_model_path(self.model_path)
        self.policy_runner = PolicyRunner(
            backend=self.policy_backend,
            model_path=resolved_model_path,
            obs_dim=self.obs_dim,
            action_dim=self.action_dim,
            obs_clip=self.obs_clip,
            require_model=self.require_model,
        )

        # ---------------- Runtime state ----------------
        self.state_lock = threading.Lock()

        self.seq = 0
        self.gait_phase = 0.0
        self.gait_cycle_count = 0

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        self.prev_actions = [0.0] * 12
        self.prev_target_rad = list(self.default_joint_angles)

        self.joint_position = list(self.default_joint_angles)
        self.joint_velocity = [0.0] * 12

        self.base_ang_vel = [0.0, 0.0, 0.0]
        self.projected_gravity = [0.0, 0.0, -1.0]

        self.last_joint_feedback_time: Optional[float] = None
        self.last_imu_time: Optional[float] = None
        self.last_status_time: Optional[float] = None
        self.last_status_ok = False
        self.last_status_code = RobotStatus.STATUS_FAULT

        self.missed_deadline_count = 0
        self.policy_error_count = 0
        self.last_loop_time = time.perf_counter()
        self.last_debug_pub_time = 0.0
        self.last_warn_time = {}

        self.last_obs = [0.0] * self.obs_dim
        self.last_raw_action = [0.0] * self.action_dim
        self.last_ik_ref = list(self.default_joint_angles)
        self.last_target_rad = list(self.default_joint_angles)

        # ---------------- ROS IO ----------------
        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            self.control_qos,
        )

        self.joint_feedback_sub = self.create_subscription(
            JointFeedback,
            "/control/actuator/joint_feedback",
            self.joint_feedback_callback,
            self.control_qos,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/control/actuator/imu",
            self.imu_callback,
            self.control_qos,
        )

        self.status_sub = self.create_subscription(
            RobotStatus,
            "/control/actuator/status",
            self.status_callback,
            self.control_qos,
        )

        self.target_pub = self.create_publisher(
            JointTarget,
            "/control/rl/joint_target",
            self.control_qos,
        )

        self.debug_pub = self.create_publisher(
            RlDebug,
            "/control/rl/debug",
            self.control_qos,
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            "rl_locomotion_node FINAL started: "
            f"{self.policy_rate_hz:.1f} Hz, "
            f"ik_profile={self.ik_profile}, "
            f"backend={self.policy_runner.backend_name()}, "
            f"providers={self.policy_runner.provider_names()}, "
            f"model={resolved_model_path}, "
            f"safe_mode={self.safe_mode}"
        )

    def _validate_config(self):
        if self.obs_dim != 47:
            raise RuntimeError(f"exp043 obs_dim must be 47, got {self.obs_dim}")
        if self.action_dim != 12:
            raise RuntimeError(f"action_dim must be 12, got {self.action_dim}")
        if self.policy_backend != "onnx":
            raise RuntimeError(
                f"Final code supports only policy_backend=onnx, got {self.policy_backend}"
            )
        if not self.require_model:
            raise RuntimeError("Final code requires require_model=true")

        for name, arr in [
            ("joint_names", self.joint_names),
            ("default_joint_angles", self.default_joint_angles),
            ("joint_min_rad", self.joint_min_rad),
            ("joint_max_rad", self.joint_max_rad),
        ]:
            if len(arr) != 12:
                raise RuntimeError(f"{name} must have 12 elements, got {len(arr)}")

        for i in range(12):
            if self.joint_min_rad[i] >= self.joint_max_rad[i]:
                raise RuntimeError(
                    f"invalid joint limit index {i}: "
                    f"{self.joint_min_rad[i]} >= {self.joint_max_rad[i]}"
                )
            if not (
                self.joint_min_rad[i]
                <= self.default_joint_angles[i]
                <= self.joint_max_rad[i]
            ):
                raise RuntimeError(
                    f"default angle out of limit index {i}: "
                    f"default={self.default_joint_angles[i]}, "
                    f"min={self.joint_min_rad[i]}, "
                    f"max={self.joint_max_rad[i]}"
                )

        valid_safe_modes = {
            JointTarget.MODE_DISABLE,
            JointTarget.MODE_STAND,
            JointTarget.MODE_CROUCH,
            JointTarget.MODE_E_STOP,
        }
        if self.safe_mode not in valid_safe_modes:
            raise RuntimeError(f"invalid safe_mode: {self.safe_mode}")

        valid_ik_profiles = {"exp043", "legacy", "old", "lateral", "lateral_ik", "spotmicro_test"}
        if self.ik_profile not in valid_ik_profiles:
            raise RuntimeError(f"invalid ik_profile: {self.ik_profile}")

        for name, arr in [
            ("leg_origin_x", self.leg_origin_x),
            ("leg_origin_y", self.leg_origin_y),
            ("shoulder_sign", self.shoulder_sign),
        ]:
            if len(arr) != 4:
                raise RuntimeError(f"{name} must have 4 elements, got {len(arr)}")

        if self.max_stride_x <= 0.0:
            raise RuntimeError("max_stride_x must be positive")
        if self.max_stride_y < 0.0:
            raise RuntimeError("max_stride_y must be non-negative")
        if self.shoulder_ref_limit < 0.0:
            raise RuntimeError("shoulder_ref_limit must be non-negative")

    def resolve_model_path(self, model_path: str) -> str:
        if not model_path:
            return ""

        expanded = os.path.expanduser(model_path)
        if os.path.isabs(expanded):
            return expanded

        pkg_share = get_package_share_directory("rl_locomotion")
        return os.path.join(pkg_share, expanded)

    # -------------------------------------------------------------------------
    # ROS callbacks
    # -------------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        with self.state_lock:
            self.cmd_vx = clamp(float(msg.linear.x), self.vx_min, self.vx_max)
            self.cmd_vy = clamp(float(msg.linear.y), self.vy_min, self.vy_max)
            self.cmd_wz = clamp(float(msg.angular.z), self.wz_min, self.wz_max)

    def joint_feedback_callback(self, msg: JointFeedback):
        if not finite_list(msg.position_rad, 12):
            self._warn_throttled("joint_pos_len", "invalid JointFeedback.position_rad")
            return

        if not finite_list(msg.velocity_rad_s, 12):
            self._warn_throttled("joint_vel_len", "invalid JointFeedback.velocity_rad_s")
            return

        with self.state_lock:
            self.joint_position = [float(v) for v in msg.position_rad]
            self.joint_velocity = [float(v) for v in msg.velocity_rad_s]
            self.last_joint_feedback_time = time.perf_counter()

    def imu_callback(self, msg: Imu):
        # IMU 데이터는 이미 robot body frame 으로 들어옴.
        # (STM 펌웨어의 BNO055 P6 axis remap 이 chip 차원에서 변환 적용됨)

        ang = [
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        ]

        if not finite_list(ang, 3):
            self._warn_throttled("imu_ang", "invalid IMU angular velocity")
            return

        # projected_gravity 는 raw accel 기반.
        # BNO055 quat 의 fusion reference drift 회피 (IMUPLUS 모드 한계).
        # 보행 중 linear accel 노이즈는 작아서 50Hz 통계적으로 OK.
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
        if not math.isfinite(accel_mag) or accel_mag < 1.0:
            self._warn_throttled("imu_accel", "invalid IMU linear acceleration")
            return

        projected_gravity = [-ax / accel_mag, -ay / accel_mag, -az / accel_mag]

        with self.state_lock:
            self.base_ang_vel = ang
            self.projected_gravity = projected_gravity
            self.last_imu_time = time.perf_counter()

    def status_callback(self, msg: RobotStatus):
        with self.state_lock:
            self.last_status_time = time.perf_counter()
            self.last_status_code = int(msg.status)
            self.last_status_ok = (
                msg.status == RobotStatus.STATUS_OK
                and bool(msg.torque_enabled)
                and bool(msg.spi_connected)
                and bool(msg.servo_connected)
            )

    # -------------------------------------------------------------------------
    # Snapshot / readiness
    # -------------------------------------------------------------------------
    def _age_ms_locked(self, t: Optional[float], now: float) -> float:
        if t is None:
            return 9999.0
        return (now - t) * 1000.0

    def snapshot_state(self):
        now = time.perf_counter()
        with self.state_lock:
            feedback_age_ms = self._age_ms_locked(self.last_joint_feedback_time, now)
            imu_age_ms = self._age_ms_locked(self.last_imu_time, now)
            status_age_ms = self._age_ms_locked(self.last_status_time, now)

            snapshot = {
                "cmd_vx": self.cmd_vx,
                "cmd_vy": self.cmd_vy,
                "cmd_wz": self.cmd_wz,
                "joint_position": list(self.joint_position),
                "joint_velocity": list(self.joint_velocity),
                "base_ang_vel": list(self.base_ang_vel),
                "projected_gravity": list(self.projected_gravity),
                "feedback_age_ms": feedback_age_ms,
                "imu_age_ms": imu_age_ms,
                "status_age_ms": status_age_ms,
                "last_status_ok": self.last_status_ok,
                "last_status_code": self.last_status_code,
            }
        return snapshot

    def policy_ready(self, snapshot) -> Tuple[bool, str]:
        if snapshot["feedback_age_ms"] > self.feedback_timeout_ms:
            return False, "joint feedback timeout"

        if snapshot["imu_age_ms"] > self.imu_timeout_ms:
            return False, "imu timeout"

        if snapshot["status_age_ms"] > self.status_timeout_ms:
            return False, "status timeout"

        if not snapshot["last_status_ok"]:
            return False, f"actuator status not OK: status={snapshot['last_status_code']}"

        return True, "ok"

    # -------------------------------------------------------------------------
    # Policy pipeline
    # -------------------------------------------------------------------------
    def update_gait_phase(self, cmd_vx: float, cmd_vy: float, cmd_wz: float):
        prev_phase = self.gait_phase
        self.gait_phase = self.gait_phase_gen.update(
            dt=self.dt,
            cmd_vx=cmd_vx,
            cmd_vy=cmd_vy,
            cmd_wz=cmd_wz,
        )
        if self.gait_phase < prev_phase:
            self.gait_cycle_count += 1

    def build_observation(self, snapshot, ik_ref: List[float]) -> List[float]:
        return self.obs_builder.build(
            base_ang_vel=snapshot["base_ang_vel"],
            projected_gravity=snapshot["projected_gravity"],
            cmd_vx=snapshot["cmd_vx"],
            cmd_vy=snapshot["cmd_vy"],
            cmd_wz=snapshot["cmd_wz"],
            dof_pos=snapshot["joint_position"],
            dof_vel=snapshot["joint_velocity"],
            ik_ref=ik_ref,
            prev_actions=self.prev_actions,
            gait_phase=self.gait_phase,
        )

    def postprocess_action(
        self,
        raw_action: Sequence[float],
        ik_ref: Sequence[float],
    ) -> List[float]:
        if not finite_list(raw_action, 12):
            raise RuntimeError("policy output is invalid")

        clipped_action = [
            clamp(float(a), -self.action_clip, self.action_clip)
            for a in raw_action
        ]

        target = [
            float(ik_ref[i]) + clipped_action[i] * self.action_scale
            for i in range(12)
        ]

        target = [
            clamp(target[i], self.joint_min_rad[i], self.joint_max_rad[i])
            for i in range(12)
        ]

        limited = []
        for i in range(12):
            delta = target[i] - self.prev_target_rad[i]
            delta = clamp(delta, -self.max_delta_rad[i], self.max_delta_rad[i])
            limited.append(self.prev_target_rad[i] + delta)

        self.prev_target_rad = list(limited)
        self.prev_actions = list(clipped_action)

        return limited

    def publish_target(
        self,
        target_rad: List[float],
        mode: int,
        stamp_msg,
    ):
        msg = JointTarget()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = "base_link"
        msg.seq = self.seq
        msg.mode = int(mode)
        msg.flags = 0
        msg.target_rad = list(target_rad)
        msg.max_delta_rad = list(self.max_delta_rad)
        msg.gait_phase = float(self.gait_phase)
        msg.gait_cycle_count = int(self.gait_cycle_count)
        self.target_pub.publish(msg)

    def publish_safe_target(self, reason: str, stamp_msg):
        self.publish_target(
            target_rad=list(self.default_joint_angles),
            mode=self.safe_mode,
            stamp_msg=stamp_msg,
        )

        self.prev_target_rad = list(self.default_joint_angles)
        self.prev_actions = [0.0] * 12
        self.last_raw_action = [0.0] * 12
        self.last_ik_ref = list(self.default_joint_angles)
        self.last_target_rad = list(self.default_joint_angles)

        self._warn_throttled("safe_target", f"publishing safe target: {reason}")

    # -------------------------------------------------------------------------
    # Debug
    # -------------------------------------------------------------------------
    def publish_debug(
        self,
        stamp_msg,
        snapshot,
        obs: List[float],
        raw_action: List[float],
        ik_ref: List[float],
        target_rad: List[float],
        obs_build_ms: float,
        inference_ms: float,
        postprocess_ms: float,
        total_loop_ms: float,
        timer_elapsed_ms: float,
        timer_jitter_ms: float,
        emergency_stop: bool,
    ):
        now = time.perf_counter()
        if now - self.last_debug_pub_time < 1.0 / self.debug_publish_rate_hz:
            return

        dbg = RlDebug()
        dbg.header.stamp = stamp_msg
        dbg.header.frame_id = "base_link"
        dbg.observation = list(obs)
        dbg.raw_action = list(raw_action)
        dbg.ik_ref_rad = list(ik_ref)
        dbg.target_rad = list(target_rad)
        dbg.gait_phase = float(self.gait_phase)
        dbg.cmd_vx = float(snapshot["cmd_vx"])
        dbg.cmd_vy = float(snapshot["cmd_vy"])
        dbg.cmd_wz = float(snapshot["cmd_wz"])
        dbg.feedback_age_ms = float(snapshot["feedback_age_ms"])
        dbg.obs_build_ms = float(obs_build_ms)
        dbg.inference_ms = float(inference_ms)
        dbg.postprocess_ms = float(postprocess_ms)
        dbg.total_loop_ms = float(total_loop_ms)
        dbg.timer_elapsed_ms = float(timer_elapsed_ms)
        dbg.timer_jitter_ms = float(timer_jitter_ms)
        dbg.missed_deadline_count = int(self.missed_deadline_count)
        dbg.emergency_stop = bool(emergency_stop)

        self.debug_pub.publish(dbg)
        self.last_debug_pub_time = now

    def _warn_throttled(self, key: str, msg: str, period_s: float = 1.0):
        now = time.perf_counter()
        last = self.last_warn_time.get(key, 0.0)
        if now - last >= period_s:
            self.get_logger().warn(msg)
            self.last_warn_time[key] = now

    # -------------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------------
    def control_loop(self):
        loop_start = time.perf_counter()
        elapsed = loop_start - self.last_loop_time
        self.last_loop_time = loop_start

        timer_elapsed_ms = elapsed * 1000.0
        timer_jitter_ms = (elapsed - self.dt) * 1000.0

        if elapsed > self.dt * 1.5:
            self.missed_deadline_count += 1

        stamp_msg = self.get_clock().now().to_msg()
        snapshot = self.snapshot_state()

        ready, reason = self.policy_ready(snapshot)

        if not ready:
            self.publish_safe_target(reason, stamp_msg)
            total_loop_ms = (time.perf_counter() - loop_start) * 1000.0
            self.publish_debug(
                stamp_msg=stamp_msg,
                snapshot=snapshot,
                obs=self.last_obs,
                raw_action=[0.0] * 12,
                ik_ref=list(self.default_joint_angles),
                target_rad=list(self.default_joint_angles),
                obs_build_ms=0.0,
                inference_ms=0.0,
                postprocess_ms=0.0,
                total_loop_ms=total_loop_ms,
                timer_elapsed_ms=timer_elapsed_ms,
                timer_jitter_ms=timer_jitter_ms,
                emergency_stop=True,
            )
            self.seq = (self.seq + 1) & 0xFFFF
            return

        self.update_gait_phase(
            cmd_vx=snapshot["cmd_vx"],
            cmd_vy=snapshot["cmd_vy"],
            cmd_wz=snapshot["cmd_wz"],
        )

        try:
            t0 = time.perf_counter()

            ik_ref = self.ik_reference.get_reference(
                phase=self.gait_phase,
                cmd_vx=snapshot["cmd_vx"],
                cmd_vy=snapshot["cmd_vy"],
                cmd_wz=snapshot["cmd_wz"],
            )

            obs = self.build_observation(snapshot, ik_ref)
            t1 = time.perf_counter()

            raw_action = self.policy_runner.infer(obs)
            t2 = time.perf_counter()

            target_rad = self.postprocess_action(raw_action, ik_ref)
            t3 = time.perf_counter()

            self.publish_target(
                target_rad=target_rad,
                mode=JointTarget.MODE_RL,
                stamp_msg=stamp_msg,
            )

            self.last_obs = list(obs)
            self.last_raw_action = list(raw_action)
            self.last_ik_ref = list(ik_ref)
            self.last_target_rad = list(target_rad)

            self.publish_debug(
                stamp_msg=stamp_msg,
                snapshot=snapshot,
                obs=obs,
                raw_action=raw_action,
                ik_ref=ik_ref,
                target_rad=target_rad,
                obs_build_ms=(t1 - t0) * 1000.0,
                inference_ms=(t2 - t1) * 1000.0,
                postprocess_ms=(t3 - t2) * 1000.0,
                total_loop_ms=(time.perf_counter() - loop_start) * 1000.0,
                timer_elapsed_ms=timer_elapsed_ms,
                timer_jitter_ms=timer_jitter_ms,
                emergency_stop=False,
            )

        except Exception as exc:
            self.policy_error_count += 1
            self.publish_safe_target(f"policy pipeline error: {exc}", stamp_msg)

            total_loop_ms = (time.perf_counter() - loop_start) * 1000.0
            self.publish_debug(
                stamp_msg=stamp_msg,
                snapshot=snapshot,
                obs=self.last_obs,
                raw_action=[0.0] * 12,
                ik_ref=list(self.default_joint_angles),
                target_rad=list(self.default_joint_angles),
                obs_build_ms=0.0,
                inference_ms=0.0,
                postprocess_ms=0.0,
                total_loop_ms=total_loop_ms,
                timer_elapsed_ms=timer_elapsed_ms,
                timer_jitter_ms=timer_jitter_ms,
                emergency_stop=True,
            )

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


if __name__ == "__main__":
    main()
