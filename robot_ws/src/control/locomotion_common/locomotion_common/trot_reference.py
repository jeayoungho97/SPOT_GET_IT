import math
from typing import List, Optional, Sequence


NUM_LEGS = 4
NUM_JOINTS = 12


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def expand_leg_values(value, default: float, name: str) -> List[float]:
    if value is None:
        return [float(default)] * NUM_LEGS
    if isinstance(value, (float, int)):
        return [float(value)] * NUM_LEGS
    values = [float(v) for v in value]
    if len(values) != NUM_LEGS:
        raise ValueError(f"{name} must have {NUM_LEGS} elements")
    return values


class SharedTrotReference:
    """
    Shared diagonal-trot foot reference and offset-link IK.

    This is intended to be the single reference used by both classic control
    and future RL retraining. Classic publishes this output directly; RL can
    use it as ik_ref and add policy residuals.
    """

    def __init__(
        self,
        gait_period: float = 1.0,
        duty_factor: float = 0.55,
        body_height: float = 0.170,
        step_height: Sequence[float] = (0.013, 0.013, 0.016, 0.016),
        default_foot_x: Sequence[float] = (-0.010, -0.010, -0.010, -0.010),
        default_foot_y: Sequence[float] = (0.0, 0.0, 0.0, 0.0),
        leg_origin_x: Sequence[float] = (0.093, 0.093, -0.093, -0.093),
        leg_origin_y: Sequence[float] = (0.036, -0.036, 0.036, -0.036),
        shoulder_sign: Sequence[float] = (1.0, -1.0, 1.0, -1.0),
        phase_offsets: Sequence[float] = (0.0, 0.5, 0.5, 0.0),
        max_stride_x: float = 0.07,
        max_stride_y: float = 0.03,
        upper_link_x: float = 0.0,
        upper_link_z: float = 0.105,
        lower_link: float = 0.130,
        shoulder_y_gain: float = 1.0,
        shoulder_limit: float = 0.16,
        joint_min: Optional[Sequence[float]] = None,
        joint_max: Optional[Sequence[float]] = None,
    ):
        if gait_period <= 0.0:
            raise ValueError("gait_period must be positive")
        if not 0.0 < duty_factor < 1.0:
            raise ValueError("duty_factor must be in (0, 1)")
        if upper_link_z <= 0.0 or lower_link <= 0.0:
            raise ValueError("link lengths must be positive")

        self.gait_period = float(gait_period)
        self.duty_factor = float(duty_factor)
        self.body_height = expand_leg_values(body_height, body_height, "body_height")
        self.step_height = expand_leg_values(step_height, 0.03, "step_height")
        self.default_foot_x = expand_leg_values(default_foot_x, 0.0, "default_foot_x")
        self.default_foot_y = expand_leg_values(default_foot_y, 0.0, "default_foot_y")
        self.leg_origin_x = expand_leg_values(leg_origin_x, 0.0, "leg_origin_x")
        self.leg_origin_y = expand_leg_values(leg_origin_y, 0.0, "leg_origin_y")
        self.shoulder_sign = expand_leg_values(shoulder_sign, 1.0, "shoulder_sign")
        self.phase_offsets = expand_leg_values(phase_offsets, 0.0, "phase_offsets")

        self.max_stride_x = float(max_stride_x)
        self.max_stride_y = float(max_stride_y)
        self.upper_link_x = float(upper_link_x)
        self.upper_link_z = float(upper_link_z)
        self.lower_link = float(lower_link)
        self.shoulder_y_gain = float(shoulder_y_gain)
        self.shoulder_limit = float(shoulder_limit)

        self.joint_min = list(joint_min) if joint_min is not None else None
        self.joint_max = list(joint_max) if joint_max is not None else None
        if (self.joint_min is None) != (self.joint_max is None):
            raise ValueError("joint_min and joint_max must be provided together")
        if self.joint_min is not None:
            if len(self.joint_min) != NUM_JOINTS or len(self.joint_max) != NUM_JOINTS:
                raise ValueError("joint limits must have 12 elements")

    def get_reference(
        self,
        phase: float,
        cmd_vx: float,
        cmd_vy: float,
        cmd_wz: float,
    ) -> List[float]:
        stance_time = self.gait_period * self.duty_factor
        target = [0.0] * NUM_JOINTS

        for leg in range(NUM_LEGS):
            foot_vx = cmd_vx - cmd_wz * self.leg_origin_y[leg]
            foot_vy = cmd_vy + cmd_wz * self.leg_origin_x[leg]

            stride_x = clamp(
                foot_vx * stance_time,
                -self.max_stride_x,
                self.max_stride_x,
            )
            stride_y = clamp(
                foot_vy * stance_time,
                -self.max_stride_y,
                self.max_stride_y,
            )

            local_phase = (phase + self.phase_offsets[leg]) % 1.0
            x_off, y_off, z_off = self._foot_offset(
                local_phase,
                stride_x,
                stride_y,
                self.step_height[leg],
            )

            x = self.default_foot_x[leg] + x_off
            y = self.default_foot_y[leg] + y_off
            z = -self.body_height[leg] + z_off

            shoulder, thigh, knee = self._leg_ik(leg, x, y, z)
            base = leg * 3
            target[base + 0] = shoulder
            target[base + 1] = thigh
            target[base + 2] = knee

        return self._clamp_joints(target)

    def _foot_offset(
        self,
        leg_phase: float,
        stride_x: float,
        stride_y: float,
        step_height: float,
    ):
        if leg_phase < self.duty_factor:
            s = leg_phase / self.duty_factor
            x = stride_x * (0.5 - s)
            y = stride_y * (0.5 - s)
            return x, y, 0.0

        s = (leg_phase - self.duty_factor) / (1.0 - self.duty_factor)
        # Smooth x/y endpoints, but use a sine lift to avoid low mid-swing clearance.
        ss = s * s * (3.0 - 2.0 * s)
        x = stride_x * (-0.5 + ss)
        y = stride_y * (-0.5 + ss)
        z = step_height * math.sin(math.pi * s)
        return x, y, z

    def _leg_ik(self, leg: int, x: float, y: float, z: float):
        shoulder_raw = self.shoulder_y_gain * math.atan2(y, -z)
        shoulder = clamp(shoulder_raw, -self.shoulder_limit, self.shoulder_limit)
        shoulder *= self.shoulder_sign[leg]

        z_eff = -math.sqrt(max(z * z + y * y, 1.0e-9))
        thigh, knee = self._solve_sagittal_ik(x, z_eff)
        return shoulder, thigh, knee

    def _solve_sagittal_ik(self, x: float, z: float):
        # If upper_link_x is zero, this is the original classic 2-link solver.
        # A non-zero upper_link_x is supported by folding it into an effective
        # upper link and angle offset, but the default intentionally matches
        # the classic controller because it is the hardware baseline.
        upper_eff = math.sqrt(
            self.upper_link_x * self.upper_link_x
            + self.upper_link_z * self.upper_link_z
        )
        upper_alpha = math.atan2(self.upper_link_x, self.upper_link_z)

        r2 = x * x + z * z
        cos_knee = (
            r2 - upper_eff * upper_eff - self.lower_link * self.lower_link
        ) / (2.0 * upper_eff * self.lower_link)
        cos_knee = clamp(cos_knee, -1.0, 1.0)
        sin_knee = math.sqrt(max(0.0, 1.0 - cos_knee * cos_knee))
        knee_raw = math.atan2(sin_knee, cos_knee)

        a = upper_eff + self.lower_link * cos_knee
        b = self.lower_link * sin_knee
        det = a * a + b * b
        z_neg = -z

        sin_thigh = (a * x - b * z_neg) / det
        cos_thigh = (b * x + a * z_neg) / det
        thigh = math.atan2(sin_thigh, cos_thigh) - upper_alpha
        knee = knee_raw + upper_alpha

        return thigh, knee

    def _clamp_joints(self, target: Sequence[float]) -> List[float]:
        if self.joint_min is None:
            return [float(v) for v in target]
        return [
            clamp(float(target[i]), self.joint_min[i], self.joint_max[i])
            for i in range(NUM_JOINTS)
        ]


class TorchSharedTrotReference:
    """
    Vectorized torch implementation of SharedTrotReference.

    Keep this class numerically aligned with SharedTrotReference so Isaac Gym
    training and ROS runtime use the same gait/IK model without a Python loop
    over thousands of environments.
    """

    def __init__(
        self,
        device,
        dtype=None,
        gait_period: float = 1.0,
        duty_factor: float = 0.55,
        body_height: float = 0.170,
        step_height: Sequence[float] = (0.013, 0.013, 0.016, 0.016),
        default_foot_x: Sequence[float] = (-0.010, -0.010, -0.010, -0.010),
        default_foot_y: Sequence[float] = (0.0, 0.0, 0.0, 0.0),
        leg_origin_x: Sequence[float] = (0.093, 0.093, -0.093, -0.093),
        leg_origin_y: Sequence[float] = (0.036, -0.036, 0.036, -0.036),
        shoulder_sign: Sequence[float] = (1.0, -1.0, 1.0, -1.0),
        phase_offsets: Sequence[float] = (0.0, 0.5, 0.5, 0.0),
        max_stride_x: float = 0.07,
        max_stride_y: float = 0.03,
        upper_link_x: float = 0.0,
        upper_link_z: float = 0.105,
        lower_link: float = 0.130,
        shoulder_y_gain: float = 1.0,
        shoulder_limit: float = 0.16,
        joint_min: Optional[Sequence[float]] = None,
        joint_max: Optional[Sequence[float]] = None,
    ):
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError(
                "TorchSharedTrotReference requires torch to be installed"
            ) from exc

        if gait_period <= 0.0:
            raise ValueError("gait_period must be positive")
        if not 0.0 < duty_factor < 1.0:
            raise ValueError("duty_factor must be in (0, 1)")
        if upper_link_z <= 0.0 or lower_link <= 0.0:
            raise ValueError("link lengths must be positive")

        self.torch = torch
        self.device = device
        self.dtype = dtype or torch.float32

        self.gait_period = float(gait_period)
        self.duty_factor = float(duty_factor)
        self.max_stride_x = float(max_stride_x)
        self.max_stride_y = float(max_stride_y)
        self.upper_link_x = float(upper_link_x)
        self.upper_link_z = float(upper_link_z)
        self.lower_link = float(lower_link)
        self.shoulder_y_gain = float(shoulder_y_gain)
        self.shoulder_limit = float(shoulder_limit)

        self.body_height = self._leg_tensor(
            expand_leg_values(body_height, body_height, "body_height")
        )
        self.step_height = self._leg_tensor(
            expand_leg_values(step_height, 0.03, "step_height")
        )
        self.default_foot_x = self._leg_tensor(
            expand_leg_values(default_foot_x, 0.0, "default_foot_x")
        )
        self.default_foot_y = self._leg_tensor(
            expand_leg_values(default_foot_y, 0.0, "default_foot_y")
        )
        self.leg_origin_x = self._leg_tensor(
            expand_leg_values(leg_origin_x, 0.0, "leg_origin_x")
        )
        self.leg_origin_y = self._leg_tensor(
            expand_leg_values(leg_origin_y, 0.0, "leg_origin_y")
        )
        self.shoulder_sign = self._leg_tensor(
            expand_leg_values(shoulder_sign, 1.0, "shoulder_sign")
        )
        self.phase_offsets = self._leg_tensor(
            expand_leg_values(phase_offsets, 0.0, "phase_offsets")
        )

        self.joint_min = None
        self.joint_max = None
        if (joint_min is None) != (joint_max is None):
            raise ValueError("joint_min and joint_max must be provided together")
        if joint_min is not None:
            if len(joint_min) != NUM_JOINTS or len(joint_max) != NUM_JOINTS:
                raise ValueError("joint limits must have 12 elements")
            self.joint_min = self._joint_tensor(joint_min)
            self.joint_max = self._joint_tensor(joint_max)

    def _leg_tensor(self, values: Sequence[float]):
        return self.torch.tensor(
            values,
            device=self.device,
            dtype=self.dtype,
        ).view(1, NUM_LEGS)

    def _joint_tensor(self, values: Sequence[float]):
        return self.torch.tensor(
            values,
            device=self.device,
            dtype=self.dtype,
        ).view(1, NUM_JOINTS)

    def get_reference(self, phase, commands):
        torch = self.torch
        if phase.dim() == 1:
            phase = phase.unsqueeze(1)
        if commands.dim() != 2 or commands.shape[1] < 3:
            raise ValueError("commands must have shape [num_envs, >=3]")

        vx = commands[:, 0:1]
        vy = commands[:, 1:2]
        wz = commands[:, 2:3]

        stance_time = self.gait_period * self.duty_factor
        foot_vx = vx - wz * self.leg_origin_y
        foot_vy = vy + wz * self.leg_origin_x

        stride_x = torch.clamp(
            foot_vx * stance_time,
            -self.max_stride_x,
            self.max_stride_x,
        )
        stride_y = torch.clamp(
            foot_vy * stance_time,
            -self.max_stride_y,
            self.max_stride_y,
        )

        leg_phase = (phase + self.phase_offsets) % 1.0
        x_off, y_off, z_off = self._foot_offset(leg_phase, stride_x, stride_y)

        x = self.default_foot_x + x_off
        y = self.default_foot_y + y_off
        z = -self.body_height + z_off

        shoulder, thigh, knee = self._leg_ik(x, y, z)
        target = torch.empty(
            (commands.shape[0], NUM_JOINTS),
            device=self.device,
            dtype=self.dtype,
        )
        target[:, 0::3] = shoulder
        target[:, 1::3] = thigh
        target[:, 2::3] = knee

        return self._clamp_joints(target)

    def _foot_offset(self, leg_phase, stride_x, stride_y):
        torch = self.torch
        is_stance = leg_phase < self.duty_factor

        s_stance = leg_phase / self.duty_factor
        x_stance = stride_x * (0.5 - s_stance)
        y_stance = stride_y * (0.5 - s_stance)
        z_stance = torch.zeros_like(x_stance)

        s_swing = (leg_phase - self.duty_factor) / (1.0 - self.duty_factor)
        ss = s_swing * s_swing * (3.0 - 2.0 * s_swing)
        x_swing = stride_x * (-0.5 + ss)
        y_swing = stride_y * (-0.5 + ss)
        z_swing = self.step_height * torch.sin(math.pi * s_swing)

        x = torch.where(is_stance, x_stance, x_swing)
        y = torch.where(is_stance, y_stance, y_swing)
        z = torch.where(is_stance, z_stance, z_swing)
        return x, y, z

    def _leg_ik(self, x, y, z):
        torch = self.torch
        shoulder_raw = self.shoulder_y_gain * torch.atan2(y, -z)
        shoulder = torch.clamp(
            shoulder_raw,
            -self.shoulder_limit,
            self.shoulder_limit,
        )
        shoulder = shoulder * self.shoulder_sign

        z_eff = -torch.sqrt(torch.clamp(z * z + y * y, min=1.0e-9))
        thigh, knee = self._solve_sagittal_ik(x, z_eff)
        return shoulder, thigh, knee

    def _solve_sagittal_ik(self, x, z):
        torch = self.torch
        upper_eff = math.sqrt(
            self.upper_link_x * self.upper_link_x
            + self.upper_link_z * self.upper_link_z
        )
        upper_alpha = math.atan2(self.upper_link_x, self.upper_link_z)

        r2 = x * x + z * z
        cos_knee = (
            r2 - upper_eff * upper_eff - self.lower_link * self.lower_link
        ) / (2.0 * upper_eff * self.lower_link)
        cos_knee = torch.clamp(cos_knee, -1.0, 1.0)
        sin_knee = torch.sqrt(torch.clamp(1.0 - cos_knee * cos_knee, min=0.0))
        knee_raw = torch.atan2(sin_knee, cos_knee)

        a = upper_eff + self.lower_link * cos_knee
        b = self.lower_link * sin_knee
        det = a * a + b * b
        z_neg = -z

        sin_thigh = (a * x - b * z_neg) / det
        cos_thigh = (b * x + a * z_neg) / det
        thigh = torch.atan2(sin_thigh, cos_thigh) - upper_alpha
        knee = knee_raw + upper_alpha

        return thigh, knee

    def _clamp_joints(self, target):
        if self.joint_min is None:
            return target
        return self.torch.max(self.torch.min(target, self.joint_max), self.joint_min)
