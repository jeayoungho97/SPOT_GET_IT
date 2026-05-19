from typing import List, Sequence, Tuple

from classic_control.constants import NUM_JOINTS, NUM_LEGS, PHASE_OFFSET
from classic_control.leg_kinematics import LegKinematics
from classic_control.math_utils import clamp, smoothstep


class TrotGaitGenerator:
    def __init__(
        self,
        kinematics: LegKinematics,
        leg_origin_x_m: Sequence[float],
        leg_origin_y_m: Sequence[float],
        body_height_mm: float,
        default_foot_x_mm: float,
        default_foot_y_mm: float,
        gait_period_sec: float,
        duty_factor: float,
        lift_z_mm: float,
        max_stride_x_mm: float,
        max_stride_y_mm: float,
    ):
        self.kinematics = kinematics
        self.leg_origin_x_m = list(leg_origin_x_m)
        self.leg_origin_y_m = list(leg_origin_y_m)
        self.default_foot_x_mm = default_foot_x_mm
        self.default_foot_y_mm = default_foot_y_mm
        self.default_foot_z_mm = -body_height_mm
        self.gait_period_sec = gait_period_sec
        self.duty_factor = duty_factor
        self.lift_z_mm = lift_z_mm
        self.max_stride_x_mm = max_stride_x_mm
        self.max_stride_y_mm = max_stride_y_mm

    def compute_foot_offset(
        self,
        leg_phase: float,
        stride_x_mm: float,
        stride_y_mm: float,
    ) -> Tuple[float, float, float]:
        if leg_phase < self.duty_factor:
            s = leg_phase / self.duty_factor
            x = stride_x_mm * (0.5 - s)
            y = stride_y_mm * (0.5 - s)
            return x, y, 0.0

        s = (leg_phase - self.duty_factor) / (1.0 - self.duty_factor)
        s = smoothstep(s)
        oms = 1.0 - s

        b0 = oms * oms * oms
        b1 = 3.0 * oms * oms * s
        b2 = 3.0 * oms * s * s
        b3 = s * s * s

        x = b0 * (-stride_x_mm * 0.5) + b3 * (stride_x_mm * 0.5)
        y = b0 * (-stride_y_mm * 0.5) + b3 * (stride_y_mm * 0.5)

        z_ctrl = (4.0 / 3.0) * self.lift_z_mm
        z = b1 * z_ctrl + b2 * z_ctrl
        return x, y, z

    def compute_targets(
        self,
        phase: float,
        vx_mps: float,
        vy_mps: float,
        wz_radps: float,
    ) -> List[float]:
        stance_time = self.gait_period_sec * self.duty_factor
        target = [0.0] * NUM_JOINTS

        for leg in range(NUM_LEGS):
            leg_x = self.leg_origin_x_m[leg]
            leg_y = self.leg_origin_y_m[leg]

            foot_vx = vx_mps - wz_radps * leg_y
            foot_vy = vy_mps + wz_radps * leg_x

            stride_x_mm = clamp(
                foot_vx * stance_time * 1000.0,
                -self.max_stride_x_mm,
                self.max_stride_x_mm,
            )
            stride_y_mm = clamp(
                foot_vy * stance_time * 1000.0,
                -self.max_stride_y_mm,
                self.max_stride_y_mm,
            )

            local_phase = (phase + PHASE_OFFSET[leg]) % 1.0
            x_off, y_off, z_off = self.compute_foot_offset(
                local_phase,
                stride_x_mm,
                stride_y_mm,
            )

            foot_x = self.default_foot_x_mm + x_off
            foot_y = self.default_foot_y_mm + y_off
            foot_z = self.default_foot_z_mm + z_off

            shoulder, thigh, knee = self.kinematics.leg_ik_3d(
                leg,
                foot_x,
                foot_y,
                foot_z,
            )

            base = leg * 3
            target[base + 0] = shoulder
            target[base + 1] = thigh
            target[base + 2] = knee

        return self.kinematics.clamp_joint_targets(target)
