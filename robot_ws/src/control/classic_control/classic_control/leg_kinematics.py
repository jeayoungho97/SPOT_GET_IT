import math
from typing import List, Sequence, Tuple

from classic_control.constants import NUM_JOINTS
from classic_control.math_utils import clamp


class LegKinematics:
    def __init__(
        self,
        l1_mm: float,
        l2_mm: float,
        shoulder_sign: Sequence[float],
        shoulder_y_gain: float,
        shoulder_limit_rad: float,
        joint_min_rad: Sequence[float],
        joint_max_rad: Sequence[float],
    ):
        self.l1_mm = l1_mm
        self.l2_mm = l2_mm
        self.shoulder_sign = list(shoulder_sign)
        self.shoulder_y_gain = shoulder_y_gain
        self.shoulder_limit_rad = shoulder_limit_rad
        self.joint_min_rad = list(joint_min_rad)
        self.joint_max_rad = list(joint_max_rad)

    def ik_2link(self, foot_x_mm: float, foot_z_mm: float) -> Tuple[float, float]:
        r2 = foot_x_mm * foot_x_mm + foot_z_mm * foot_z_mm
        cos_k = (
            (r2 - self.l1_mm * self.l1_mm - self.l2_mm * self.l2_mm)
            / (2.0 * self.l1_mm * self.l2_mm)
        )
        cos_k = clamp(cos_k, -1.0, 1.0)
        sin_k = math.sqrt(max(0.0, 1.0 - cos_k * cos_k))
        theta_k = math.atan2(sin_k, cos_k)

        a = self.l1_mm + self.l2_mm * cos_k
        b = self.l2_mm * sin_k
        det = a * a + b * b
        foot_z_neg = -foot_z_mm
        sin_t = (a * foot_x_mm - b * foot_z_neg) / det
        cos_t = (b * foot_x_mm + a * foot_z_neg) / det
        theta_t = math.atan2(sin_t, cos_t)
        return theta_t, theta_k

    def leg_ik_3d(self, leg: int, x_mm: float, y_mm: float, z_mm: float):
        shoulder = self.shoulder_y_gain * math.atan2(y_mm, -z_mm)
        shoulder = clamp(shoulder, -self.shoulder_limit_rad, self.shoulder_limit_rad)
        shoulder *= self.shoulder_sign[leg]

        z_eff = -math.sqrt(max(z_mm * z_mm + y_mm * y_mm, 1.0e-6))
        thigh, knee = self.ik_2link(x_mm, z_eff)
        return shoulder, thigh, knee

    def clamp_joint_targets(self, target: Sequence[float]) -> List[float]:
        return [
            clamp(float(target[i]), self.joint_min_rad[i], self.joint_max_rad[i])
            for i in range(NUM_JOINTS)
        ]
