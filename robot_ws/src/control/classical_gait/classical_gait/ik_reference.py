import math
from typing import List


class TrotIkReference:
    """
    Spot Micro 4-leg trot IK reference 생성기.

    cmd_vel + gait_phase → 12 joint angle (rad).
    rl_locomotion 의 TrotIkReference 와 동일한 식을 따르되,
    L1_X / L1_Z / L2 / body_height 는 **실제 robot 측정값** 사용.

    학습 URDF 값 (참고):
      L1_X=0.01, L1_Z=0.12, L2=0.115, body_height=0.206

    실제 측정값 (이번 패키지 default):
      L1_X=0.03, L1_Z=0.13, L2=0.110, body_height=0.20

    Leg order: FL, FR, RL, RR
    Phase offsets: [0.0, 0.5, 0.5, 0.0]  (diagonal trot)
    """

    def __init__(
        self,
        default_joint_angles: List[float],
        gait_period: float = 0.6,
        duty_factor: float = 0.5,
        step_height: float = 0.03,
        body_height: float = 0.20,
        robot_width: float = 0.15,
        blend_cmd_norm: float = 0.1,
        l1_x: float = 0.03,
        l1_z: float = 0.13,
        l2: float = 0.110,
    ):
        if len(default_joint_angles) != 12:
            raise ValueError("default_joint_angles must have 12 elements")

        self.default_joint_angles = list(default_joint_angles)

        self.l1_x = l1_x
        self.l1_z = l1_z
        self.l2 = l2
        self.l1_eff = math.sqrt(l1_x ** 2 + l1_z ** 2)
        self.alpha = math.atan2(l1_x, l1_z)

        self.robot_width = robot_width
        self.gait_period = gait_period
        self.duty_factor = duty_factor
        self.step_height = step_height
        self.body_height = body_height
        self.blend_cmd_norm = blend_cmd_norm

        # FL, FR, RL, RR
        self.phase_offsets = [0.0, 0.5, 0.5, 0.0]

    def get_reference(
        self,
        phase: float,
        cmd_vx: float,
        cmd_vy: float,
        cmd_wz: float,
    ) -> List[float]:
        """phase + cmd_vel → 12 joint angle. cmd_norm 작으면 default 와 blend."""
        v_left = cmd_vx - (cmd_wz * self.robot_width / 2.0)
        v_right = cmd_vx + (cmd_wz * self.robot_width / 2.0)

        stance_time = self.gait_period * self.duty_factor
        stride_l = v_left * stance_time
        stride_r = v_right * stance_time

        # FL, FR, RL, RR
        strides = [stride_l, stride_r, stride_l, stride_r]

        ref = [0.0] * 12

        for leg_idx in range(4):
            local_phase = (phase + self.phase_offsets[leg_idx]) % 1.0
            stride = strides[leg_idx]

            z = -self.body_height

            if local_phase < self.duty_factor:
                # stance
                t_stance = local_phase / self.duty_factor
                x = stride * (0.5 - t_stance)
            else:
                # swing
                t_swing = (local_phase - self.duty_factor) / (1.0 - self.duty_factor)
                x = stride * (-0.5 + t_swing)
                z = -self.body_height + self.step_height * math.sin(math.pi * t_swing)

            theta_leg, theta_foot = self._solve_leg_ik(x, z)

            base = leg_idx * 3
            ref[base + 0] = 0.0          # shoulder
            ref[base + 1] = theta_leg
            ref[base + 2] = theta_foot

        # 정지 / 저속에서는 default pose 와 blend
        cmd_norm = math.sqrt(cmd_vx * cmd_vx + cmd_vy * cmd_vy + cmd_wz * cmd_wz)
        blend = max(0.0, min(1.0, cmd_norm / self.blend_cmd_norm))

        ref = [
            blend * ref[i] + (1.0 - blend) * self.default_joint_angles[i]
            for i in range(12)
        ]

        return ref

    def _solve_leg_ik(self, x: float, z: float):
        """2-link IK with L1 X-offset (alpha correction). x, z 는 hip frame."""
        d = math.sqrt(x * x + z * z)

        cos_q2 = (d * d - self.l1_eff * self.l1_eff - self.l2 * self.l2) / (
            2.0 * self.l1_eff * self.l2
        )
        cos_q2 = max(-0.999, min(0.999, cos_q2))

        q2 = math.acos(cos_q2)
        beta = math.atan2(x, -z)
        alpha_k = math.atan2(
            self.l2 * math.sin(q2),
            self.l1_eff + self.l2 * math.cos(q2),
        )
        q1 = beta - alpha_k

        theta_leg = q1 - self.alpha
        theta_foot = q2 + self.alpha

        return theta_leg, theta_foot
