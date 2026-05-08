import math
from typing import List, Optional, Tuple

class TrotIkReference:
    """
    exp043 학습 코드의 SpotmicroTest._get_ik_target()을
    ROS2 단일 로봇용 scalar Python 코드로 포팅한 버전.

    학습 코드 기준:
      - v_left  = vx - wz * robot_width / 2
      - v_right = vx + wz * robot_width / 2
      - stride_l/r = v_left/right * gait_period * duty_factor
      - leg order = FL, FR, RL, RR
      - phase offsets = [0.0, 0.5, 0.5, 0.0]
      - ref_dof_pos[:, 1::3] = theta_leg
      - ref_dof_pos[:, 2::3] = theta_foot
      - shoulder ref = 0
      - 정지/저속에서는 default pose와 IK pose를 blend
    """

    def __init__(
        self,
        default_joint_angles: List[float],
        gait_period: float = 0.6,
        duty_factor: float = 0.5,
        step_height: float = 0.03,
        body_height: float = 0.206,
        robot_width: float = 0.15,
        blend_cmd_norm: float = 0.1,
    ):
        if len(default_joint_angles) != 12:
            raise ValueError("default_joint_angles must have 12 elements")

        self.default_joint_angles = list(default_joint_angles)

        # 학습 코드 _init_buffers()와 동일한 값
        self.L1_X = 0.01
        self.L1_Z = 0.12
        self.L2 = 0.115
        self.L1_EFF = math.sqrt(self.L1_X ** 2 + self.L1_Z ** 2)
        self.ALPHA = math.atan2(self.L1_X, self.L1_Z)

        self.robot_width = robot_width
        self.gait_period = gait_period
        self.duty_factor = duty_factor
        self.step_height = step_height
        self.body_height = body_height
        self.blend_cmd_norm = blend_cmd_norm

        # FL, FR, RL, RR
        self.phase_offsets = [0.0, 0.5, 0.5, 0.0]

    def get_reference(self, phase: float, cmd_vx: float, cmd_vy: float, cmd_wz: float) -> List[float]:
        """
        학습 코드의 _get_ik_target()과 같은 의미의 ref_dof_pos[12] 반환.

        cmd_vy는 현재 exp043 구조에서 observation command에는 들어가지만,
        IK target 계산에는 직접 사용하지 않는다.
        다만 blend의 cmd_norm에는 학습 코드처럼 commands[:, :3] norm이 들어가므로 포함한다.
        """

        vx = cmd_vx
        wz = cmd_wz

        v_left = vx - (wz * self.robot_width / 2.0)
        v_right = vx + (wz * self.robot_width / 2.0)

        stance_time = self.gait_period * self.duty_factor

        stride_l = v_left * stance_time
        stride_r = v_right * stance_time

        # 학습 코드: strides = [stride_l, stride_r, stride_l, stride_r]
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
            ref[base + 0] = 0.0
            ref[base + 1] = theta_leg
            ref[base + 2] = theta_foot

        # 학습 코드:
        # cmd_norm = torch.norm(self.commands[:, :3], dim=1, keepdim=True)
        # blend = torch.clamp(cmd_norm / 0.1, 0.0, 1.0)
        cmd_norm = math.sqrt(cmd_vx * cmd_vx + cmd_vy * cmd_vy + cmd_wz * cmd_wz)
        blend = max(0.0, min(1.0, cmd_norm / self.blend_cmd_norm))

        ref = [
            blend * ref[i] + (1.0 - blend) * self.default_joint_angles[i]
            for i in range(12)
        ]

        return ref

    def _solve_leg_ik(self, x: float, z: float):
        d = math.sqrt(x * x + z * z)

        cos_q2 = (d * d - self.L1_EFF * self.L1_EFF - self.L2 * self.L2) / (
            2.0 * self.L1_EFF * self.L2
        )

        # 학습 코드와 동일하게 -0.999 ~ 0.999로 clamp
        cos_q2 = max(-0.999, min(0.999, cos_q2))

        q2 = math.acos(cos_q2)
        beta = math.atan2(x, -z)
        alpha_k = math.atan2(
            self.L2 * math.sin(q2),
            self.L1_EFF + self.L2 * math.cos(q2),
        )
        q1 = beta - alpha_k

        theta_leg = q1 - self.ALPHA
        theta_foot = q2 + self.ALPHA

        return theta_leg, theta_foot
