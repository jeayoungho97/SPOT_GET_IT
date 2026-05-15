import math


class GaitPhaseGenerator:
    """
    cmd_vel norm 에 비례해서 gait phase 를 0~1 사이로 누적 진행.

      cmd_norm = sqrt(vx^2 + vy^2 + wz^2)
      phase_scale = clamp(cmd_norm / phase_cmd_norm, 0.0, 1.0)
      phase = (phase + dt / gait_period * phase_scale) % 1.0

    cmd_vel = 0 이면 phase 진행 멈춤 → IK reference 가 default 자세 유지.
    rl_locomotion 의 GaitPhaseGenerator 와 동일한 식.
    """

    def __init__(self, gait_period: float, phase_cmd_norm: float = 0.1):
        if gait_period <= 0.0:
            raise ValueError("gait_period must be positive")
        if phase_cmd_norm <= 0.0:
            raise ValueError("phase_cmd_norm must be positive")

        self.gait_period = gait_period
        self.phase_cmd_norm = phase_cmd_norm
        self.phase = 0.0

    def reset(self):
        self.phase = 0.0

    def update(self, dt: float, cmd_vx: float, cmd_vy: float, cmd_wz: float) -> float:
        cmd_norm = math.sqrt(cmd_vx * cmd_vx + cmd_vy * cmd_vy + cmd_wz * cmd_wz)
        phase_scale = max(0.0, min(1.0, cmd_norm / self.phase_cmd_norm))
        self.phase = (self.phase + (dt / self.gait_period) * phase_scale) % 1.0
        return self.phase

    def sin_cos(self):
        return (
            math.sin(2.0 * math.pi * self.phase),
            math.cos(2.0 * math.pi * self.phase),
        )
