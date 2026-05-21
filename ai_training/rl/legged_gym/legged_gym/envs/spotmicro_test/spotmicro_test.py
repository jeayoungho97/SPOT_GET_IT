from legged_gym.envs.base.legged_robot import LeggedRobot
from isaacgym.torch_utils import torch_rand_float, quat_from_euler_xyz
from isaacgym import gymtorch
from pathlib import Path
import math
import sys
import torch


def _ensure_locomotion_common_on_path():
    try:
        from locomotion_common import TorchSharedTrotReference  # noqa: F401
        return
    except ImportError:
        pass

    for parent in Path(__file__).resolve().parents:
        candidate = parent / "robot_ws" / "src" / "control" / "locomotion_common"
        if candidate.exists():
            sys.path.insert(0, str(candidate))
            return


_ensure_locomotion_common_on_path()
from locomotion_common import TorchSharedTrotReference


class SpotmicroTest(LeggedRobot):
    def _init_buffers(self):
        super()._init_buffers()
        rigid_body_state = self.gym.acquire_rigid_body_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.rigid_body_states = gymtorch.wrap_tensor(rigid_body_state).view(
            self.num_envs, self.num_bodies, 13)
        self._init_ik_reference()

        self.gait_phase = torch.zeros(self.num_envs, 1, dtype=torch.float, device=self.device)
        self.commands_scale = torch.tensor(
            [self.obs_scales.lin_vel, self.obs_scales.lin_vel, self.obs_scales.ang_vel],
            device=self.device)
        # ==== Step 5: 서보 응답 지연 (substep 단위, dt=5ms 해상도) ====
        if self.cfg.domain_rand.action_delay:
            delay_range = self.cfg.domain_rand.action_delay_range
            decimation = self.cfg.control.decimation
            assert delay_range[1] < decimation, (
                f"action_delay_range max({delay_range[1]})은 "
                f"decimation({decimation})보다 작아야 합니다. "
                f"그래야 한 policy step 안에서 처리됩니다.")
            
            # 환경별 지연 substep 수 (정수)
            self.action_delay_substeps = torch.randint(
                delay_range[0], delay_range[1] + 1,
                (self.num_envs,), device=self.device)
            
            dt_ms = self.sim_params.dt * 1000
            print(f"[Action Delay] 활성화: "
                  f"substep 단위, dt={dt_ms:.1f}ms, "
                  f"range={delay_range[0]*dt_ms:.0f}~{delay_range[1]*dt_ms:.0f}ms")
                  
        # ==== Recovery assist randomization ====
        # Stage 1: 너무 세게 시작하지 말 것
        self.recovery_roll_pitch_range = math.radians(
            getattr(self.cfg.domain_rand, "recovery_roll_pitch_range_deg", 10.0)
        )
        self.recovery_yaw_range = math.pi
        self.recovery_lin_vel_xy_range = getattr(self.cfg.domain_rand, "recovery_lin_vel_xy_range", 0.10)
        self.recovery_lin_vel_z_range = getattr(self.cfg.domain_rand, "recovery_lin_vel_z_range", 0.03)
        self.recovery_ang_vel_xy_range = getattr(self.cfg.domain_rand, "recovery_ang_vel_xy_range", 0.60)
        self.recovery_ang_vel_z_range = getattr(self.cfg.domain_rand, "recovery_ang_vel_z_range", 0.30)
        
        # ==== Recovery diagnostic용 reset 상태 기록 ====
        self.last_reset_roll = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float
        )
        self.last_reset_pitch = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float
        )
        self.last_reset_tilt = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float
        )
        self.last_tilt_metric = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float
        )
        self.last_ang_vel_xy_metric = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float
        )

    def _init_ik_reference(self):
        ik_cfg = self.cfg.ik
        self.gait_period = float(ik_cfg.gait_period)
        self.duty_factor = float(ik_cfg.duty_factor)
        self.phase_cmd_norm = float(ik_cfg.phase_cmd_norm)
        self.blend_cmd_norm = float(ik_cfg.blend_cmd_norm)
        self.phase_offsets = torch.tensor(
            ik_cfg.phase_offsets,
            device=self.device,
            dtype=torch.float,
        )
        self.ik_reference = TorchSharedTrotReference(
            device=self.device,
            dtype=torch.float,
            gait_period=ik_cfg.gait_period,
            duty_factor=ik_cfg.duty_factor,
            body_height=ik_cfg.body_height,
            step_height=ik_cfg.step_height,
            default_foot_x=ik_cfg.default_foot_x,
            default_foot_y=ik_cfg.default_foot_y,
            leg_origin_x=ik_cfg.leg_origin_x,
            leg_origin_y=ik_cfg.leg_origin_y,
            shoulder_sign=ik_cfg.shoulder_sign,
            phase_offsets=ik_cfg.phase_offsets,
            max_stride_x=ik_cfg.max_stride_x,
            max_stride_y=ik_cfg.max_stride_y,
            upper_link_x=ik_cfg.upper_link_x,
            upper_link_z=ik_cfg.upper_link_z,
            lower_link=ik_cfg.lower_link,
            shoulder_y_gain=ik_cfg.shoulder_y_gain,
            shoulder_limit=ik_cfg.shoulder_limit,
            joint_min=ik_cfg.joint_min,
            joint_max=ik_cfg.joint_max,
        )


    def step(self, actions):
        """서보 응답 지연을 substep 단위로 적용
        
        decimation 루프의 각 substep에서:
          - i < delay인 환경 → 이전 액션(last_actions) 사용
          - i >= delay인 환경 → 현재 액션 사용
        
        이렇게 하면 5ms 해상도로 지연을 시뮬레이션할 수 있음.
        self.actions는 현재 액션 그대로 유지 (observation, reward용).
        """
        clip_actions = self.cfg.normalization.clip_actions
        self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device)
        
        self.render()
        for i in range(self.cfg.control.decimation):
            if self.cfg.domain_rand.action_delay:
                # i < delay인 환경: 서보에 아직 새 명령 안 도착 → 이전 액션
                # i >= delay인 환경: 새 명령 도착 → 현재 액션
                use_old = (i < self.action_delay_substeps).unsqueeze(1)  # [num_envs, 1]
                torque_actions = torch.where(use_old, self.last_actions, self.actions)
            else:
                torque_actions = self.actions

            self.torques = self._compute_torques(torque_actions).view(self.torques.shape)
            self.gym.set_dof_actuation_force_tensor(
                self.sim, gymtorch.unwrap_tensor(self.torques))
            self.gym.simulate(self.sim)
            if self.device == 'cpu':
                self.gym.fetch_results(self.sim, True)
            self.gym.refresh_dof_state_tensor(self.sim)
        self.post_physics_step()

        clip_obs = self.cfg.normalization.clip_observations
        self.obs_buf = torch.clip(self.obs_buf, -clip_obs, clip_obs)
        if self.privileged_obs_buf is not None:
            self.privileged_obs_buf = torch.clip(self.privileged_obs_buf, -clip_obs, clip_obs)
        return self.obs_buf, self.privileged_obs_buf, self.rew_buf, self.reset_buf, self.extras

    def post_physics_step(self):
        self.gym.refresh_rigid_body_state_tensor(self.sim)

        # 속도 명령 크기에 비례하여 gait phase 진행
        cmd_norm = torch.norm(self.commands[:, :3], dim=1, keepdim=True)  # [num_envs, 1]
        phase_scale = torch.clamp(cmd_norm / self.phase_cmd_norm, 0.0, 1.0)  # phase_cmd_norm 이하면 감속→정지

        dt_phase = self.dt / self.gait_period
        self.gait_phase = (self.gait_phase + dt_phase * phase_scale) % 1.0
        super().post_physics_step()
        self.last_tilt_metric = torch.norm(self.projected_gravity[:, :2], dim=1)
        self.last_ang_vel_xy_metric = torch.norm(self.base_ang_vel[:, :2], dim=1)
              
    def check_termination(self):
        super().check_termination()

        base_height = self.root_states[:, 2]
        min_base_height = getattr(self.cfg.rewards, "min_base_height", 0.155)
        max_base_tilt_deg = getattr(self.cfg.rewards, "max_base_tilt_deg", 75.0)
        max_tilt_gravity_z = -math.cos(math.radians(max_base_tilt_deg))
        self.reset_buf |= (base_height < min_base_height)
        self.reset_buf |= (self.projected_gravity[:, 2] > max_tilt_gravity_z)
        
    def _reset_dofs(self, env_ids):
        # recovery 학습 첫 단계에서는 관절은 기본 자세 근처에서 시작
        joint_noise = torch_rand_float(
            -0.05, 0.05,
            (len(env_ids), self.num_dof),
            device=self.device,
        )
        self.dof_pos[env_ids] = self.default_dof_pos + joint_noise
        self.dof_vel[env_ids] = 0.

        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(
            self.sim,
            gymtorch.unwrap_tensor(self.dof_state),
            gymtorch.unwrap_tensor(env_ids_int32),
            len(env_ids_int32),
        )

        if hasattr(self, 'pair_air_time'):
            self.pair_air_time[env_ids] = 0.
        if hasattr(self, 'max_feet_height'):
            self.max_feet_height[env_ids] = 0.
        if hasattr(self, 'feet_swing_contact_time'):
            self.feet_swing_contact_time[env_ids] = 0.

        # recovery에서는 phase mismatch도 학습해야 하므로 reset마다 랜덤화
        self.gait_phase[env_ids] = torch_rand_float(
            0.0, 1.0,
            (len(env_ids), 1),
            device=self.device,
        )

        # Action Delay: 리셋 환경의 지연 재랜덤화
        if self.cfg.domain_rand.action_delay:
            delay_range = self.cfg.domain_rand.action_delay_range
            self.action_delay_substeps[env_ids] = torch.randint(
                delay_range[0],
                delay_range[1] + 1,
                (len(env_ids),),
                device=self.device,
            )

    
    def _reset_root_states(self, env_ids):
        """Recovery assist용 reset:
        - base를 살짝 기울어진 상태로 시작
        - roll/pitch angular velocity 부여
        - 아직 완전히 넘어진 상태는 만들지 않음
        """
        num = len(env_ids)

        self.root_states[env_ids] = self.base_init_state
        self.root_states[env_ids, :3] += self.env_origins[env_ids]

        # plane이면 필요 없지만, trimesh 확장 고려해서 z는 초기 높이 유지
        self.root_states[env_ids, 2] = self.base_init_state[2] + self.env_origins[env_ids, 2]

        # roll/pitch/yaw randomization
        roll = torch_rand_float(
            -self.recovery_roll_pitch_range,
            self.recovery_roll_pitch_range,
            (num, 1),
            device=self.device,
        ).squeeze(1)

        pitch = torch_rand_float(
            -self.recovery_roll_pitch_range,
            self.recovery_roll_pitch_range,
            (num, 1),
            device=self.device,
        ).squeeze(1)

        yaw = torch_rand_float(
            -self.recovery_yaw_range,
            self.recovery_yaw_range,
            (num, 1),
            device=self.device,
        ).squeeze(1)

        self.root_states[env_ids, 3:7] = quat_from_euler_xyz(roll, pitch, yaw)
        
        # diagnostic에서 reset 직후의 실제 perturbation을 initial tilt로 쓰기 위해 저장
        self.last_reset_roll[env_ids] = roll
        self.last_reset_pitch[env_ids] = pitch
        self.last_reset_tilt[env_ids] = torch.maximum(torch.abs(roll), torch.abs(pitch))

        # base linear velocity
        self.root_states[env_ids, 7:9] = torch_rand_float(
            -self.recovery_lin_vel_xy_range,
            self.recovery_lin_vel_xy_range,
            (num, 2),
            device=self.device,
        )
        self.root_states[env_ids, 9] = torch_rand_float(
            -self.recovery_lin_vel_z_range,
            self.recovery_lin_vel_z_range,
            (num, 1),
            device=self.device,
        ).squeeze(1)

        # base angular velocity: roll/pitch 방향을 중점적으로 교란
        self.root_states[env_ids, 10:12] = torch_rand_float(
            -self.recovery_ang_vel_xy_range,
            self.recovery_ang_vel_xy_range,
            (num, 2),
            device=self.device,
        )
        self.root_states[env_ids, 12] = torch_rand_float(
            -self.recovery_ang_vel_z_range,
            self.recovery_ang_vel_z_range,
            (num, 1),
            device=self.device,
        ).squeeze(1)

        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(
            self.sim,
            gymtorch.unwrap_tensor(self.root_states),
            gymtorch.unwrap_tensor(env_ids_int32),
            len(env_ids_int32),
        )

    def _push_robots(self):
        """주행 중 전환 복구 상황을 만들기 위해 선속도와 roll/pitch 각속도 impulse를 더한다."""
        max_lin = self.cfg.domain_rand.max_push_vel_xy
        lin_clip = getattr(self.cfg.domain_rand, "push_lin_vel_clip", max_lin)
        lin_delta = torch_rand_float(
            -max_lin, max_lin, (self.num_envs, 2), device=self.device)
        self.root_states[:, 7:9] = torch.clamp(
            self.root_states[:, 7:9] + lin_delta,
            min=-lin_clip,
            max=lin_clip,
        )

        max_ang_xy = getattr(self.cfg.domain_rand, "max_push_ang_vel_xy", 0.0)
        if max_ang_xy > 0.0:
            ang_xy_clip = getattr(self.cfg.domain_rand, "push_ang_vel_xy_clip", max_ang_xy)
            ang_xy_delta = torch_rand_float(
                -max_ang_xy, max_ang_xy, (self.num_envs, 2), device=self.device)
            self.root_states[:, 10:12] = torch.clamp(
                self.root_states[:, 10:12] + ang_xy_delta,
                min=-ang_xy_clip,
                max=ang_xy_clip,
            )

        max_ang_z = getattr(self.cfg.domain_rand, "max_push_ang_vel_z", 0.0)
        if max_ang_z > 0.0:
            ang_z_clip = getattr(self.cfg.domain_rand, "push_ang_vel_z_clip", max_ang_z)
            ang_z_delta = torch_rand_float(
                -max_ang_z, max_ang_z, (self.num_envs, 1), device=self.device).squeeze(1)
            self.root_states[:, 12] = torch.clamp(
                self.root_states[:, 12] + ang_z_delta,
                min=-ang_z_clip,
                max=ang_z_clip,
            )

        self.last_transition_push_step = int(self.common_step_counter)
        self.last_transition_push_lin = float(max_lin)
        self.last_transition_push_ang_xy = float(max_ang_xy)
        self.last_transition_push_ang_z = float(max_ang_z)

        self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(self.root_states))
        if not hasattr(self, '_push_count'):
            self._push_count = 0
        self._push_count += 1
        if self._push_count <= 3:
            print(
                f"[DR] Push #{self._push_count} at step {self.common_step_counter}, "
                f"lin={max_lin}, ang_xy={max_ang_xy}, ang_z={max_ang_z}")

    def _resample_commands(self, env_ids):
        if len(env_ids) == 0:
            return

        self.commands[env_ids, 0] = torch_rand_float(
            self.command_ranges["lin_vel_x"][0],
            self.command_ranges["lin_vel_x"][1],
            (len(env_ids), 1),
            device=self.device,
        ).squeeze(1)
        self.commands[env_ids, 1] = torch_rand_float(
            self.command_ranges["lin_vel_y"][0],
            self.command_ranges["lin_vel_y"][1],
            (len(env_ids), 1),
            device=self.device,
        ).squeeze(1)
        if self.cfg.commands.heading_command:
            self.commands[env_ids, 3] = torch_rand_float(
                self.command_ranges["heading"][0],
                self.command_ranges["heading"][1],
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)
        else:
            self.commands[env_ids, 2] = torch_rand_float(
                self.command_ranges["ang_vel_yaw"][0],
                self.command_ranges["ang_vel_yaw"][1],
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)

        deadband = getattr(self.cfg.commands, "command_deadband", 0.02)
        if deadband <= 0.0:
            return

        lin_norm = torch.norm(self.commands[env_ids, :2], dim=1)
        self.commands[env_ids, :2] *= (lin_norm > deadband).unsqueeze(1)
        
    def compute_observations(self):
        ref_dof_pos = self._get_ik_target()
        phase_sin = torch.sin(2 * torch.pi * self.gait_phase)
        phase_cos = torch.cos(2 * torch.pi * self.gait_phase)
        self.obs_buf = torch.cat([
            self.base_ang_vel * self.obs_scales.ang_vel,           # 3
            self.projected_gravity,                                 # 3
            self.commands[:, :3] * self.commands_scale,            # 3
            (self.dof_pos - ref_dof_pos) * self.obs_scales.dof_pos,  # 12
            self.dof_vel * self.obs_scales.dof_vel,                # 12
            self.actions,                                           # 12 (현재 액션)
            phase_sin, phase_cos,                                   # 2
        ], dim=-1)
        if self.add_noise:
            self.obs_buf += (2 * torch.rand_like(self.obs_buf) - 1) * self.noise_scale_vec
            
    def _get_noise_scale_vec(self, cfg):
        noise_vec = torch.zeros_like(self.obs_buf[0])
        self.add_noise = self.cfg.noise.add_noise
        noise_scales = self.cfg.noise.noise_scales
        noise_level = self.cfg.noise.noise_level

        # obs layout:
        # 0:3   base_ang_vel
        # 3:6   projected_gravity
        # 6:9   commands
        # 9:21  dof_pos - ref_dof_pos
        # 21:33 dof_vel
        # 33:45 actions
        # 45:47 phase_sin, phase_cos

        noise_vec[:3] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
        noise_vec[3:6] = noise_scales.gravity * noise_level
        noise_vec[6:9] = 0.0
        noise_vec[9:21] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
        noise_vec[21:33] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
        noise_vec[33:45] = 0.0
        noise_vec[45:47] = 0.0
        if self.cfg.terrain.measure_heights:
            noise_vec[48:235] = noise_scales.height_measurements* noise_level * self.obs_scales.height_measurements

        return noise_vec
        
    def _reward_feet_air_time(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        self.last_contacts = contact
        first_contact = (self.feet_air_time > 0.) * contact_filt
        self.feet_air_time += self.dt
        rew_airTime = torch.sum((self.feet_air_time - 0.15) * first_contact, dim=1)
        rew_airTime *= torch.norm(self.commands[:, :3], dim=1) > self.blend_cmd_norm
        self.feet_air_time *= ~contact_filt
        return rew_airTime
        
    def _reward_trot_symmetry(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        sync_0 = (contact[:, 0] == contact[:, 3]).float()
        sync_1 = (contact[:, 1] == contact[:, 2]).float()  
        anti_phase = (contact[:, 0] != contact[:, 1]).float()
        reward = (sync_0 + sync_1 + anti_phase) / 3.0
        reward *= (torch.norm(self.commands[:, :3], dim=1) > self.blend_cmd_norm).float()
        return reward
        
    def _reward_no_stuck_feet(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        phases = (self.gait_phase + self.phase_offsets) % 1.0
        desired_air = phases >= self.duty_factor
        swing_contact = desired_air & contact_filt
        if not hasattr(self, 'feet_swing_contact_time'):
            self.feet_swing_contact_time = torch.zeros(
                self.num_envs, len(self.feet_indices), device=self.device)
        self.feet_swing_contact_time = (
            self.feet_swing_contact_time + self.dt
        ) * swing_contact.float()
        grace_time = getattr(self.cfg.rewards, "swing_contact_grace_time", 0.03)
        penalty = torch.sum(
            torch.clamp(self.feet_swing_contact_time - grace_time, min=0.),
            dim=1,
        )
        penalty *= (torch.norm(self.commands[:, :3], dim=1) > self.blend_cmd_norm).float()
        return penalty

    def _reward_symmetric_gait(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        if not hasattr(self, 'pair_air_ema'):
            self.pair_air_ema = torch.zeros(self.num_envs, 2, device=self.device)
        pair0_air = (~contact_filt[:, 0] & ~contact_filt[:, 3]).float()
        pair1_air = (~contact_filt[:, 1] & ~contact_filt[:, 2]).float()
        alpha = 0.02
        self.pair_air_ema[:, 0] = (1 - alpha) * self.pair_air_ema[:, 0] + alpha * pair0_air
        self.pair_air_ema[:, 1] = (1 - alpha) * self.pair_air_ema[:, 1] + alpha * pair1_air
        diff = torch.abs(self.pair_air_ema[:, 0] - self.pair_air_ema[:, 1])
        total = self.pair_air_ema[:, 0] + self.pair_air_ema[:, 1]
        balance = torch.where(
            total > 0.02,
            1.0 - diff / (total + 1e-6),
            torch.zeros_like(diff)
        )
        reward = balance
        reward *= (torch.norm(self.commands[:, :3], dim=1) > self.blend_cmd_norm).float()
        return reward

    def _reward_feet_clearance(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        if not hasattr(self, 'max_feet_height'):
            self.max_feet_height = torch.zeros(
                self.num_envs, len(self.feet_indices), device=self.device)
        feet_z = self.rigid_body_states[:, self.feet_indices, 2]
        is_air = (~contact_filt).float()
        self.max_feet_height = torch.max(self.max_feet_height, feet_z * is_air)
        first_contact = (self.max_feet_height > 0.) * contact_filt
        clearance_min = getattr(self.cfg.rewards, "feet_clearance_min", 0.02)
        clearance_cap = getattr(self.cfg.rewards, "feet_clearance_cap", 0.03)
        height_reward = torch.clamp(
            self.max_feet_height - clearance_min,
            min=0.,
            max=clearance_cap,
        )
        reward = torch.sum(height_reward * first_contact.float(), dim=1)
        self.max_feet_height *= is_air
        reward *= (torch.norm(self.commands[:, :3], dim=1) > self.blend_cmd_norm).float()
        return reward

    def _reward_swing_contact(self):
        phases = (self.gait_phase + self.phase_offsets) % 1.0
        desired_air = phases >= self.duty_factor
        actual_contact = self.contact_forces[:, self.feet_indices, 2] > 1.0
        dragging = (desired_air & actual_contact).float()
        cmd_norm = torch.norm(self.commands[:, :3], dim=1, keepdim=True)
        is_moving = (cmd_norm > self.blend_cmd_norm).float()
        return torch.sum(dragging * is_moving, dim=1) / 4.0
        
    
    def _reward_stand_still(self):
        cmd_norm = torch.norm(self.commands[:, :3], dim=1)
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1) * (cmd_norm < self.blend_cmd_norm)
    '''
    def _reward_stand_still(self):
        cmd_norm = torch.norm(self.commands[:, :3], dim=1)
        is_stand = (cmd_norm < 0.1).float()

        lin_penalty = torch.sum(torch.square(self.base_lin_vel[:, :2]), dim=1)
        yaw_penalty = torch.square(self.base_ang_vel[:, 2])
        pose_penalty = 0.2 * torch.sum(
            torch.square(self.dof_pos - self.default_dof_pos), dim=1
        )

        return (lin_penalty + 0.5 * yaw_penalty + pose_penalty) * is_stand
    '''
    def _get_ik_target(self):
        ref_dof_pos = self.ik_reference.get_reference(
            self.gait_phase,
            self.commands[:, :3],
        )

        # 정지 시 default pose로 블렌딩
        cmd_norm = torch.norm(self.commands[:, :3], dim=1, keepdim=True)
        blend = torch.clamp(cmd_norm / self.blend_cmd_norm, 0.0, 1.0)

        ref_dof_pos = blend * ref_dof_pos + (1.0 - blend) * self.default_dof_pos

        return ref_dof_pos
        
    def _compute_torques(self, actions):
        actions_scaled = actions * self.cfg.control.action_scale
        ref_dof_pos = self._get_ik_target()
        torques = self.p_gains * (actions_scaled + ref_dof_pos - self.dof_pos) - self.d_gains * self.dof_vel
        return torch.clip(torques, -self.torque_limits, self.torque_limits)
        
    def _reward_tracking_ik(self):
        # 관절별 페널티 가중치: [Shoulder, Leg, Foot] 순서
        # 어깨(0.1)는 자유롭게 움직이도록 허용하고, Leg와 Foot(1.0)은 IK를 잘 따르도록 강제함
        weights = torch.tensor([1.0, 1.0, 1.0] * 4, device=self.device)
    
        # action에 가중치를 곱해서 에러 계산
        weighted_actions = self.actions * weights
        error = torch.sum(torch.square(weighted_actions), dim=1)
        sigma = 2.0
        return torch.exp(-error / sigma)
        
    def _reward_tracking_ang_vel(self):
        ang_vel_error = torch.square(
            self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(
            -ang_vel_error / self.cfg.rewards.tracking_sigma_ang_vel)

    def _reward_trot_contact(self):
        phases = (self.gait_phase + self.phase_offsets) % 1.0
        desired_contact = phases < self.duty_factor
        actual_contact = self.contact_forces[:, self.feet_indices, 2] > 1.0
        match = (actual_contact == desired_contact).float()
        cmd_norm = torch.norm(self.commands[:, :3], dim=1, keepdim=True)
        is_moving = (cmd_norm > self.blend_cmd_norm).float()
        return torch.sum(match * is_moving, dim=1) / 4.0

    def _recovery_tilt_mask(self):
        threshold_deg = getattr(self.cfg.rewards, "recovery_reward_tilt_threshold_deg", 4.0)
        threshold = math.sin(math.radians(threshold_deg))
        tilt = torch.norm(self.projected_gravity[:, :2], dim=1)
        return tilt, (tilt > threshold).float()

    def _reward_tilt_recovery(self):
        tilt, mask = self._recovery_tilt_mask()
        improvement = torch.clamp(self.last_tilt_metric - tilt, min=0.0, max=0.05)
        return improvement * mask

    def _reward_ang_vel_xy_recovery(self):
        _, mask = self._recovery_tilt_mask()
        ang_vel_xy = torch.norm(self.base_ang_vel[:, :2], dim=1)
        damping = torch.clamp(self.last_ang_vel_xy_metric - ang_vel_xy, min=0.0, max=0.5)
        return damping * mask
