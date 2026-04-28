from legged_gym.envs.base.legged_robot import LeggedRobot
from isaacgym.torch_utils import torch_rand_float
from isaacgym import gymtorch
import torch
  
class SpotmicroTest(LeggedRobot):
    def _init_buffers(self):
        super()._init_buffers()
        rigid_body_state = self.gym.acquire_rigid_body_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.rigid_body_states = gymtorch.wrap_tensor(rigid_body_state).view(
            self.num_envs, self.num_bodies, 13)
        self.gait_freq = 2.0

        # ====IK 변수====
        self.L1_X = 0.01
        self.L1_Z = 0.12
        self.L2 = 0.115
        self.L1_EFF = (0.01**2 + 0.12**2)**0.5
        self.ALPHA = torch.atan2(torch.tensor(0.01), torch.tensor(0.12)).item()
        self.robot_width = 0.15 

        self.gait_period = 0.6
        self.duty_factor = 0.5
        self.step_height = 0.03
        self.body_height = 0.206

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
        phase_scale = torch.clamp(cmd_norm / 0.1, 0.0, 1.0)  # 0.1 이하면 감속→정지

        dt_phase = self.dt / self.gait_period
        self.gait_phase = (self.gait_phase + dt_phase * phase_scale) % 1.0
        super().post_physics_step()
              
    def _reset_dofs(self, env_ids):
        self.dof_pos[env_ids] = self.default_dof_pos * torch_rand_float(
            0.5, 1.5, (len(env_ids), self.num_dof), device=self.device)
        self.dof_vel[env_ids] = 0.

        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(
            self.sim,
            gymtorch.unwrap_tensor(self.dof_state),
            gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))
        if hasattr(self, 'pair_air_time'):
            self.pair_air_time[env_ids] = 0.
        if hasattr(self, 'max_feet_height'):
            self.max_feet_height[env_ids] = 0.
        
        # Action Delay: 리셋 환경의 지연 재랜덤화
        if self.cfg.domain_rand.action_delay:
            delay_range = self.cfg.domain_rand.action_delay_range
            self.action_delay_substeps[env_ids] = torch.randint(
                delay_range[0], delay_range[1] + 1,
                (len(env_ids),), device=self.device)

    
    def _reset_root_states(self, env_ids):
        """base 속도를 0으로 리셋"""
        self.root_states[env_ids] = self.base_init_state
        self.root_states[env_ids, :3] += self.env_origins[env_ids]
        self.root_states[env_ids, 7:13] = torch_rand_float(
            -0.3, 0.3, (len(env_ids), 6), device=self.device)

        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(
            self.sim,
            gymtorch.unwrap_tensor(self.root_states),
            gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))
    

    def check_termination(self):
        super().check_termination()
        base_height = self.root_states[:, 2]
        self.reset_buf |= (base_height < 0.155)
        self.reset_buf |= (self.projected_gravity[:, 2] > 0.0)
        
    def _resample_commands(self, env_ids):
        self.commands[env_ids, 0] = torch_rand_float(
            self.command_ranges["lin_vel_x"][0], self.command_ranges["lin_vel_x"][1],
            (len(env_ids), 1), device=self.device).squeeze(1)
        self.commands[env_ids, 1] = torch_rand_float(
            self.command_ranges["lin_vel_y"][0], self.command_ranges["lin_vel_y"][1],
            (len(env_ids), 1), device=self.device).squeeze(1)
        if self.cfg.commands.heading_command:
            self.commands[env_ids, 3] = torch_rand_float(
                self.command_ranges["heading"][0], self.command_ranges["heading"][1],
                (len(env_ids), 1), device=self.device).squeeze(1)
        else:
            self.commands[env_ids, 2] = torch_rand_float(
                self.command_ranges["ang_vel_yaw"][0], self.command_ranges["ang_vel_yaw"][1],
                (len(env_ids), 1), device=self.device).squeeze(1)
        self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.05).unsqueeze(1)
        
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
            
    def _reward_feet_air_time(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        self.last_contacts = contact
        first_contact = (self.feet_air_time > 0.) * contact_filt
        self.feet_air_time += self.dt
        rew_airTime = torch.sum((self.feet_air_time - 0.15) * first_contact, dim=1)
        rew_airTime *= torch.norm(self.commands[:, :2], dim=1) > 0.1
        self.feet_air_time *= ~contact_filt
        return rew_airTime
        
    def _reward_trot_symmetry(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        sync_0 = (contact[:, 0] == contact[:, 3]).float()
        sync_1 = (contact[:, 1] == contact[:, 2]).float()  
        anti_phase = (contact[:, 0] != contact[:, 1]).float()
        reward = (sync_0 + sync_1 + anti_phase) / 3.0
        reward *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()   
        return reward
        
    def _reward_no_stuck_feet(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        if not hasattr(self, 'feet_ground_time'):
            self.feet_ground_time = torch.zeros(
                self.num_envs, len(self.feet_indices), device=self.device)
        self.feet_ground_time = (self.feet_ground_time + self.dt) * contact_filt.float()
        penalty = torch.sum(torch.clamp(self.feet_ground_time - 0.15, min=0.), dim=1)
        penalty *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
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
        reward *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
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
        height_reward = torch.clamp(self.max_feet_height - 0.02, min=0., max=0.03)
        reward = torch.sum(height_reward * first_contact.float(), dim=1)
        self.max_feet_height *= is_air
        reward *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward
        
    def _reward_stand_still(self):
        cmd_norm = torch.norm(self.commands[:, :3], dim=1)
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1) * (cmd_norm < 0.1)

    def _get_ik_target(self):
        vx = self.commands[:, 0]
        wz = self.commands[:, 2] 
        v_left = vx - (wz * self.robot_width / 2.0)
        v_right = vx + (wz * self.robot_width / 2.0)
        stance_time = self.gait_period * self.duty_factor
        stride_l = v_left * stance_time
        stride_r = v_right * stance_time
        strides = torch.stack([stride_l, stride_r, stride_l, stride_r], dim=1)
        offsets = torch.tensor([0.0, 0.5, 0.5, 0.0], device=self.device)
        phases = (self.gait_phase + offsets) % 1.0
        x = torch.zeros((self.num_envs, 4), device=self.device)
        z = torch.full((self.num_envs, 4), -self.body_height, device=self.device)
        is_stance = phases < self.duty_factor
        is_swing = ~is_stance
        t_stance = phases / self.duty_factor
        x[is_stance] = strides[is_stance] * (0.5 - t_stance[is_stance])
        t_swing = (phases - self.duty_factor) / (1.0 - self.duty_factor)
        x[is_swing] = strides[is_swing] * (-0.5 + t_swing[is_swing])
        z[is_swing] = -self.body_height + self.step_height * torch.sin(torch.pi * t_swing[is_swing])
        d = torch.sqrt(x**2 + z**2)
        cos_q2 = (d**2 - self.L1_EFF**2 - self.L2**2) / (2 * self.L1_EFF * self.L2)
        cos_q2 = torch.clamp(cos_q2, -0.999, 0.999)
        q2 = torch.acos(cos_q2)
        beta = torch.atan2(x, -z)
        alpha_k = torch.atan2(self.L2 * torch.sin(q2), self.L1_EFF + self.L2 * torch.cos(q2))
        q1 = beta - alpha_k
        theta_leg = q1 - self.ALPHA
        theta_foot = q2 + self.ALPHA
        ref_dof_pos = torch.zeros((self.num_envs, 12), device=self.device)
        ref_dof_pos[:, 1::3] = theta_leg  
        ref_dof_pos[:, 2::3] = theta_foot 

        # 정지 시 default pose로 블렌딩
        cmd_norm = torch.norm(self.commands[:, :3], dim=1, keepdim=True)  # [num_envs, 1]
        blend = torch.clamp(cmd_norm / 0.1, 0.0, 1.0)  # 0=정지→default, 1=이동→IK
        ref_dof_pos = blend * ref_dof_pos + (1.0 - blend) * self.default_dof_pos
    
        return ref_dof_pos
        
    def _compute_torques(self, actions):
        actions_scaled = actions * self.cfg.control.action_scale
        ref_dof_pos = self._get_ik_target()
        torques = self.p_gains * (actions_scaled + ref_dof_pos - self.dof_pos) - self.d_gains * self.dof_vel
        return torch.clip(torques, -self.torque_limits, self.torque_limits)
        
    def _reward_tracking_ik(self):
        error = torch.sum(torch.square(self.actions), dim=1)
        sigma = 2.0 
        return torch.exp(-error / sigma)

    def _reward_trot_contact(self):
        offsets = torch.tensor([0.0, 0.5, 0.5, 0.0], device=self.device)
        phases = (self.gait_phase + offsets) % 1.0
        desired_contact = phases < self.duty_factor
        actual_contact = self.contact_forces[:, self.feet_indices, 2] > 1.0
        match = (actual_contact == desired_contact).float()
        cmd_norm = torch.norm(self.commands[:, :2], dim=1, keepdim=True)
        is_moving = (cmd_norm > 0.1).float()
        return torch.sum(match * is_moving, dim=1) / 4.0
