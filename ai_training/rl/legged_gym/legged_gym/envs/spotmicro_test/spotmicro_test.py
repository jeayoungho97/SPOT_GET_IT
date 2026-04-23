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
        self.gait_freq = 2.0  # Trot 주파수 (Hz)
        
        # ====IK 변수====
        # IK Constants (미리 계산된 상수 캐싱)
        self.L1_X = 0.01
        self.L1_Z = 0.12
        self.L2 = 0.115
        
        # math 라이브러리 없이 순수 파이썬/토치로 한 번만 계산하여 상수화
        self.L1_EFF = (0.01**2 + 0.12**2)**0.5       # 0.1204159
        self.ALPHA = torch.atan2(torch.tensor(0.01), torch.tensor(0.12)).item() # 0.0831412 rad
        self.robot_width = 0.15 
        
        self.gait_period = 0.6
        self.duty_factor = 0.5
        self.step_height = 0.03
        self.body_height = 0.206
        
        # Phase Tracker (0.0 ~ 1.0)
        self.gait_phase = torch.zeros(self.num_envs, 1, dtype=torch.float, device=self.device)
        self.commands_scale = torch.tensor(
            [self.obs_scales.lin_vel, self.obs_scales.lin_vel, self.obs_scales.ang_vel],
            device=self.device)
        

    def post_physics_step(self):
        # rigid body state 매 step refresh
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        #====IK====
        dt_phase = self.dt / self.gait_period
        self.gait_phase = (self.gait_phase + dt_phase) % 1.0
        super().post_physics_step()
              
    def _reset_dofs(self, env_ids):
        """관절을 정확히 default 위치로 리셋 (랜덤화 없음)
        Phase 5(DR)에서 부모 클래스 버전으로 되돌리거나,
        randomize_dof_pos 플래그로 제어할 수 있음
        """
        self.dof_pos[env_ids] = self.default_dof_pos
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
    def _reset_root_states(self, env_ids):
        """base 속도를 0으로 리셋 (랜덤화 없음)
        Phase 5(DR)에서 부모 클래스 버전으로 되돌림
        """
        self.root_states[env_ids] = self.base_init_state
        self.root_states[env_ids, :3] += self.env_origins[env_ids]
        # 부모 클래스는 여기서 [7:13]을 [-0.5, 0.5]로 랜덤화하지만,
        # Phase 4에서는 0으로 유지
        self.root_states[env_ids, 7:13] = 0.

        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(
            self.sim,
            gymtorch.unwrap_tensor(self.root_states),
            gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))
    def check_termination(self):
        # 부모 클래스의 기본 종료 조건 먼저 실행 (timeout + contact 종료)
        super().check_termination()
        
        # 높이 기반 종료
        base_height = self.root_states[:, 2]
        self.reset_buf |= (base_height < 0.155)
        
        # 기울기 기반 종료 (뒤집힘 감지)
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
        # SpotMicro용 필터링 임계값 (0.2 → 0.05)
        self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.05).unsqueeze(1)
        
    def compute_observations(self):
        ref_dof_pos = self._get_ik_target()
    
        # phase 정보 (sin/cos으로 연속적으로 표현)
        phase_sin = torch.sin(2 * torch.pi * self.gait_phase)
        phase_cos = torch.cos(2 * torch.pi * self.gait_phase)
    
        self.obs_buf = torch.cat([
            #self.base_lin_vel * self.obs_scales.lin_vel,           # 3
            self.base_ang_vel * self.obs_scales.ang_vel,           # 3
            self.projected_gravity,                                 # 3
            self.commands[:, :3] * self.commands_scale,            # 3
            (self.dof_pos - ref_dof_pos) * self.obs_scales.dof_pos,  # 12 (IK 기준!)
            self.dof_vel * self.obs_scales.dof_vel,                # 12
            self.actions,                                           # 12
            phase_sin, phase_cos,                                   # 2 (gait phase)
        ], dim=-1)
        if self.add_noise:
            self.obs_buf += (2 * torch.rand_like(self.obs_buf) - 1) * self.noise_scale_vec
            
    def _reward_feet_air_time(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        self.last_contacts = contact
        first_contact = (self.feet_air_time > 0.) * contact_filt
        self.feet_air_time += self.dt
        # 0.5초 → 0.15초로 변경
        rew_airTime = torch.sum((self.feet_air_time - 0.15) * first_contact, dim=1)
        rew_airTime *= torch.norm(self.commands[:, :2], dim=1) > 0.1
        self.feet_air_time *= ~contact_filt
        return rew_airTime
        
    def _reward_trot_symmetry(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
    
        # 대각선 쌍 동기화
        sync_0 = (contact[:, 0] == contact[:, 3]).float()
        sync_1 = (contact[:, 1] == contact[:, 2]).float()  
        # 반위상: 두 대각선 쌍이 반대 상태
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
    
        # 속도 명령이 있을 때만 발동
        penalty *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    
        return penalty

    def _reward_symmetric_gait(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
    
        # 1. 대각선 쌍별 공중시간 추적
        if not hasattr(self, 'pair_air_ema'):
            self.pair_air_ema = torch.zeros(self.num_envs, 2, device=self.device)
    
        # FL(0)-RR(3) 쌍: 둘 다 공중이면 누적
        pair0_air = (~contact_filt[:, 0] & ~contact_filt[:, 3]).float()
        # FR(1)-RL(2) 쌍: 둘 다 공중이면 누적
        pair1_air = (~contact_filt[:, 1] & ~contact_filt[:, 2]).float()
    
        alpha = 0.02
        self.pair_air_ema[:, 0] = (1 - alpha) * self.pair_air_ema[:, 0] + alpha * pair0_air
        self.pair_air_ema[:, 1] = (1 - alpha) * self.pair_air_ema[:, 1] + alpha * pair1_air
    
        # 균형 보상: 1초간 양쪽 쌍의 공중시간이 비슷하면 보상
        diff = torch.abs(self.pair_air_ema[:, 0] - self.pair_air_ema[:, 1])
        total = self.pair_air_ema[:, 0] + self.pair_air_ema[:, 1]
    
        # 양쪽 다 들었고(total > 0) 균형적이면(diff 작으면) 높은 보상
        balance = torch.where(
            total > 0.02,
            1.0 - diff / (total + 1e-6),  # 균형이면 ~1, 편중이면 ~0
            torch.zeros_like(diff)          # 아무것도 안 들면 0
        )
    
     
        # 3. 통합
        reward = balance
        # 속도 명령 있을 때만
        reward *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    
        return reward
        

    def _reward_feet_clearance(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
    
        # swing 중 최대 높이 추적
        if not hasattr(self, 'max_feet_height'):
            self.max_feet_height = torch.zeros(
                self.num_envs, len(self.feet_indices), device=self.device)
    
        feet_z = self.rigid_body_states[:, self.feet_indices, 2]
    
        # 공중이면 최대 높이 갱신, 접지면 리셋
        is_air = (~contact_filt).float()
        self.max_feet_height = torch.max(self.max_feet_height, feet_z * is_air)
    
        # 착지 순간 감지 (이전 공중 → 현재 접지)
        first_contact = (self.max_feet_height > 0.) * contact_filt
    
        # 착지 순간에만 높이 기반 보상 (2cm 이상, 최대 5cm)
        height_reward = torch.clamp(self.max_feet_height - 0.02, min=0., max=0.03)
        reward = torch.sum(height_reward * first_contact.float(), dim=1)
    
        # 착지한 발의 최대 높이 리셋
        self.max_feet_height *= is_air
    
        # 속도 명령 있을 때만
        reward *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    
        return reward
        
    def _reward_stand_still(self):
        # lin_vel과 ang_vel 모두 작을 때만 발동
        cmd_norm = torch.norm(self.commands[:, :3], dim=1)  # x, y, yaw 전부 체크
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
        
        return ref_dof_pos
        
    def _compute_torques(self, actions):
        actions_scaled = actions * self.cfg.control.action_scale
        
        # 1. 완벽한 IK 궤적을 가져옵니다.
        ref_dof_pos = self._get_ik_target()
        
        # 2. 잔차 제어(Residual Control): 
        # 기존의 self.default_dof_pos 대신 ref_dof_pos를 베이스로 사용합니다.
        # AI(actions)는 이 베이스에서 얼마나 더 꺾을지만 결정합니다.
        torques = self.p_gains * (actions_scaled + ref_dof_pos - self.dof_pos) - self.d_gains * self.dof_vel
        
        return torch.clip(torques, -self.torque_limits, self.torque_limits)
        
    def _reward_tracking_ik(self):
        # AI가 출력한 보정값(actions)이 0에 가까울수록 높은 보상 부여
        # 즉, "어지간하면 네가 짠 완벽한 IK 궤적에 맡기고 개입하지 마"라는 뜻입니다.
        error = torch.sum(torch.square(self.actions), dim=1)
        sigma = 2.0 
        return torch.exp(-error / sigma)

    def _reward_trot_contact(self):
        # 대각선 발(FL-RR / FR-RL)의 목표 위상 오프셋
        offsets = torch.tensor([0.0, 0.5, 0.5, 0.0], device=self.device)
        phases = (self.gait_phase + offsets) % 1.0
        
        # Duty factor (0.5)를 기준으로, 0.5 미만일 때는 땅에 닿아야 함 (Stance)
        desired_contact = phases < self.duty_factor
        
        # 실제 발의 접촉 상태 (z축 힘이 1.0 이상이면 접촉)
        actual_contact = self.contact_forces[:, self.feet_indices, 2] > 1.0
        
        # 목표와 실제가 일치하면 보상 (각 발당 0.25점씩, 총 1.0점)
        match = (actual_contact == desired_contact).float()
        
        # 로봇이 앞으로 가라는 명령을 받았을 때만 박자 보상 부여
        cmd_norm = torch.norm(self.commands[:, :2], dim=1, keepdim=True)
        is_moving = (cmd_norm > 0.1).float()
        
        return torch.sum(match * is_moving, dim=1) / 4.0

    #def step(self, actions):
        # 모든 액션을 0으로 강제 → 순수 default_joint_angles만 적용
        #zero_actions = torch.zeros_like(actions)
        #return super().step(zero_actions)

