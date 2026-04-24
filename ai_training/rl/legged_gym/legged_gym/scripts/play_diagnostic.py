# play_diagnostic.py
# SpotMicro RL 학습 결과 종합 진단 스크립트 v3
# 사용법: python legged_gym/scripts/play_diagnostic.py --task=spotmicro_test
#         python legged_gym/scripts/play_diagnostic.py --task=spotmicro_test --checkpoint /path/to/model_500.pt --lightweight
#
# 출력:
#   1. 터미널에 종합 진단 리포트
#   2. diagnostics/{experiment_name}/ 에 상세 그래프 저장
#
# v2 추가 항목:
#   - [6] 회전 추종 분석 (Step 2용)
#   - [7] 자세 각도 분석 - roll/pitch (Step 3용)
#   - [8] 동작 부드러움 분석 (Step 4용)
#   - [9] 보행 패턴 분석 - 발 접촉/공중 시간 (Step 5용)
#   - [10] 에너지 효율 분석 (Step 4~6용)
#   - 자세 안정성 판정 버그 수정 (높이 목표 대비 편차 반영)
#
# v3 추가 항목:
#   - --checkpoint / --lightweight 옵션 (중간 iter 경량 스냅샷)
#   - Gait FFT 주파수 / L-R 비대칭 / 대각 동기화율
#   - 관절별 사용 범위/편향 JSON 저장
#   - 관절별 전력 분배 / 에너지 상세 JSON 저장
 
from legged_gym import LEGGED_GYM_ROOT_DIR
import os
import re
import isaacgym
from legged_gym.envs import *
from legged_gym.utils import get_args, task_registry
import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict
 
 
def run_diagnostic(args, checkpoint_path=None, lightweight=False):
    # ============ 환경 설정 ============
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 64)
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = False
 
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()
 
    # --- Checkpoint 로드 (항목 1-A) ---
    train_cfg.runner.resume = True
    if checkpoint_path:
        # checkpoint 경로에서 run 디렉토리와 모델 번호 추출
        ck_dir = os.path.dirname(checkpoint_path)
        ck_basename = os.path.basename(checkpoint_path)  # e.g. model_500.pt
        train_cfg.runner.load_run = ck_dir
        # 모델 번호 추출 (model_500.pt → 500)
        m = re.search(r'model_(\d+)\.pt', ck_basename)
        if m:
            train_cfg.runner.checkpoint = int(m.group(1))

    ppo_runner, train_cfg = task_registry.make_alg_runner(
        env=env, name=args.task, args=args, train_cfg=train_cfg
    )
    policy = ppo_runner.get_inference_policy(device=env.device)
 
    # ============ 출력 폴더 (logs 바깥) ============
    diag_dir = os.path.join(
        LEGGED_GYM_ROOT_DIR, 'diagnostics',
        train_cfg.runner.experiment_name
    )
    os.makedirs(diag_dir, exist_ok=True)
 
    # ============ 관절 이름 & 한계 정보 ============
    joint_names = env.dof_names
    num_joints = env.num_dof
    num_envs = env.num_envs
    num_feet = env.feet_indices.shape[0]
 
    torque_limits = env.torque_limits.cpu().numpy()
    dof_pos_lower = env.dof_pos_limits[:, 0].cpu().numpy()
    dof_pos_upper = env.dof_pos_limits[:, 1].cpu().numpy()
 
    # URDF 원본 한계 역산
    soft_ratio = env.cfg.rewards.soft_dof_pos_limit
    urdf_lower = np.zeros_like(dof_pos_lower)
    urdf_upper = np.zeros_like(dof_pos_upper)
    for i in range(num_joints):
        mid = (dof_pos_lower[i] + dof_pos_upper[i]) / 2
        r_soft = dof_pos_upper[i] - dof_pos_lower[i]
        r_urdf = r_soft / soft_ratio if soft_ratio > 0 else r_soft
        urdf_lower[i] = mid - r_urdf / 2
        urdf_upper[i] = mid + r_urdf / 2
 
    # foot 이름 추출
    try:
        foot_names = [env.gym.get_actor_rigid_body_name(env.envs[0], env.actor_handles[0], idx)
                      for idx in env.feet_indices.cpu().numpy()]
    except:
        foot_names = [f'foot_{i}' for i in range(num_feet)]
 
    # --- 경량/풀 모드 헤더 ---
    iter_number = 0
    if checkpoint_path:
        m = re.search(r'model_(\d+)\.pt', os.path.basename(checkpoint_path))
        if m:
            iter_number = int(m.group(1))

    if not lightweight:
        print(f"\n{'='*60}")
        print(f"  SpotMicro RL 진단 v3")
        print(f"  Envs: {num_envs}, Joints: {num_joints}, Feet: {num_feet}")
        print(f"  Foot names: {foot_names}")
        if checkpoint_path:
            print(f"  Checkpoint: {os.path.basename(checkpoint_path)}")
        print(f"{'='*60}\n")
    else:
        print(f"  [경량] iter={iter_number}, checkpoint={os.path.basename(checkpoint_path) if checkpoint_path else 'latest'}")
 
    # ============ 데이터 수집 ============
    # 경량 모드: 1 에피소드 (max_episode_length steps)
    # 풀 모드: 3 에피소드 분량
    if lightweight:
        NUM_STEPS = int(env.max_episode_length)
    else:
        NUM_STEPS = int(env.max_episode_length) * 3
 
    data = defaultdict(list)
 
    # 관절별 누적 통계
    torque_saturation_count = np.zeros(num_joints)
    torque_max_seen = np.zeros(num_joints)
    dof_pos_min_seen = np.full(num_joints, 1e10)
    dof_pos_max_seen = np.full(num_joints, -1e10)
    dof_near_limit_count = np.zeros(num_joints)
 
    total_steps = 0
    episode_lengths = []
    episode_returns = []
    current_ep_return = torch.zeros(num_envs, device=env.device)
    current_ep_length = torch.zeros(num_envs, device=env.device)
 
    vel_errors_x = []
    vel_errors_y = []
    ang_vel_errors = []
 
    # 동작 부드러움 추적
    action_rates = []
    prev_actions = None
 
    # 보행 패턴 추적 (발별 공중/접촉 시간)
    feet_contact_count = np.zeros(num_feet)
    feet_air_count = np.zeros(num_feet)
 
    # roll/pitch 추적
    roll_list = []
    pitch_list = []
 
    # 에너지 추적
    power_list = []

    # --- 관절별 전력 추적 (항목 6-A) ---
    per_joint_power_sum = np.zeros(num_joints)
    peak_power = 0.0
 
    for step in range(NUM_STEPS):
        actions = policy(obs.detach())
        obs, _, rews, dones, infos = env.step(actions.detach())
        total_steps += 1
 
        # --- 토크 분석 ---
        torques_np = env.torques.cpu().numpy()
        torque_abs = np.abs(torques_np)
 
        for j in range(num_joints):
            col = torque_abs[:, j]
            torque_saturation_count[j] += np.sum(col > torque_limits[j] * 0.9)
            torque_max_seen[j] = max(torque_max_seen[j], col.max())
 
        saturated_ratio = np.mean(torque_abs > torque_limits[None, :] * 0.9)
        data['torque_saturation_ratio'].append(saturated_ratio)
 
        # --- 관절 위치 분석 ---
        dof_pos_np = env.dof_pos.cpu().numpy()
 
        for j in range(num_joints):
            col = dof_pos_np[:, j]
            dof_pos_min_seen[j] = min(dof_pos_min_seen[j], col.min())
            dof_pos_max_seen[j] = max(dof_pos_max_seen[j], col.max())
            margin = (dof_pos_upper[j] - dof_pos_lower[j]) * 0.05
            near_lower = np.sum(col < dof_pos_lower[j] + margin)
            near_upper = np.sum(col > dof_pos_upper[j] - margin)
            dof_near_limit_count[j] += near_lower + near_upper
 
        # --- 선속도 추종 ---
        cmd_x = env.commands[:, 0].cpu().numpy()
        cmd_y = env.commands[:, 1].cpu().numpy()
        vel_x = env.base_lin_vel[:, 0].cpu().numpy()
        vel_y = env.base_lin_vel[:, 1].cpu().numpy()
        vel_errors_x.append(np.mean(np.abs(cmd_x - vel_x)))
        vel_errors_y.append(np.mean(np.abs(cmd_y - vel_y)))
 
        data['cmd_vel_x'].append(np.mean(cmd_x))
        data['actual_vel_x'].append(np.mean(vel_x))
 
        # --- 각속도 추종 ---
        cmd_yaw = env.commands[:, 2].cpu().numpy()
        actual_yaw = env.base_ang_vel[:, 2].cpu().numpy()
        ang_vel_errors.append(np.mean(np.abs(cmd_yaw - actual_yaw)))
        data['cmd_ang_vel'].append(np.mean(np.abs(cmd_yaw)))
        data['actual_ang_vel'].append(np.mean(actual_yaw))
 
        # --- Roll/Pitch ---
        projected_grav = env.projected_gravity.cpu().numpy()
        gx = projected_grav[:, 0]
        gy = projected_grav[:, 1]
        gz = projected_grav[:, 2]
        roll = np.arctan2(gy, -gz)
        pitch = np.arctan2(-gx, -gz)
        roll_list.append(np.mean(np.abs(roll)))
        pitch_list.append(np.mean(np.abs(pitch)))
        data['roll_abs'].append(np.mean(np.abs(roll)))
        data['pitch_abs'].append(np.mean(np.abs(pitch)))
 
        # --- 높이/수직속도 ---
        data['base_height'].append(env.root_states[:, 2].mean().item())
        data['base_vel_z'].append(env.base_lin_vel[:, 2].mean().item())
 
        # --- 동작 부드러움 ---
        current_actions = env.actions.cpu().numpy()
        if prev_actions is not None:
            rate = np.mean(np.square(current_actions - prev_actions))
            action_rates.append(rate)
            data['action_rate'].append(rate)
        prev_actions = current_actions.copy()
 
        # --- 보행 패턴 (발 접촉/공중) ---
        contact_forces = env.contact_forces[:, env.feet_indices, 2].cpu().numpy()
        in_contact = contact_forces > 1.0
        for f in range(num_feet):
            feet_contact_count[f] += np.sum(in_contact[:, f])
            feet_air_count[f] += np.sum(~in_contact[:, f])
 
        contact_pattern = in_contact[0]
        data['contact_pattern'].append(contact_pattern.copy())
 
        # --- 에너지 (Power = torque * velocity) ---
        dof_vel_np = env.dof_vel.cpu().numpy()
        power = np.mean(np.sum(np.abs(torques_np * dof_vel_np), axis=1))
        power_list.append(power)
        data['power'].append(power)

        # --- 관절별 전력 (항목 6-A) ---
        per_joint_power = np.mean(np.abs(torques_np * dof_vel_np), axis=0)  # (num_joints,)
        per_joint_power_sum += per_joint_power

        # peak_power 추적
        step_total_power = np.sum(np.abs(torques_np * dof_vel_np), axis=1).mean()
        peak_power = max(peak_power, step_total_power)
 
        # --- 에피소드 통계 ---
        current_ep_return += rews
        current_ep_length += 1
        done_ids = dones.nonzero(as_tuple=False).flatten()
        if len(done_ids) > 0:
            episode_lengths.extend(current_ep_length[done_ids].cpu().tolist())
            episode_returns.extend(current_ep_return[done_ids].cpu().tolist())
            current_ep_return[done_ids] = 0
            current_ep_length[done_ids] = 0
 
        # --- alive 비율 ---
        alive = (env.reset_buf == 0).sum().item()
        data['alive_ratio'].append(alive / num_envs)

    # ============ 공통 후처리 ============
    total_samples = total_steps * num_envs

    # --- 속도 오차 요약 ---
    mean_err_x = np.mean(vel_errors_x)
    mean_err_y = np.mean(vel_errors_y)
    mean_ang_err = np.mean(ang_vel_errors)

    # --- 자세 요약 ---
    mean_height = np.mean(data['base_height'])
    std_height = np.std(data['base_height'])
    mean_vel_z = np.mean(np.abs(data['base_vel_z']))
    height_target = env.cfg.rewards.base_height_target

    # --- Roll/Pitch 요약 ---
    mean_roll = np.mean(roll_list)
    mean_pitch = np.mean(pitch_list)
    max_roll = np.max(roll_list)
    max_pitch = np.max(pitch_list)

    # --- 토크 요약 ---
    overall_sat = np.mean(data['torque_saturation_ratio']) * 100

    # --- 에너지 요약 ---
    mean_power = np.mean(power_list)
    std_power = np.std(power_list)
    mean_vel = np.mean(np.abs(data['actual_vel_x']))
    robot_mass = 2.6
    cot = mean_power / (robot_mass * 9.81 * mean_vel) if mean_vel > 0.01 else 0.0

    # --- 에피소드 통계 ---
    timeout_rate = 0.0
    early_death_rate = 0.0
    if episode_lengths:
        timeout_rate = np.sum(np.array(episode_lengths) >= env.max_episode_length) / len(episode_lengths) * 100
        early_death_rate = np.sum(np.array(episode_lengths) < 50) / len(episode_lengths) * 100

    # --- Gait FFT 주파수 (항목 3-A) ---
    contact_patterns = np.array(data['contact_pattern'])  # (steps, num_feet)
    gait_frequency = 0.0
    gait_period_steps = 0

    if len(contact_patterns) > 100:
        signal = contact_patterns[:, 0].astype(float)  # FL foot
        signal = signal - signal.mean()  # DC 제거

        dt_step = env.dt * env.cfg.control.decimation  # 1 step의 실제 시간(초)

        from numpy.fft import fft, fftfreq
        N = len(signal)
        yf = np.abs(fft(signal))[:N//2]
        xf = fftfreq(N, d=dt_step)[:N//2]

        # DC(0Hz) 제외하고 피크 찾기
        yf[0] = 0
        peak_idx = np.argmax(yf)
        gait_frequency = float(xf[peak_idx])  # Hz
        gait_period_steps = int(1.0 / (gait_frequency * dt_step)) if gait_frequency > 0 else 0

    # --- L/R 비대칭 (항목 3-B) ---
    left_contact = []
    right_contact = []
    for f in range(num_feet):
        total = feet_contact_count[f] + feet_air_count[f]
        pct = feet_contact_count[f] / total * 100 if total > 0 else 0
        fn_lower = foot_names[f].lower()
        if 'left' in fn_lower or '_l_' in fn_lower:
            left_contact.append(pct)
        elif 'right' in fn_lower or '_r_' in fn_lower:
            right_contact.append(pct)

    lr_asymmetry = abs(np.mean(left_contact) - np.mean(right_contact)) if left_contact and right_contact else 0.0

    # --- 대각 동기화율 (항목 3-C) ---
    overall_trot = 0.0
    diag1_sync = 0.0
    diag2_sync = 0.0
    all_ground = True
    for f in range(num_feet):
        total = feet_contact_count[f] + feet_air_count[f]
        air_pct = feet_air_count[f] / total * 100 if total > 0 else 0
        if air_pct > 5:
            all_ground = False

    if not all_ground and num_feet == 4 and len(contact_patterns) > 100:
        diag1_sync = float(np.mean(contact_patterns[:, 0] == contact_patterns[:, 3]))
        diag2_sync = float(np.mean(contact_patterns[:, 1] == contact_patterns[:, 2]))
        overall_trot = (diag1_sync + diag2_sync) / 2

    # ================================================================
    # 경량 모드: JSON 저장만 하고 리턴 (항목 1-B, 1-C)
    # ================================================================
    if lightweight:
        import json as _json

        # 발별 접촉 비율
        feet_contact_pct_dict = {}
        for f in range(num_feet):
            fname = foot_names[f]
            total = feet_contact_count[f] + feet_air_count[f]
            feet_contact_pct_dict[fname] = float(feet_contact_count[f] / total * 100) if total > 0 else 0.0

        snapshot = {
            'iter': iter_number,
            'timeout_pct': float(timeout_rate),
            'vel_error_x': float(mean_err_x),
            'torque_saturation_pct': float(overall_sat),
            'mean_roll_deg': float(np.degrees(mean_roll)),
            'mean_pitch_deg': float(np.degrees(mean_pitch)),
            'mean_power': float(mean_power),
            'episode_return': float(np.mean(episode_returns)) if episode_returns else 0.0,
            'feet_contact_pct': feet_contact_pct_dict,
            'diagonal_sync': float(overall_trot * 100),
        }

        snapshot_path = os.path.join(diag_dir, f'snapshot_iter{iter_number:04d}.json')
        with open(snapshot_path, 'w', encoding='utf-8') as jf:
            _json.dump(snapshot, jf, indent=2, ensure_ascii=False)
        print(f"    -> 스냅샷 저장: {snapshot_path}")
        return  # 경량 모드 종료

    # ============ 리포트 출력 (풀 모드만) ============
 
    # --- [1] 토크 분석 ---
    print(f"\n{'='*60}")
    print(f"  [1] 토크 분석 (Torque Analysis)")
    print(f"{'='*60}")
    print(f"  {'Joint':<30} {'Limit':>7} {'MaxSeen':>8} {'Sat%':>7} {'상태':>6}")
    print(f"  {'-'*58}")
    for j in range(num_joints):
        sat_pct = torque_saturation_count[j] / total_samples * 100
        limit = torque_limits[j]
        max_seen = torque_max_seen[j]
        if sat_pct > 20:
            status = "⚠ 심각"
        elif sat_pct > 5:
            status = "⚠ 주의"
        else:
            status = "✓ 정상"
        print(f"  {joint_names[j]:<30} {limit:>7.3f} {max_seen:>8.3f} {sat_pct:>6.1f}% {status}")
 
    print(f"\n  전체 평균 토크 포화율: {overall_sat:.1f}%")
    if overall_sat > 10:
        print(f"  → 토크 한계에 자주 도달. PD 게인 또는 action_scale 확인 필요.")
    elif overall_sat > 3:
        print(f"  → 가끔 포화. 정상 범위이나 모니터링 필요.")
    else:
        print(f"  → 토크 여유 충분.")
 
    # --- [2] 관절 한계 ---
    print(f"\n{'='*60}")
    print(f"  [2] 관절 한계 분석 (Joint Limit Analysis)")
    print(f"{'='*60}")
    print(f"  {'Joint':<30} {'URDF Lo':>8} {'URDF Hi':>8} {'MinSeen':>8} {'MaxSeen':>8} {'Near%':>7}")
    print(f"  {'-'*69}")
    for j in range(num_joints):
        near_pct = dof_near_limit_count[j] / total_samples * 100
        print(f"  {joint_names[j]:<30} {urdf_lower[j]:>8.3f} {urdf_upper[j]:>8.3f} "
              f"{dof_pos_min_seen[j]:>8.3f} {dof_pos_max_seen[j]:>8.3f} {near_pct:>6.1f}%")
 
    # --- [3] 선속도 추종 ---
    print(f"\n{'='*60}")
    print(f"  [3] 선속도 추종 분석 (Linear Velocity Tracking)")
    print(f"{'='*60}")
    print(f"  평균 X속도 오차: {mean_err_x:.4f} m/s")
    print(f"  평균 Y속도 오차: {mean_err_y:.4f} m/s")
    if mean_err_x < 0.03:
        print(f"  → ✓ X속도 추종 우수")
    elif mean_err_x < 0.08:
        print(f"  → X속도 추종 보통")
    else:
        print(f"  → ⚠ X속도 추종 미흡")
 
    # --- [4] 에피소드 통계 ---
    print(f"\n{'='*60}")
    print(f"  [4] 에피소드 통계 (Episode Stats)")
    print(f"{'='*60}")
    if episode_lengths:
        print(f"  에피소드 수: {len(episode_lengths)}")
        print(f"  평균 길이: {np.mean(episode_lengths):.1f} steps "
              f"(max: {env.max_episode_length} = timeout)")
        print(f"  최소/최대 길이: {np.min(episode_lengths):.0f} / {np.max(episode_lengths):.0f}")
        print(f"  평균 리턴: {np.mean(episode_returns):.3f}")
        print(f"  Timeout 비율: {timeout_rate:.1f}%")
        print(f"  조기 종료 비율 (<50 steps): {early_death_rate:.1f}%")
        if early_death_rate > 30:
            print(f"  → ⚠ 리셋 직후 넘어짐 빈번.")
    else:
        print(f"  에피소드 완료 없음")
 
    # --- [5] 자세 안정성 ---
    print(f"\n{'='*60}")
    print(f"  [5] 자세 안정성 (Posture Stability)")
    print(f"{'='*60}")
    height_error = abs(mean_height - height_target)
    print(f"  평균 높이: {mean_height:.4f} m (목표: {height_target} m, 편차: {height_error:.4f} m)")
    print(f"  높이 표준편차: {std_height:.4f} m")
    print(f"  평균 수직 속도(abs): {mean_vel_z:.4f} m/s")
    if mean_height < 0.13:
        print(f"  → ⚠ 주저앉음. Kp 부족 또는 토크 포화 확인.")
    elif height_error > 0.04:
        print(f"  → ⚠ 목표 높이와 차이 큼. base_height reward 추가 고려.")
    elif mean_vel_z > 0.1:
        print(f"  → ⚠ 수직 바운싱. lin_vel_z 페널티 추가 고려.")
    elif std_height > 0.03:
        print(f"  → ⚠ 높이 변동 큼. 안정성 reward 추가 고려.")
    else:
        print(f"  → ✓ 안정적.")
 
    # --- [6] 회전 추종 ---
    print(f"\n{'='*60}")
    print(f"  [6] 회전 추종 분석 (Angular Velocity Tracking)")
    print(f"{'='*60}")
    mean_cmd_yaw = np.mean(np.abs(data['cmd_ang_vel']))
    print(f"  평균 yaw 명령(abs): {mean_cmd_yaw:.4f} rad/s")
    print(f"  평균 yaw 오차: {mean_ang_err:.4f} rad/s")
    if mean_cmd_yaw < 0.01:
        print(f"  → yaw 명령 없음 (Step 2 이전이면 정상)")
    elif mean_ang_err < 0.05:
        print(f"  → ✓ 회전 추종 우수")
    elif mean_ang_err < 0.15:
        print(f"  → 회전 추종 보통")
    else:
        print(f"  → ⚠ 회전 추종 미흡")
 
    # --- [7] Roll/Pitch ---
    print(f"\n{'='*60}")
    print(f"  [7] 자세 각도 분석 (Roll/Pitch)")
    print(f"{'='*60}")
    print(f"  평균 |roll|:  {np.degrees(mean_roll):.2f}° (최대: {np.degrees(max_roll):.2f}°)")
    print(f"  평균 |pitch|: {np.degrees(mean_pitch):.2f}° (최대: {np.degrees(max_pitch):.2f}°)")
    if mean_roll > np.radians(15) or mean_pitch > np.radians(15):
        print(f"  → ⚠ 기울어짐 심각. orientation reward 강화 필요.")
    elif mean_roll > np.radians(8) or mean_pitch > np.radians(8):
        print(f"  → ⚠ 기울어짐 있음. orientation reward 고려.")
    else:
        print(f"  → ✓ 자세 안정적.")
 
    # --- [8] 동작 부드러움 ---
    print(f"\n{'='*60}")
    print(f"  [8] 동작 부드러움 분석 (Action Smoothness)")
    print(f"{'='*60}")
    if action_rates:
        mean_action_rate = np.mean(action_rates)
        max_action_rate = np.max(action_rates)
        print(f"  평균 action rate (MSE): {mean_action_rate:.6f}")
        print(f"  최대 action rate (MSE): {max_action_rate:.6f}")
        if mean_action_rate > 0.1:
            print(f"  → ⚠ 동작 급변 심함. 떨림 가능성. action_rate 페널티 추가 고려.")
        elif mean_action_rate > 0.01:
            print(f"  → 보통. action_rate 페널티로 개선 가능.")
        else:
            print(f"  → ✓ 동작 부드러움.")
 
    # --- [9] 보행 패턴 ---
    print(f"\n{'='*60}")
    print(f"  [9] 보행 패턴 분석 (Gait Pattern)")
    print(f"{'='*60}")
    print(f"  {'Foot':<25} {'접촉%':>8} {'공중%':>8} {'상태':>10}")
    print(f"  {'-'*51}")
    for f in range(num_feet):
        total = feet_contact_count[f] + feet_air_count[f]
        contact_pct = feet_contact_count[f] / total * 100 if total > 0 else 0
        air_pct = feet_air_count[f] / total * 100 if total > 0 else 0
        if air_pct < 2:
            status = "⚠ 끌림"
        elif air_pct < 10:
            status = "△ 약간 듦"
        elif air_pct < 50:
            status = "✓ 정상"
        else:
            status = "⚠ 과도"
        print(f"  {foot_names[f]:<25} {contact_pct:>7.1f}% {air_pct:>7.1f}% {status}")

    if all_ground:
        print(f"\n  → 모든 발이 바닥에 붙어 있음. Step 5 이전이면 정상.")
    else:
        if num_feet == 4 and len(contact_patterns) > 100:
            print(f"\n  대각선 동기화율: {overall_trot*100:.1f}%")
            print(f"    (FL-RR: {diag1_sync*100:.1f}%, FR-RL: {diag2_sync*100:.1f}%)")
            if overall_trot > 0.7:
                print(f"  → ✓ Trot gait 패턴 감지!")
            elif overall_trot > 0.5:
                print(f"  → 부분적 trot 패턴. 학습 더 필요.")
            else:
                print(f"  → Trot이 아닌 다른 gait. 안정적이면 OK.")

    # --- Gait FFT 출력 (항목 3) ---
    print(f"\n  Gait 주파수: {gait_frequency:.2f} Hz (주기: {gait_period_steps} steps)")
    print(f"  L/R 비대칭: {lr_asymmetry:.1f}%p")
 
    # --- [10] 에너지 효율 ---
    print(f"\n{'='*60}")
    print(f"  [10] 에너지 효율 분석 (Energy/Power)")
    print(f"{'='*60}")
    print(f"  평균 소비 전력: {mean_power:.4f} W")
    print(f"  전력 표준편차: {std_power:.4f} W")
    print(f"  피크 전력: {peak_power:.4f} W")
    if mean_vel > 0.01:
        print(f"  Cost of Transport: {cot:.2f}")
        if cot < 2.0:
            print(f"  → ✓ 에너지 효율 우수")
        elif cot < 10.0:
            print(f"  → 보통 (소형 로봇 일반적 수준)")
        else:
            print(f"  → ⚠ 비효율적. torques 페널티 추가 고려.")
    else:
        print(f"  → 속도 거의 0이라 CoT 계산 불가")

    # --- [11] Trot 상세 분석 ---
    print(f"\n{'='*60}")
    print(f"  [11] Trot 상세 분석 (Trot Detail)")
    print(f"{'='*60}")
    contact_history = np.array(data['contact_pattern'])
    for f in range(num_feet):
        air_durations = []
        ground_durations = []
        count = 0
        in_air = not contact_history[0][f]
        for t in range(len(contact_history)):
            is_air = not contact_history[t][f]
            if is_air == in_air:
                count += 1
            else:
                duration = count * env.dt * env.cfg.control.decimation
                if in_air:
                    air_durations.append(duration)
                else:
                    ground_durations.append(duration)
                count = 1
                in_air = is_air
        mean_air = np.mean(air_durations) if air_durations else 0
        mean_gnd = np.mean(ground_durations) if ground_durations else 0
        print(f"  {foot_names[f]}: 공중 {mean_air:.3f}s / 접지 {mean_gnd:.3f}s")

    print(f"\n  FL-RR 평균 공중시간 vs FR-RL 평균 공중시간")
    print(f"  비율이 1.0에 가까울수록 대칭")

    # --- [12] 속도 구간별 추종 ---
    print(f"\n{'='*60}")
    print(f"  [12] 속도 구간별 추종 (Per-speed Tracking)")
    print(f"{'='*60}")
    cmd_x_all = np.array(data['cmd_vel_x'])
    actual_x_all = np.array(data['actual_vel_x'])
    for low, high, label in [(0.0, 0.2, '저속'), (0.2, 0.3, '중속'), (0.3, 0.5, '고속')]:
        mask = (cmd_x_all >= low) & (cmd_x_all < high)
        if np.sum(mask) > 0:
            err = np.mean(np.abs(cmd_x_all[mask] - actual_x_all[mask]))
            print(f"  {label} ({low}~{high} m/s): 오차 {err:.4f} m/s, 샘플 {np.sum(mask)}")

    # --- [13] 넘어짐 원인 분석 ---
    print(f"\n{'='*60}")
    print(f"  [13] 넘어짐 원인 분석 (Fall Analysis)")
    print(f"{'='*60}")
    if 'fall_heights' not in data:
        data['fall_heights'] = []
        data['fall_rolls'] = []
        data['fall_pitches'] = []
    if data['fall_heights']:
        print(f"  넘어질 때 평균 높이: {np.mean(data['fall_heights']):.4f} m")
        print(f"  넘어질 때 평균 |roll|: {np.degrees(np.mean(data['fall_rolls'])):.1f}°")
        print(f"  넘어질 때 평균 |pitch|: {np.degrees(np.mean(data['fall_pitches'])):.1f}°")
        if np.mean(data['fall_rolls']) > np.mean(data['fall_pitches']):
            print(f"  → 좌우 전도가 주 원인. orientation 또는 ang_vel_xy 강화 고려")
        else:
            print(f"  → 전후 전도가 주 원인. 속도 범위 또는 lin_vel_z 확인")

    # --- [14] Shoulder 관절 분석 ---
    print(f"\n{'='*60}")
    print(f"  [14] Shoulder 관절 상세 (Shoulder Analysis)")
    print(f"{'='*60}")
    shoulder_indices = [j for j, name in enumerate(joint_names) if 'shoulder' in name]
    for j in shoulder_indices:
        range_used = dof_pos_max_seen[j] - dof_pos_min_seen[j]
        range_total = urdf_upper[j] - urdf_lower[j]
        usage_pct = range_used / range_total * 100
        bias = (dof_pos_max_seen[j] + dof_pos_min_seen[j]) / 2
        print(f"  {joint_names[j]}: 사용범위 {usage_pct:.0f}%, 편향 {bias:+.3f} rad")
        if abs(bias) > 0.1:
            print(f"    → ⚠ 한쪽으로 편향. 비대칭 보행 원인 가능")

    # ============ 그래프 1: 종합 대시보드 ============
    fig, axes = plt.subplots(4, 2, figsize=(16, 20))
    fig.suptitle('SpotMicro RL Diagnostic Report v3', fontsize=16, fontweight='bold')
    steps_range = range(len(data['cmd_vel_x']))

    ax = axes[0, 0]
    sat_pcts = torque_saturation_count / total_samples * 100
    colors = ['red' if s > 20 else 'orange' if s > 5 else 'green' for s in sat_pcts]
    short_names = [n.replace('front_', 'F').replace('rear_', 'R')
                    .replace('_left_', 'L_').replace('_right_', 'R_') for n in joint_names]
    ax.barh(short_names, sat_pcts, color=colors)
    ax.set_xlabel('Saturation %')
    ax.set_title('Torque Saturation per Joint (>90% limit)')
    ax.axvline(x=10, color='orange', linestyle='--', alpha=0.5)

    ax = axes[0, 1]
    for j in range(num_joints):
        ax.plot([j, j], [urdf_lower[j], urdf_upper[j]], 'b-', linewidth=6, alpha=0.2)
        ax.plot([j, j], [dof_pos_min_seen[j], dof_pos_max_seen[j]], 'r-', linewidth=3)
    ax.set_xticks(range(num_joints))
    ax.set_xticklabels(short_names, rotation=45, ha='right', fontsize=8)
    ax.set_ylabel('Angle (rad)')
    ax.set_title('Joint Range: URDF(blue) vs Actual(red)')

    ax = axes[1, 0]
    ax.plot(steps_range, data['cmd_vel_x'], 'b-', alpha=0.7, label='Command X')
    ax.plot(steps_range, data['actual_vel_x'], 'r-', alpha=0.7, label='Actual X')
    ax.set_xlabel('Step'); ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Linear Velocity Tracking'); ax.legend()

    ax = axes[1, 1]
    ax.plot(steps_range, data['cmd_ang_vel'], 'b-', alpha=0.7, label='Command Yaw')
    ax.plot(steps_range, data['actual_ang_vel'], 'r-', alpha=0.7, label='Actual Yaw')
    ax.set_xlabel('Step'); ax.set_ylabel('Angular Vel (rad/s)')
    ax.set_title('Angular Velocity Tracking'); ax.legend()

    ax = axes[2, 0]
    ax.plot(steps_range, [np.degrees(r) for r in data['roll_abs']], 'r-', alpha=0.7, label='|Roll|')
    ax.plot(steps_range, [np.degrees(p) for p in data['pitch_abs']], 'b-', alpha=0.7, label='|Pitch|')
    ax.axhline(y=15, color='red', linestyle='--', alpha=0.3, label='Warning 15°')
    ax.set_xlabel('Step'); ax.set_ylabel('Angle (deg)')
    ax.set_title('Roll / Pitch Stability'); ax.legend()

    ax = axes[2, 1]
    ax.plot(steps_range, data['base_height'], 'g-', alpha=0.7, label='Height')
    ax.axhline(y=height_target, color='blue', linestyle='--', alpha=0.5, label=f'Target ({height_target}m)')
    ax.axhline(y=0.12, color='red', linestyle='--', alpha=0.5, label='Termination')
    ax.set_xlabel('Step'); ax.set_ylabel('Height (m)')
    ax.set_title('Base Height'); ax.legend()

    ax = axes[3, 0]
    show_steps = min(200, len(contact_patterns))
    if show_steps > 0:
        pattern_slice = contact_patterns[-show_steps:]
        for f in range(num_feet):
            y_vals = pattern_slice[:, f].astype(float) * (num_feet - f)
            ax.fill_between(range(show_steps), y_vals - 0.4, y_vals + 0.4,
                          alpha=0.6, label=foot_names[f])
        ax.set_xlabel('Step (last 200)'); ax.set_ylabel('Foot (filled=contact)')
        ax.set_title('Gait Pattern (Robot #0)'); ax.legend(fontsize=7); ax.set_yticks([])

    ax = axes[3, 1]
    ax.plot(steps_range, data['power'], 'orange', alpha=0.7)
    ax.set_xlabel('Step'); ax.set_ylabel('Power (W)')
    ax.set_title('Mechanical Power Consumption')

    plt.tight_layout()
    save_path = os.path.join(diag_dir, 'diagnostic_report.png')
    plt.savefig(save_path, dpi=150)
    print(f"\n  종합 그래프 저장: {save_path}")

    # ============ 그래프 2: 관절별 토크 상세 ============
    print(f"  관절별 토크 히스토리 수집 중...")
    torque_history = []
    dof_pos_history = []
    obs = env.get_observations()
    for _ in range(500):
        actions = policy(obs.detach())
        obs, _, rews, dones, infos = env.step(actions.detach())
        torque_history.append(env.torques[0].cpu().numpy())
        dof_pos_history.append(env.dof_pos[0].cpu().numpy())

    torque_history = np.array(torque_history)
    dof_pos_history = np.array(dof_pos_history)

    fig2, axes2 = plt.subplots(4, 3, figsize=(18, 16))
    fig2.suptitle('Per-Joint Torque & Position (Robot #0, last 500 steps)', fontsize=14)
    for j in range(min(num_joints, 12)):
        row = j // 3; col = j % 3
        ax = axes2[row, col]; ax_pos = ax.twinx()
        t_steps = range(len(torque_history))
        ax.plot(t_steps, torque_history[:, j], 'b-', alpha=0.7, linewidth=0.8)
        ax.axhline(y=torque_limits[j], color='red', linestyle='--', alpha=0.3)
        ax.axhline(y=-torque_limits[j], color='red', linestyle='--', alpha=0.3)
        ax.set_ylabel('Torque (Nm)', color='blue', fontsize=8)
        ax.set_ylim(-torque_limits[j] * 1.2, torque_limits[j] * 1.2)
        ax_pos.plot(t_steps, dof_pos_history[:, j], 'g-', alpha=0.5, linewidth=0.8)
        ax_pos.axhline(y=urdf_lower[j], color='orange', linestyle=':', alpha=0.3)
        ax_pos.axhline(y=urdf_upper[j], color='orange', linestyle=':', alpha=0.3)
        ax_pos.set_ylabel('Pos (rad)', color='green', fontsize=8)
        ax.set_title(joint_names[j], fontsize=9)
        ax.tick_params(axis='both', labelsize=7)

    plt.tight_layout()
    save_path2 = os.path.join(diag_dir, 'joint_detail.png')
    plt.savefig(save_path2, dpi=150)
    print(f"  관절 상세 그래프 저장: {save_path2}")

    # ============ 그래프 3: 동작 부드러움 ============
    if action_rates:
        fig3, axes3 = plt.subplots(1, 2, figsize=(14, 5))
        fig3.suptitle('Action Smoothness Analysis', fontsize=14)
        ax = axes3[0]
        ax.plot(action_rates, 'purple', alpha=0.7, linewidth=0.5)
        ax.set_xlabel('Step'); ax.set_ylabel('Action Rate (MSE)')
        ax.set_title('Action Rate Over Time')
        ax = axes3[1]
        ax.hist(action_rates, bins=50, color='purple', edgecolor='white', alpha=0.7)
        ax.set_xlabel('Action Rate (MSE)'); ax.set_ylabel('Count')
        ax.set_title('Action Rate Distribution')
        plt.tight_layout()
        save_path3 = os.path.join(diag_dir, 'action_smoothness.png')
        plt.savefig(save_path3, dpi=150)
        print(f"  동작 부드러움 그래프 저장: {save_path3}")

    print(f"\n{'='*60}")
    print(f"  진단 완료!")
    print(f"  그래프 위치: {diag_dir}/")
    print(f"{'='*60}\n")

    # ============ JSON 요약 저장 (experiment_report.py 연동용) ============
    import json as _json
    from datetime import datetime as _dt

    diagnostic_summary = {
        'timestamp': _dt.now().isoformat(),
        'run_name': train_cfg.runner.run_name,
        'experiment_name': train_cfg.runner.experiment_name,
        'max_iterations': train_cfg.runner.max_iterations,
        'num_envs': num_envs,
        'metrics': {
            'timeout_pct': float(timeout_rate),
            'early_death_pct': float(early_death_rate),
            'vel_error_x': float(mean_err_x),
            'vel_error_y': float(mean_err_y),
            'ang_vel_error': float(mean_ang_err),
            'torque_saturation_pct': float(overall_sat),
            'mean_height': float(mean_height),
            'height_std': float(std_height),
            'height_target': float(height_target),
            'mean_vel_z_abs': float(mean_vel_z),
            'mean_roll_deg': float(np.degrees(mean_roll)),
            'mean_pitch_deg': float(np.degrees(mean_pitch)),
            'max_roll_deg': float(np.degrees(max_roll)),
            'max_pitch_deg': float(np.degrees(max_pitch)),
            'mean_action_rate': float(np.mean(action_rates)) if action_rates else 0.0,
            'max_action_rate': float(np.max(action_rates)) if action_rates else 0.0,
            'mean_power': float(mean_power),
            'power_std': float(std_power),
            'cost_of_transport': float(cot) if mean_vel > 0.01 else None,
            'mean_episode_length': float(np.mean(episode_lengths)) if episode_lengths else 0.0,
            'mean_episode_return': float(np.mean(episode_returns)) if episode_returns else 0.0,
            'num_episodes': len(episode_lengths),
        },
        'config': {
            'control_type': env.cfg.control.control_type,
            'action_scale': float(env.cfg.control.action_scale),
            'decimation': int(env.cfg.control.decimation),
            'stiffness': {k: float(v) for k, v in env.cfg.control.stiffness.items()},
            'damping': {k: float(v) for k, v in env.cfg.control.damping.items()},
            'base_height_target': float(env.cfg.rewards.base_height_target),
            'soft_dof_pos_limit': float(env.cfg.rewards.soft_dof_pos_limit),
            'lin_vel_x': list(env.cfg.commands.ranges.lin_vel_x),
            'lin_vel_y': list(env.cfg.commands.ranges.lin_vel_y),
            'ang_vel_yaw': list(env.cfg.commands.ranges.ang_vel_yaw),
        },
        'reward_scales': {},
        'torque_per_joint': {},
        'joint_limit_near_pct': {},
        'gait': {
            'feet_contact_pct': {},
            'feet_air_pct': {},
        },
    }

    # reward scales 추출
    for attr in dir(env.cfg.rewards.scales):
        if not attr.startswith('_'):
            val = getattr(env.cfg.rewards.scales, attr)
            if isinstance(val, (int, float)):
                diagnostic_summary['reward_scales'][attr] = float(val)

    # 관절별 상세 (항목 5-A: 사용 범위/편향 추가)
    for j in range(num_joints):
        jname = joint_names[j]
        sat_pct = float(torque_saturation_count[j] / total_samples * 100)
        near_pct = float(dof_near_limit_count[j] / total_samples * 100)

        range_used = float(dof_pos_max_seen[j] - dof_pos_min_seen[j])
        range_total = float(urdf_upper[j] - urdf_lower[j])
        usage_pct = range_used / range_total * 100 if range_total > 0 else 0
        bias = float((dof_pos_max_seen[j] + dof_pos_min_seen[j]) / 2)

        diagnostic_summary['torque_per_joint'][jname] = {
            'saturation_pct': sat_pct,
            'max_torque_seen': float(torque_max_seen[j]),
            'torque_limit': float(torque_limits[j]),
            'range_used_rad': range_used,
            'range_total_rad': range_total,
            'range_usage_pct': float(usage_pct),
            'position_bias_rad': bias,
        }
        diagnostic_summary['joint_limit_near_pct'][jname] = near_pct

    # 보행 패턴
    for f in range(num_feet):
        fname = foot_names[f]
        total = feet_contact_count[f] + feet_air_count[f]
        diagnostic_summary['gait']['feet_contact_pct'][fname] = \
            float(feet_contact_count[f] / total * 100) if total > 0 else 0
        diagnostic_summary['gait']['feet_air_pct'][fname] = \
            float(feet_air_count[f] / total * 100) if total > 0 else 0

    # Gait 분석 데이터 (항목 3-D)
    diagnostic_summary['gait']['frequency_hz'] = float(gait_frequency)
    diagnostic_summary['gait']['period_steps'] = int(gait_period_steps)
    diagnostic_summary['gait']['diagonal_sync_pct'] = float(overall_trot * 100)
    diagnostic_summary['gait']['lr_asymmetry_pct'] = float(lr_asymmetry)

    # 에너지 상세 (항목 6-B)
    diagnostic_summary['energy'] = {
        'mean_power': float(mean_power),
        'peak_power': float(peak_power),
        'peak_mean_ratio': float(peak_power / mean_power) if mean_power > 0 else None,
        'cost_of_transport': float(cot) if mean_vel > 0.01 else None,
        'per_joint_power': {},
    }
    total_power_sum = per_joint_power_sum.sum()
    for j in range(num_joints):
        avg_power = per_joint_power_sum[j] / total_steps
        pct = per_joint_power_sum[j] / total_power_sum * 100 if total_power_sum > 0 else 0
        diagnostic_summary['energy']['per_joint_power'][joint_names[j]] = {
            'mean_power_w': float(avg_power),
            'share_pct': float(pct),
        }

    # JSON 저장
    json_save_path = os.path.join(diag_dir, 'diagnostic_summary.json')
    with open(json_save_path, 'w', encoding='utf-8') as jf:
        _json.dump(diagnostic_summary, jf, indent=2, ensure_ascii=False)
    print(f"  JSON 요약 저장: {json_save_path}")


# ============================================================
if __name__ == '__main__':
    import sys

    args = get_args()

    # --- 항목 1-A: 추가 CLI 인자 파싱 ---
    checkpoint_path = None
    lightweight = False

    # get_args()가 처리하지 않는 인자를 수동으로 파싱
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        if argv[i] == '--checkpoint' and i + 1 < len(argv):
            checkpoint_path = argv[i + 1]
            i += 2
        elif argv[i].startswith('--checkpoint='):
            checkpoint_path = argv[i].split('=', 1)[1]
            i += 1
        elif argv[i] == '--lightweight':
            lightweight = True
            i += 1
        else:
            i += 1

    run_diagnostic(args, checkpoint_path=checkpoint_path, lightweight=lightweight)
