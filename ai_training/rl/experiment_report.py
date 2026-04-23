#!/usr/bin/env python3
"""
SpotMicro RL 실험 보고서 자동 생성 + Git 자동 커밋
=====================================================

사용법:
  # 방법 1: 목적을 CLI로 전달
  python experiment_report.py --task=spotmicro_test \
      --purpose "Step4: torques/action_rate 페널티 추가로 동작 부드럽게"

  # 방법 2: 목적을 파일로 전달 (학습 전에 미리 작성)
  echo "Step4: torques/action_rate 페널티 추가" > .experiment_purpose.txt
  python experiment_report.py --task=spotmicro_test

  # 방법 3: 목적 없이 실행 (보고서에 TODO로 표시)
  python experiment_report.py --task=spotmicro_test

동작:
  1. diagnostics/ 에서 최신 diagnostic_summary.json 읽기
  2. logs/ 에서 tensorboard 스칼라 + 그래프 추출
  3. git diff로 이전 버전 대비 config 변경점 자동 감지
  4. 이전 실험 보고서에서 수치 읽어서 비교표 생성
  5. pass/fail 기준에 따라 결론 자동 생성
  6. 마크다운 보고서 생성 → experiments/ 폴더
  7. git add → commit → push
"""

import os
import sys
import json
import glob
import re
import subprocess
import argparse
from datetime import datetime
from pathlib import Path

# ============================================================
# 설정
# ============================================================
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

# legged_gym이 다른 경로에 있으면 여기서 수정
# PROJECT_ROOT를 legged_gym 루트로 바꾸거나, 아래 경로들을 조정
LOGS_DIR = os.path.join(PROJECT_ROOT, 'legged_gym', 'logs')
DIAG_DIR = os.path.join(PROJECT_ROOT, 'legged_gym', 'diagnostics')
EXPERIMENTS_DIR = os.path.join(PROJECT_ROOT, 'experiments')
PURPOSE_FILE = os.path.join(PROJECT_ROOT, '.experiment_purpose.txt')


def parse_args():
    parser = argparse.ArgumentParser(description='RL 실험 보고서 자동 생성')
    parser.add_argument('--task', type=str, default='spotmicro_test',
                        help='legged_gym task 이름')
    parser.add_argument('--purpose', type=str, default=None,
                        help='이번 실험의 목적 (한 줄)')
    parser.add_argument('--no-git', action='store_true',
                        help='Git commit/push 건너뛰기')
    parser.add_argument('--no-push', action='store_true',
                        help='Git push만 건너뛰기 (commit은 함)')
    parser.add_argument('--exp-id', type=str, default=None,
                        help='실험 ID 직접 지정 (미지정시 자동 채번)')
    return parser.parse_args()


# ============================================================
# 1. 실험 ID 채번
# ============================================================
def get_next_experiment_id(experiments_dir):
    """experiments/ 폴더에서 마지막 번호를 찾아 +1"""
    os.makedirs(experiments_dir, exist_ok=True)
    existing = glob.glob(os.path.join(experiments_dir, 'exp*.md'))
    if not existing:
        return 1

    max_id = 0
    for f in existing:
        basename = os.path.basename(f)
        match = re.match(r'exp(\d+)', basename)
        if match:
            max_id = max(max_id, int(match.group(1)))
    return max_id + 1


# ============================================================
# 2. 목적(purpose) 읽기
# ============================================================
def get_purpose(args):
    """CLI 인자 > .experiment_purpose.txt > TODO 순서로 읽기"""
    if args.purpose:
        return args.purpose

    if os.path.exists(PURPOSE_FILE):
        with open(PURPOSE_FILE, 'r', encoding='utf-8') as f:
            purpose = f.read().strip()
        if purpose:
            return purpose

    return "TODO: 실험 목적을 여기에 작성하세요"


# ============================================================
# 3. Diagnostic JSON 읽기
# ============================================================
def load_diagnostic_summary(experiment_name):
    """play_diagnostic.py가 저장한 diagnostic_summary.json 로드"""
    json_path = os.path.join(DIAG_DIR, experiment_name, 'diagnostic_summary.json')

    if not os.path.exists(json_path):
        print(f"[경고] {json_path} 없음.")
        print(f"  play_diagnostic.py 에 JSON 저장 패치를 적용했는지 확인하세요.")
        print(f"  패치 없이도 보고서 골격은 생성합니다.")
        return None

    with open(json_path, 'r', encoding='utf-8') as f:
        return json.load(f)

# ============================================================
# 4. Tensorboard 스칼라 추출
# ============================================================
def extract_tensorboard_data(experiment_name, run_name=None):
    """최신 run의 tensorboard event 파일에서 스칼라 추출"""
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except ImportError:
        print("[경고] tensorboard 패키지 없음. pip install tensorboard")
        return None

    log_base = os.path.join(LOGS_DIR, experiment_name)
    if not os.path.exists(log_base):
        print(f"[경고] 로그 디렉토리 없음: {log_base}")
        return None

    # 최신 run 찾기 (수정 시간 기준)
    if run_name:
        run_dirs = [os.path.join(log_base, run_name)]
    else:
        all_runs = sorted(
            [os.path.join(log_base, d) for d in os.listdir(log_base)
             if os.path.isdir(os.path.join(log_base, d))],
            key=os.path.getmtime
        )
        if not all_runs:
            print(f"[경고] run 디렉토리 없음: {log_base}")
            return None
        run_dirs = [all_runs[-1]]  # 최신 run

    run_dir = run_dirs[0]
    run_name_actual = os.path.basename(run_dir)
    print(f"  Tensorboard 읽는 중: {run_dir}")

    # event 파일 찾기
    event_files = glob.glob(os.path.join(run_dir, 'events.out.tfevents.*'))
    if not event_files:
        # 하위 폴더 탐색
        event_files = glob.glob(os.path.join(run_dir, '**', 'events.out.tfevents.*'), recursive=True)

    if not event_files:
        print(f"[경고] event 파일 없음: {run_dir}")
        return None

    ea = EventAccumulator(run_dir)
    ea.Reload()

    tb_data = {
        'run_name': run_name_actual,
        'run_dir': run_dir,
        'scalars': {},
    }

    # 주요 스칼라 태그 추출
    scalar_tags = ea.Tags().get('scalars', [])
    for tag in scalar_tags:
        events = ea.Scalars(tag)
        if events:
            values = [e.value for e in events]
            steps = [e.step for e in events]
            tb_data['scalars'][tag] = {
                'final': values[-1],
                'max': max(values),
                'min': min(values),
                'mean_last_10pct': sum(values[int(len(values)*0.9):]) / max(1, len(values) - int(len(values)*0.9)),
                'total_steps': steps[-1] if steps else 0,
            }

    return tb_data


# ============================================================
# 5. Tensorboard 그래프 저장
# ============================================================
def save_tensorboard_graphs(tb_data, save_dir):
    """주요 reward 커브를 PNG로 저장"""
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("[경고] matplotlib 또는 tensorboard 없음. 그래프 건너뜀.")
        return []

    if not tb_data or 'run_dir' not in tb_data:
        return []

    ea = EventAccumulator(tb_data['run_dir'])
    ea.Reload()

    scalar_tags = ea.Tags().get('scalars', [])
    if not scalar_tags:
        return []

    os.makedirs(save_dir, exist_ok=True)
    saved_files = []

    # --- 그래프 1: Total Reward + Episode Length ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle(f"Training Curves: {tb_data['run_name']}", fontsize=14)

    reward_tags = [t for t in scalar_tags if 'reward' in t.lower() or 'return' in t.lower()]
    length_tags = [t for t in scalar_tags if 'length' in t.lower() or 'episode_length' in t.lower()]

    # 메인 reward
    for tag in reward_tags[:3]:  # 상위 3개만
        events = ea.Scalars(tag)
        steps = [e.step for e in events]
        values = [e.value for e in events]
        label = tag.split('/')[-1] if '/' in tag else tag
        axes[0].plot(steps, values, alpha=0.8, label=label)
    axes[0].set_xlabel('Iteration')
    axes[0].set_ylabel('Reward')
    axes[0].set_title('Training Reward')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Episode length
    for tag in length_tags[:2]:
        events = ea.Scalars(tag)
        steps = [e.step for e in events]
        values = [e.value for e in events]
        label = tag.split('/')[-1] if '/' in tag else tag
        axes[1].plot(steps, values, alpha=0.8, label=label)
    axes[1].set_xlabel('Iteration')
    axes[1].set_ylabel('Episode Length')
    axes[1].set_title('Episode Length')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    path1 = os.path.join(save_dir, 'tb_training_curves.png')
    plt.savefig(path1, dpi=150)
    plt.close()
    saved_files.append(path1)

    # --- 그래프 2: 개별 Reward 항목들 ---
    rew_tags = [t for t in scalar_tags if t.startswith('rew_') or '/rew_' in t]
    if rew_tags:
        n_plots = len(rew_tags)
        cols = 3
        rows = (n_plots + cols - 1) // cols
        fig, axes = plt.subplots(rows, cols, figsize=(15, 4 * rows))
        fig.suptitle(f"Individual Rewards: {tb_data['run_name']}", fontsize=14)

        if rows == 1 and cols == 1:
            axes = [[axes]]
        elif rows == 1:
            axes = [axes]

        for i, tag in enumerate(rew_tags):
            r, c = i // cols, i % cols
            ax = axes[r][c] if rows > 1 else axes[0][c]
            events = ea.Scalars(tag)
            steps = [e.step for e in events]
            values = [e.value for e in events]
            label = tag.split('/')[-1] if '/' in tag else tag
            label = label.replace('rew_', '')
            ax.plot(steps, values, alpha=0.8)
            ax.set_title(label, fontsize=10)
            ax.grid(True, alpha=0.3)

        # 빈 서브플롯 제거
        for i in range(n_plots, rows * cols):
            r, c = i // cols, i % cols
            ax = axes[r][c] if rows > 1 else axes[0][c]
            ax.set_visible(False)

        plt.tight_layout()
        path2 = os.path.join(save_dir, 'tb_individual_rewards.png')
        plt.savefig(path2, dpi=150)
        plt.close()
        saved_files.append(path2)

    return saved_files

def copy_final_model(experiment_name, dest_dir):
    """최신 run의 최종 모델을 experiments 폴더로 복사"""
    import shutil
    log_base = os.path.join(LOGS_DIR, experiment_name)
    if not os.path.exists(log_base):
        return None

    runs = sorted(
        [os.path.join(log_base, d) for d in os.listdir(log_base)
         if os.path.isdir(os.path.join(log_base, d))],
        key=os.path.getmtime
    )
    if not runs:
        return None

    latest_run = runs[-1]
    models = sorted(glob.glob(os.path.join(latest_run, 'model_*.pt')))
    if not models:
        return None

    final_model = models[-1]
    dest_path = os.path.join(dest_dir, 'model_final.pt')
    shutil.copy2(final_model, dest_path)
    return dest_path
    
# ============================================================
# 6. Git Diff로 변경점 자동 감지
# ============================================================
def get_config_diff():
    """git diff로 마지막 커밋 대비 config 변경 사항 추출"""
    try:
        # staged + unstaged 변경 모두 포함
        result = subprocess.run(
            ['git', 'diff', 'HEAD', '--', '*.py'],
            capture_output=True, text=True, cwd=PROJECT_ROOT
        )
        if result.returncode != 0:
            # 첫 커밋이거나 git이 없는 경우
            result = subprocess.run(
                ['git', 'diff', '--cached', '--', '*.py'],
                capture_output=True, text=True, cwd=PROJECT_ROOT
            )

        diff_text = result.stdout.strip()
        if not diff_text:
            # unstaged만 있을 수 있음
            result = subprocess.run(
                ['git', 'diff', '--', '*.py'],
                capture_output=True, text=True, cwd=PROJECT_ROOT
            )
            diff_text = result.stdout.strip()

        return diff_text if diff_text else "(변경 사항 없음 또는 git 미설정)"

    except FileNotFoundError:
        return "(git이 설치되지 않음)"


def parse_config_changes(diff_text):
    """diff에서 reward scales, hyperparams 등 주요 변경 사항을 사람이 읽기 좋게 요약"""
    if diff_text.startswith("("):
        return diff_text

    changes = []
    lines = diff_text.split('\n')

    for i, line in enumerate(lines):
        if line.startswith('+') and not line.startswith('+++'):
            content = line[1:].strip()
            if content and not content.startswith('#'):
                changes.append(f"  + {content}")
        elif line.startswith('-') and not line.startswith('---'):
            content = line[1:].strip()
            if content and not content.startswith('#'):
                changes.append(f"  - {content}")

    if not changes:
        return "(코드 변경 없음)"

    return '\n'.join(changes)


# ============================================================
# 7. 이전 실험과 비교
# ============================================================
def load_previous_experiment(experiments_dir, current_id):
    """직전 실험 보고서에서 수치 읽기"""
    prev_id = current_id - 1
    if prev_id < 1:
        return None

    # JSON 사이드카 파일 찾기
    pattern = os.path.join(experiments_dir, f'exp{prev_id:03d}_*.json')
    files = glob.glob(pattern)
    if files:
        with open(files[0], 'r', encoding='utf-8') as f:
            return json.load(f)
    return None


# ============================================================
# 8. Pass/Fail 자동 판정
# ============================================================
def auto_judge(metrics, run_name=''):
    """phase4_detailed_guide의 통과 기준에 따라 자동 판정"""
    if not metrics:
        return "⚠ 수치 데이터 없음 — 수동 판정 필요", []

    judgments = []
    all_pass = True

    # Timeout%
    timeout = metrics.get('timeout_pct', 0)
    if timeout >= 80:
        judgments.append(f"✅ Timeout: {timeout:.1f}% (≥80%)")
    elif timeout >= 60:
        judgments.append(f"⚠️ Timeout: {timeout:.1f}% (60~80%, 보통)")
        all_pass = False
    else:
        judgments.append(f"❌ Timeout: {timeout:.1f}% (<60%, 미달)")
        all_pass = False

    # 속도 오차 X
    vel_err = metrics.get('vel_error_x', 999)
    if vel_err < 0.12:
        judgments.append(f"✅ 속도오차 X: {vel_err:.4f} m/s (<0.12)")
    elif vel_err < 0.2:
        judgments.append(f"⚠️ 속도오차 X: {vel_err:.4f} m/s (0.12~0.2, 보통)")
    else:
        judgments.append(f"❌ 속도오차 X: {vel_err:.4f} m/s (>0.2, 미달)")
        all_pass = False

    # 토크 포화율
    torque_sat = metrics.get('torque_saturation_pct', 0)
    if torque_sat < 10:
        judgments.append(f"✅ 토크포화: {torque_sat:.1f}% (<10%)")
    elif torque_sat < 40:
        judgments.append(f"⚠️ 토크포화: {torque_sat:.1f}% (10~40%)")
    else:
        judgments.append(f"❌ 토크포화: {torque_sat:.1f}% (>40%)")
        all_pass = False

    # 자세 안정성
    roll = metrics.get('mean_roll_deg', 0)
    pitch = metrics.get('mean_pitch_deg', 0)
    if roll < 8 and pitch < 8:
        judgments.append(f"✅ 자세: roll {roll:.1f}°, pitch {pitch:.1f}° (안정)")
    elif roll < 15 and pitch < 15:
        judgments.append(f"⚠️ 자세: roll {roll:.1f}°, pitch {pitch:.1f}° (보통)")
    else:
        judgments.append(f"❌ 자세: roll {roll:.1f}°, pitch {pitch:.1f}° (불안정)")
        all_pass = False

    # 조기 종료
    early_death = metrics.get('early_death_pct', 0)
    if early_death < 5:
        judgments.append(f"✅ 조기종료: {early_death:.1f}% (<5%)")
    elif early_death < 20:
        judgments.append(f"⚠️ 조기종료: {early_death:.1f}% (5~20%)")
    else:
        judgments.append(f"❌ 조기종료: {early_death:.1f}% (>20%)")
        all_pass = False

    overall = "✅ PASS" if all_pass else "❌ FAIL (일부 기준 미달)"
    return overall, judgments


# ============================================================
# 9. 보고서 생성
# ============================================================
def generate_report(exp_id, purpose, diag_data, tb_data, diff_text,
                    prev_data, tb_graphs, task_name):
    """마크다운 보고서 생성"""

    run_name = ''
    if tb_data:
        run_name = tb_data.get('run_name', '')
    elif diag_data:
        run_name = diag_data.get('run_name', '')

    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M')
    date_str = datetime.now().strftime('%Y-%m-%d')

    # 파일명
    safe_run = re.sub(r'[^\w\-]', '_', run_name) if run_name else 'unknown'
    filename = f"exp{exp_id:03d}_{safe_run}.md"
    json_filename = f"exp{exp_id:03d}_{safe_run}.json"

    metrics = diag_data.get('metrics', {}) if diag_data else {}
    config_snapshot = diag_data.get('config', {}) if diag_data else {}
    reward_scales = diag_data.get('reward_scales', {}) if diag_data else {}

    # 자동 판정
    overall_judge, judgments = auto_judge(metrics, run_name)

    # --- 변경점 요약 ---
    changes_summary = parse_config_changes(diff_text)

    # --- 이전 대비 비교표 ---
    comparison = ""
    if prev_data and prev_data.get('metrics'):
        pm = prev_data['metrics']
        comparison = f"""
## 이전 실험 대비 비교

| 지표 | 이전 (exp{exp_id-1:03d}) | 현재 (exp{exp_id:03d}) | 변화 |
|------|-------|-------|------|
| Timeout% | {pm.get('timeout_pct', 'N/A'):.1f}% | {metrics.get('timeout_pct', 'N/A'):.1f}% | {_delta(metrics.get('timeout_pct'), pm.get('timeout_pct'), '%', higher_better=True)} |
| 속도오차 X | {pm.get('vel_error_x', 'N/A'):.4f} | {metrics.get('vel_error_x', 'N/A'):.4f} | {_delta(metrics.get('vel_error_x'), pm.get('vel_error_x'), 'm/s', higher_better=False)} |
| 토크포화 | {pm.get('torque_saturation_pct', 'N/A'):.1f}% | {metrics.get('torque_saturation_pct', 'N/A'):.1f}% | {_delta(metrics.get('torque_saturation_pct'), pm.get('torque_saturation_pct'), '%', higher_better=False)} |
| Roll | {pm.get('mean_roll_deg', 'N/A'):.1f}° | {metrics.get('mean_roll_deg', 'N/A'):.1f}° | {_delta(metrics.get('mean_roll_deg'), pm.get('mean_roll_deg'), '°', higher_better=False)} |
| Pitch | {pm.get('mean_pitch_deg', 'N/A'):.1f}° | {metrics.get('mean_pitch_deg', 'N/A'):.1f}° | {_delta(metrics.get('mean_pitch_deg'), pm.get('mean_pitch_deg'), '°', higher_better=False)} |
| 평균 전력 | {pm.get('mean_power', 'N/A'):.4f}W | {metrics.get('mean_power', 'N/A'):.4f}W | {_delta(metrics.get('mean_power'), pm.get('mean_power'), 'W', higher_better=False)} |
"""

    # --- Tensorboard 요약 ---
    tb_summary = ""
    if tb_data and tb_data.get('scalars'):
        tb_rows = []
        for tag, vals in sorted(tb_data['scalars'].items()):
            short_tag = tag.split('/')[-1] if '/' in tag else tag
            tb_rows.append(
                f"| {short_tag} | {vals['final']:.4f} | "
                f"{vals['max']:.4f} | {vals['min']:.4f} | "
                f"{vals['mean_last_10pct']:.4f} |"
            )
        tb_table = '\n'.join(tb_rows)
        tb_summary = f"""
## Tensorboard 학습 곡선 요약

| 스칼라 | 최종값 | 최대 | 최소 | 후반10% 평균 |
|--------|--------|------|------|-------------|
{tb_table}

총 학습 iteration: {tb_data['scalars'].get(list(tb_data['scalars'].keys())[0], {}).get('total_steps', 'N/A')}
"""

    # --- 그래프 참조 ---
    graph_refs = ""
    exp_dir_rel = f"experiments/exp{exp_id:03d}_{safe_run}"
    if tb_graphs:
        graph_lines = []
        for gp in tb_graphs:
            gname = os.path.basename(gp)
            graph_lines.append(f"![{gname}]({exp_dir_rel}/{gname})")
        graph_refs = '\n'.join(graph_lines)

    # diagnostic 그래프 참조
    diag_graph_dir = os.path.join(DIAG_DIR, task_name)
    diag_graphs_ref = ""
    if os.path.exists(diag_graph_dir):
        for img_name in ['diagnostic_report.png', 'joint_detail.png', 'action_smoothness.png']:
            img_path = os.path.join(diag_graph_dir, img_name)
            if os.path.exists(img_path):
                diag_graphs_ref += f"![{img_name}]({exp_dir_rel}/{img_name})\n"

    # --- Reward Scales 스냅샷 ---
    reward_table = ""
    if reward_scales:
        reward_rows = []
        for k, v in sorted(reward_scales.items()):
            if v != 0.0:
                reward_rows.append(f"| {k} | {v} |")
        if reward_rows:
            reward_table = "| Reward | Scale |\n|--------|-------|\n" + '\n'.join(reward_rows)

    # --- 보고서 조립 ---
    report = f"""# 실험 {exp_id:03d}: {run_name}

- **날짜:** {timestamp}
- **Task:** {task_name}
- **Run name:** `{run_name}`
- **판정:** {overall_judge}

---

## 실험 목적

{purpose}

---

## 변경점 (이전 커밋 대비)

```diff
{diff_text[:3000]}
```

**변경 요약:**
{changes_summary}

---

## 현재 Reward Scales

{reward_table if reward_table else "(데이터 없음 — diagnostic JSON 확인)"}

---

## 진단 결과 (Diagnostic)

### 핵심 지표

{chr(10).join('- ' + j for j in judgments) if judgments else "(데이터 없음)"}

### 상세 수치

| 지표 | 값 |
|------|-----|
| Timeout% | {metrics.get('timeout_pct', 'N/A')} |
| 조기종료% | {metrics.get('early_death_pct', 'N/A')} |
| 속도오차 X | {metrics.get('vel_error_x', 'N/A')} m/s |
| 속도오차 Y | {metrics.get('vel_error_y', 'N/A')} m/s |
| 각속도오차 | {metrics.get('ang_vel_error', 'N/A')} rad/s |
| 토크포화% | {metrics.get('torque_saturation_pct', 'N/A')} |
| 평균 높이 | {metrics.get('mean_height', 'N/A')} m |
| Roll (평균) | {metrics.get('mean_roll_deg', 'N/A')}° |
| Pitch (평균) | {metrics.get('mean_pitch_deg', 'N/A')}° |
| Action Rate | {metrics.get('mean_action_rate', 'N/A')} |
| 평균 전력 | {metrics.get('mean_power', 'N/A')} W |
| CoT | {metrics.get('cost_of_transport', 'N/A')} |
{comparison}
{tb_summary}

---

## 그래프

### Tensorboard 학습 곡선
{graph_refs if graph_refs else "(그래프 없음)"}

### Diagnostic 결과
{diag_graphs_ref if diag_graphs_ref else "(그래프 없음)"}

---

## 결론 및 다음 실험 방향

**자동 판정:** {overall_judge}

{_auto_conclusion(metrics, judgments)}

> 💡 위 결론은 자동 생성되었습니다. 필요시 수정하세요.

---

## 메모

_(추가 메모가 있으면 여기에 작성)_

"""
    return report, filename, json_filename


def _delta(current, previous, unit, higher_better=True):
    """변화량을 화살표로 표시"""
    if current is None or previous is None:
        return "N/A"
    try:
        diff = float(current) - float(previous)
        if abs(diff) < 0.0001:
            return "→ 유지"
        arrow = "↑" if diff > 0 else "↓"
        good = (diff > 0) == higher_better
        emoji = "✅" if good else "⚠️"
        return f"{emoji} {arrow} {abs(diff):.4f}{unit}"
    except (TypeError, ValueError):
        return "N/A"


def _auto_conclusion(metrics, judgments):
    """수치 기반 자동 결론 생성"""
    if not metrics:
        return "데이터 없음. play_diagnostic.py를 실행 후 다시 시도하세요."

    issues = [j for j in judgments if j.startswith('❌')]
    warnings = [j for j in judgments if j.startswith('⚠️')]
    passes = [j for j in judgments if j.startswith('✅')]

    lines = []

    if not issues and not warnings:
        lines.append("모든 기준을 통과했습니다. 다음 Step으로 진행 가능합니다.")
    elif not issues:
        lines.append("대부분 기준을 통과했으나 일부 항목이 보통 수준입니다.")
        for w in warnings:
            lines.append(f"  - {w}")
    else:
        lines.append("일부 기준을 통과하지 못했습니다. 조정이 필요합니다.")
        for i in issues:
            lines.append(f"  - {i}")

        # 구체적 제안
        timeout = metrics.get('timeout_pct', 100)
        if timeout < 60:
            lines.append("")
            lines.append("**제안:** Timeout이 낮습니다. 최근 추가한 페널티를 절반으로 줄여보세요.")

        vel_err = metrics.get('vel_error_x', 0)
        if vel_err > 0.2:
            lines.append("**제안:** 속도 추종이 미흡합니다. tracking_lin_vel 가중치를 높여보세요.")

        torque_sat = metrics.get('torque_saturation_pct', 0)
        if torque_sat > 40:
            lines.append("**제안:** 토크 포화가 심합니다. action_scale을 줄이거나 Kp를 낮춰보세요.")

    return '\n'.join(lines)


# ============================================================
# 10. EXPERIMENT_LOG.md 업데이트
# ============================================================
def update_experiment_log(exp_id, run_name, purpose, metrics, overall_judge):
    """전체 실험 요약 테이블에 한 줄 추가"""
    log_path = os.path.join(EXPERIMENTS_DIR, 'EXPERIMENT_LOG.md')

    date_str = datetime.now().strftime('%m/%d')
    timeout = f"{metrics.get('timeout_pct', 'N/A'):.1f}%" if metrics else "N/A"
    vel_err = f"{metrics.get('vel_error_x', 'N/A'):.3f}" if metrics else "N/A"
    torque = f"{metrics.get('torque_saturation_pct', 'N/A'):.1f}%" if metrics else "N/A"
    result = "✅" if "PASS" in overall_judge else "❌"

    # 목적 줄이기 (테이블에 들어갈 만큼)
    short_purpose = purpose[:40] + "..." if len(purpose) > 40 else purpose

    new_row = (
        f"| {exp_id:03d} | {date_str} | {run_name} | {short_purpose} "
        f"| {timeout} | {vel_err} | {torque} | {result} |"
    )

    if os.path.exists(log_path):
        with open(log_path, 'r', encoding='utf-8') as f:
            content = f.read()
        content = content.rstrip() + '\n' + new_row + '\n'
    else:
        header = """# SpotMicro RL 실험 로그

| # | 날짜 | Run Name | 핵심 변경 | Timeout% | 속도오차X | 토크포화 | 결과 |
|---|------|----------|----------|----------|----------|---------|------|
"""
        content = header + new_row + '\n'

    with open(log_path, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"  실험 로그 업데이트: {log_path}")


# ============================================================
# 11. Git Commit & Push
# ============================================================
def git_commit_push(exp_id, run_name, overall_judge, no_push=False):
    """git add → commit → push"""
    try:
        # 전체 변경사항 스테이징
        subprocess.run(['git', 'add', '-A'], cwd=PROJECT_ROOT, check=True)

        # 커밋 메시지
        result_tag = "PASS" if "PASS" in overall_judge else "FAIL"
        msg = f"[exp{exp_id:03d}] {run_name} — {result_tag}"

        subprocess.run(
            ['git', 'commit', '-m', msg],
            cwd=PROJECT_ROOT, check=True
        )
        print(f"  Git commit: {msg}")

        if not no_push:
            result = subprocess.run(
                ['git', 'push'],
                cwd=PROJECT_ROOT, capture_output=True, text=True
            )
            if result.returncode == 0:
                print(f"  Git push 완료!")
            else:
                print(f"  Git push 실패: {result.stderr}")
                print(f"  수동으로 'git push' 해주세요.")
        else:
            print(f"  Git push 건너뜀 (--no-push)")

    except subprocess.CalledProcessError as e:
        print(f"  Git 에러: {e}")
        print(f"  수동으로 커밋해주세요.")
    except FileNotFoundError:
        print(f"  Git이 설치되지 않았습니다.")


# ============================================================
# MAIN
# ============================================================
def main():
    args = parse_args()

    print("\n" + "=" * 60)
    print("  SpotMicro RL 실험 보고서 자동 생성")
    print("=" * 60)

    # --- 실험 ID ---
    if args.exp_id:
        exp_id = int(args.exp_id)
    else:
        exp_id = get_next_experiment_id(EXPERIMENTS_DIR)
    print(f"\n  실험 ID: exp{exp_id:03d}")

    # --- 목적 ---
    purpose = get_purpose(args)
    print(f"  목적: {purpose}")

    # --- Diagnostic JSON ---
    print(f"\n[1/6] Diagnostic 데이터 로드...")
    diag_data = load_diagnostic_summary(args.task)

    # --- Tensorboard ---
    print(f"\n[2/6] Tensorboard 데이터 추출...")
    tb_data = extract_tensorboard_data(args.task)

    # --- 보고서용 그래프 저장 ---
    print(f"\n[3/6] 그래프 저장...")
    metrics = diag_data.get('metrics', {}) if diag_data else {}
    run_name = ''
    if tb_data:
        run_name = tb_data.get('run_name', '')
    elif diag_data:
        run_name = diag_data.get('run_name', '')

    safe_run = re.sub(r'[^\w\-]', '_', run_name) if run_name else 'unknown'
    exp_assets_dir = os.path.join(EXPERIMENTS_DIR, f"exp{exp_id:03d}_{safe_run}")
    os.makedirs(exp_assets_dir, exist_ok=True)

    tb_graphs = save_tensorboard_graphs(tb_data, exp_assets_dir)
    
    # 최종 모델 복사
    model_path = copy_final_model(args.task, exp_assets_dir)
    if model_path:
        print(f"    최종 모델 복사: {os.path.basename(model_path)}")

    # diagnostic 그래프 복사
    diag_graph_dir = os.path.join(DIAG_DIR, args.task)
    if os.path.exists(diag_graph_dir):
        import shutil
        for img_name in ['diagnostic_report.png', 'joint_detail.png', 'action_smoothness.png']:
            src = os.path.join(diag_graph_dir, img_name)
            if os.path.exists(src):
                shutil.copy2(src, os.path.join(exp_assets_dir, img_name))
                print(f"    복사: {img_name}")

    # --- Config Diff ---
    print(f"\n[4/6] Config 변경점 분석...")
    diff_text = get_config_diff()

    # --- 이전 실험 비교 ---
    prev_data = load_previous_experiment(EXPERIMENTS_DIR, exp_id)


    # --- 보고서 생성 ---
    print(f"\n[5/6] 보고서 생성...")
    report, md_filename, json_filename = generate_report(
        exp_id, purpose, diag_data, tb_data, diff_text,
        prev_data, tb_graphs, args.task
    )

    # 보고서 저장
    md_path = os.path.join(EXPERIMENTS_DIR, md_filename)
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write(report)
    print(f"    보고서: {md_path}")

    # JSON 사이드카 저장 (다음 실험 비교용)
    json_path = os.path.join(EXPERIMENTS_DIR, json_filename)
    sidecar = {
        'exp_id': exp_id,
        'run_name': run_name,
        'purpose': purpose,
        'timestamp': datetime.now().isoformat(),
        'metrics': metrics,
    }
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(sidecar, f, indent=2, ensure_ascii=False)

    # 실험 로그 업데이트
    overall_judge, _ = auto_judge(metrics, run_name)
    update_experiment_log(exp_id, run_name, purpose, metrics, overall_judge)

    # 임시 파일 정리
    if os.path.exists(PURPOSE_FILE):
        os.remove(PURPOSE_FILE)
        print(f"    .experiment_purpose.txt 삭제")

    # --- Git ---
    if not args.no_git:
        print(f"\n[6/6] Git commit & push...")
        git_commit_push(exp_id, run_name, overall_judge, no_push=args.no_push)
    else:
        print(f"\n[6/6] Git 건너뜀 (--no-git)")

    # --- 완료 ---
    print(f"\n{'=' * 60}")
    print(f"  ✅ 완료! 보고서: {md_path}")
    print(f"{'=' * 60}\n")


if __name__ == '__main__':
    main()
