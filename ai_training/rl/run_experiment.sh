#!/bin/bash
# ============================================================
# run_experiment.sh
# SpotMicro RL 실험 원커맨드 자동화
# ============================================================
#
# 사용법:
#
#   === 풀 파이프라인 (학습 → play 시청 → 체크리스트 → 진단 → 보고서 → git) ===
#   ./run_experiment.sh --purpose "Step4: torques 페널티 추가"
#
#   === 학습 건너뛰기 (이미 학습 완료) ===
#   ./run_experiment.sh --skip-train --purpose "Step4: torques 페널티 추가"
#
#   === 육안 확인 건너뛰기 (수치만 기록) ===
#   ./run_experiment.sh --skip-visual --purpose "빠른 테스트"
#
#   === 학습만 (보고서는 나중에) ===
#   ./run_experiment.sh --only-train --purpose "Step4: torques 페널티 추가"
#
# 환경 변수:
#   LEGGED_GYM_DIR  - legged_gym 루트 경로 (기본: 현재 디렉토리)
#

set -e

# ---- 기본값 ----
TASK="spotmicro_test"
PURPOSE=""
SKIP_TRAIN=false
ONLY_TRAIN=false
NO_PUSH=false
EXTRA_TRAIN_ARGS=""
LEGGED_GYM_DIR="${LEGGED_GYM_DIR:-./legged_gym}"

# ---- 인자 파싱 ----
while [[ $# -gt 0 ]]; do
    case $1 in
        --task)
            TASK="$2"; shift 2 ;;
        --purpose)
            PURPOSE="$2"; shift 2 ;;
        --skip-train)
            SKIP_TRAIN=true; shift ;;
        --only-train)
            ONLY_TRAIN=true; shift ;;
        --no-push)
            NO_PUSH=true; shift ;;
        --train-args)
            EXTRA_TRAIN_ARGS="$2"; shift 2 ;;
        -h|--help)
            head -25 "$0" | tail -20
            exit 0 ;;
        *)
            echo "알 수 없는 옵션: $1"; exit 1 ;;
    esac
done

# ---- 목적 확인 ----
if [ -z "$PURPOSE" ]; then
    echo ""
    echo "  ⚠ --purpose 가 지정되지 않았습니다."
    echo "  이번 실험의 목적을 한 줄로 입력하세요:"
    read -r -p "  목적: " PURPOSE
    echo ""
fi
[ -z "$PURPOSE" ] && PURPOSE="(목적 미작성)"

echo "$PURPOSE" > "${LEGGED_GYM_DIR}/.experiment_purpose.txt"

echo ""
echo "=========================================="
echo "  SpotMicro RL 실험 자동화"
echo "=========================================="
echo "  Task:    $TASK"
echo "  목적:    $PURPOSE"
echo "  학습:    $([ "$SKIP_TRAIN" = true ] && echo '건너뜀' || echo '실행')"
echo "=========================================="
echo ""

# ============================================================
# 1. 학습
# ============================================================
if [ "$SKIP_TRAIN" = false ]; then
    echo "[1/3] 학습 시작..."
    echo "----------------------------------------------"
    cd "$LEGGED_GYM_DIR"
    python legged_gym/scripts/train.py --task="$TASK" $EXTRA_TRAIN_ARGS
    echo ""
    echo "  ✅ 학습 완료!"
    echo ""
fi

if [ "$ONLY_TRAIN" = true ]; then
    echo "  --only-train 모드. 보고서를 나중에 생성하려면:"
    echo "    ./run_experiment.sh --skip-train --purpose \"$PURPOSE\""
    exit 0
fi

# ============================================================
# 3. Diagnostic 실행 (headless — 수치 수집)
# ============================================================
echo ""
echo "[2/3] Diagnostic 실행 (수치 수집)..."
echo "----------------------------------------------"

cd "$LEGGED_GYM_DIR"
python legged_gym/scripts/play_diagnostic.py --task="$TASK" 2>&1 | tee /tmp/diagnostic_output.txt

echo ""
echo "  ✅ Diagnostic 완료!"
echo ""

# ============================================================
# 3. 보고서 생성 + Git
# ============================================================
echo "[3/3] 보고서 생성 + Git..."
echo "----------------------------------------------"

REPORT_ARGS="--task=$TASK"
[ "$NO_PUSH" = true ] && REPORT_ARGS="$REPORT_ARGS --no-push"

cd "$LEGGED_GYM_DIR"
python experiment_report.py $REPORT_ARGS --purpose "$PURPOSE"

echo ""
echo "=========================================="
echo "  ✅ 전체 파이프라인 완료!"
echo "=========================================="
echo ""
echo "  💡 AI 분석이 필요하면:"
echo "     생성된 보고서(.md)를 Claude 채팅에 붙여넣으세요."
echo ""
