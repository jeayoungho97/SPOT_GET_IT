# robot_ws

Jetson Orin Nano용 ROS2 Humble 워크스페이스.

## 빌드 도메인
- 배포판: ROS2 Humble Hawksbill
- 빌드 도구: colcon
- 타겟 OS: Ubuntu 22.04 (aarch64)

## 예정 구조 (팀 ROS2 컨벤션 기준)
- `src/vendor/` — 외부 드라이버 (원본 유지)
- `src/interfaces/robot_interfaces/` — 🔒 모든 msg/srv/action
- `src/common/` — 공통 유틸
- `src/perception/` — LiDAR, Camera, Fusion
- `src/localization/` — 측위
- `src/planning/` — 경로 계획
- `src/decision/` — 행동 결정
- `src/control/` — motion_controller, actuator_bridge
- `src/network/` — 관제 통신
- `src/bringup/` — 통합 실행
- `src/tests/`

## 빌드
```bash
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

담당: RL, LiDAR, Depth, 관제, 통신 (계층별 분담)