# sim_executor

## 역할

시뮬 로봇(sim_02, sim_03)을 global path를 따라 이동시키고 `/localization/robot/state`를 발행한다.

Isaac Sim과 GUI는 이 토픽을 구독하여 위치를 표출한다.

---

## 패키지 구조

```
sim_executor/
├── sim_executor/
│   ├── __init__.py
│   └── sim_executor_node.py
├── config/
│   └── sim_executor.param.yaml
├── launch/
│   └── sim_executor.launch.py
├── resource/
│   └── sim_executor
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Subscribe

| 토픽 | 타입 | QoS |
|------|------|-----|
| `/planning/global_path/sim_02` | `nav_msgs/Path` | RELIABLE, TRANSIENT_LOCAL |
| `/planning/global_path/sim_03` | `nav_msgs/Path` | RELIABLE, TRANSIENT_LOCAL |

## Publish

| 토픽 | 타입 | QoS |
|------|------|-----|
| `/localization/robot/state` | `robot_interfaces/RobotLocalization` | RELIABLE |

---

## 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `robot_id` | `2` | 로봇 식별자 (2: sim_02 / 3: sim_03) |
| `start_x` | `2.0` | 출발 x (m) — launch에서 로봇별로 주입 |
| `start_y` | `2.0` | 출발 y (m) — launch에서 로봇별로 주입 |
| `speed` | `0.1` | 이동 속도 (m/s) |
| `arrival_dist` | `0.7` | waypoint 도착 판정 거리 (m) |
| `tick_rate` | `20.0` | 위치 갱신 주기 (Hz) |
| `frame_id` | `"map"` | header frame_id |

---

## 빌드

```bash
colcon build --packages-select sim_executor
source install/setup.bash
```

## 실행

```bash
ros2 launch sim_executor sim_executor.launch.py
```

> `global_path_manager`가 먼저 실행되어 있어야 한다.

## 위치 확인

```bash
ros2 topic echo /localization/robot/state
```
