# global_path_manager

## 역할

시연장 맵을 기반으로 로봇별 출발지에서 수백 개의 후보 경로를 사전 생성하고, 로봇별로 경로를 선정해 발행한다.

노드는 발행 후에도 살아있어 TRANSIENT_LOCAL 캐시를 유지한다. 늦게 subscribe하는 노드도 경로를 즉시 수신할 수 있다.

---

## 패키지 구조

```
global_path_manager/
├── global_path_manager/
│   ├── __init__.py
│   ├── path_set.py                  # 경로 생성 / 유효성 / 선정 핵심 로직
│   └── global_path_manager_node.py  # ROS2 노드
├── config/
│   ├── map.yaml                     # 맵 경계 / 장애물 / 로봇별 출발·도착 좌표
│   └── global_path_manager.param.yaml
├── launch/
│   └── global_path_manager.launch.py
├── resource/
│   └── global_path_manager
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Subscribe

없음.

## Publish

| 토픽 | 타입 | QoS |
|------|------|-----|
| `/planning/global_path/robot_01` | `nav_msgs/Path` | RELIABLE, TRANSIENT_LOCAL |
| `/planning/global_path/sim_02` | `nav_msgs/Path` | RELIABLE, TRANSIENT_LOCAL |
| `/planning/global_path/sim_03` | `nav_msgs/Path` | RELIABLE, TRANSIENT_LOCAL |

> 토픽은 `map.yaml`의 `starts` 키 기준으로 자동 생성된다.

---

## 파라미터

### config/map.yaml

| 항목 | 설명 |
|------|------|
| `starts` | 로봇별 출발 좌표 |
| `end` | 공통 도착 좌표 |
| `lobby` | 커버리지 대상 구역 경계 (이동 가능 구역) |
| `waypoint_sampling` | 경유지 격자 샘플링 범위 및 간격 |
| `obstacles` | 장애물 목록 (id, x_min, x_max, y_min, y_max, clearance) |

### config/global_path_manager.param.yaml

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `frame_id` | `"map"` | Path header frame_id |
| `map_config_path` | `""` | 비워두면 패키지 기본값 사용 |

---

## 빌드

```bash
colcon build --packages-select global_path_manager
source install/setup.bash
```

## 실행

```bash
ros2 launch global_path_manager global_path_manager.launch.py
```

## 맵 변경 시

`config/map.yaml` 수정 후 재빌드 및 재실행.

## 경로 확인

```bash
ros2 topic echo --once /planning/global_path/robot_01
ros2 topic echo --once /planning/global_path/sim_02
ros2 topic echo --once /planning/global_path/sim_03
```
