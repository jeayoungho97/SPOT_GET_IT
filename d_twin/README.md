# d_twin

Isaac Sim 기반 디지털 트윈 구성 파일 모음.

ROS2(`/localization/robot/state`)로 수신한 로봇 위치를 Isaac Sim 내 Spot prim에 실시간 반영하고, 시뮬 로봇(sim_02, sim_03)의 이동을 시각화한다.

---

## 구조

```
d_twin/
├── scripts/          # Isaac Sim Script Node용 Python 스크립트
└── isaac_projects/   # Isaac Sim 씬 파일 및 실행 스크립트
```

## 하위 디렉토리 요약

### scripts/
Isaac Sim Action Graph의 Script Node에 연결되는 Python 스크립트.
ROS2 토픽 구독 및 prim 위치/방향 반영 로직 포함.
→ 상세 내용: [scripts/README.md](scripts/README.md)

### isaac_projects/
Isaac Sim 씬 파일(.usd)과 Isaac Sim 실행 스크립트(.sh) 관리.
→ 상세 내용: [isaac_projects/README.md](isaac_projects/README.md)

---

## 실행

**ROS2 노드 실행 (PC)**
```bash
ros2 launch global_path_manager global_path_manager.launch.py
ros2 launch sim_executor sim_executor.launch.py
```

**Isaac Sim 실행**
```bash
SPOT_GET_IT/d_twin/isaac_projects/launch_isaac.sh
```
- Play → ScriptNode Warning → **Yes**

---

## 환경

| 항목 | 내용 |
|------|------|
| Isaac Sim | 5.1.0 |
| Python (Isaac Sim 내장) | 3.11.13 |
| ROS2 | Humble |
| robot_interfaces 빌드 | `robot_ws_311/` (Python 3.11 전용) |
