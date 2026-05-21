# classic_control

SpotMicro용 open-loop 고전 제어 trot 보행 컨트롤러.

## 역할

`classic_control_node`는 RL policy를 사용하지 않고 body-frame 속도 명령을
12개 관절 target으로 변환한다. 유지보수를 쉽게 하기 위해 ROS 통신, 상태
전환, gait 생성, leg IK를 작은 모듈로 분리했다.

- `classic_control_node.py`: ROS 파라미터, subscription, publication,
  STAND/STANDUP/DWELL/WALK/SETTLE 상태 전환.
- `locomotion_common/SharedTrotReference`: diagonal trot foot trajectory와
  offset-link inverse kinematics. RL 재학습용 reference와 같은 구현을 쓴다.
- `math_utils.py`: 작은 수치 계산 helper.

## 인터페이스

Subscribe:

- `/control/cmd_vel/spot_01` (`geometry_msgs/msg/Twist`): 최종 속도 명령.
- `/control/actuator/joint_feedback` (`robot_interfaces/msg/JointFeedback`):
  사용 가능한 경우 transition 시작점으로 쓰는 최신 관절 위치.

Publish:

- `/control/classic_control/joint_target` (`robot_interfaces/msg/JointTarget`):
  `motion_manager`의 `CLASSIC` mode가 선택할 고전 제어 관절 target.
  message `mode`는 `JointTarget.MODE_CLASSIC` 값을 사용한다.

`motion_state`는 이 패키지에서 publish하지 않는다. 현재 시스템에서는
`actuator_bridge_node`가 같은 `/control/cmd_vel/spot_01` 명령을 구독해
STM command의 `motion_state`를 만들고, STM feedback 기반
`/localization/spot_motion`을 publish한다.

## 파라미터

기본 파라미터는 `config/classic_control.yaml`에 있다.

주요 그룹:

- command 제한과 deadband: `vx_*`, `vy_*`, `wz_*`, `*_deadband_*`
- 로봇 기구 치수: `upper_link_x_mm`, `upper_link_z_mm`, `lower_link_mm`,
  `body_height_mm_per_leg`, `leg_origin_x_m`, `leg_origin_y_m`, `shoulder_sign`
- gait 형태: `gait_period_sec`, `duty_factor`, `lift_z_mm_per_leg`,
  `default_foot_x_mm_per_leg`, `max_stride_x_mm`, `max_stride_y_mm`
- transition과 관절 안전 제한: `stand_dwell_sec`, `min_transition_sec`,
  `max_transition_sec`, `joint_min_rad`, `joint_max_rad`

## 실행

단독 실행:

```bash
ros2 launch classic_control classic_control.launch.py
```

motion manager에서 이 target을 선택하려면 `CLASSIC` mode를 사용한다:

```bash
ros2 topic pub /control/behavior/mode std_msgs/msg/String "data: 'CLASSIC'" --once
```

## 참고

이 노드는 `Twist.linear.x`, `Twist.linear.y`, `Twist.angular.z`를 이용해 전진,
후진, 좌우 이동, yaw 회전, 혼합 명령을 지원한다. 다만 현재 upstream의
`safety_supervisor_node` 경로에서 일부 축이 clamp되거나 제거될 수 있으므로,
실제로 이 노드까지 전달되는 최종 `/control/cmd_vel/spot_01` 값을 확인해야
한다.
