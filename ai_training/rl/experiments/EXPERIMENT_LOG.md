# SpotMicro RL 실험 로그

| # | 날짜 | Run Name | 핵심 변경 | Timeout% | 속도오차X | 토크포화 | 결과 |
|---|------|----------|----------|----------|----------|---------|------|
| 001 | 04/24 | spotmicro_v1_0_ik_tracking | V1.0: default IK 동작을 모방하여 보행, 환경에 따른 변수를... | 100.0% | 0.103 | 16.4% | ✅ |
| 002 | 04/24 | spotmicro_v1_1_0_lin_vel_improve | V1.1.0: 직선속도 추종 정도 향상을 위한 reward 가중치 및 a... | 100.0% | 0.085 | 16.0% | ✅ |
| 003 | 04/24 | spotmicro_v1_1_1_lin_vel_improve | V1.1.1: 직선속도 추종 정도는 유지하며 몸체의 흔들림 감소 | 100.0% | 0.082 | 14.2% | ✅ |
| 004 | 04/24 | spotmicro_v1_1_2_orientation_improve | V1.1.2: 추종을 유지하며 줄 수 있는 페널티 수치 확인 | 100.0% | 0.087 | 12.7% | ✅ |
| 005 | 04/24 | spotmicro_v1_1_3_vel_improve | V1.1.3: 직선속도, 각속도 추종 오차 최소화 | 81.5% | 0.089 | 47.5% | ❌ |
| 006 | 04/27 | spotmicro_v1_1_4_sigma_desc | V1.1.4: tracking sigma 조정으로 민감도를 올림, 속도 ... | 100.0% | 0.060 | 13.6% | ✅ |
| 007 | 04/27 | spotmicro_v2_0_DR_friction | V2.0: 도메인 랜덤화 적용. 우선 지면 마찰계수만 적용 | 100.0% | 0.069 | 12.8% | ✅ |
| 008 | 04/27 | spotmicro_v2_1_DR_mass | V2.0: 질량 랜덤화 적용하여 오차와 안정성을 유지하는지 확인 | 100.0% | 0.063 | 14.8% | ✅ |
| 009 | 04/27 | spotmicro_v2_2_DR_external_push | V2.2: 외부의 충격량 적용. | 100.0% | 0.070 | 14.4% | ✅ |
| 010 | 04/27 | spotmicro_v2_3_DR_sensor_noise | V2.3: 센서 노이즈 값 추가. | 99.2% | 0.077 | 12.9% | ✅ |
| 011 | 04/27 | spotmicro_v2_3_1_DR_sensor_noise | V2.3.1: sigma값 낮춰서 노이즈 환경에서 성능이 좋아지는지 확인... | 100.0% | 0.053 | 14.3% | ✅ |
| 012 | 04/27 | spotmicro_v2_3_2_DR_sensor_noise | V2.3.2: 각속도 추종 reward를 직선속도 추종 reward랑 비... | 100.0% | 0.051 | 20.0% | ✅ |
| 013 | 04/27 | spotmicro_v2_4_DR_servo_delay | V2.4: 실제로 서보모터 응답의 지연을 이전 step의 액션을 적용하는... | 100.0% | 0.061 | 12.9% | ✅ |
| 014 | 04/27 | spotmicro_v2_5_stand_still | V2.5: 속도 범위를 0.0 포함하는 범위로 수정, 가만히 있는 동작 ... | 5.7% | 0.156 | 31.0% | ❌ |
| 015 | 04/27 | spotmicro_v2_5_1_stand_still | V2.5.1: gait 진행을 속도 비례로 수정, 정지 명령 때 trot... | 100.0% | 0.042 | 12.4% | ✅ |
| 016 | 04/28 | spotmicro_v2_5_2_stand_turn | V2.5.2: 제자리에서 회전할 수 있도록 설정 | 100.0% | 0.045 | 15.9% | ✅ |
| 017 | 04/28 | spotmicro_v2_6_reset_state | V2.6: 리셋 상태 랜덤화 | 96.2% | 0.053 | 18.2% | ✅ |
| 018 | 04/28 | spotmicro_v2_7_stab_ang_improve | V2.7: 나빠진 안정성, 각속도 오차 성능 향상 | 97.7% | 0.049 | 16.3% | ✅ |
| 019 | 04/28 | spotmicro_v2_7_1_stab_ang_improve | V2.7.1: 나빠진 안정성, 각속도 오차 성능 향상, iteration... | 100.0% | 0.047 | 18.1% | ✅ |
| 020 | 04/28 | spotmicro_v2_8_ang_shoulder | V2.8: ik shoulder 관절 목표값 설정, 회전 오차 감소하는지... | 5.7% | 0.077 | 22.2% | ❌ |
| 021 | 04/28 | spotmicro_v2_9_angle_weight | V2.9: tracking_ik의 관절 각도 가중치 설정, 어깨 관절에 ... | 98.5% | 0.052 | 28.3% | ✅ |
| 022 | 04/28 | spotmicro_v2_10_ang_improve | V2.10: 가중치 복구, trot contact reward 제자리 회... | 98.5% | 0.047 | 17.6% | ✅ |
| 023 | 04/28 | spotmicro_v2_11_resample_mode | V2.11: 여러 경우의 동작을 mode로 구분하여 학습시킴 | 96.2% | 0.074 | 37.2% | ✅ |
| 024 | 04/28 | spotmicro_v3_0_change_urdf | V3.0: urdf 실제 무게 반영 | 97.7% | 0.053 | 33.8% | ✅ |
| 025 | 04/28 | spotmicro_v3_1_PD_change | V3.1: urdf 변경으로 인한 변경점을 보정하기 위한 PD 변경. | 92.1% | 0.043 | 18.1% | ✅ |
| 026 | 04/29 | spotmicro_v3_1_1_urdf_fix | V3.1.1: urdf inertia 오류 수정. | 98.5% | 0.050 | 20.5% | ✅ |
| 027 | 04/29 | spotmicro_v3_1_2_PD_adjust | V3.1.2: 수정했던 PD를 원복. | 92.1% | 0.050 | 34.3% | ✅ |
| 028 | 04/29 | spotmicro_v3_1_3_PD_adjust | V3.1.3: PD수치 조정을 통해 토크 포화 보정 시도. | 93.4% | 0.048 | 19.0% | ✅ |
| 029 | 04/29 | spotmicro_v3_2_noise_order | V3.2: observation 조정으로 인한 노이즈 함수 순서 조정 처... | 96.2% | 0.046 | 20.3% | ✅ |
| 030 | 04/29 | spotmicro_v3_3_ik_desc | V3.3: IK 구조가 토크를 과도하게 먹는 구조인지 확인하기 위한 IK... | 94.8% | 0.044 | 24.7% | ✅ |
| 031 | 04/29 | spotmicro_v3_3_1_torque_improve | V3.3.1: torque 페널티 증가시켜 leg, foot 토크 완화 ... | 97.0% | 0.047 | 19.2% | ✅ |
| 032 | 04/29 | spotmicro_v3_3_2_torque_improve | V3.3.2: torque  페널티 적당히 조절, leg, foot 토크... | 6.0% | 0.096 | 26.4% | ❌ |
| 033 | 05/04 | spotmicro_v3_4_ik_sigma_asc | V3.4: tracking_ik 의 sigma를 감소한 pd gain 값... | 99.2% | 0.041 | 22.2% | ✅ |
| 034 | 05/04 | spotmicro_v4_0_new_tracking_ik | V4.0: tracking_ik 의 방식을 action에 따른 rewar... | 94.8% | 0.053 | 18.8% | ✅ |
| 035 | 05/04 | spotmicro_v4_0_1_shoulder_weight | V4.0.11: shoulder 관절 weight를 0.9로 변경. 좀 ... | 96.2% | 0.033 | 23.3% | ✅ |
| 036 | 05/04 | spotmicro_v4_1_new_IK | V4.1: IK를 2D에서 quasi-3D로 변경. | 98.5% | 0.032 | 24.7% | ✅ |
| 037 | 05/04 | spotmicro_v4_2_height_reward | V4.2: 높이 reward 추가하여 뒷다리 토크 문제 해결 시도 | 90.8% | 0.032 | 22.4% | ✅ |
| 038 | 05/04 | spotmicro_v4_3_body_height | V4.3: IK의 body height를 낮춰 토크 포화 감소 시도 | 49.6% | 0.039 | 25.0% | ❌ |
| 039 | 05/04 | spotmicro_v4_4_pitch_z | V4.4: pitch/roll 에 따라 leg의 z값 보정, 균형 맞춰서... | 87.1% | 0.037 | 21.3% | ✅ |
| 040 | 05/06 | spotmicro_v4_4_pitch_z | V4.5: IK를 좀 덜 공격적이게 변경, 추종도 낮춰서 토크 포화 해결... | 74.9% | 0.035 | 19.4% | ❌ |
