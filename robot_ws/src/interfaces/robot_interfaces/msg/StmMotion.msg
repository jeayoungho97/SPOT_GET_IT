# STM motion source data for gait-based odometry

std_msgs/Header header

uint32 timestamp_ms         # STM 기준 패킷 생성 시간
uint32 seq                  # 패킷 순번

uint8 motion_state          # 현재 동작 상태

float32 gait_phase          # 현재 보행 Cycle 진행률
uint32 gait_cycle_count     # 완료된 Gait cycle 누적 수

float32 imu_yaw_rad         # 현재 로봇의 yaw 방향
float32 gyro_z_rad_s        # z축 각속도

# Motion State 
uint8 STOP=0                # 정지
uint8 WALK_FORWARD=1        # 전진
uint8 WALK_BACKWARD=2       # 후진
uint8 TURN_LEFT=3           # 좌회전
uint8 TURN_RIGHT=4          # 우회전
uint8 STRAFE_LEFT=5         # 왼쪽 횡이동
uint8 STRAFE_RIGHT=6        # 오른쪽 횡이동
uint8 TRANSITION=7          # 동작 전환중
uint8 UNKNOWN=8             # 상태불명