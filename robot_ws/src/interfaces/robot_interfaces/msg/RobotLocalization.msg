# robot_interfaces/msg/RobotLocalization.msg
# 로봇 위치 및 방향 정보
# Publisher  : localization_fusion (라이다 파트)
# Subscriber : perception/camera_perception, d_twin (SIM 파트)

std_msgs/Header header

uint8 robot_id      # 1: robot_01, 2: sim_02, 3: sim_03
float32 x           # 공통 좌표계 기준 x (m)
float32 y           # 공통 좌표계 기준 y (m)
float32 yaw         # 공통 좌표계 기준 yaw (rad)
