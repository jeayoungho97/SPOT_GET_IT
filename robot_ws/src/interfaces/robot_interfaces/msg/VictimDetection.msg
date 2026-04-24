# robot_interfaces/msg/VictimDetection.msg
# 인명 탐지 결과 메시지
# Publisher  : camera_perception_node
# Subscriber : decision, network

std_msgs/Header header

float32 bbox_x          # bbox 좌측 상단 x (정규화, 0~1)
float32 bbox_y          # bbox 좌측 상단 y (정규화, 0~1)
float32 bbox_w          # bbox 너비 (정규화, 0~1)
float32 bbox_h          # bbox 높이 (정규화, 0~1)
float32 distance        # 추정 거리 (m) - Stereo 미포함으로 현재 0.0 고정
float32 confidence      # 탐지 신뢰도 (개발 중 사용, 배포 시 제거 예정)
