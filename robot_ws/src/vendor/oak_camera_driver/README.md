# oak_camera_driver

OAK-D Lite 카메라 드라이버 노드. RGB RAW 스트림과 H.264 인코딩 스트림을 ROS2 토픽으로 publish합니다.

---

## 역할

- OAK-D Lite 디바이스 직접 점유 및 제어
- RGB RAW 스트림 publish → `camera_perception` 추론용
- H.264 인코딩 스트림 publish → `image_stream_sender` 관제 송출용

---

## 패키지 구조

```
oak_camera_driver/
├── package.xml
├── setup.py
├── setup.cfg
├── oak_camera_driver/
│   ├── __init__.py
│   └── oak_camera_driver_node.py
├── launch/
│   └── oak_camera_driver.launch.py
├── config/
│   └── oak_camera_driver.param.yaml
└── README.md
```

---

## Subscribe

없음 (카메라 직접 제어)

## Publish

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/vendor/camera/image_raw` | `sensor_msgs/Image` | RAW BGR 스트림 |
| `/vendor/camera/encoded` | `sensor_msgs/CompressedImage` | MJPEG 인코딩 스트림 (format=jpeg) |

---

## 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `fps` | 15 | 카메라 프레임 레이트 |
| `rgb_width` | 480 | RGB 출력 너비 (victim_detection bbox 정규화 기준) |
| `rgb_height` | 270 | RGB 출력 높이 (victim_detection bbox 정규화 기준) |
| `show_preview` | false | imshow 미리보기 활성화 |
| `preview_width` | 480 | 미리보기 창 너비 |
| `preview_height` | 270 | 미리보기 창 높이 |

---

## 의존성

| 패키지 | 버전 | 비고 |
|--------|------|------|
| `depthai` | v3.5.0 | **반드시 v3 사용. v2와 API 호환 불가** |
| `rclpy` | - | ROS2 Humble |
| `sensor_msgs` | - | |
| `cv_bridge` | - | |
| `opencv-python` | - | |

---

## 빌드 및 실행

```bash
cd ~/robot_ws
colcon build --packages-select oak_camera_driver
source install/setup.bash
ros2 launch oak_camera_driver oak_camera_driver.launch.py
```

### 토픽 확인

```bash
ros2 topic hz /perception/camera/image_raw
ros2 topic hz /perception/camera/encoded
```

---

## 참고

- MJPEG 인코딩 포맷 사용 중. 변경 필요 시 VideoEncoder Profile 및 `msg.format` 수정으로 전환 가능 (H.264, H.265)
- Stereo Depth는 OAK-D Lite 온보드 메모리 한계로 미포함. RGB + H.264 + Stereo 동시 운용 불가
