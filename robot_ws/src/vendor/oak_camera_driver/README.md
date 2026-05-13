# oak_camera_driver

## 역할

OAK-D Lite 카메라를 직접 점유하여 RGB RAW 스트림과 MJPEG 인코딩 스트림을 ROS2 토픽으로 publish한다.

카메라 제어 로직을 이 패키지에 격리함으로써 `camera_perception`과 `image_stream_sender`가 동일한 스트림을 독립적으로 subscribe할 수 있다.

---

## Publish 토픽

| 토픽 | 타입 | fps | 설명 |
|------|------|-----|------|
| `/vendor/camera/image_raw` | `sensor_msgs/Image` | `image_raw_fps` (기본 5) | RAW BGR 스트림. perception 전용 |
| `/vendor/camera/encoded` | `sensor_msgs/CompressedImage` | `fps` (기본 15) | MJPEG 인코딩 스트림. 영상 송출 전용 |

> `image_raw`는 Jetson 부하 경감을 위해 `image_raw_fps`로 스킵 publish한다.
> `encoded`는 캡처 fps 그대로 publish한다.

---

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `fps` | int | 15 | 카메라 캡처 fps. `encoded` publish fps와 동일 |
| `image_raw_fps` | int | 5 | `image_raw` publish fps. `fps`보다 크면 `fps`로 클램프 |
| `rgb_width` | int | 480 | RGB 출력 너비. `camera_perception` bbox 정규화 기준 해상도 |
| `rgb_height` | int | 270 | RGB 출력 높이. `camera_perception` bbox 정규화 기준 해상도 |
| `show_preview` | bool | false | imshow 미리보기 활성화. 원격 접속 환경에서는 false 유지 |
| `preview_width` | int | 480 | 미리보기 창 너비 |
| `preview_height` | int | 270 | 미리보기 창 높이 |

---

## 빌드 및 실행

```bash
cd ~/robot_ws
colcon build --packages-select oak_camera_driver
source install/setup.bash
```

```bash
ros2 launch oak_camera_driver oak_camera_driver.launch.py
```

---

## 토픽 확인

```bash
ros2 topic hz /vendor/camera/image_raw
ros2 topic hz /vendor/camera/encoded
```

---

## 의존성

| 패키지 | 용도 |
|--------|------|
| `depthai` | OAK-D Lite SDK (v3.5.0). **v2와 API 호환 불가, 반드시 v3 사용** |
| `rclpy` | ROS2 Python 클라이언트 |
| `sensor_msgs` | Image, CompressedImage 메시지 타입 |
| `cv_bridge` | OpenCV ↔ ROS2 이미지 변환 |
| `opencv-python` | 이미지 처리 |

---

## 설계 결정

**depthai-ros 공식 드라이버 미사용** : DepthAI v3 API 기준 공식 ROS2 드라이버 지원 여부가 불확실하고, SDK 기반 동작이 이미 검증되어 SDK 직접 사용 방식을 채택했다.

**Stereo Depth 미포함** : OAK-D Lite 온보드 메모리 한계로 RGB + MJPEG + Stereo 동시 운용이 불가능하다. 영상 송출 우선순위가 거리 추정보다 높다고 판단하여 Stereo를 제외했다.

**image_raw와 encoded fps 분리** : `image_raw`는 perception 추론용으로 Jetson 부하 경감을 위해 낮은 fps로 publish하고, `encoded`는 영상 송출용으로 캡처 fps 그대로 유지한다.

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `X_LINK_DEVICE_NOT_FOUND` | OAK-D Lite 미연결 또는 udev rules 미설정 | udev rules 추가 후 재연결 |
| H.264 인코더 해상도 오류 | H264는 width 32의 배수, height 8의 배수 필요 | 인코더 입력 해상도 자동 보정 (현재 코드에 적용됨) |
| Stereo 메모리 부족 | RGB + MJPEG + Stereo 동시 운용 불가 | Stereo 제거 |
| 원격 접속 끊김 | imshow 부하 | `show_preview: false` 유지 |
