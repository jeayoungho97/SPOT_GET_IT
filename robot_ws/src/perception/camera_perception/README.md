# camera_perception

RGB 스트림을 수신하여 YOLOv11 + TensorRT 기반 인명 탐지를 수행하고 결과를 publish합니다.

> ⚠️ 실행 전 TensorRT 엔진 파일 배포가 필요합니다. [엔진 파일 배포](#엔진-파일-배포) 섹션을 반드시 확인하십시오.

---

## 역할

- `/perception/camera/image_raw` subscribe
- YOLOv11n + TensorRT FP16 추론
- `/perception/camera/victim_detection` publish

---

## 패키지 구조

```
camera_perception/
├── package.xml
├── setup.py
├── setup.cfg
├── camera_perception/
│   ├── __init__.py
│   ├── camera_perception_node.py
│   └── trt_inference.py
├── launch/
│   └── camera_perception.launch.py
├── config/
│   └── camera_perception.param.yaml
└── README.md
```

---

## Subscribe

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/perception/camera/image_raw` | `sensor_msgs/Image` | RAW BGR 스트림 (oak_camera_driver 발행) |

## Publish

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/perception/camera/victim_detection` | `robot_interfaces/VictimDetection` | 인명 탐지 결과 |

---

## 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `engine_path` | `/home/jetson/models/best.engine` | TensorRT 엔진 파일 경로 |
| `conf_threshold` | 0.25 | 탐지 confidence 임계값 |
| `iou_threshold` | 0.45 | NMS IOU 임계값 |
| `infer_size` | 480 | 추론 입력 크기 (정사각형) |
| `show_preview` | false | imshow 미리보기 활성화 |
| `preview_width` | 480 | 미리보기 창 너비 |
| `preview_height` | 270 | 미리보기 창 높이 |

---

## 의존성

| 패키지 | 비고 |
|--------|------|
| `rclpy` | ROS2 Humble |
| `sensor_msgs` | |
| `cv_bridge` | |
| `robot_interfaces` | VictimDetection 메시지 |
| `tensorrt` | |
| `opencv-python` | |

---

## 빌드 및 실행

```bash
cd ~/robot_ws
colcon build --packages-select camera_perception
source install/setup.bash
ros2 launch camera_perception camera_perception.launch.py
```

### 토픽 확인

```bash
ros2 topic echo /perception/camera/victim_detection
```

---

## 엔진 파일 배포

TensorRT 엔진 파일은 `ai_training/vision/models/best.engine`에 있습니다.
`camera_perception.param.yaml`의 `engine_path`가 `/home/jetson/models/best.engine`을 가리키므로 배포 전 아래 명령어를 실행하십시오.

```bash
mkdir -p /home/jetson/models
cp ~/vision_test/SPOT_GET_IT/ai_training/vision/models/best.engine /home/jetson/models/best.engine
```

---

## 참고

- bbox 좌표는 정규화(0~1) 좌표. 기준 해상도는 `oak_camera_driver.param.yaml`의 `rgb_width`, `rgb_height`
- 전처리는 Letterbox 방식 적용 (비율 유지, 회색 패딩 114)
- `confidence` 필드는 개발 중 튜닝 및 디버깅 용도로 포함. 배포 시 제거 예정
