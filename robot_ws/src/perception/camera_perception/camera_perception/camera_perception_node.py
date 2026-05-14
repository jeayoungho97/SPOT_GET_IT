import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from camera_perception.trt_inference import TRTInference


class CameraPerceptionNode(Node):

    def __init__(self):
        super().__init__('camera_perception_node')

        # 파라미터 선언
        self.declare_parameter('engine_path',       '/home/jetson/models/best.engine')
        self.declare_parameter('conf_threshold',    0.7)
        self.declare_parameter('iou_threshold',     0.45)
        self.declare_parameter('infer_size',        480)
        self.declare_parameter('show_preview',      False)
        self.declare_parameter('preview_width',     480)
        self.declare_parameter('preview_height',    270)
        self.declare_parameter('robot_id',          'spot_01')
        self.declare_parameter('detect_consec_n',   5)        # N프레임 연속 탐지 기준

        engine_path          = self.get_parameter('engine_path').value
        conf_threshold       = self.get_parameter('conf_threshold').value
        iou_threshold        = self.get_parameter('iou_threshold').value
        self.infer_size      = self.get_parameter('infer_size').value
        self.show_preview    = self.get_parameter('show_preview').value
        self.preview_width   = self.get_parameter('preview_width').value
        self.preview_height  = self.get_parameter('preview_height').value
        self.robot_id        = self.get_parameter('robot_id').value
        self.detect_consec_n = self.get_parameter('detect_consec_n').value

        self.bridge          = CvBridge()

        # 연속 탐지 카운터
        self._consec_count    = 0
        self._person_detected = False  # 발견 상태 (중복 발행 방지)

        # TensorRT 추론 모듈 초기화
        self.get_logger().info(f'엔진 로드 중: {engine_path}')
        self.trt = TRTInference(
            engine_path    = engine_path,
            infer_size     = self.infer_size,
            conf_threshold = conf_threshold,
            iou_threshold  = iou_threshold,
        )
        self.get_logger().info('엔진 로드 완료')

        # Subscriber
        self.rgb_sub = self.create_subscription(
            Image,
            '/vendor/camera/image_raw',
            self.rgb_callback,
            1
        )

        # Publisher
        self.detected_pub = self.create_publisher(
            Bool, f'/perception/person_detected/{self.robot_id}', 1
        )

        self.get_logger().info(
            f'camera_perception_node 시작 | robot_id: {self.robot_id} '
            f'| 연속 탐지 기준: {self.detect_consec_n}프레임'
        )

    def rgb_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.trt.infer(frame)

        if not results:
            # 탐지 끊김 → True 상태였을 때만 False 1회 발행
            if self._person_detected:
                self._person_detected = False
                msg = Bool()
                msg.data = False
                self.detected_pub.publish(msg)
                self.get_logger().info(f'[person_detected] {self.robot_id} | False')
            self._consec_count = 0
            self.get_logger().debug('탐지 없음')
            return

        best = max(results, key=lambda x: x['conf'])
        self._consec_count += 1

        self.get_logger().info(
            f'탐지 | conf: {best["conf"]:.3f} '
            f'| 연속: {self._consec_count}/{self.detect_consec_n}'
        )

        # 5프레임 달성 시점에만 True 1회 발행
        if self._consec_count == self.detect_consec_n and not self._person_detected:
            self._person_detected = True
            msg = Bool()
            msg.data = True
            self.detected_pub.publish(msg)
            self.get_logger().info(
                f'[person_detected] {self.robot_id} | True | '
                f'{self.detect_consec_n}프레임 연속 탐지 확정'
            )

        # 미리보기
        if self.show_preview:
            import cv2
            bbox = best['bbox']
            vis  = frame.copy()
            h, w = vis.shape[:2]
            x1 = int(bbox[0] * w)
            y1 = int(bbox[1] * h)
            x2 = int((bbox[0] + bbox[2]) * w)
            y2 = int((bbox[1] + bbox[3]) * h)
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f'conf:{best["conf"]:.2f} [{self._consec_count}/{self.detect_consec_n}]',
                (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
            vis = cv2.resize(vis, (self.preview_width, self.preview_height))
            cv2.imshow('camera_perception', vis)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
