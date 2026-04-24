import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_interfaces.msg import VictimDetection

from camera_perception.trt_inference import TRTInference


class CameraPerceptionNode(Node):

    def __init__(self):
        super().__init__('camera_perception_node')

        # 파라미터 선언
        self.declare_parameter('engine_path',      '/home/jetson/models/best.engine')
        self.declare_parameter('conf_threshold',   0.25)
        self.declare_parameter('iou_threshold',    0.45)
        self.declare_parameter('infer_size',       480)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('preview_width', 480)
        self.declare_parameter('preview_height', 270)

        engine_path         = self.get_parameter('engine_path').value
        conf_threshold      = self.get_parameter('conf_threshold').value
        iou_threshold       = self.get_parameter('iou_threshold').value
        self.infer_size     = self.get_parameter('infer_size').value
        self.show_preview    = self.get_parameter('show_preview').value
        self.preview_width   = self.get_parameter('preview_width').value
        self.preview_height  = self.get_parameter('preview_height').value

        self.bridge         = CvBridge()

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
            '/perception/camera/image_raw',
            self.rgb_callback,
            1
        )
        # Publisher
        self.victim_pub = self.create_publisher(VictimDetection, '/perception/camera/victim_detection', 1)

        self.get_logger().info('camera_perception_node 시작')

    def rgb_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.trt.infer(frame)

        if not results:
            self.get_logger().debug('탐지 없음')
            return

        # 가장 confidence 높은 탐지 결과 선택
        best = max(results, key=lambda x: x['conf'])
        bbox = best['bbox']  # [x, y, w, h] 정규화

        self.get_logger().info(
            f'탐지 | bbox: [{bbox[0]:.3f}, {bbox[1]:.3f}, {bbox[2]:.3f}, {bbox[3]:.3f}] '
            f'conf: {best["conf"]:.3f}'
        )

        # 미리보기
        if self.show_preview:
            import cv2
            vis = frame.copy()
            h, w = vis.shape[:2]
            x1 = int(bbox[0] * w)
            y1 = int(bbox[1] * h)
            x2 = int((bbox[0] + bbox[2]) * w)
            y2 = int((bbox[1] + bbox[3]) * h)
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis, f'conf:{best["conf"]:.2f}',
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            vis = cv2.resize(vis, (self.preview_width, self.preview_height))
            cv2.imshow('camera_perception', vis)
            cv2.waitKey(1)

        msg = VictimDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.bbox_x     = bbox[0]
        msg.bbox_y     = bbox[1]
        msg.bbox_w     = bbox[2]
        msg.bbox_h     = bbox[3]
        msg.distance   = 0.0
        msg.confidence = best['conf']
        self.victim_pub.publish(msg)

    def _estimate_distance(self, bbox: list) -> float:
        """거리 추정 - Stereo 미포함으로 현재 0.0 고정"""
        return 0.0


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
