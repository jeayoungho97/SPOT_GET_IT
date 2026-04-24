import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import depthai as dai


class OakCameraDriverNode(Node):

    def __init__(self):
        super().__init__('oak_camera_driver_node')

        # 파라미터 선언
        self.declare_parameter('fps', 15)
        self.declare_parameter('rgb_width', 480)
        self.declare_parameter('rgb_height', 270)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('preview_width', 1280)
        self.declare_parameter('preview_height', 720)

        self.fps            = self.get_parameter('fps').value
        self.rgb_width      = self.get_parameter('rgb_width').value
        self.rgb_height     = self.get_parameter('rgb_height').value
        self.show_preview   = self.get_parameter('show_preview').value
        self.preview_width  = self.get_parameter('preview_width').value
        self.preview_height = self.get_parameter('preview_height').value

        # Publisher
        self.rgb_pub     = self.create_publisher(Image,           '/perception/camera/image_raw', 1)
        self.encoded_pub = self.create_publisher(CompressedImage, '/perception/camera/encoded',   1)

        self.bridge = CvBridge()

        self.pipeline = self._build_pipeline()
        self.pipeline.start()
        self.get_logger().info(
            f'OAK 카메라 시작 | {self.rgb_width}x{self.rgb_height} @ {self.fps}fps'
        )

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def _build_pipeline(self) -> dai.Pipeline:
        pipeline = dai.Pipeline()

        # RGB 카메라
        cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

        # RAW RGB 출력
        rgb_out = cam_rgb.requestOutput(
            (self.rgb_width, self.rgb_height),
            dai.ImgFrame.Type.BGR888p
        )
        self.rgb_queue = rgb_out.createOutputQueue(maxSize=1, blocking=False)

        # H.264 인코더 (width: 32의 배수, height: 8의 배수)
        enc_w = (self.rgb_width  + 31) // 32 * 32
        enc_h = (self.rgb_height +  7) //  8 *  8
        encoder = pipeline.create(dai.node.VideoEncoder)
        encoder.setDefaultProfilePreset(
            self.fps,
            dai.VideoEncoderProperties.Profile.MJPEG
        )
        encoder_input = cam_rgb.requestOutput(
            (enc_w, enc_h),
            dai.ImgFrame.Type.NV12
        )
        encoder_input.link(encoder.input)
        self.encoded_queue = encoder.bitstream.createOutputQueue(maxSize=1, blocking=False)

        return pipeline

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # RAW RGB publish
        rgb_data = self.rgb_queue.tryGet()
        if rgb_data is not None:
            frame = rgb_data.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = now
            msg.header.frame_id = 'oak_camera'
            self.rgb_pub.publish(msg)

        # H.264 인코딩 스트림 publish
        encoded_data = self.encoded_queue.tryGet()
        if encoded_data is not None:
            msg = CompressedImage()
            msg.header.stamp = now
            msg.header.frame_id = 'oak_camera'
            msg.format = 'jpeg'
            msg.data = encoded_data.getData().tobytes()
            self.encoded_pub.publish(msg)

        # 미리보기
        if self.show_preview and rgb_data is not None:
            import cv2
            preview = cv2.resize(rgb_data.getCvFrame(), (self.preview_width, self.preview_height))
            cv2.imshow('oak_camera_driver preview', preview)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakCameraDriverNode()
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
