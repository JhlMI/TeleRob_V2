#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time


class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Pipeline GStreamer (RECEPTOR)
        pipeline = (
            "udpsrc port=5000 caps=application/x-rtp,encoding-name=H264,payload=96 ! "
            "rtph264depay ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "appsink drop=true sync=false max-buffers=1"
        )

        # Crear captura
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("❌ No se pudo abrir el stream")
            return

        # FPS
        self.last_time = time.time()
        self.fps = 0.0

        self.timer = self.create_timer(0.01, self.update)

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # ---- FPS ----
        now = time.time()
        dt = now - self.last_time
        if dt > 0:
            self.fps = 1.0 / dt
        self.last_time = now

        # ---- Dibujar FPS ----
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 440),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)

        # Mostrar
        cv2.imshow("Video RX", frame)
        cv2.waitKey(1)

        # Publicar ROS2
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
