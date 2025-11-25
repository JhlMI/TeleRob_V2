import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import subprocess


class VideoStreamer(Node):
    def __init__(self):
        super().__init__('video_streamer')

        # Estado del streaming
        self.streaming = False
        self.process = None

        # Suscripción al comando desde PS4
        self.create_subscription(Float32MultiArray, '/stream_cmd', self.cmd_callback, 10)

        self.get_logger().info("VideoStreamer listo — Toggle con SHARE")

    def start_stream(self):
        if self.streaming:
            return

        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=640,height=480,framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
            "rtph264pay pt=96 ! "
            "udpsink host=192.168.100.150 port=5000 sync=false async=false"
        )

        self.process = subprocess.Popen(pipeline, shell=True)
        self.streaming = True
        self.get_logger().info("📡 Streaming ACTIVADO")

    def stop_stream(self):
        if not self.streaming:
            return

        if self.process:
            self.process.terminate()
            self.process = None

        self.streaming = False
        self.get_logger().info("🛑 Streaming DESACTIVADO")

    def cmd_callback(self, msg):
        share = int(msg.data[0])

        # Si SHARE == 1 → TOGGLE
        if share == 1:
            if self.streaming:
                self.stop_stream()
            else:
                self.start_stream()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
