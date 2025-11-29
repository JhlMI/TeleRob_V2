#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import time
import threading


class VideoReceiver(Node):
    def __init__(self):
        super().__init__("video_receiver")

        print(">>> Nodo ROS2 creado")

        self.running = True

        print(">>> Lanzando thread de GStreamer…")
        self.thread = threading.Thread(target=self.video_loop)
        self.thread.daemon = True
        self.thread.start()

    def video_loop(self):
        print(">>> Thread GStreamer iniciado")

        try:
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
            print(">>> GStreamer importado correctamente")
        except Exception as e:
            print(f"!!! ERROR al importar GStreamer: {e}")
            return

        Gst.init(None)
        print(">>> GStreamer inicializado")

        pipeline_str = (
            'udpsrc port=5000 caps="application/x-rtp,encoding-name=H264,payload=96" ! '
            'rtph264depay ! avdec_h264 ! videoconvert ! '
            'video/x-raw,format=BGR ! appsink name=appsink0 emit-signals=false '
            'sync=false max-buffers=1 drop=true'
        )

        print(">>> Creando pipeline…")
        try:
            pipeline = Gst.parse_launch(pipeline_str)
            print(">>> Pipeline creado OK")
        except Exception as e:
            print(f"!!! ERROR creando pipeline: {e}")
            return

        appsink = pipeline.get_by_name("appsink0")
        if appsink is None:
            print("!!! ERROR appsink0 no encontrado")
            return

        print(">>> Iniciando pipeline PLAYING…")
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("!!! ERROR al poner pipeline en PLAYING")
            return

        last_time = 0.0
        print(">>> Loop de recepción iniciado (esperando video)…")

        while self.running:
            try:
                sample = appsink.emit("pull-sample")
            except Exception as e:
                print(f"!!! ERROR en pull-sample: {e}")
                continue

            if sample is None:
                continue

            print(">>> Frame recibido")

            buf = sample.get_buffer()
            caps = sample.get_caps()
            s = caps.get_structure(0)

            width = s.get_int("width")[1]
            height = s.get_int("height")[1]

            data = buf.extract_dup(0, buf.get_size())

            frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3)).copy()

            now = time.time()
            fps = 1.0 / (now - last_time) if last_time else 0
            last_time = now

            print(f">>> FPS: {fps:.1f}")

            cv2.imshow("Video RX", frame)
            cv2.waitKey(1)

        print(">>> Deteniendo pipeline…")
        pipeline.set_state(Gst.State.NULL)
        print(">>> Pipeline apagado")

    def destroy_node(self):
        print(">>> Apagando nodo…")
        self.running = False
        self.thread.join(timeout=1)
        print(">>> Thread terminado")
        super().destroy_node()


def main(args=None):
    print(">>> Inicializando ROS2…")
    rclpy.init(args=args)
    print(">>> ROS2 INIT OK")

    node = VideoReceiver()
    print(">>> Nodo VideoReceiver listo")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    print(">>> Nodo destruido")

    rclpy.shutdown()
    print(">>> ROS2 apagado")


if __name__ == "__main__":
    main()
