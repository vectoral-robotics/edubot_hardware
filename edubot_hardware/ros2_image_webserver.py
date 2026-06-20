#!/usr/bin/env python3
import threading
import time
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response
from rclpy.node import Node
from sensor_msgs.msg import Image


# ---------- ROS2 part ----------
class ImageBufferNode(Node):
    def __init__(self):
        super().__init__("image_webserver_node")

        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_stamp = 0.0

        self.sub = self.create_subscription(Image, self.image_topic, self.cb, 10)
        self.get_logger().info(f"Subscribing to: {self.image_topic}")

    def cb(self, msg: Image):
        try:
            # Convert ROS Image -> OpenCV (BGR)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Encode as JPEG
            ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                return

            with self.lock:
                self.latest_jpeg = jpg.tobytes()
                self.latest_stamp = time.time()
        except Exception as e:
            self.get_logger().warn(f"Failed converting/encoding image: {e}")


# ---------- Flask part ----------
def create_app(node: ImageBufferNode) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index():
        return """
        <html>
          <head><title>ROS2 Camera Stream</title></head>
          <body style="font-family: sans-serif;">
            <h2>ROS2 Camera Stream</h2>
            <p>MJPEG stream:</p>
            <img src="/stream" style="max-width: 100%; height: auto;" />
            <p>Single JPEG: <a href="/image">/image</a></p>
          </body>
        </html>
        """

    @app.get("/image")
    def image():
        with node.lock:
            data = node.latest_jpeg
        if data is None:
            return Response("No image received yet. Is cam2image running?", status=503)
        return Response(data, mimetype="image/jpeg")

    @app.get("/stream")
    def stream():
        def gen():
            boundary = "frame"
            while True:
                with node.lock:
                    data = node.latest_jpeg
                if data is None:
                    time.sleep(0.1)
                    continue

                yield (
                    b"--" + boundary.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n"
                )
                time.sleep(0.05)  # ~20 FPS max, je nach cam2image

        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def main():
    rclpy.init()
    node = ImageBufferNode()

    app = create_app(node)

    # ROS spin in background thread so Flask can run in main thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    host = node.get_parameter("host").get_parameter_value().string_value
    port = node.get_parameter("port").get_parameter_value().integer_value
    node.get_logger().info(f"Webserver running on http://{host}:{port} (open via your device IP)")

    try:
        # threaded=True allows concurrent requests
        app.run(host=host, port=port, threaded=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
