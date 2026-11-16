#!/usr/bin/env python3
"""Bridge Unitree Go2 built-in front camera (VideoHub RPC) to ROS 2 Image topics.

Publishes:
  - /go2/front_camera/image_raw/compressed (sensor_msgs/msg/CompressedImage, JPEG)
  - /go2/front_camera/image_raw (sensor_msgs/msg/Image, bgr8, published on demand)
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from unitree_api.msg import Request, Response

VIDEO_API_ID_GET_IMAGE_SAMPLE = 1001


class Go2FrontCamNode(Node):
    """Polls Go2 VideoHub JPEG samples and publishes standard ROS 2 image topics."""

    def __init__(self) -> None:
        super().__init__("go2_front_cam_node")

        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("raw_width", 640)
        self.declare_parameter("raw_height", 360)
        self.declare_parameter("frame_id", "front_camera")

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._raw_width = int(self.get_parameter("raw_width").value)
        self._raw_height = int(self.get_parameter("raw_height").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_req = self.create_publisher(Request, "/api/videohub/request", 10)
        self._sub_resp = self.create_subscription(
            Response, "/api/videohub/response", self._on_response, 10
        )
        self._pub_compressed = self.create_publisher(
            CompressedImage, "/go2/front_camera/image_raw/compressed", img_qos
        )
        self._pub_raw = self.create_publisher(
            Image, "/go2/front_camera/image_raw", img_qos
        )

        self._in_flight_id = None
        self._in_flight_at = 0.0
        self._frame_count = 0

        period = 1.0 / max(rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "Go2 front camera bridge started: "
            "/go2/front_camera/image_raw[/compressed] "
            f"(target {rate_hz:.1f} Hz)"
        )

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self._in_flight_id is not None:
            if now - self._in_flight_at < self._timeout_sec:
                return
            self.get_logger().warn(
                "VideoHub response timed out; sending next request.",
                throttle_duration_sec=5.0,
            )
            self._in_flight_id = None

        if (
            self._pub_req.get_subscription_count() == 0
            or self.count_publishers("/api/videohub/response") == 0
        ):
            self.get_logger().info(
                "Waiting for Go2 /api/videohub endpoints...",
                throttle_duration_sec=5.0,
            )
            return

        req = Request()
        req_id = time.monotonic_ns()
        req.header.identity.id = req_id
        req.header.identity.api_id = VIDEO_API_ID_GET_IMAGE_SAMPLE
        self._in_flight_id = req_id
        self._in_flight_at = now
        self._pub_req.publish(req)

    def _on_response(self, msg: Response) -> None:
        if msg.header.identity.id != self._in_flight_id:
            return
        self._in_flight_id = None

        if msg.header.status.code != 0 or not msg.binary:
            self.get_logger().warn(
                f"VideoHub error code={msg.header.status.code}",
                throttle_duration_sec=5.0,
            )
            return

        jpg_bytes = np.asarray(msg.binary, dtype=np.int8).tobytes()
        stamp = self.get_clock().now().to_msg()

        comp = CompressedImage()
        comp.header.stamp = stamp
        comp.header.frame_id = self._frame_id
        comp.format = "jpeg"
        comp.data = jpg_bytes
        self._pub_compressed.publish(comp)

        if self._pub_raw.get_subscription_count() > 0:
            img = cv2.imdecode(
                np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if img is not None:
                if (
                    self._raw_width > 0
                    and self._raw_height > 0
                    and (img.shape[1] != self._raw_width or img.shape[0] != self._raw_height)
                ):
                    img = cv2.resize(
                        img,
                        (self._raw_width, self._raw_height),
                        interpolation=cv2.INTER_AREA,
                    )
                raw = Image()
                raw.header.stamp = stamp
                raw.header.frame_id = self._frame_id
                raw.height = img.shape[0]
                raw.width = img.shape[1]
                raw.encoding = "bgr8"
                raw.is_bigendian = 0
                raw.step = img.shape[1] * 3
                raw.data = img.tobytes()
                self._pub_raw.publish(raw)

        self._frame_count += 1
        if self._frame_count == 1:
            self.get_logger().info(
                f"First frame received ({len(jpg_bytes)} bytes JPEG)."
            )


def main() -> None:
    rclpy.init()
    node = Go2FrontCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
