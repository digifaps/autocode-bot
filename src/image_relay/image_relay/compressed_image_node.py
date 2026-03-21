#!/usr/bin/env python3
"""
CompressedImage relay node: subscribes to raw Image topics, publishes JPEG CompressedImage.
Use the compressed topics in Foxglove/rosbridge to reduce memory and bandwidth.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2


class CompressedImageNode(Node):
    def __init__(self):
        super().__init__("compressed_image_node")
        self.declare_parameter("left_topic", "camera/left/image_raw")
        self.declare_parameter("right_topic", "camera/right/image_raw")
        self.declare_parameter("left_compressed", "camera/left/compressed")
        self.declare_parameter("right_compressed", "camera/right/compressed")
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("max_hz", 10.0)

        left_in = self.get_parameter("left_topic").value
        right_in = self.get_parameter("right_topic").value
        left_out = self.get_parameter("left_compressed").value
        right_out = self.get_parameter("right_compressed").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.max_hz = self.get_parameter("max_hz").value

        self._bridge = CvBridge()
        self._last_pub = {}
        self._min_interval = 1.0 / self.max_hz if self.max_hz > 0 else 0.0

        self._pub_left = self.create_publisher(CompressedImage, left_out, 2)
        self._pub_right = self.create_publisher(CompressedImage, right_out, 2)
        self._sub_left = self.create_subscription(Image, left_in, self._make_cb("left"), 2)
        self._sub_right = self.create_subscription(Image, right_in, self._make_cb("right"), 2)

        self.get_logger().info(
            "Relay: %s -> %s, %s -> %s (JPEG quality=%d, max %.1f Hz)"
            % (left_in, left_out, right_in, right_out, self.jpeg_quality, self.max_hz)
        )

    def _make_cb(self, side):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            last = self._last_pub.get(side, 0.0)
            if self._min_interval > 0 and (now - last) < self._min_interval:
                return
            self._last_pub[side] = now
            try:
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().warn("cv_bridge imgmsg_to_cv2: %s" % e)
                return
            _, jpeg = cv2.imencode(".jpg", cv_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not _:
                return
            out = CompressedImage()
            out.header = msg.header
            out.format = "jpeg"
            out.data = jpeg.tobytes()
            if side == "left":
                self._pub_left.publish(out)
            else:
                self._pub_right.publish(out)
        return cb


def main():
    rclpy.init()
    node = CompressedImageNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
