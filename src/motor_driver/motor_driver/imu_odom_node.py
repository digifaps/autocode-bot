#!/usr/bin/env python3
"""
IMU-based odom frame publisher.

Publishes the transform odom -> base_link using the IMU:
- Orientation: integrated from gyro (yaw only) for 2D mapping.
- Position: fixed at origin (no wheel encoders); slam_toolbox corrects pose via map->odom.

Use with slam_toolbox for basic LiDAR + IMU mapping. TF chain: map -> odom (slam_toolbox) -> base_link (this node) -> base_laser.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuOdomNode(Node):
    """Publishes odom -> base_link from IMU gyro (yaw integration)."""

    def __init__(self):
        super().__init__("imu_odom_node")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 50.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        rate = self.get_parameter("publish_rate").value

        self._yaw = 0.0
        self._last_stamp = None
        self._tf_broadcaster = TransformBroadcaster(self)
        self._have_imu = False

        self._sub = self.create_subscription(
            Imu,
            "imu/data_raw",
            self._imu_cb,
            10,
        )
        self._timer = self.create_timer(1.0 / rate, self._publish_tf)

        self.get_logger().info(
            "imu_odom_node: publishing %s -> %s from IMU yaw (position fixed at origin)"
            % (self.odom_frame, self.base_frame)
        )

    def _imu_cb(self, msg):
        if self._last_stamp is not None and msg.header.stamp:
            try:
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                last_t = self._last_stamp
                dt = t - last_t
                if 0 < dt < 1.0:  # ignore backward or huge jumps
                    gz = msg.angular_velocity.z
                    self._yaw += gz * dt
                    self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
            except Exception:
                pass
        if msg.header.stamp:
            self._last_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._have_imu = True

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # Yaw only (2D)
        cy = math.cos(self._yaw * 0.5)
        sy = math.sin(self._yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
