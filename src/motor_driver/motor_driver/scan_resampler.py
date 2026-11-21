#!/usr/bin/env python3
"""
Resample /scan onto a fixed angular grid centred on straight ahead, for rf2o_laser_odometry.

The D500 driver publishes 0..360 deg with a point count that changes from scan to scan
(501-505), so the angle of index i drifts between scans. rf2o assumes a fixed number of
columns at angles -fov/2 + i*fov/(n-1) (centred on the laser's x axis). This node maps every
output column to the nearest input beam by angle and marks invalid ranges as 0, which rf2o
treats as "no measurement".
"""

import math
import signal

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanResampler(Node):
    def __init__(self):
        super().__init__("scan_resampler")
        self.declare_parameter("input_topic", "scan")
        self.declare_parameter("output_topic", "scan_fixed")
        self.declare_parameter("num_beams", 500)  # 0.72 deg; the D500 delivers ~503 per turn
        n = int(self.get_parameter("num_beams").value)
        self.n = n
        # Full circle without duplicating +-180 deg: fov = 2*pi*(n-1)/n, symmetric around 0
        self.fov = 2.0 * math.pi * (n - 1) / n
        self.angles = -0.5 * self.fov + np.arange(n) * (2.0 * math.pi / n)
        self.pub = self.create_publisher(LaserScan, self.get_parameter("output_topic").value, 5)
        self.create_subscription(LaserScan, self.get_parameter("input_topic").value, self._cb,
                                 qos_profile_sensor_data)
        self.get_logger().info("scan_resampler: %s -> %s, %d beams over %.2f deg centred on 0"
                               % (self.get_parameter("input_topic").value,
                                  self.get_parameter("output_topic").value, n, math.degrees(self.fov)))

    def _cb(self, msg):
        src = np.asarray(msg.ranges, dtype=np.float32)
        m = src.size
        if m < 2 or msg.angle_increment == 0.0:
            return
        src = np.nan_to_num(src, nan=0.0, posinf=0.0, neginf=0.0)
        valid = (src >= max(msg.range_min, 1e-3)) & (src <= msg.range_max)
        src = np.where(valid, src, 0.0).astype(np.float32)
        # Index of the input beam nearest to each output angle (input covers angle_min + i*inc)
        rel = np.mod(self.angles - msg.angle_min, 2.0 * math.pi)
        idx = np.rint(rel / msg.angle_increment).astype(np.int64)
        span = msg.angle_increment * m
        if span >= 2.0 * math.pi - 1.5 * abs(msg.angle_increment):
            idx %= m                       # full turn: wrap around
            out = src[idx]
        else:
            inside = idx < m               # partial scan: columns outside it get no measurement
            out = np.zeros(self.n, np.float32)
            out[inside] = src[idx[inside]]
        o = LaserScan()
        o.header = msg.header
        o.angle_min = float(-0.5 * self.fov)
        o.angle_max = float(0.5 * self.fov)
        o.angle_increment = float(2.0 * math.pi / self.n)
        o.time_increment = msg.time_increment * m / float(self.n)
        o.scan_time = msg.scan_time
        o.range_min = msg.range_min
        o.range_max = msg.range_max
        o.ranges = out.tolist()
        self.pub.publish(o)


def _sigint_once(signum, frame):
    # launch sends SIGINT twice on Ctrl+C; ignore the second one so cleanup is not interrupted
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None):
    signal.signal(signal.SIGINT, _sigint_once)  # rclpy chains to this handler
    rclpy.init(args=args)
    node = None
    try:
        node = ScanResampler()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
