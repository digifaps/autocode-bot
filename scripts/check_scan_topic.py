#!/usr/bin/env python3
"""One-off: subscribe to /scan for 10s and report message count. Run on Jetson with workspace sourced."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def main():
    rclpy.init()
    node = Node("check_scan")
    count = [0]
    def cb(msg):
        count[0] += 1
        if count[0] == 1:
            node.get_logger().info("First /scan: %d ranges" % len(msg.ranges))
    node.create_subscription(LaserScan, "scan", cb, 10)
    node.get_logger().info("Listening for /scan for 10 s...")
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("/scan messages received: %d" % count[0])
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
