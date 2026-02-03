#!/usr/bin/env python3
"""
Bridge ROS2 topics /scan and /imu/data_raw to WebSocket (port 9090).
Web viewer expects the same JSON message format as combined_server (fake) mode.
Run after sourcing the ROS2 workspace; requires ROS2 to be running (robot + LiDAR).
"""

import asyncio
import json
import math
import sys
import threading

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan, Imu
except ImportError:
    print("rclpy not found. Source your ROS2 workspace: source install/setup.bash", file=sys.stderr)
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("Install websockets: pip3 install websockets", file=sys.stderr)
    sys.exit(1)


def _safe_float(v, default=0.0):
    """Return a JSON-serializable float (no nan/inf)."""
    f = float(v)
    if math.isfinite(f):
        return f
    return default


def imu_to_dict(msg):
    """Convert sensor_msgs/Imu to dict for web viewer. Sanitize nan/inf for JSON."""
    return {
        "header": {"frame_id": msg.header.frame_id},
        "linear_acceleration": {
            "x": _safe_float(msg.linear_acceleration.x),
            "y": _safe_float(msg.linear_acceleration.y),
            "z": _safe_float(msg.linear_acceleration.z),
        },
        "angular_velocity": {
            "x": _safe_float(msg.angular_velocity.x),
            "y": _safe_float(msg.angular_velocity.y),
            "z": _safe_float(msg.angular_velocity.z),
        },
    }


def scan_to_dict(msg):
    """Convert sensor_msgs/LaserScan to dict for web viewer. Sanitize nan/inf in ranges for JSON."""
    range_max = float(msg.range_max)
    if not math.isfinite(range_max) or range_max <= 0:
        range_max = 12.0
    ranges = [_safe_float(r, range_max) for r in msg.ranges]
    return {
        "angle_min": _safe_float(msg.angle_min),
        "angle_max": _safe_float(msg.angle_max),
        "angle_increment": _safe_float(msg.angle_increment),
        "range_min": _safe_float(msg.range_min),
        "range_max": range_max,
        "ranges": ranges,
    }


class ROS2WebSocketBridge(Node):
    def __init__(self, loop, clients_ref):
        super().__init__("ros2_websocket_bridge")
        self._loop = loop
        self._clients = clients_ref
        self._scan_sub = self.create_subscription(
            LaserScan, "scan", self._scan_cb, 10
        )
        self._imu_sub = self.create_subscription(
            Imu, "imu/data_raw", self._imu_cb, 10
        )
        self.get_logger().info("Subscribed to /scan and /imu/data_raw")

    def _broadcast(self, topic, msg_dict):
        try:
            payload = json.dumps({"op": "publish", "topic": topic, "msg": msg_dict})
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f"JSON serialize failed for {topic}: {e}")
            return
        for ws in list(self._clients):
            try:
                asyncio.run_coroutine_threadsafe(ws.send(payload), self._loop)
            except Exception:
                pass

    def _scan_cb(self, msg):
        self._broadcast("/scan", scan_to_dict(msg))

    def _imu_cb(self, msg):
        self._broadcast("/imu/data_raw", imu_to_dict(msg))


def run_ros_spin(node):
    rclpy.spin(node)


async def main_async():
    rclpy.init()
    clients = set()

    loop = asyncio.get_running_loop()
    node = ROS2WebSocketBridge(loop, clients)

    spin_thread = threading.Thread(target=run_ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    async def handler(websocket):
        clients.add(websocket)
        print(f"[CONN] Client: {websocket.remote_address}")
        try:
            async for _ in websocket:
                pass
        finally:
            clients.discard(websocket)
            print("[CONN] Disconnected")

    print("ROS2 WebSocket bridge on ws://0.0.0.0:9090 (real /scan, /imu/data_raw)")
    async with websockets.serve(handler, "0.0.0.0", 9090):
        await asyncio.Future()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
