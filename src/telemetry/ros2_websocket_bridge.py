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
    from rclpy.qos import qos_profile_sensor_data
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
    def __init__(self, loop, ws_connections_ref):
        super().__init__("ros2_websocket_bridge")
        self._loop = loop
        self._ws_connections = ws_connections_ref  # set of websockets; do not name like _clients (rclpy may iterate node attrs)
        # Subscribe with both default (RELIABLE) and sensor_data (BEST_EFFORT) so we receive regardless of publisher QoS.
        self._imu_sub = self.create_subscription(Imu, "imu/data_raw", self._imu_cb, 10)
        self._imu_sub_be = self.create_subscription(Imu, "imu/data_raw", self._imu_cb, qos_profile_sensor_data)
        self._scan_sub = self.create_subscription(
            LaserScan, "scan", self._scan_cb, 10
        )
        self._scan_sub_be = self.create_subscription(
            LaserScan, "scan", self._scan_cb, qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /scan and /imu/data_raw")

    def _broadcast(self, topic, msg_dict):
        try:
            payload = json.dumps({"op": "publish", "topic": topic, "msg": msg_dict})
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f"JSON serialize failed for {topic}: {e}")
            return
        conns = list(self._ws_connections)
        if not getattr(self, "_broadcast_logged", False) and conns:
            self._broadcast_logged = True
            self.get_logger().info("Broadcasting to %d client(s)" % len(conns))
        for ws in conns:
            try:
                asyncio.run_coroutine_threadsafe(ws.send(payload), self._loop)
            except Exception:
                pass

    def _scan_cb(self, msg):
        try:
            if not getattr(self, "_scan_logged", False):
                self._scan_logged = True
                self.get_logger().info("First /scan received (%d ranges)" % len(getattr(msg, "ranges", [])))
            self._broadcast("/scan", scan_to_dict(msg))
        except Exception as e:
            self.get_logger().warn("scan_cb failed: %s" % e)

    def _imu_cb(self, msg):
        try:
            if not getattr(self, "_imu_logged", False):
                self._imu_logged = True
                self.get_logger().info("First /imu/data_raw received")
            self._broadcast("/imu/data_raw", imu_to_dict(msg))
        except Exception as e:
            self.get_logger().warn("imu_cb failed: %s" % e)


def run_ros_spin(node):
    rclpy.spin(node)


async def main_async():
    rclpy.init()
    ws_connections = set()

    loop = asyncio.get_running_loop()
    node = ROS2WebSocketBridge(loop, ws_connections)

    spin_thread = threading.Thread(target=run_ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    async def handler(websocket):
        ws_connections.add(websocket)
        print(f"[CONN] Client: {websocket.remote_address}")
        try:
            async for _ in websocket:
                pass
        finally:
            ws_connections.discard(websocket)
            print("[CONN] Disconnected")

    print("ROS2 WebSocket bridge on ws://0.0.0.0:9090 (real /scan, /imu/data_raw)")
    try:
        async with websockets.serve(handler, "0.0.0.0", 9090):
            await asyncio.Future()
    except OSError as e:
        if e.errno == 98:  # Address already in use
            print("Port 9090 already in use. Stop the other process first:", file=sys.stderr)
            print("  pkill -f ros2_websocket_bridge; pkill -f combined_server; sleep 2", file=sys.stderr)
            sys.exit(1)
        raise


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
