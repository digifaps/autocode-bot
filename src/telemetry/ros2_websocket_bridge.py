#!/usr/bin/env python3
"""
Bridge ROS2 topics /scan, /imu/data_raw, and camera images to WebSocket (port 9090).
Used by Foxglove Studio (Rosbridge) and the web telemetry viewer.
Run after sourcing the ROS2 workspace; requires ROS2 to be running (robot + LiDAR + stereo).
"""

import asyncio
import base64
import json
import math
import sys
import threading
import time

# Optional: send JPEG-compressed images (smaller; often better for Foxglove over WebSocket)
try:
    import cv2
    import numpy as np
    _HAVE_CV2 = True
except ImportError:
    _HAVE_CV2 = False

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan, Imu
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
    """Convert sensor_msgs/Imu to dict for Foxglove (Rosbridge JSON). Full message + sanitized numbers."""
    # ROS2 uses header.stamp.sec and .nanosec; Rosbridge/ROS1 expects secs/nsecs
    stamp = getattr(msg.header.stamp, "sec", 0) or 0
    nanosec = getattr(msg.header.stamp, "nanosec", 0) or 0
    # Exactly 9 elements per covariance (schema float64[9]) to avoid DataView offset errors in Foxglove
    def cov9(arr):
        out = [_safe_float(x) for x in list(arr)[:9]]
        while len(out) < 9:
            out.append(0.0)
        return out[:9]
    return {
        "header": {
            "seq": int(getattr(msg.header, "seq", 0)),
            "stamp": {"secs": int(stamp), "nsecs": int(nanosec)},
            "frame_id": msg.header.frame_id or "",
        },
        "orientation": {
            "x": _safe_float(msg.orientation.x),
            "y": _safe_float(msg.orientation.y),
            "z": _safe_float(msg.orientation.z),
            "w": _safe_float(msg.orientation.w, 1.0),
        },
        "orientation_covariance": cov9(msg.orientation_covariance),
        "angular_velocity": {
            "x": _safe_float(msg.angular_velocity.x),
            "y": _safe_float(msg.angular_velocity.y),
            "z": _safe_float(msg.angular_velocity.z),
        },
        "angular_velocity_covariance": cov9(msg.angular_velocity_covariance),
        "linear_acceleration": {
            "x": _safe_float(msg.linear_acceleration.x),
            "y": _safe_float(msg.linear_acceleration.y),
            "z": _safe_float(msg.linear_acceleration.z),
        },
        "linear_acceleration_covariance": cov9(msg.linear_acceleration_covariance),
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


def image_to_dict(msg):
    """Convert sensor_msgs/Image to dict for Foxglove (Rosbridge). data as base64."""
    stamp_sec = getattr(msg.header.stamp, "sec", 0) or 0
    stamp_nanosec = getattr(msg.header.stamp, "nanosec", 0) or 0
    data = bytes(msg.data) if hasattr(msg.data, "__iter__") else msg.data
    # Rosbridge/Foxglove: support both ROS1 (secs/nsecs) and ROS2 (sec/nanosec) stamp
    stamp = {
        "secs": int(stamp_sec),
        "nsecs": int(stamp_nanosec),
        "sec": int(stamp_sec),
        "nanosec": int(stamp_nanosec),
    }
    return {
        "header": {
            "frame_id": msg.header.frame_id or "",
            "stamp": stamp,
        },
        "height": int(msg.height),
        "width": int(msg.width),
        "encoding": str(msg.encoding) if msg.encoding else "bgr8",
        "is_bigendian": int(getattr(msg, "is_bigendian", 0)),
        "step": int(msg.step),
        "data": base64.b64encode(data).decode("ascii"),
    }


def image_to_compressed_dict(msg):
    """Convert sensor_msgs/Image to sensor_msgs/CompressedImage dict (JPEG) for smaller WebSocket payloads."""
    stamp_sec = getattr(msg.header.stamp, "sec", 0) or 0
    stamp_nanosec = getattr(msg.header.stamp, "nanosec", 0) or 0
    stamp = {
        "secs": int(stamp_sec),
        "nsecs": int(stamp_nanosec),
        "sec": int(stamp_sec),
        "nanosec": int(stamp_nanosec),
    }
    if not _HAVE_CV2:
        return None
    data = bytes(msg.data) if hasattr(msg.data, "__iter__") else msg.data
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    enc = (str(msg.encoding) if msg.encoding else "bgr8").lower()
    try:
        arr = np.frombuffer(data, dtype=np.uint8)
        if "bgr" in enc or "rgb" in enc:
            arr = arr.reshape((h, step))[:, : w * 3].reshape((h, w, 3))
            if "rgb" in enc:
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        else:
            arr = arr.reshape((h, step))[:, :w].reshape((h, w))
        _, jpeg = cv2.imencode(".jpg", arr)
        return {
            "header": {"frame_id": msg.header.frame_id or "", "stamp": stamp},
            "format": "jpeg",
            "data": base64.b64encode(jpeg.tobytes()).decode("ascii"),
        }
    except Exception:
        return None


# Throttle camera images over WebSocket (Hz)
IMAGE_BRIDGE_HZ = 5.0
IMAGE_MIN_INTERVAL = 1.0 / IMAGE_BRIDGE_HZ if IMAGE_BRIDGE_HZ > 0 else 1.0
# If True and cv2 available, send sensor_msgs/CompressedImage (jpeg) on .../compressed; smaller and often works better in Foxglove
SEND_COMPRESSED_IMAGE = True


class ROS2WebSocketBridge(Node):
    def __init__(self, loop, ws_connections_ref):
        super().__init__("ros2_websocket_bridge")
        self._loop = loop
        self._ws_connections = ws_connections_ref
        self._image_last_send = {}  # topic -> last send time for throttling
        self._last_msg = {}  # topic_str -> msg_dict (so we can send to new subscribers immediately)
        # Subscribe with both default (RELIABLE) and sensor_data (BEST_EFFORT) so we receive regardless of publisher QoS.
        self._imu_sub = self.create_subscription(Imu, "imu/data_raw", self._imu_cb, 10)
        self._imu_sub_be = self.create_subscription(Imu, "imu/data_raw", self._imu_cb, qos_profile_sensor_data)
        self._scan_sub = self.create_subscription(
            LaserScan, "scan", self._scan_cb, 10
        )
        self._scan_sub_be = self.create_subscription(
            LaserScan, "scan", self._scan_cb, qos_profile_sensor_data
        )
        self._img_left_sub = self.create_subscription(
            Image, "camera/left/image_raw", self._make_image_cb("camera/left/image_raw"), 10
        )
        self._img_right_sub = self.create_subscription(
            Image, "camera/right/image_raw", self._make_image_cb("camera/right/image_raw"), 10
        )
        self.get_logger().info(
            "Subscribed to /scan, /imu/data_raw, camera/left+right (images @ %.1f Hz%s)"
            % (IMAGE_BRIDGE_HZ, ", also sending .../compressed (JPEG)" if (SEND_COMPRESSED_IMAGE and _HAVE_CV2) else "")
        )

    def _broadcast(self, topic, msg_dict):
        topic_str = topic if topic.startswith("/") else "/" + topic
        try:
            payload = json.dumps({"op": "publish", "topic": topic_str, "msg": msg_dict})
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f"JSON serialize failed for {topic}: {e}")
            return
        self._last_msg[topic_str] = msg_dict  # cache for new subscribers
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

    def _make_image_cb(self, topic):
        def cb(msg):
            now = time.monotonic()
            last = self._image_last_send.get(topic, 0)
            if now - last < IMAGE_MIN_INTERVAL:
                return
            self._image_last_send[topic] = now
            try:
                if not getattr(self, "_img_logged", False):
                    self._img_logged = True
                    self.get_logger().info("First image received on %s" % topic)
                self._broadcast(topic, image_to_dict(msg))
                if SEND_COMPRESSED_IMAGE and _HAVE_CV2:
                    compressed = image_to_compressed_dict(msg)
                    if compressed is not None:
                        self._broadcast(topic + "/compressed", compressed)
            except Exception as e:
                self.get_logger().warn("image_cb %s failed: %s" % (topic, e))

        return cb


def run_ros_spin(node):
    rclpy.spin(node)


async def main_async():
    rclpy.init()
    ws_connections = set()

    loop = asyncio.get_running_loop()
    node = ROS2WebSocketBridge(loop, ws_connections)

    spin_thread = threading.Thread(target=run_ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    # Topics we publish (reply to get_topics / get_topics_and_raw_types so Foxglove doesn't time out)
    BRIDGE_TOPICS = [
        "/scan",
        "/imu/data_raw",
        "/camera/left/image_raw",
        "/camera/right/image_raw",
        "/camera/left/image_raw/compressed",
        "/camera/right/image_raw/compressed",
    ]
    BRIDGE_TYPES = [
        "sensor_msgs/LaserScan",
        "sensor_msgs/Imu",
        "sensor_msgs/Image",
        "sensor_msgs/Image",
        "sensor_msgs/CompressedImage",
        "sensor_msgs/CompressedImage",
    ]
    # ROS2-style message definitions (builtin_interfaces/Time, no seq) so Foxglove detects ROS 2
    _HEADER_ROS2 = """builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec"""
    TYPEDEF_LASERSCAN = """std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
================================================================================
MSG: std_msgs/Header
""" + _HEADER_ROS2
    TYPEDEF_IMU = """std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
""" + _HEADER_ROS2 + """
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z"""
    TYPEDEF_IMAGE = """std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
================================================================================
MSG: std_msgs/Header
""" + _HEADER_ROS2
    TYPEDEF_COMPRESSED_IMAGE = """std_msgs/Header header
string format
uint8[] data
================================================================================
MSG: std_msgs/Header
""" + _HEADER_ROS2
    # Omit full typedef for Imu so Foxglove uses our JSON as-is (avoids "DataView offset" binary-parse error)
    BRIDGE_TYPEDEFS = [
        TYPEDEF_LASERSCAN,
        "",  # sensor_msgs/Imu: no typedef → JSON message used directly
        TYPEDEF_IMAGE,
        TYPEDEF_IMAGE,
        TYPEDEF_COMPRESSED_IMAGE,
        TYPEDEF_COMPRESSED_IMAGE,
    ]

    async def handler(websocket):
        ws_connections.add(websocket)
        print(f"[CONN] Client: {websocket.remote_address}")
        # Hint ROS 2 so Foxglove can detect version (it doesn't call get_ros_version; may use status or response shape)
        try:
            await websocket.send(json.dumps({"op": "status", "level": "info", "msg": "ROS 2"}))
        except Exception:
            pass
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    op = data.get("op", "")
                    if op == "subscribe":
                        # Send last cached message so the panel shows data right away
                        topic = data.get("topic", "")
                        if topic:
                            key = topic if topic.startswith("/") else "/" + topic
                            if key in node._last_msg:
                                await websocket.send(
                                    json.dumps(
                                        {"op": "publish", "topic": key, "msg": node._last_msg[key]}
                                    )
                                )
                        continue
                    if op != "call_service":
                        continue
                    service = data.get("service", "")
                    call_id = data.get("id", "")
                    if "get_topics" in service or "topics" in service or "topics_and_raw_types" in service:
                        values = {
                            "topics": BRIDGE_TOPICS,
                            "types": BRIDGE_TYPES,
                            "ros_version": 2,
                        }
                        if "raw_types" in service or "topics_and_raw_types" in service:
                            values["typedefs_full_text"] = BRIDGE_TYPEDEFS
                        response = {
                            "op": "service_response",
                            "id": call_id,
                            "service": service,
                            "result": True,
                            "values": values,
                        }
                        await websocket.send(json.dumps(response))
                        continue
                    # /rosapi/get_ros_version — Foxglove uses this to detect ROS 1 vs 2; must respond for version detection
                    if "get_ros_version" in service or service == "/rosapi/get_ros_version":
                        response = {
                            "op": "service_response",
                            "id": call_id,
                            "service": service,
                            "result": True,
                            "values": {"version": "2"},
                        }
                        await websocket.send(json.dumps(response))
                        continue
                    # /rosapi/get_time
                    if "get_time" in service or service == "/rosapi/get_time":
                        t = time.time()
                        secs = int(t)
                        nsecs = int((t - secs) * 1e9)
                        response = {
                            "op": "service_response",
                            "id": call_id,
                            "service": service,
                            "result": True,
                            "values": {"sec": secs, "nanosec": nsecs, "secs": secs, "nsecs": nsecs},
                        }
                        await websocket.send(json.dumps(response))
                except (json.JSONDecodeError, KeyError, TypeError):
                    pass
        finally:
            ws_connections.discard(websocket)
            print("[CONN] Disconnected")

    print("ROS2 WebSocket bridge on ws://0.0.0.0:9090 (/scan, /imu/data_raw, camera/left+right images)")
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
