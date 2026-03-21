# Telemetry Web Viewer – Debug Notes

## Symptoms

- **LiDAR:** Webpage shows "waiting for scan" (no scan data).
- **IMU:** Webpage shows dashes (––) instead of numbers.

## Data flow

1. **ROS2** publishes `/scan` (LaserScan) and `/imu/data_raw` (Imu).
2. **ros2_websocket_bridge.py** subscribes to those topics and forwards messages over WebSocket (port 9090) as `{ "op": "publish", "topic": "...", "msg": { ... } }`.
3. **web_viewer.html** connects to `ws://<host>:9090`, sends `subscribe` for `/scan` and `/imu/data_raw`, and on each `publish` message updates the LiDAR canvas and IMU fields using `msg.ranges`, `msg.linear_acceleration`, `msg.angular_velocity`.

## Debug checklist

### 1. Are the topics publishing on the Jetson?

From the Jetson (with workspace sourced):

```bash
source ~/ros2_humble/install/setup.bash   # if you use Humble from source
source ~/ros2_ws/install/setup.bash

ros2 topic list | grep -E 'scan|imu'
ros2 topic hz /scan
ros2 topic hz /imu/data_raw
```

- If **/scan** does not appear or has 0 Hz: LiDAR driver or USB port issue; bridge will never get scan data.
- If **/imu/data_raw** does not appear or has 0 Hz: `motor_driver` (IMU node) not running or IMU hardware not connected; bridge will never get IMU data → viewer keeps dashes.

### 2. QoS: bridge must use sensor profile to receive

- Many sensor drivers (LiDAR, IMU) publish with **BEST_EFFORT** reliability. The default rclpy subscription uses **RELIABLE**, so the bridge may not receive any messages.
- **Fix applied:** The bridge now subscribes with `qos_profile_sensor_data` (BEST_EFFORT, VOLATILE) for both `/scan` and `/imu/data_raw`.

### 3. Is the bridge actually running on 9090?

```bash
ss -tlnp | grep 9090
pgrep -af ros2_websocket_bridge
```

If you see `combined_server.py` on 9090 instead of `ros2_websocket_bridge.py`, the browser is getting **fake** data (or nothing useful). The real-data bridge must be the process bound to 9090.

### 4. Bridge message format vs viewer

- **Viewer expects:** `data.op === 'publish'`, `data.msg`, and for IMU: `msg.linear_acceleration.x/y/z`, `msg.angular_velocity.x/y/z`. For scan: `msg.ranges`, `msg.angle_min`, etc.
- **Bridge sends:** Same structure (`imu_to_dict` and `scan_to_dict`). So format is correct **if** the bridge sends at all.

### 5. LiDAR “waiting for scan” – NaN in ranges

- **LaserScan.ranges** often contains `float('nan')` or `inf` for “no return” or out-of-range.
- **Python `json.dumps()`** turns `float('nan')` into the literal `NaN` in the output string. That is **invalid JSON** (RFC 8259).
- **Browser:** `JSON.parse('{"ranges":[1.2,NaN,3.4]}')` can throw (or behave badly) on invalid JSON. So even if the bridge sends a message, the viewer’s `onmessage` may throw when parsing, and the LiDAR panel never updates → “waiting for scan”.
- **Conclusion:** If `/scan` is publishing but the page still shows “waiting for scan”, the most likely cause is **NaN (or inf) in `ranges`** → invalid JSON → parse error in the browser. Fix (when you change code): sanitize `ranges` (e.g. replace NaN/inf with a valid number or omit) before `json.dumps()` in the bridge.

### 6. IMU dashes (fixed: add imu_node to robot.launch.py)

- Dashes mean the viewer never receives a `publish` message for `/imu/data_raw`. **Fix:** Add `imu_node` to `robot.launch.py` so `/imu/data_raw` is published.

### 7. LiDAR "waiting for scan" – no /scan messages

- If the bridge never logs "First /scan received", the LiDAR driver is not publishing.
- **Check on Jetson:** `ls /dev/ttyUSB*` — if empty, no USB LiDAR device. Connect the D500 via USB-to-UART and ensure the adapter is recognized (`/dev/ttyUSB0` or `/dev/ttyACM0`).
- Run `scripts/check_scan_topic.py` (with workspace sourced) to count /scan messages in 10 s; 0 means the driver isn't publishing (USB/port/power or driver error).

## Summary

| Symptom            | Most likely cause |
|---------------------|-------------------|
| LiDAR “waiting for scan” | (1) `/scan` not publishing, or (2) `ranges` contain NaN/inf → invalid JSON → parse error in browser. |
| IMU dashes          | `/imu/data_raw` not publishing (motor_driver/IMU not running or not connected). |

**Next steps when you’re ready to change code:**  
(1) Verify topics with `ros2 topic list` and `ros2 topic hz` on the Jetson.  
(2) In the bridge: sanitize LaserScan `ranges` (replace or drop NaN/inf) before `json.dumps()`.  
(3) Ensure the process on port 9090 is `ros2_websocket_bridge.py` and that robot + LiDAR (+ IMU) launches are running in the same ROS 2 environment.

---

## Foxglove Studio (accelerometer + cameras)

The same WebSocket bridge (port 9090) is used by **Foxglove Studio** via **Rosbridge (WebSocket)**. The bridge forwards:

| Topic | Message type | Notes |
|-------|--------------|--------|
| `/imu/data_raw` | `sensor_msgs/Imu` | Full message (header.stamp, orientation, linear_acceleration, angular_velocity, covariances). Use **Raw Messages** or **Plot** for accelerometer/gyro. |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR; use **Image** (range) or **3D** / **Map** panels. |
| `/camera/left/image_raw` | `sensor_msgs/Image` | Left stereo image (throttled to 5 Hz). Use leading slash in Foxglove. |
| `/camera/right/image_raw` | `sensor_msgs/Image` | Right stereo image (throttled to 5 Hz). |
| `/camera/left/image_raw/compressed` | `sensor_msgs/CompressedImage` | Left as JPEG (smaller; try this if raw shows no video). |
| `/camera/right/image_raw/compressed` | `sensor_msgs/CompressedImage` | Right as JPEG. |

### How to view in Foxglove

1. **Start robot + bridge** on the Jetson: `scripts/start_telemetry_real.sh` (or run `robot.launch.py` then `ros2_websocket_bridge.py` on port 9090).
2. **Open Foxglove Studio** → Add connection → **Rosbridge (WebSocket)** → URL: `ws://<JETSON_IP>:9090` (e.g. `ws://192.168.8.171:9090`).
3. **Panels:** Use **Raw Messages** or **Plot** for `/imu/data_raw`; **Image** for `/camera/left/image_raw` and `/camera/right/image_raw` (use the leading slash). If no video appears, try `/camera/left/image_raw/compressed` and `/camera/right/image_raw/compressed` (JPEG, smaller). **3D** or **Map** for `/scan`.

Image topics are throttled to 5 Hz; compressed topics are JPEG for lower bandwidth.
