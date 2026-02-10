#!/bin/bash
# Start ROS (stereo + IMU + LiDAR) and the WebSocket bridge for Foxglove only. No web panel, no MJPEG server.
# Run from ROS2 workspace root (e.g. cd ~/ros2_ws) or from autocode-bot repo root.
# Connect Foxglove Studio → Rosbridge (WebSocket) → ws://<JETSON_IP>:9090

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$BOT_DIR/../../install/setup.bash" ]; then
    WS_ROOT="$(cd "$BOT_DIR/../.." && pwd)"
    BOT_REL="$BOT_DIR"
elif [ -f "$BOT_DIR/../install/setup.bash" ]; then
    WS_ROOT="$(cd "$BOT_DIR/.." && pwd)"
    BOT_REL="$BOT_DIR"
elif [ -f "install/setup.bash" ]; then
    WS_ROOT="$(pwd)"
    BOT_REL="$WS_ROOT/src/autocode-bot"
    [ -d "$BOT_REL" ] || BOT_REL="$BOT_DIR"
else
    echo "Could not find ROS2 workspace (install/setup.bash). Run from workspace root or autocode-bot repo."
    exit 1
fi

cd "$WS_ROOT"
source install/setup.bash

PIDS=()
cleanup() {
    echo "Stopping all..."
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Workspace: $WS_ROOT | Foxglove bridge only (no web panel)"

# 1) Robot (stereo cameras + IMU)
ros2 launch robot_bringup robot.launch.py &
PIDS+=($!)
sleep 3

# 2) LiDAR over USB (optional: set LIDAR_USB_PORT=/dev/ttyUSB1 if needed)
LIDAR_PORT="${LIDAR_USB_PORT:-/dev/ttyUSB0}"
ros2 launch robot_bringup d500_lidar.launch.py use_usb:=true port_name:="$LIDAR_PORT" &
PIDS+=($!)
sleep 2

# 3) Rosbridge WebSocket for Foxglove (port 9090): prefer official rosbridge_suite, else custom bridge
pkill -f ros2_websocket_bridge 2>/dev/null || true
pkill -f combined_server 2>/dev/null || true
pkill -f rosbridge_websocket 2>/dev/null || true
sleep 2
if command -v ss &>/dev/null; then
  PID=$(ss -tlnp 2>/dev/null | awk -F'[=,]' '/:9090 /{print $2}' | tr -d ' ')
  [ -n "$PID" ] && kill -9 "$PID" 2>/dev/null || true
  sleep 1
fi

USE_ROSBRIDGE=false
if ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
  USE_ROSBRIDGE=true
fi
if [ "$USE_ROSBRIDGE" = true ]; then
  echo "Starting official Rosbridge WebSocket server (port 9090)..."
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0 &
  PIDS+=($!)
else
  echo "Rosbridge not installed; using custom WebSocket bridge. Install for full Foxglove support:"
  echo "  ./src/autocode-bot/scripts/install_rosbridge.sh"
  python3 "$BOT_REL/src/telemetry/ros2_websocket_bridge.py" &
  PIDS+=($!)
fi

IP=$(hostname -I | awk '{print $1}')
echo ""
echo "Foxglove: connect to ws://${IP}:9090 (Rosbridge WebSocket)"
echo "Topics: /imu/data_raw, /scan, /camera/left/image_raw, /camera/right/image_raw"
echo "        (if no video in Foxglove, try /camera/left/image_raw/compressed and .../right/.../compressed)"
echo "Press Ctrl+C to stop."
wait
