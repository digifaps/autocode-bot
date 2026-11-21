#!/bin/bash
# Start ROS (stereo + IMU + LiDAR) and the WebSocket bridge for Foxglove only. No web panel, no MJPEG server.
# Run from ROS2 workspace root (e.g. cd ~/ros2_ws) or from autocode-bot repo root.
# Bridge: foxglove_bridge if built (Foxglove Studio → Foxglove WebSocket → ws://<JETSON_IP>:8765),
#         else rosbridge / custom bridge (Foxglove Studio → Rosbridge → ws://<JETSON_IP>:9090).
# Force a bridge with FOXGLOVE_BRIDGE=foxglove|rosbridge|custom.

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

# 1) Robot (stereo cameras + IMU + D500 LiDAR; LiDAR is included by robot.launch.py)
# LiDAR over USB: /dev/lidar (udev rule scripts/udev/99-d500-lidar.rules), else /dev/ttyUSB0; override with LIDAR_USB_PORT
if [ -e /dev/lidar ]; then DEFAULT_LIDAR_PORT=/dev/lidar; else DEFAULT_LIDAR_PORT=/dev/ttyUSB0; fi
LIDAR_PORT="${LIDAR_USB_PORT:-$DEFAULT_LIDAR_PORT}"
ros2 launch robot_bringup robot.launch.py lidar_use_usb:=true lidar_port_name:="$LIDAR_PORT" &
PIDS+=($!)
sleep 3

# 2) WebSocket bridge for Foxglove
free_port() {
  if command -v ss &>/dev/null; then
    local pid
    pid=$(ss -tlnp 2>/dev/null | awk -F'[=,]' "/:$1 /{print \$2}" | tr -d ' ')
    [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
  fi
}
pkill -x foxglove_bridge 2>/dev/null || true
pkill -f ros2_websocket_bridge 2>/dev/null || true
pkill -f combined_server 2>/dev/null || true
pkill -f rosbridge_websocket 2>/dev/null || true
sleep 2
free_port 8765
free_port 9090

BRIDGE="${FOXGLOVE_BRIDGE:-}"
if [ -z "$BRIDGE" ]; then
  if ros2 pkg executables foxglove_bridge 2>/dev/null | grep -q "foxglove_bridge foxglove_bridge"; then
    BRIDGE=foxglove
  elif ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
    BRIDGE=rosbridge
  else
    BRIDGE=custom
  fi
fi

IP=$(hostname -I | awk '{print $1}')
case "$BRIDGE" in
  foxglove)
    echo "Starting foxglove_bridge (port 8765)..."
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0 &
    PIDS+=($!)
    CONNECT="Foxglove WebSocket -> ws://${IP}:8765"
    ;;
  rosbridge)
    echo "Starting official Rosbridge WebSocket server (port 9090)..."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0 &
    PIDS+=($!)
    CONNECT="Rosbridge -> ws://${IP}:9090 (use compressed image topics)"
    ;;
  *)
    echo "foxglove_bridge / rosbridge not installed; using custom WebSocket bridge (limited). Install:"
    echo "  ./src/autocode-bot/scripts/install_foxglove_bridge.sh"
    python3 "$BOT_REL/src/telemetry/ros2_websocket_bridge.py" &
    PIDS+=($!)
    CONNECT="Rosbridge -> ws://${IP}:9090"
    ;;
esac

echo ""
echo "Foxglove Studio: Open connection -> ${CONNECT}"
echo "Topics: /imu/data_raw, /scan, /camera/{left,right}/compressed, /camera/{left,right}/image_raw, /tf, /robot_description"
echo "Press Ctrl+C to stop."
wait
