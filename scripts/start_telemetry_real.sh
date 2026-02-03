#!/bin/bash
# Start ROS (stereo + IMU + LiDAR over USB) and telemetry so the webpage shows real data.
# Run from your ROS2 workspace root (e.g. cd ~/ros2_ws) or from autocode-bot repo root.
# LiDAR: use_usb:=true (default /dev/ttyUSB0). Set permissions: sudo chmod 666 /dev/ttyUSB0

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Find workspace root (where install/setup.bash is)
if [ -f "$BOT_DIR/../../install/setup.bash" ]; then
    WS_ROOT="$(cd "$BOT_DIR/../.." && pwd)"
    BOT_REL="$BOT_DIR"
elif [ -f "$BOT_DIR/../install/setup.bash" ]; then
    WS_ROOT="$(cd "$BOT_DIR/.." && pwd)"
    BOT_REL="$BOT_DIR"
elif [ -f "install/setup.bash" ]; then
    WS_ROOT="$(pwd)"
    BOT_REL="$WS_ROOT/src/autocode-bot"
    if [ ! -d "$BOT_REL" ]; then
        BOT_REL="$BOT_DIR"
    fi
else
    echo "Could not find ROS2 workspace (install/setup.bash). Run from workspace root or from autocode-bot repo."
    exit 1
fi

cd "$WS_ROOT"
source install/setup.bash

PIDS=()
cleanup() {
    echo "Stopping all..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Workspace: $WS_ROOT"
echo "Robot package: $BOT_REL"

# 1) Robot (stereo cameras + IMU + depth_processor + motor_driver)
ros2 launch robot_bringup robot.launch.py &
PIDS+=($!)
sleep 3

# 2) LiDAR over USB
ros2 launch robot_bringup d500_lidar.launch.py use_usb:=true &
PIDS+=($!)
sleep 2

# 3) ROS2 -> WebSocket bridge (real /scan, /imu/data_raw)
python3 "$BOT_REL/src/telemetry/ros2_websocket_bridge.py" &
PIDS+=($!)
sleep 1

# 4) MJPEG video server (stereo cameras via GStreamer)
python3 "$BOT_REL/src/telemetry/video_server.py" &
PIDS+=($!)

# 5) HTTP server for web viewer
(cd "$BOT_REL/src/telemetry" && python3 -m http.server 8080) &
PIDS+=($!)

IP=$(hostname -I | awk '{print $1}')
echo ""
echo "Telemetry running with real data (LiDAR USB, stereo, IMU)."
echo "Open: http://${IP}:8080/web_viewer.html"
echo "Press Ctrl+C to stop all."
wait
