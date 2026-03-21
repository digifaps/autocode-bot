#!/bin/bash
# Install official Rosbridge WebSocket server for ROS2 Humble (for Foxglove with proper ROS version detection).
# Run from workspace root: ./src/autocode-bot/scripts/install_rosbridge.sh
# Or: cd ~/ros2_ws && ./src/autocode-bot/scripts/install_rosbridge.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$BOT_DIR/../../install/setup.bash" ]; then
    WS_ROOT="$(cd "$BOT_DIR/../.." && pwd)"
elif [ -f "$BOT_DIR/../install/setup.bash" ]; then
    WS_ROOT="$(cd "$BOT_DIR/.." && pwd)"
elif [ -f "install/setup.bash" ]; then
    WS_ROOT="$(pwd)"
else
    echo "Run from ROS2 workspace root (e.g. cd ~/ros2_ws)"
    exit 1
fi

echo "Workspace: $WS_ROOT"

# 1) Try apt (ROS Humble)
if apt-cache show ros-humble-rosbridge-suite &>/dev/null; then
    echo "Installing ros-humble-rosbridge-suite and dependencies..."
    sudo apt-get update -qq
    sudo apt-get install -y ros-humble-rosbridge-suite \
        python3-tornado python3-autobahn python3-twisted 2>/dev/null || true
    for f in /opt/ros/humble/setup.bash "$HOME/ros2_humble/install/setup.bash"; do
        [ -f "$f" ] && source "$f" && break
    done
    if ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
        echo "Done. Rosbridge installed via apt. Use: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
        exit 0
    fi
fi

# 2) Build from source (with Python 3.8 compatibility patches applied)
echo "Installing rosbridge_suite from source..."
# Python deps (rosbridge_library + server): bson via pymongo, cbor2, tornado, numpy, etc.
pip3 install typing_extensions pymongo cbor2 tornado numpy Pillow ujson --user 2>/dev/null || true
cd "$WS_ROOT/src"
if [ ! -d rosbridge_suite ]; then
    git clone --depth 1 -b humble https://github.com/RobotWebTools/rosbridge_suite.git
fi
cd "$WS_ROOT"
for f in /opt/ros/humble/setup.bash "$HOME/ros2_humble/install/setup.bash"; do
    [ -f "$f" ] && source "$f" && break
done
rosdep install -y --from-paths src/rosbridge_suite --ignore-src 2>/dev/null || true
colcon build --packages-up-to rosbridge_server --symlink-install
source install/setup.bash
echo "Done. Rosbridge built from source."
echo "Usage: source ~/ros2_humble/install/setup.bash && source install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
