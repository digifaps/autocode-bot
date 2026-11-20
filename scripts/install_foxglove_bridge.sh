#!/bin/bash
# Build foxglove_bridge (native Foxglove WebSocket server, port 8765) for ROS2 Humble built from source.
# Run from workspace root: cd ~/ros2_ws && ./src/autocode-bot/scripts/install_foxglove_bridge.sh
#
# Pinned to the commits tested on the Jetson (JetPack 5.1.3 / Ubuntu 20.04 / Python 3.8):
#   foxglove-sdk        51e14b5 (foxglove_bridge 3.2.4; downloads the prebuilt aarch64 Foxglove C++ SDK)
#   rosx_introspection  0eb68de (2.0.1 + CMake fix)
# rosx_introspection must be built with BUILD_TESTING=OFF: with tests on, upstream links
# sensor_msgs/geometry_msgs into the library's exported targets and foxglove_bridge fails to configure
# ("links to target sensor_msgs::... but the target was not found").

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

FOXGLOVE_SDK_COMMIT=51e14b5c82b2ca500da8956b3c1cdb4e13a28e29
ROSX_COMMIT=0eb68de757eac3b0504f3ec237c9abf857579246

# Clone a repo at an exact commit (shallow), unless the directory already exists.
fetch_at() {
    local url="$1" dir="$2" commit="$3"
    if [ -d "$dir" ]; then
        echo "$dir already present ($(git -C "$dir" rev-parse --short HEAD 2>/dev/null || echo '?')), leaving as is."
        return
    fi
    git init -q "$dir"
    git -C "$dir" remote add origin "$url"
    git -C "$dir" fetch -q --depth 1 origin "$commit"
    git -C "$dir" checkout -q FETCH_HEAD
    echo "Fetched $dir at ${commit:0:7}"
}

# System deps used by foxglove_bridge (standalone ASIO, nlohmann-json, websocketpp)
sudo apt-get install -y libasio-dev nlohmann-json3-dev libwebsocketpp-dev 2>/dev/null || true

cd "$WS_ROOT/src"
fetch_at https://github.com/foxglove/foxglove-sdk.git foxglove-sdk "$FOXGLOVE_SDK_COMMIT"
fetch_at https://github.com/facontidavide/rosx_introspection.git rosx_introspection "$ROSX_COMMIT"

cd "$WS_ROOT"
for f in /opt/ros/humble/setup.bash "$HOME/ros2_humble/install/setup.bash"; do
    [ -f "$f" ] && source "$f" && break
done

colcon build --symlink-install --packages-select rosx_introspection \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# foxglove_bridge takes ~5 min on a Xavier NX
colcon build --symlink-install --packages-select foxglove_msgs foxglove_bridge \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

if ros2 pkg executables foxglove_bridge 2>/dev/null | grep -q "foxglove_bridge foxglove_bridge"; then
    echo "Done. foxglove_bridge built. scripts/start_foxglove.sh will use it (ws://<JETSON_IP>:8765)."
else
    echo "foxglove_bridge build did not produce an executable; check log/latest_build/foxglove_bridge" >&2
    exit 1
fi
