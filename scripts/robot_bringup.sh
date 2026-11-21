#!/bin/bash
# Starts the whole robot stack (robot_bringup/bringup.launch.py). Run by autocode-bot.service at
# boot; also fine to run by hand when the service is stopped.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROBOT_WS:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"   # <ws>/src/autocode-bot/scripts
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "ROS 2 workspace not found at $WS (set ROBOT_WS)"; exit 1
fi
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
# Keep ROS log directories from piling up (one per launch)
find "$HOME/.ros/log" -mindepth 1 -maxdepth 1 -mtime +14 -exec rm -rf {} + 2>/dev/null || true
exec ros2 launch robot_bringup bringup.launch.py "$@"
