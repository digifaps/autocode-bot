#!/bin/bash
# Start the robot stack by hand (sensors, odometry, foxglove_bridge on port 8765, health,
# on-demand mapping). On the robot this normally runs at boot as the autocode-bot service.
# Foxglove: Open connection -> Foxglove WebSocket -> ws://<robot-ip>:8765 (ws://10.42.0.1:8765 on
# the robot's Wi-Fi), then import config/foxglove/autocode-bot.json as layout.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if systemctl is-active --quiet autocode-bot 2>/dev/null; then
  echo "The robot stack is already running as the autocode-bot service (started at boot)."
  echo "  Logs:    journalctl -u autocode-bot -f"
  echo "  Restart: sudo systemctl restart autocode-bot"
  echo "  To run it by hand instead: sudo systemctl stop autocode-bot, then run this script again."
  exit 1
fi
IP=$(hostname -I | awk '{print $1}')
echo "Foxglove: ws://${IP}:8765  (robot Wi-Fi: ws://10.42.0.1:8765). Ctrl+C to stop."
exec "$SCRIPT_DIR/robot_bringup.sh" "$@"
