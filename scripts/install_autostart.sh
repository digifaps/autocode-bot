#!/bin/bash
# Start the robot stack at boot (systemd service autocode-bot) and install the LiDAR udev rule.
# Usage: sudo scripts/install_autostart.sh            install + enable + start
#        sudo scripts/install_autostart.sh --remove   stop, disable, remove
# Logs: journalctl -u autocode-bot -f      Control: sudo systemctl {stop|start|restart} autocode-bot
# Network (eth0 + Wi-Fi access point) is set up separately: sudo scripts/setup_network.sh <password>
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT=/etc/systemd/system/autocode-bot.service
if [ "$(id -u)" -ne 0 ]; then echo "Run with sudo: sudo $0"; exit 1; fi
RUN_USER="${SUDO_USER:-$(stat -c %U "$REPO")}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"

if [ "${1:-}" = "--remove" ]; then
  systemctl disable --now autocode-bot 2>/dev/null || true
  rm -f "$UNIT" /etc/sysctl.d/60-autocode-bot-dds.conf
  systemctl daemon-reload
  echo "autocode-bot service removed"
  exit 0
fi

WS="$(cd "$REPO/../.." && pwd)"
[ -f "$WS/install/setup.bash" ] || { echo "Build the workspace first ($WS/install/setup.bash missing)"; exit 1; }

echo "[1/4] LiDAR udev rule (/dev/lidar)"
install -m 644 "$REPO/scripts/udev/99-d500-lidar.rules" /etc/udev/rules.d/99-d500-lidar.rules
udevadm control --reload-rules && udevadm trigger --subsystem-match=tty

echo "[2/4] socket buffers for Cyclone DDS (raw camera images over UDP)"
cat > /etc/sysctl.d/60-autocode-bot-dds.conf <<'SYSCTL'
# Cyclone DDS (config/cyclonedds.xml) asks for 16 MB receive buffers; without them raw camera
# frames are dropped (image_raw ~10 Hz instead of 20)
net.core.rmem_max = 33554432
net.core.wmem_max = 33554432
SYSCTL
sysctl -q -p /etc/sysctl.d/60-autocode-bot-dds.conf

echo "[3/4] systemd unit $UNIT (user $RUN_USER)"
sed -e "s|@USER@|$RUN_USER|g" -e "s|@HOME@|$RUN_HOME|g" -e "s|@REPO@|$REPO|g" \
  "$REPO/scripts/autocode-bot.service.in" > "$UNIT"
chmod 644 "$UNIT"
systemctl daemon-reload

echo "[4/4] enable + (re)start"
systemctl enable autocode-bot >/dev/null
systemctl restart autocode-bot
sleep 2
systemctl --no-pager --lines=0 status autocode-bot | head -5
echo ""
echo "The robot stack now starts at boot. Logs: journalctl -u autocode-bot -f"
