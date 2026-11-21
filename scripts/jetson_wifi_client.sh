#!/bin/bash
# Temporarily use wlan0 as a Wi-Fi client (e.g. your home network) instead of the robot's access
# point. The access point comes back on the next boot, or now with:
#   sudo nmcli connection up autocode-bot-ap
# Usage: sudo scripts/jetson_wifi_client.sh [connection-name]   (list: nmcli connection show)
set -e
CLIENT_CONNECTION="${1:-${JETSON_CLIENT_CONNECTION:-zander}}"
if [ "$(id -u)" -ne 0 ]; then echo "Run with sudo: sudo $0 [connection-name]"; exit 1; fi
command -v nmcli >/dev/null || { echo "NetworkManager (nmcli) not found."; exit 1; }
echo "[1/2] Stopping the access point..."
nmcli connection down autocode-bot-ap 2>/dev/null || true
sleep 2
echo "[2/2] Connecting to '$CLIENT_CONNECTION'..."
if ! nmcli connection up "$CLIENT_CONNECTION"; then
  echo "Failed to connect to '$CLIENT_CONNECTION'. Available connections:"
  nmcli connection show
  exit 1
fi
JETSON_IP=$(ip -4 -br addr show wlan0 | awk '{print $3}' | cut -d/ -f1)
echo "Client mode: '$CLIENT_CONNECTION', wlan0 $JETSON_IP. Access point returns on reboot."
