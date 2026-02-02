#!/bin/bash
# Switch Jetson wlan0 back to client mode and connect to your WiFi (e.g. zander).
# Run with: sudo ./jetson_wifi_client.sh
# To switch to AP mode: sudo ./jetson_wifi_ap.sh

set -e

# Connection name to bring up (nmcli connection show). Default: zander (your SSID)
CLIENT_CONNECTION="${JETSON_CLIENT_CONNECTION:-zander}"

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0"
  exit 1
fi

if ! command -v nmcli &>/dev/null; then
  echo "NetworkManager (nmcli) not found."
  exit 1
fi

echo "[1/2] Stopping Access Point (JetsonAP)..."
nmcli connection down JetsonAP 2>/dev/null || true
nmcli device disconnect wlan0 2>/dev/null || true
sleep 2

echo "[2/2] Connecting to '$CLIENT_CONNECTION'..."
if ! nmcli connection up "$CLIENT_CONNECTION"; then
  echo "Failed to connect to '$CLIENT_CONNECTION'. Available connections:"
  nmcli connection show
  echo "Set another: sudo JETSON_CLIENT_CONNECTION='YourSSID' $0"
  exit 1
fi

JETSON_IP=$(ip -4 -br addr show wlan0 | awk '{print $3}' | cut -d/ -f1)
echo ""
echo "--- Client mode ---"
echo "  Connected to: $CLIENT_CONNECTION"
echo "  Jetson (wlan0): $JETSON_IP"
echo "  SSH: ssh fabrice@$JETSON_IP"
echo ""
