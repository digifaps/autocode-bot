#!/bin/bash
# Switch Jetson wlan0 to Access Point mode. Connect your device to the Jetson's WiFi
# and reach it at 10.42.0.1 (SSH, web viewer, etc.).
# Run with: sudo ./jetson_wifi_ap.sh
# To switch back to client (zander): sudo ./jetson_wifi_client.sh

set -e

# Config (change if needed)
AP_SSID="${JETSON_AP_SSID:-JetsonAP}"
AP_PASSWORD="${JETSON_AP_PASSWORD:-jetsonap123}"
AP_IP="10.42.0.1/24"
CONNECTION_NAME="JetsonAP"

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0"
  exit 1
fi

if ! command -v nmcli &>/dev/null; then
  echo "NetworkManager (nmcli) not found. Install: sudo apt install network-manager"
  exit 1
fi

echo "[1/3] Disconnecting wlan0 from current connection..."
nmcli device disconnect wlan0 2>/dev/null || true
sleep 2

echo "[2/3] Creating/updating AP connection '$CONNECTION_NAME' (SSID: $AP_SSID)..."
# Remove existing AP connection so we can recreate with current config
nmcli connection delete "$CONNECTION_NAME" 2>/dev/null || true

nmcli connection add type wifi ifname wlan0 con-name "$CONNECTION_NAME" autoconnect no \
  wifi.mode ap \
  wifi.ssid "$AP_SSID" \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "$AP_PASSWORD" \
  ipv4.addresses "$AP_IP" \
  ipv4.method shared

echo "[3/3] Starting Access Point..."
nmcli connection up "$CONNECTION_NAME"

echo ""
echo "--- Access Point is running ---"
echo "  SSID:     $AP_SSID"
echo "  Password: $AP_PASSWORD"
echo "  Jetson:   $AP_IP (e.g. ssh fabrice@10.42.0.1)"
echo "  Web:      http://10.42.0.1:8080/web_viewer.html"
echo ""
echo "To switch back to WiFi client (zander): sudo $(dirname "$0")/jetson_wifi_client.sh"
echo ""
