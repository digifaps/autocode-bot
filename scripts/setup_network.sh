#!/bin/bash
# Permanent network setup for the robot (NetworkManager):
#   eth0  : normal networking (DHCP from your router, default route, internet)
#   wlan0 : always the robot's Wi-Fi access point, 10.42.0.1/24 with DHCP for clients
#           (Foxglove: ws://10.42.0.1:8765). AP clients get internet through eth0 when it is up.
# Other Wi-Fi client profiles are kept but no longer autoconnect, so wlan0 stays an AP.
#
# Usage: sudo scripts/setup_network.sh <ap-password> [ssid]
#   password: 8-63 characters (WPA2); it is stored only in NetworkManager, not in this repo.
#   ssid    : default autocode-bot. Channel: 2.4 GHz channel 6 (AP_CHANNEL / AP_BAND to override;
#             5 GHz needs the Wi-Fi country set, e.g. sudo iw reg set BE and /etc/default/crda).
# Temporarily back to a Wi-Fi client: sudo scripts/jetson_wifi_client.sh (AP returns on reboot).
set -euo pipefail

AP_PASSWORD="${1:-}"
AP_SSID="${2:-autocode-bot}"
AP_CON="autocode-bot-ap"
AP_IP="10.42.0.1/24"
AP_BAND="${AP_BAND:-bg}"
AP_CHANNEL="${AP_CHANNEL:-6}"
WIFI_IF="${WIFI_IF:-wlan0}"
ETH_IF="${ETH_IF:-eth0}"

if [ "$(id -u)" -ne 0 ]; then echo "Run with sudo: sudo $0 <ap-password> [ssid]"; exit 1; fi
if [ ${#AP_PASSWORD} -lt 8 ] || [ ${#AP_PASSWORD} -gt 63 ]; then
  echo "AP password must be 8-63 characters (WPA2). Usage: sudo $0 <ap-password> [ssid]"; exit 1
fi
command -v nmcli >/dev/null || { echo "NetworkManager (nmcli) not found"; exit 1; }

echo "[1/4] Ethernet ($ETH_IF): DHCP, autoconnect, preferred default route"
ETH_CON=$(nmcli -t -f NAME,TYPE,DEVICE con show | awk -F: -v d="$ETH_IF" '$2=="802-3-ethernet" && $3==d {print $1; exit}')
if [ -z "$ETH_CON" ]; then
  ETH_CON=$(nmcli -t -f NAME,TYPE con show | awk -F: '$2=="802-3-ethernet" {print $1; exit}')
fi
if [ -n "$ETH_CON" ]; then
  nmcli con modify "$ETH_CON" connection.autoconnect yes ipv4.method auto ipv4.route-metric 100
else
  nmcli con add type ethernet ifname "$ETH_IF" con-name "Wired $ETH_IF" connection.autoconnect yes \
    ipv4.method auto ipv4.route-metric 100
fi

echo "[2/4] Other Wi-Fi client profiles: no autoconnect (kept for jetson_wifi_client.sh)"
nmcli -t -f NAME,TYPE con show | awk -F: '$2=="802-11-wireless" {print $1}' | while read -r c; do
  [ "$c" = "$AP_CON" ] && continue
  if [ "$(nmcli -g 802-11-wireless.mode con show "$c")" = "ap" ]; then
    echo "      removing old AP profile '$c'"; nmcli con delete "$c" >/dev/null
  else
    nmcli con modify "$c" connection.autoconnect no; echo "      '$c': autoconnect off"
  fi
done

echo "[3/4] Access point '$AP_SSID' on $WIFI_IF ($AP_IP, band $AP_BAND channel $AP_CHANNEL)"
nmcli con delete "$AP_CON" >/dev/null 2>&1 || true
nmcli con add type wifi ifname "$WIFI_IF" con-name "$AP_CON" \
  connection.autoconnect yes connection.autoconnect-priority 100 connection.autoconnect-retries 0 \
  802-11-wireless.mode ap 802-11-wireless.ssid "$AP_SSID" \
  802-11-wireless.band "$AP_BAND" 802-11-wireless.channel "$AP_CHANNEL" 802-11-wireless.powersave 2 \
  wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
  wifi-sec.psk "$AP_PASSWORD" \
  ipv4.method shared ipv4.addresses "$AP_IP" ipv6.method ignore >/dev/null

echo "[4/4] Starting the access point"
nmcli device disconnect "$WIFI_IF" >/dev/null 2>&1 || true
sleep 1
nmcli con up "$AP_CON" >/dev/null
sleep 2
iw dev "$WIFI_IF" info | grep -E "ssid|type|channel" | sed 's/^\s*/      /'
echo ""
echo "Done. Wi-Fi '$AP_SSID' -> robot at 10.42.0.1 (Foxglove ws://10.42.0.1:8765, ssh $(logname 2>/dev/null || echo user)@10.42.0.1)."
echo "Ethernet stays the normal network; the AP comes back on every boot."
