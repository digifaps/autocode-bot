# Jetson WiFi: Client vs Access Point

Switch the Jetson between **client mode** (connected to your WiFi, e.g. zander) and **Access Point mode** (Jetson acts as a hotspot you connect to).

## Requirements

- NetworkManager: `sudo apt install network-manager`
- WiFi interface `wlan0` (typical on Jetson Xavier NX)

## Scripts (run on the Jetson)

From the repo root (e.g. `~/ros2_ws/src/autocode-bot` or wherever you cloned):

### 1. Switch to Access Point (Jetson as hotspot)

```bash
sudo scripts/jetson_wifi_ap.sh
```

- Creates/activates connection **JetsonAP** (SSID: `JetsonAP`, default password: `jetsonap123`).
- Jetson gets IP **10.42.0.1**; clients get DHCP from 10.42.0.x.
- Connect your laptop/phone to WiFi **JetsonAP**, then:
  - SSH: `ssh fabrice@10.42.0.1`
  - Web viewer: http://10.42.0.1:8080/web_viewer.html

**Custom SSID/password (before running):**

```bash
sudo JETSON_AP_SSID=MyRobot JETSON_AP_PASSWORD=mypass scripts/jetson_wifi_ap.sh
```

### 2. Switch back to Client (connect to zander)

```bash
sudo scripts/jetson_wifi_client.sh
```

- Stops the AP and connects to **zander** (or the connection name you set).
- Jetson gets an IP from your router (e.g. 192.168.8.110); use that for SSH/web.

**Different WiFi (e.g. other SSID):**

```bash
sudo JETSON_CLIENT_CONNECTION="YourSSID" scripts/jetson_wifi_client.sh
```

(List connections: `nmcli connection show`)

## Summary

| Mode   | Command                    | Jetson IP (typical) | You connect to   |
|--------|----------------------------|----------------------|------------------|
| Client | `sudo scripts/jetson_wifi_client.sh` | 192.168.8.110 (wlan0) | Your WiFi (zander) |
| AP     | `sudo scripts/jetson_wifi_ap.sh`     | 10.42.0.1            | WiFi "JetsonAP"  |

## Troubleshooting

- **"WiFi is disabled" / no wlan0:** Enable in BIOS or `rfkill unblock wifi`.
- **AP fails to start:** Some drivers don’t support AP mode; try a USB WiFi dongle that supports AP, or use **create_ap** (hostapd + dnsmasq) as fallback.
- **No internet when connected to JetsonAP:** AP mode is local-only by default; for internet you’d add NAT/forwarding from eth0 (optional, not in these scripts).
