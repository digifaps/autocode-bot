# Robot networking: Ethernet + Wi-Fi access point

| Interface | Role | Address |
|---|---|---|
| `eth0` | Normal networking: DHCP from your router, internet, SSH | from your router (e.g. 192.168.8.174) |
| `wlan0` | Always the robot's Wi-Fi access point | **10.42.0.1**, DHCP for clients (10.42.0.x) |

Both are NetworkManager connections that come up at boot. Connect a laptop or phone to the robot's Wi-Fi and use:

- Foxglove: `ws://10.42.0.1:8765`
- SSH: `ssh <user>@10.42.0.1`

When Ethernet is plugged in, clients on the access point also get internet through it (NetworkManager `shared` mode). The robot stack (`autocode-bot` service) does not depend on either network: it starts without a cable and with no Wi-Fi client connected.

## Setup (once)

```bash
sudo ~/ros2_ws/src/autocode-bot/scripts/setup_network.sh <wifi-password> [ssid]
```

- **SSID** defaults to `autocode-bot`; the **password** must be 8–63 characters (WPA2). It is stored only in NetworkManager on the robot (`/etc/NetworkManager/system-connections/`), never in this repo.
- Creates the connection `autocode-bot-ap` (autoconnect, highest priority, 2.4 GHz channel 6, power saving off) and removes older access point profiles.
- Keeps other Wi-Fi client profiles (e.g. your home network) but turns their autoconnect off, so `wlan0` stays an access point after every boot.
- Ethernet: makes sure the wired connection uses DHCP, autoconnects and carries the default route.

**Band:** the Wi-Fi country is not set on this Jetson (regulatory domain "world"), which only allows an access point on 2.4 GHz. For 5 GHz set the country first (e.g. `sudo iw reg set BE`, persistently in `/etc/default/crda`), then run the script with `AP_BAND=a AP_CHANNEL=36`.

Change the password later by running the script again with the new one.

## Temporarily use Wi-Fi as a client

```bash
sudo ~/ros2_ws/src/autocode-bot/scripts/jetson_wifi_client.sh [connection-name]   # e.g. your home Wi-Fi
sudo nmcli connection up autocode-bot-ap                                           # back to access point
```

The access point also comes back on the next boot.

## Checks

```bash
nmcli connection show --active      # autocode-bot-ap on wlan0, wired connection on eth0
iw dev wlan0 info                   # type AP, ssid, channel
iw dev wlan0 station dump           # connected clients
```

Foxglove's Diagnostics panel shows the same under **Network** (access point and number of clients, Ethernet address).
