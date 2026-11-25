#!/bin/bash
# Run the robot without the desktop and without services it does not use; frees ~1 GB of RAM.
#
#   sudo scripts/headless.sh          boot to the console (no GNOME), stop the desktop now, disable unused services
#   sudo scripts/headless.sh --undo   desktop at boot again and re-enable exactly the services this script disabled
#   sudo scripts/headless.sh --status show what is on / off
#
# The desktop is not removed: `sudo systemctl start gdm` starts it any time (monitor login or remote), without
# changing the boot default. The robot stack (autocode-bot.service), ssh, the Wi-Fi access point, avahi
# (<host>.local), nvargus (cameras) and fan control are not touched.
set -euo pipefail

# Services the robot does not use. Socket units too, or they start their service again on demand.
UNITS=(
  docker.socket docker.service containerd.service   # no containers in use
  bluetooth.service                                  # no Bluetooth devices
  ModemManager.service                               # no mobile modem
  rpcbind.socket rpcbind.service                     # NFS helper; no NFS mounts
  kerneloops.service                                 # crash report uploader
  switcheroo-control.service                         # dual-GPU switching for the desktop
  apt-daily.timer apt-daily-upgrade.timer            # daily package list refresh (upgrades are off anyway)
)
STATE_DIR=/var/lib/autocode-bot
STATE="$STATE_DIR/headless.disabled"   # the units this script disabled, for --undo

[[ $EUID -eq 0 ]] || { echo "run with sudo" >&2; exit 1; }

status() {
  echo "boot target: $(systemctl get-default)   desktop (gdm): $(systemctl is-active gdm 2>/dev/null || true)"
  for u in "${UNITS[@]}"; do
    printf "  %-28s %-9s %s\n" "$u" "$(systemctl is-enabled "$u" 2>/dev/null || true)" "$(systemctl is-active "$u" 2>/dev/null || true)"
  done
  free -m | awk '/^Mem:/ {printf "memory: %d MB used of %d MB\n", $3, $2}'
}

case "${1:-}" in
  --status)
    status
    ;;
  --undo)
    systemctl set-default graphical.target
    if [[ -f "$STATE" ]]; then
      mapfile -t units < "$STATE"
      [[ ${#units[@]} -gt 0 ]] && systemctl enable --now "${units[@]}"
      rm -f "$STATE"
    fi
    systemctl start gdm || true
    echo "desktop back at boot (started now too); re-enabled: ${units[*]:-nothing}"
    ;;
  "")
    mkdir -p "$STATE_DIR"
    touch "$STATE"
    for u in "${UNITS[@]}"; do
      systemctl list-unit-files "$u" --no-legend 2>/dev/null | grep -q . || continue   # not installed
      if [[ "$(systemctl is-enabled "$u" 2>/dev/null || true)" == enabled ]]; then
        systemctl disable --now "$u"
        grep -qx "$u" "$STATE" || echo "$u" >> "$STATE"
      else
        systemctl stop "$u" 2>/dev/null || true
      fi
    done
    systemctl set-default multi-user.target
    systemctl stop gdm || true   # ends the auto-login desktop session now; the robot stack keeps running
    echo "headless: console at boot, desktop stopped, unused services off (undo: sudo $0 --undo)"
    status
    ;;
  *)
    sed -n '2,10p' "$0"
    exit 1
    ;;
esac
