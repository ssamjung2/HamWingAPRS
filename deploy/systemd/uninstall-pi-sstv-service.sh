#!/usr/bin/env bash
set -euo pipefail

# Remove pi-sstv service artifacts.
# Run on the Pi:
#   sudo bash deploy/systemd/uninstall-pi-sstv-service.sh

SERVICE_DST="/etc/systemd/system/pi-sstv.service"
ENV_DST="/etc/default/pi-sstv"
WATCHDOG_DST="/etc/systemd/system.conf.d/90-runtime-watchdog.conf"

echo "Stopping/disabling pi-sstv.service if present"
systemctl stop pi-sstv.service || true
systemctl disable pi-sstv.service || true

echo "Removing files"
rm -f "${SERVICE_DST}" "${ENV_DST}" "${WATCHDOG_DST}"

echo "Reloading systemd"
systemctl daemon-reload
systemctl daemon-reexec

echo "Done"
