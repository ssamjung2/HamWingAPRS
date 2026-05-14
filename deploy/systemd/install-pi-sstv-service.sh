#!/usr/bin/env bash
set -euo pipefail

# Install pi-sstv service + environment + system watchdog drop-in.
# Run on the Pi from repository root:
#   sudo bash deploy/systemd/install-pi-sstv-service.sh

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

SERVICE_SRC="${REPO_ROOT}/deploy/systemd/pi-sstv.service"
ENV_SRC="${REPO_ROOT}/deploy/systemd/pi-sstv.env"
WATCHDOG_SRC="${REPO_ROOT}/deploy/systemd/90-runtime-watchdog.conf"

SERVICE_DST="/etc/systemd/system/pi-sstv.service"
ENV_DST="/etc/default/pi-sstv"
WATCHDOG_DIR="/etc/systemd/system.conf.d"
WATCHDOG_DST="${WATCHDOG_DIR}/90-runtime-watchdog.conf"

if [[ ! -f "${SERVICE_SRC}" ]]; then
  echo "Missing source file: ${SERVICE_SRC}" >&2
  exit 1
fi
if [[ ! -f "${ENV_SRC}" ]]; then
  echo "Missing source file: ${ENV_SRC}" >&2
  exit 1
fi
if [[ ! -f "${WATCHDOG_SRC}" ]]; then
  echo "Missing source file: ${WATCHDOG_SRC}" >&2
  exit 1
fi

echo "Installing service unit to ${SERVICE_DST}"
install -m 0644 "${SERVICE_SRC}" "${SERVICE_DST}"

echo "Installing environment file to ${ENV_DST}"
install -m 0644 "${ENV_SRC}" "${ENV_DST}"

echo "Installing watchdog drop-in to ${WATCHDOG_DST}"
install -d -m 0755 "${WATCHDOG_DIR}"
install -m 0644 "${WATCHDOG_SRC}" "${WATCHDOG_DST}"

echo "Reloading systemd"
systemctl daemon-reload
systemctl daemon-reexec

echo "Enabling pi-sstv service"
systemctl enable pi-sstv.service

echo "Restarting pi-sstv service"
systemctl restart pi-sstv.service

echo "Status"
systemctl --no-pager --full status pi-sstv.service || true

echo "Done. Tail logs with: journalctl -u pi-sstv.service -f"
