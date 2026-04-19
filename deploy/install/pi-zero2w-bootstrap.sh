#!/usr/bin/env bash
set -euo pipefail

# Fresh Pi Zero 2 W bootstrap for HamWing SSTV stack.
# Usage:
#   sudo bash deploy/install/pi-zero2w-bootstrap.sh
#   sudo bash deploy/install/pi-zero2w-bootstrap.sh --repo /home/pi-user/Desktop/HamWingAPRS --user pi-user --skip-upgrade

REPO_DIR=""
APP_USER="pi-user"
SKIP_UPGRADE=0
SLOWFRAME_BIN=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo)
      REPO_DIR="$2"
      shift 2
      ;;
    --user)
      APP_USER="$2"
      shift 2
      ;;
    --skip-upgrade)
      SKIP_UPGRADE=1
      shift
      ;;
    --slowframe-bin)
      SLOWFRAME_BIN="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run as root (sudo)." >&2
  exit 1
fi

if [[ -z "${REPO_DIR}" ]]; then
  REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fi

if [[ ! -d "${REPO_DIR}" ]]; then
  echo "Repo directory not found: ${REPO_DIR}" >&2
  exit 1
fi

if ! id "${APP_USER}" >/dev/null 2>&1; then
  echo "User does not exist: ${APP_USER}" >&2
  exit 1
fi

APP_HOME="$(getent passwd "${APP_USER}" | cut -d: -f6)"
if [[ -z "${APP_HOME}" || ! -d "${APP_HOME}" ]]; then
  echo "Unable to resolve home directory for user ${APP_USER}" >&2
  exit 1
fi

CONFIG_TEMPLATE_SRC="${REPO_DIR}/deploy/config/pi_sstv.cfg"
CONFIG_TEMPLATE_DST="${APP_HOME}/pi_sstv.cfg"
AUDIO_SNIPPET_SRC="${REPO_DIR}/deploy/config/config.txt.audio-snippet.conf"
SERVICE_INSTALLER="${REPO_DIR}/deploy/systemd/install-pi-sstv-service.sh"

if [[ -z "${SLOWFRAME_BIN}" ]]; then
  SLOWFRAME_BIN="${APP_HOME}/Desktop/Slowframe/bin/slowframe"
fi

for required in \
  "${CONFIG_TEMPLATE_SRC}" \
  "${AUDIO_SNIPPET_SRC}" \
  "${SERVICE_INSTALLER}" \
  "${REPO_DIR}/pi_sstv.py"; do
  if [[ ! -f "${required}" ]]; then
    echo "Missing required file: ${required}" >&2
    exit 1
  fi
done

echo "[1/10] apt update"
apt-get update

if [[ "${SKIP_UPGRADE}" -eq 0 ]]; then
  echo "[2/10] apt full-upgrade"
  DEBIAN_FRONTEND=noninteractive apt-get -y full-upgrade
else
  echo "[2/10] skipping apt full-upgrade (--skip-upgrade set)"
fi

echo "[3/10] installing software dependencies"
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  python3 \
  python3-pip \
  python3-rpi.gpio \
  python3-serial \
  alsa-utils \
  libcamera-apps \
  v4l-utils \
  i2c-tools \
  raspi-config \
  git \
  ca-certificates

echo "[4/10] enabling interfaces (I2C + UART hardware; disable UART login shell)"
if command -v raspi-config >/dev/null 2>&1; then
  raspi-config nonint do_i2c 0 || true
  raspi-config nonint do_serial_hw 0 || true
  raspi-config nonint do_serial_cons 1 || true
else
  echo "raspi-config not found; skipping interface configuration"
fi

echo "[5/10] applying audio config.txt snippet"
if [[ -f /boot/firmware/config.txt ]]; then
  BOOT_CONFIG="/boot/firmware/config.txt"
elif [[ -f /boot/config.txt ]]; then
  BOOT_CONFIG="/boot/config.txt"
else
  echo "Unable to find config.txt (checked /boot/firmware/config.txt and /boot/config.txt)" >&2
  exit 1
fi

cp -a "${BOOT_CONFIG}" "${BOOT_CONFIG}.pre-hamwing.$(date +%Y%m%d-%H%M%S).bak"

while IFS= read -r line; do
  line_trimmed="$(echo "${line}" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
  [[ -z "${line_trimmed}" ]] && continue
  [[ "${line_trimmed}" == \#* ]] && continue

  if ! grep -Fqx "${line_trimmed}" "${BOOT_CONFIG}"; then
    echo "${line_trimmed}" >> "${BOOT_CONFIG}"
    echo "  + added: ${line_trimmed}"
  fi
done < "${AUDIO_SNIPPET_SRC}"

echo "[6/10] preparing runtime directories"
install -d -m 0755 "${APP_HOME}/Desktop/HAB"
chown -R "${APP_USER}:${APP_USER}" "${APP_HOME}/Desktop/HAB"

echo "[7/10] seeding pi_sstv.cfg if missing"
if [[ ! -f "${CONFIG_TEMPLATE_DST}" ]]; then
  install -m 0644 "${CONFIG_TEMPLATE_SRC}" "${CONFIG_TEMPLATE_DST}"
  chown "${APP_USER}:${APP_USER}" "${CONFIG_TEMPLATE_DST}"
  echo "  + created ${CONFIG_TEMPLATE_DST}"
else
  echo "  = existing ${CONFIG_TEMPLATE_DST} kept"
fi

echo "[8/10] checking SlowFrame binary"
if [[ ! -x "${SLOWFRAME_BIN}" ]]; then
  echo "Missing executable SlowFrame binary: ${SLOWFRAME_BIN}" >&2
  echo "Install/copy SlowFrame first, then rerun bootstrap." >&2
  echo "Hint: use --slowframe-bin /path/to/slowframe if installed elsewhere." >&2
  exit 1
fi

echo "  + found ${SLOWFRAME_BIN}"

echo "[9/10] installing systemd service/watchdog artifacts"
bash "${SERVICE_INSTALLER}"

if [[ "${APP_USER}" != "pi-user" ]]; then
  echo "  - adjusting service for user ${APP_USER}"
  sed -i "s|^User=.*$|User=${APP_USER}|" /etc/systemd/system/pi-sstv.service
  sed -i "s|^Group=.*$|Group=${APP_USER}|" /etc/systemd/system/pi-sstv.service
  sed -i "s|^WorkingDirectory=.*$|WorkingDirectory=${APP_HOME}/Desktop|" /etc/systemd/system/pi-sstv.service
  sed -i "s|ExecStart=/usr/bin/python3 /home/pi-user/Desktop/pi_sstv.py --config /home/pi-user/pi_sstv.cfg|ExecStart=/usr/bin/python3 ${APP_HOME}/Desktop/pi_sstv.py --config ${APP_HOME}/pi_sstv.cfg|" /etc/systemd/system/pi-sstv.service
  systemctl daemon-reload
  systemctl restart pi-sstv.service
fi
if [[ "${APP_USER}" != "pi-user" ]]; then
  echo "  - adjusting service for user ${APP_USER}"
  sed -i "s|^User=.*$|User=${APP_USER}|" /etc/systemd/system/pi-sstv.service
  sed -i "s|^Group=.*$|Group=${APP_USER}|" /etc/systemd/system/pi-sstv.service
  sed -i "s|^WorkingDirectory=.*$|WorkingDirectory=${APP_HOME}/Desktop|" /etc/systemd/system/pi-sstv.service
  sed -i "s|ExecStart=/usr/bin/python3 /home/pi-user/Desktop/pi_sstv.py --config /home/pi-user/pi_sstv.cfg|ExecStart=/usr/bin/python3 ${APP_HOME}/Desktop/pi_sstv.py --config ${APP_HOME}/pi_sstv.cfg|" /etc/systemd/system/pi-sstv.service
  systemctl daemon-reload
  systemctl restart pi-sstv.service
fi

echo "[10/10] final checks"
systemctl --no-pager --full status pi-sstv.service || true

echo
echo "Bootstrap complete."
echo "Reboot is recommended before flight operation: sudo reboot"
echo "After reboot, validate with:"
echo "  systemctl status pi-sstv.service"
echo "  journalctl -u pi-sstv.service -f"
