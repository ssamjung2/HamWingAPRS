# HamWing APRS / SSTV Deployment Notes

This repository contains:

- `pi_sstv.py`: Raspberry Pi SSTV mission script (camera -> SlowFrame -> ALSA playback -> DRA818 TX)
- `HamWing_RadioConfig/HamWing_RadioConfig/HamWing_RadioConfig.ino`: Feather M0 radio config + diagnostics
- `deploy/systemd/*`: production service/watchdog units
- `deploy/install/pi-zero2w-bootstrap.sh`: fresh Pi Zero 2 W setup script
- `deploy/config/*`: install-time config templates

## Quick Start (Fresh Pi Zero 2 W)

1. Copy this repo to the Pi (for example to `/home/pi-user/Desktop/HamWingAPRS`).
2. Run the bootstrap installer as root from the repo root:

```bash
cd /home/pi-user/Desktop/HamWingAPRS
sudo bash deploy/install/pi-zero2w-bootstrap.sh
```

If SlowFrame is installed in a non-default location, pass it explicitly:

```bash
sudo bash deploy/install/pi-zero2w-bootstrap.sh --slowframe-bin /custom/path/slowframe
```

3. Reboot once after install:

```bash
sudo reboot
```

4. Verify service and logs:

```bash
sudo systemctl status pi-sstv.service
journalctl -u pi-sstv.service -f
```

## What the Bootstrap Script Configures

- Installs required packages (Python, camera tools, ALSA utilities, GPIO/serial support)
- Enables Pi interfaces needed by this project (I2C + UART hardware, disables UART login shell)
- Applies audio-related `config.txt` lines from `deploy/config/config.txt.audio-snippet.conf`
- Installs and enables `pi-sstv.service`
- Installs runtime watchdog systemd drop-in
- Seeds `/home/pi-user/pi_sstv.cfg` from template if missing
- Creates output folders and sets ownership
- Verifies SlowFrame binary exists and is executable before enabling service

## Runtime Tuning

Service environment lives in `/etc/default/pi-sstv` (installed from `deploy/systemd/pi-sstv.env`).

Useful variables:

- `PI_SSTV_ALSA_DEVICE`
- `PI_SSTV_ALSA_TARGET_VOLUME`
- `PI_SSTV_ALSA_MAX_SAFE_VOLUME`
- `PI_SSTV_ALSA_MIXER_DEVICE`
- `PI_SSTV_ALSA_MIXER_CONTROL`

## One-Time Validation

Run a transmit test manually:

```bash
cd /home/pi-user/Desktop
python3 ./pi_sstv.py --test r36 --callsign kw9d --radio uhf
```

Check playback devices:

```bash
aplay -L
```

## Crash Recovery Validation

```bash
sudo pkill -f "python3 .*pi_sstv.py"
sudo systemctl status pi-sstv.service
```

Service should restart automatically.
