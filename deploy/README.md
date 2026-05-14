# Deployment Bundle

This folder contains install assets for a fresh Pi Zero 2 W.

## Contents

- `install/pi-zero2w-bootstrap.sh`: end-to-end setup for dependencies, interfaces, service, watchdog, config seeding
- `systemd/pi-sstv.service`: watchdog-managed SSTV mission service
- `systemd/pi-sstv.env`: runtime ALSA + volume guardrail settings
- `systemd/90-runtime-watchdog.conf`: PID1/system watchdog settings
- `systemd/install-pi-sstv-service.sh`: service-only installer
- `config/pi_sstv.cfg`: mission config template
- `config/config.txt.audio-snippet.conf`: audio-related config.txt lines

## Install on Pi

From repository root:

```bash
sudo bash deploy/install/pi-zero2w-bootstrap.sh
```

If SlowFrame is not at the default path (`/home/pi-user/Desktop/Slowframe/bin/slowframe`), pass:

```bash
sudo bash deploy/install/pi-zero2w-bootstrap.sh --slowframe-bin /path/to/slowframe
```

After install:

```bash
sudo reboot
```

## Verify

```bash
sudo systemctl status pi-sstv.service
journalctl -u pi-sstv.service -f
```
