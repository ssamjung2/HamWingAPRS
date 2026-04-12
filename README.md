# HamWing SSTV HAB Payload

An open-source High Altitude Balloon (HAB) payload controller that captures images with a Raspberry Pi camera, encodes them as [SSTV](https://en.wikipedia.org/wiki/Slow-scan_television) audio, and transmits them over RF using a DRA818U transceiver on a HamWing carrier board.

> **License:** This project is released as open-source software. Contributions, bug reports, and pull requests are welcome.
>
> **Legal notice:** Radio transmission requires a valid Amateur Radio license in your jurisdiction. Comply with all applicable regulations regarding airborne operation, frequencies, power limits, and identification requirements.

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Wiring Reference](#wiring-reference)
- [Repository Contents](#repository-contents)
- [Software Requirements](#software-requirements)
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
  - [Raspberry Pi Setup](#raspberry-pi-setup)
  - [Arduino / DRA818U Programming](#arduino--dra818u-programming)
- [Configuration](#configuration)
- [SSTV Modes](#sstv-modes)
- [Transmit Schedule Profiles](#transmit-schedule-profiles)
- [Running the Payload](#running-the-payload)
- [CLI Reference](#cli-reference)
- [GPIO Pin Reference](#gpio-pin-reference)
- [Audio Path](#audio-path)
- [LED Status Codes (Arduino)](#led-status-codes-arduino)
- [Contributing](#contributing)

---

## Hardware Overview

```
┌──────────────────────────────────────────────────────────┐
│  High Altitude Balloon Gondola                           │
│                                                          │
│  ┌─────────────┐     ┌───────────────┐     ┌─────────┐  │
│  │ Raspberry Pi│────►│ HamWing Board │────►│ DRA818U │  │
│  │ + Camera    │     │ (RC LPF,      │     │ VHF/UHF │  │
│  └─────────────┘     │  GPIO break-  │     │ TRX     │  │
│  ┌─────────────┐     │  out header)  │     └────┬────┘  │
│  │ Arduino     │────►│               │          │       │
│  │ (one-time   │     └───────────────┘          │       │
│  │  programmer)│                           Antenna      │
│  └─────────────┘                                        │
└──────────────────────────────────────────────────────────┘
```

The Arduino runs `hamwing_TNC.ino` **once at power-up** to configure the DRA818U via its AT-command UART, then releases control. The Raspberry Pi runs `pi_sstv.py` for the remainder of the flight, driving PTT, power-down, and power-level via GPIO while streaming SSTV audio through the PWM pins.

---

## Wiring Reference

### Arduino → HamWing → DRA818U (programming phase)

| Arduino Pin | HamWing Net | DRA818U Pin | Notes |
|-------------|-------------|-------------|-------|
| D0 (RX) | TXD (J5) | TXD | Hardware serial |
| D1 (TX) | RXD (J5) | RXD | **Disconnect during sketch upload** |
| D4 | PD (J3) | PD | Released to hi-Z after `setup()` |

### Raspberry Pi → HamWing → DRA818U (runtime)

| Pi BCM | Physical Pin | Net | Function |
|--------|-------------|-----|----------|
| GPIO4 | Pin 7 | PD | Power-down (HIGH = active, LOW = sleep) |
| GPIO27 | Pin 13 | PTT | Push-to-talk (LOW = TX keyed) |
| GPIO22 | Pin 15 | HL | Power level (LOW = low, HIGH = high) |
| GPIO12 | Pin 32 | AudioOut L | PWM audio left channel |
| GPIO13 | Pin 33 | AudioOut R | PWM audio right channel |

---

## Repository Contents

| File | Description |
|------|-------------|
| `pi_sstv.py` | Main HAB payload controller — image capture, SSTV encoding, and radio transmission |
| `hamwing_TNC.ino` | Arduino sketch — one-time DRA818U AT-command programmer |
| `README.md` | This file |

---

## Software Requirements

### Raspberry Pi (`pi_sstv.py`)

| Dependency | Version | Install |
|-----------|---------|---------|
| Python | ≥ 3.7 | Pre-installed on Raspberry Pi OS |
| RPi.GPIO | Latest | `pip install RPi.GPIO` |
| rpicam-still | Latest | Included with Raspberry Pi OS (Bookworm+) |
| SlowFrame | Latest | See [SlowFrame installation](#installing-slowframe) |
| aplay (ALSA) | Any | `sudo apt install alsa-utils` |
| MMSSTV library | Optional | Required for PD and Robot modes — see [MMSSTV](#optional-mmsstv-library) |

### Arduino (`hamwing_TNC.ino`)

| Dependency | Notes |
|-----------|-------|
| Arduino IDE ≥ 2.x **or** arduino-cli | [arduino.cc/en/software](https://www.arduino.cc/en/software) |
| No external libraries | Uses only built-in `Serial` and `millis()` |
| Target board | Arduino Uno, Nano, or compatible |

---

## Hardware Requirements

- Raspberry Pi (any model with a camera connector and 40-pin GPIO header; Pi Zero 2 W recommended for weight)
- HamWing Feather board
- DRA818U (400–470 MHz) or DRA818V (134–174 MHz) transceiver module
- Arduino Uno or Nano Feather
- Raspberry Pi camera module (OV5647 / IMX219 / HQ Camera)
- Dipole or 1/4-wave whip antenna cut for your target frequency
- RC audio filter components: R1 270 Ω, R2 150 Ω, C1 33 nF, C2 10 µF (see [Audio Path](#audio-path))
- Power supply suitable for the payload (3.3 V–5 V regulated)

---

## Installation

### Raspberry Pi Setup

**1. Enable the PWM audio overlay**

Add the following line to `/boot/firmware/config.txt` (Bookworm) or `/boot/config.txt` (Bullseye and earlier):

```
dtoverlay=audremap,enable_jack=on
```

Reboot after editing.

**2. Clone the repository**

```bash
git clone https://github.com/your-org/HamWingAPRS.git
cd HamWingAPRS
```

**3. Install Python dependencies**

```bash
pip install RPi.GPIO
```

**4. Install ALSA utilities** (if not already present)

```bash
sudo apt update && sudo apt install -y alsa-utils
```

#### Installing SlowFrame

SlowFrame is the SSTV encoder used by `pi_sstv.py`. Follow the upstream installation instructions and ensure the `slowframe` binary is available at:

```
/home/pi-user/Desktop/Slowframe/bin/slowframe
```

Or override the path at runtime via the config file:

```ini
[paths]
slowframe = /your/path/to/slowframe
```

#### Optional: MMSSTV Library

Some SSTV modes (PD50, PD90, PD120, PD160, PD180, PD240, PD290, Robot 8 BW, Robot 12 BW) require the MMSSTV shared encoder library. Native modes (M1, M2, S1, S2, R36, R72, BW24, M4) work without it.

If the library is installed, point SlowFrame to it via environment variable:

```bash
export MMSSTV_LIB_PATH=/opt/mmsstv/lib/libsstv_encoder.so
```

Or set `lib_path` in `[mmsstv]` in your config file.

To disable MMSSTV detection and force native-only modes:

```bash
export SLOWFRAME_NO_MMSSTV=1
```

Modes that require MMSSTV will automatically fall back to their configured native fallback mode when the library is unavailable.

---

### Arduino / DRA818U Programming

> **Important:** The Arduino D0/D1 hardware serial pins share the USB programming interface. **Disconnect the DRA818 RXD wire before uploading the sketch**, then reconnect it before power-cycling.

**1. Open `hamwing_TNC.ino`** in Arduino IDE.

**2. Edit the radio configuration constants** near the top of the file:

```cpp
const float FREQ_TX  = 434.5000;  // Transmit frequency (MHz)
const float FREQ_RX  = 434.5000;  // Receive frequency (MHz)
const int   BANDWIDTH = 1;        // 0 = 12.5 kHz narrow, 1 = 25 kHz wide
const char* TX_CTCSS  = "0000";   // CTCSS tone ("0000" = none)
const char* RX_CTCSS  = "0000";
const int   SQUELCH   = 0;        // 0 = open squelch (recommended for HAB TX-only)
const int   VOLUME    = 4;        // DRA818 speaker volume, 1–8
// Filters — keep all OFF for SSTV / data:
const int   FILTER_PRE  = 0;
const int   FILTER_HIGH = 0;
const int   FILTER_LOW  = 0;
```

**3. Select your board:** Tools → Board → Arduino Uno / Arduino Nano.

**4. Upload the sketch.**

**5. Reconnect the DRA818 RXD wire, then power-cycle.**

**6. Observe the built-in LED** (see [LED Status Codes](#led-status-codes-arduino)).

Once programming is confirmed, the Arduino is no longer needed for flight. The Raspberry Pi takes over all radio control lines.

---

## Configuration

Generate a fully-commented default configuration file:

```bash
python3 pi_sstv.py --generate-config pi_sstv.cfg
```

Then edit `pi_sstv.cfg` as needed and load it at runtime:

```bash
python3 pi_sstv.py --config pi_sstv.cfg
```

### Key Configuration Sections

| Section | Key Settings |
|---------|-------------|
| `[paths]` | `output_dir`, `slowframe`, `test_image`, `data_csv` |
| `[mission]` | `schedule`, `total`, `interval`, `callsign`, `min_captures_between_transmissions` |
| `[radio]` | `max_transmit_duty_cycle`, `cooldown_scale_factor`, `radio_wake_delay_seconds` |
| `[capture]` | `quality`, `metering`, `exposure`, `awb` |
| `[encode]` | `format`, `sample_rate`, `aspect`, `verbose` |
| `[overlay]` | Timestamp and callsign text overlay position, color, size |
| `[mmsstv]` | `lib_path`, `disable` |
| `[logging]` | `debug`, `log_file`, `quiet_log_file` |

For detailed documentation on any section:

```bash
python3 pi_sstv.py --explain <topic>
# Topics: capture  encode  overlay  mmsstv  modes  schedule  tx  gpio  logging
```

---

## SSTV Modes

| Mode | Duration | Cooldown | Requires MMSSTV | Fallback | Notes |
|------|----------|----------|-----------------|---------|-------|
| `robot8bw` | 8 s | 90 s | Yes | `bw24` | Ultra-fast monochrome status frame |
| `robot12bw` | 12 s | 90 s | Yes | `bw24` | Very fast monochrome |
| `bw24` | 24 s | 120 s | No | — | Fast native monochrome |
| `m4` | 29 s | 135 s | No | — | Fast Martin color |
| `r36` | 36 s | 150 s | No | — | Fast native color |
| `m2` | 58 s | 240 s | No | — | Balanced, wide compatibility |
| `pd50` | 50 s | 240 s | Yes | `m2` | Fast PD color |
| `s2` | 71 s | 300 s | No | — | Native Scottie |
| `r72` | 72 s | 300 s | No | — | Higher-quality Robot |
| `pd90` | 90 s | 360 s | Yes | `r36` | Popular fast color |
| `m1` | 114 s | 480 s | No | — | High-quality Martin |
| `s1` | 110 s | 480 s | No | — | High-quality Scottie |
| `pd120` | 120 s | 540 s | Yes | `m1` | Wide PD color |
| `pd160` | 160 s | 660 s | Yes | `m1` | Slow quality mode |
| `pd180` | 180 s | 720 s | Yes | `m1` | High-detail |
| `pd240` | 240 s | 900 s | Yes | `m1` | Very high quality |
| `pd290` | 290 s | 1080 s | Yes | `pd180` | Highest quality; best-shot use |

Modes that require MMSSTV fall back to their native equivalent automatically if the library is unavailable.

---

## Transmit Schedule Profiles

Set `schedule` under `[mission]` in your config file to one of the following presets:

| Profile | Description |
|---------|-------------|
| `hab_climb` | Maximum update-rate, mono-heavy. Best for the steepest part of the climb. |
| `hab_rapid` | Fast color bursts + one PD shot per rotation. Best during rapid ascent. |
| `hab_cruise` | Balanced; status frames mixed with progressive quality shots. **Recommended default.** |
| `hab_float` | Quality-first PD mode rotation. For float altitude and science windows. |

The rolling duty-cycle enforcement (`max_transmit_duty_cycle`, default 25 % per hour) prevents regulatory violations regardless of which profile is selected.

---

## Running the Payload

**Dry run — capture and encode without transmitting:**

```bash
python3 pi_sstv.py --config pi_sstv.cfg --no-tx
```

**Normal flight:**

```bash
python3 pi_sstv.py --config pi_sstv.cfg
```

**With debug logging:**

```bash
python3 pi_sstv.py --config pi_sstv.cfg --debug
```

**Run as a systemd service (recommended for flight):**

Create `/etc/systemd/system/pi_sstv.service`:

```ini
[Unit]
Description=HamWing SSTV HAB Payload Controller
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi-user/HamWingAPRS/pi_sstv.py --config /home/pi-user/pi_sstv.cfg
WorkingDirectory=/home/pi-user
StandardOutput=journal
StandardError=journal
Restart=on-failure
User=pi-user

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable pi_sstv.service
sudo systemctl start pi_sstv.service
```

---

## CLI Reference

```
python3 pi_sstv.py [OPTIONS]

  --config PATH             Load settings from a .cfg file
  --generate-config [PATH]  Write a default config file and exit
  --explain TOPIC           Print documentation for a config topic and exit
  --no-tx                   Capture and encode but do not transmit
  --debug                   Enable DEBUG-level log output
  --log-file PATH           Write log to file in addition to stdout
  --quiet-log-file PATH     Write log to file only; suppress stdout
```

---

## GPIO Pin Reference

All pin numbers use BCM numbering (`GPIO.BCM`).

| BCM | Physical Pin | Function | Active State |
|-----|-------------|----------|--------------|
| GPIO4 | Pin 7 | DRA818 PD (power-down) | HIGH = module active |
| GPIO27 | Pin 13 | DRA818 PTT | LOW = transmitting |
| GPIO22 | Pin 15 | DRA818 HL (power level) | LOW = low power, HIGH = high power |
| GPIO12 | Pin 32 | PWM audio left | — |
| GPIO13 | Pin 33 | PWM audio right | — |

---

## Audio Path

Pi GPIO12/13 output PWM audio through a passive RC filter before reaching the DRA818U microphone input:

```
Pi GPIO12/13
    │
    ▼
R1 (270 Ω)         — Voltage divider: attenuates 3.3 V Pi audio to safe mic level
    │
R2 (150 Ω) + C1 (33 nF)  — RC low-pass filter, cutoff ≈ 11 kHz
    │
C2 (10 µF)         — DC-blocking capacitor
    │
    ▼
DRA818U MIC input
```

The 270 Ω / 150 Ω divider reduces the Pi's 3.3 V audio swing to a level safe for the DRA818U mic input. The 33 nF capacitor limits audio bandwidth to ≈ 11 kHz — comfortably above the highest SSTV tone (~2.5 kHz). C2 blocks any DC bias on the line.

---

## LED Status Codes (Arduino)

After the DRA818U programming sequence completes in `setup()`, the sketch blinks the built-in LED indefinitely to report the result:

| Pattern | Meaning |
|---------|---------|
| Slow blink — 500 ms on / 500 ms off | All AT commands acknowledged — programming OK |
| Fast blink — 100 ms on / 100 ms off | One or more AT commands failed or timed out |

If you see fast blinking, verify:

- UART TX/RX wires are not swapped
- DRA818 RXD wire was reconnected after sketch upload
- Baud rate matches the DRA818U default (`DRA818_BAUD = 9600`)
- DRA818U is powered and PD pin is pulled HIGH before `Serial.begin()`

---

## Contributing

1. Fork the repository and create a feature branch.
2. Test changes on hardware before submitting a pull request.
3. Follow existing code style and include comments for any hardware-specific behavior.
4. For bug reports, include your Raspberry Pi OS version, Python version (`python3 --version`), and the relevant log output.

For questions, open a GitHub Issue or reach out via the amateur radio community forums.
