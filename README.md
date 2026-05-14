<<<<<<< HEAD
<<<<<<< HEAD
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
=======
# HamWing SSTV HAB Payload

**Version 2.0**

An open-source High Altitude Balloon (HAB) payload controller. It captures images with a Raspberry Pi camera, encodes them as [SSTV](https://en.wikipedia.org/wiki/Slow-scan_television) audio using the SlowFrame encoder, and transmits them over RF through a DRA818 transceiver mounted on a HamWing carrier board.
=======
# HamWing SSTV HAB Payload

An open-source High Altitude Balloon (HAB) payload controller that captures images with a Raspberry Pi camera, encodes them as [SSTV](https://en.wikipedia.org/wiki/Slow-scan_television) audio, and transmits them over RF using a DRA818U transceiver on a HamWing carrier board.
>>>>>>> cbbea99 (HAB SSTV)

> **License:** This project is released as open-source software. Contributions, bug reports, and pull requests are welcome.
>
> **Legal notice:** Radio transmission requires a valid Amateur Radio license in your jurisdiction. Comply with all applicable regulations regarding airborne operation, frequencies, power limits, and identification requirements.

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
<<<<<<< HEAD
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Wiring Reference](#wiring-reference)
- [Audio Path](#audio-path)
- [Installation](#installation)
  - [Raspberry Pi Setup](#raspberry-pi-setup)
  - [Installing SlowFrame](#installing-slowframe)
  - [Optional: MMSSTV Library](#optional-mmsstv-library)
  - [Arduino and DRA818 Programming](#arduino-and-dra818-programming)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
  - [Generating a Config File](#generating-a-config-file)
  - [Config File Search Order](#config-file-search-order)
  - [Config Sections Reference](#config-sections-reference)
- [CLI Reference](#cli-reference)
  - [Verbs](#verbs)
  - [mission](#mission)
  - [run](#run)
  - [schedules](#schedules)
  - [config](#config)
  - [status](#status)
  - [service](#service)
  - [diag](#diag)
  - [help](#help)
  - [Legacy Flag Interface](#legacy-flag-interface)
- [SSTV Modes](#sstv-modes)
- [Transmit Schedule Profiles](#transmit-schedule-profiles)
  - [Built-in Presets](#built-in-presets)
  - [Custom Schedule Profiles](#custom-schedule-profiles)
  - [Fallback Mode Resolution](#fallback-mode-resolution)
- [Duty-Cycle and Cooldown System](#duty-cycle-and-cooldown-system)
- [Overlay System](#overlay-system)
- [Running as a systemd Service](#running-as-a-systemd-service)
- [GPIO Pin Reference](#gpio-pin-reference)
- [LED Status Codes (Arduino)](#led-status-codes-arduino)
- [Troubleshooting](#troubleshooting)
=======
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
>>>>>>> cbbea99 (HAB SSTV)
- [Contributing](#contributing)

---

## Hardware Overview

```
<<<<<<< HEAD
+----------------------------------------------------------+
|  High Altitude Balloon Gondola                           |
|                                                          |
|  +-------------+     +---------------+     +---------+  |
|  | Raspberry Pi|---->| HamWing Board |---->| DRA818  |  |
|  | + Camera    |     | (RC LPF,      |     | VHF/UHF |  |
|  +-------------+     |  GPIO header) |     |  TRX    |  |
|  +-------------+     +---------------+     +----+----+  |
|  | Arduino     |---->|                          |       |
|  | (one-time   |     |                       Antenna    |
|  |  programmer)|                                        |
|  +-------------+                                        |
+----------------------------------------------------------+
```

The Arduino runs `hamwing_TNC.ino` once at power-up to program the DRA818 via its AT-command UART, then releases all control lines. The Raspberry Pi runs `pi_sstv.py` for the remainder of the flight, managing image capture, SSTV encoding, and radio control through GPIO.

---

## System Architecture

The pipeline for each transmitted image follows this sequence:

```
rpicam-still          SlowFrame             aplay
    |                     |                   |
    v                     v                   v
Capture JPEG  -->  Encode to WAV  -->  Play audio via PWM
                   (+ text overlay)         |
                                       DRA818 mic input
                                            |
                                       RF transmission
```

The script manages two independent gating mechanisms before any transmission is attempted:

1. **Per-mode cooldown** -- each SSTV mode enforces a minimum rest period equal to its own transmission duration after it last aired.
2. **Rolling duty-cycle budget** -- total transmit time within a configurable rolling window (default: 1 hour) cannot exceed a configurable fraction (default: 25%).

Both gates must pass before a transmission proceeds. If either blocks, the current image is discarded and the mission loop continues capturing.

---

## Hardware Requirements

| Component | Notes |
|-----------|-------|
| Raspberry Pi | Any model with a 40-pin GPIO header and camera connector. Pi Zero 2 W recommended for weight. |
| HamWing carrier board | Provides RC audio filter, GPIO breakout, and DRA818 power regulation. |
| DRA818V or DRA818U | DRA818V: 134-174 MHz (VHF). DRA818U: 400-470 MHz (UHF). |
| Raspberry Pi camera | OV5647 (V1), IMX219 (V2), or HQ camera. |
| Arduino Uno or Nano | Used once at power-up to program the DRA818 via AT commands. Not required after programming. |
| Antenna | 1/4-wave whip or dipole cut for your target frequency. |
| RC filter components | R1 270 ohm, R2 150 ohm, C1 33 nF, C2 10 uF (integrated on HamWing board). |
| Power supply | 3.3-5 V regulated; sized for Pi + camera + DRA818 current draw. |

---

## Software Requirements

### Raspberry Pi

| Package | Version | How to Install |
|---------|---------|----------------|
| Python | 3.9 or later | Pre-installed on Raspberry Pi OS |
| RPi.GPIO | Latest | `pip install RPi.GPIO` |
| rpicam-still | Any | Included with Raspberry Pi OS Bookworm and later |
| SlowFrame | Latest | See [Installing SlowFrame](#installing-slowframe) |
| aplay (ALSA utils) | Any | `sudo apt install alsa-utils` |
| MMSSTV shared library | Optional | Required for PD, Robot BW, and Fax modes. See [Optional: MMSSTV Library](#optional-mmsstv-library). |

### Arduino

| Requirement | Notes |
|-------------|-------|
| Arduino IDE 2.x or arduino-cli | [arduino.cc/en/software](https://www.arduino.cc/en/software) |
| No external libraries | Uses only the built-in `Serial` object and `millis()` |
| Compatible board | Arduino Uno, Nano, or any ATmega328P-based board |
=======
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
>>>>>>> cbbea99 (HAB SSTV)

---

## Wiring Reference

<<<<<<< HEAD
All Raspberry Pi pin numbers use BCM numbering unless stated otherwise.

### Raspberry Pi to HamWing to DRA818

| BCM Pin | Physical Pin | Signal | Logic |
|---------|-------------|--------|-------|
| GPIO4 | Pin 7 | PD (power-down) | HIGH = module active, LOW = sleep |
| GPIO27 | Pin 13 | PTT VHF | LOW = transmitting (active-LOW) |
| GPIO17 | Pin 11 | PTT UHF | LOW = transmitting (active-LOW); used when `band = uhf` or `band = both` |
| GPIO22 | Pin 15 | H/L power level | LOW = 0.5 W, HIGH = 1 W |
| GPIO12 | Pin 32 | PWM audio left | Output to RC filter input |
| GPIO13 | Pin 33 | PWM audio right | Output to RC filter input |
| GPIO21 | Pin 40 | Status LED | Configurable polarity; default active-HIGH |

### Arduino to HamWing to DRA818 (programming phase only)

| Arduino Pin | HamWing Net | DRA818 Pin | Notes |
|-------------|-------------|------------|-------|
| D0 (RX) | TXD | TXD | Hardware serial receive |
| D1 (TX) | RXD | RXD | Disconnect before sketch upload |
| D4 | PD | PD | Released to hi-Z after `setup()` completes |

---

## Audio Path

Pi GPIO12 and GPIO13 produce PWM audio. The HamWing board's passive RC filter converts the PWM signal to an analog audio level suitable for the DRA818 microphone input.

```
Pi GPIO12 / GPIO13 (PWM)
        |
      [R1 270 ohm]     Series; with R2 shunt forms voltage divider (× 0.36 attenuation)
        |
        +----[R2 150 ohm]----GND    Shunt resistor; sets output impedance
        |
        +----[C1 33 nF]-----GND    Shunt cap; LPF with R_th = R1||R2 = 96 ohm
        |                           Cutoff ~50 kHz; far above highest SSTV tone (~2.5 kHz)
      [C2 10 uF]               DC-blocking capacitor; removes bias voltage from the line
        |
        v
   DRA818 MIC input
```

The `dtoverlay=audremap,enable_jack=on` kernel overlay must be active for GPIO12/GPIO13 to carry audio. The default aplay device is `plughw:CARD=Headphones,DEV=0`. Both can be overridden in the `[alsa]` config section.
=======
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
>>>>>>> cbbea99 (HAB SSTV)

---

## Installation

### Raspberry Pi Setup

<<<<<<< HEAD
**Step 1: Enable the PWM audio overlay**

Add the following line to `/boot/firmware/config.txt` (Raspberry Pi OS Bookworm) or `/boot/config.txt` (Bullseye and earlier):
=======
**1. Enable the PWM audio overlay**

Add the following line to `/boot/firmware/config.txt` (Bookworm) or `/boot/config.txt` (Bullseye and earlier):
>>>>>>> cbbea99 (HAB SSTV)

```
dtoverlay=audremap,enable_jack=on
```

<<<<<<< HEAD
Reboot after saving.

To verify the overlay loaded:

```bash
aplay -L | grep -i headphones
```

**Step 2: Clone the repository**
=======
Reboot after editing.

**2. Clone the repository**
>>>>>>> cbbea99 (HAB SSTV)

```bash
git clone https://github.com/your-org/HamWingAPRS.git
cd HamWingAPRS
```

<<<<<<< HEAD
**Step 3: Install Python dependencies**
=======
**3. Install Python dependencies**
>>>>>>> cbbea99 (HAB SSTV)

```bash
pip install RPi.GPIO
```

<<<<<<< HEAD
**Step 4: Install ALSA utilities**
=======
**4. Install ALSA utilities** (if not already present)
>>>>>>> cbbea99 (HAB SSTV)

```bash
sudo apt update && sudo apt install -y alsa-utils
```

<<<<<<< HEAD
**Step 5: Run the system status check**

```bash
python3 pi_sstv.py status
```

This checks Python version, GPIO availability, `rpicam-still`, SlowFrame, MMSSTV library detection, `aplay`, audio device presence, output directory, config file, and systemd service state in one pass.

---

### Installing SlowFrame

SlowFrame is the SSTV encoder backend used by `pi_sstv.py`. Follow the upstream SlowFrame installation instructions. After installation, the binary should be accessible at:
=======
#### Installing SlowFrame

SlowFrame is the SSTV encoder used by `pi_sstv.py`. Follow the upstream installation instructions and ensure the `slowframe` binary is available at:
>>>>>>> cbbea99 (HAB SSTV)

```
/home/pi-user/Desktop/Slowframe/bin/slowframe
```

<<<<<<< HEAD
To use a different location, override it in your config file:

```ini
[paths]
slowframe = /usr/local/bin/slowframe
```

Or pass it at runtime:

```bash
python3 pi_sstv.py mission --config pi_sstv.cfg --slowframe /usr/local/bin/slowframe
```

---

### Optional: MMSSTV Library

A subset of SSTV modes require the MMSSTV shared encoder library. Modes that do not require it are labelled "native" throughout this documentation. If the library is not installed, any scheduled mode that requires it will automatically fall back to its configured native fallback mode -- no manual intervention is needed.

**Modes requiring MMSSTV:** `robot8bw`, `robot12bw`, `pd50`, `pd90`, `pd120`, `pd160`, `pd180`, `pd240`, `pd290`, `fax480`

To point SlowFrame to the library, set the environment variable before launching:

```bash
export MMSSTV_LIB_PATH=/opt/mmsstv/lib/libsstv_encoder.so
python3 pi_sstv.py mission --config pi_sstv.cfg
```

Or set it permanently in your config file:

```ini
[mmsstv]
lib_path = /opt/mmsstv/lib/libsstv_encoder.so
```

To disable MMSSTV detection and force native-only encoding regardless of library availability:
=======
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
>>>>>>> cbbea99 (HAB SSTV)

```bash
export SLOWFRAME_NO_MMSSTV=1
```

<<<<<<< HEAD
Or in config:

```ini
[mmsstv]
disable = true
```

---

### Arduino and DRA818 Programming

The Arduino programs the DRA818 module via its AT-command UART during `setup()`. After programming, it blinks the built-in LED to report the result and holds there; the Raspberry Pi takes over all radio control lines for the rest of the flight.

> **Important:** The Arduino D0/D1 hardware serial pins share the USB programming interface. Disconnect the DRA818 RXD wire before uploading the sketch, then reconnect it before power-cycling the board.

**Step 1: Edit the radio configuration constants** near the top of `hamwing_TNC.ino`:

```cpp
const float FREQ_TX   = 434.5000;  // Transmit frequency in MHz
const float FREQ_RX   = 434.5000;  // Receive frequency in MHz
const int   BANDWIDTH = 1;         // 0 = 12.5 kHz narrow, 1 = 25 kHz wide
const char* TX_CTCSS  = "0000";    // CTCSS tone code; "0000" = none
const char* RX_CTCSS  = "0000";
const int   SQUELCH   = 0;         // 0 = open squelch (recommended for HAB TX-only)
const int   VOLUME    = 4;         // DRA818 speaker volume, 1-8
// Keep all filters OFF for SSTV and data modes:
=======
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
>>>>>>> cbbea99 (HAB SSTV)
const int   FILTER_PRE  = 0;
const int   FILTER_HIGH = 0;
const int   FILTER_LOW  = 0;
```

<<<<<<< HEAD
**Step 2:** Select your board in Arduino IDE: Tools > Board > Arduino Uno or Arduino Nano.

**Step 3:** Upload the sketch.

**Step 4:** Reconnect the DRA818 RXD wire, then power-cycle.

**Step 5:** Observe the built-in LED to confirm programming status (see [LED Status Codes](#led-status-codes-arduino)).

---

## Quick Start

**1. Run a system readiness check:**

```bash
python3 pi_sstv.py status
```

**2. Generate a config file:**

```bash
python3 pi_sstv.py config generate pi_sstv.cfg
```

**3. Edit the config** -- at minimum, set your callsign:

```ini
[mission]
callsign = W1AW-11
```

**4. Validate the config:**

```bash
python3 pi_sstv.py config validate pi_sstv.cfg
```

**5. Do a dry run** (no radio transmission):

```bash
python3 pi_sstv.py mission --config pi_sstv.cfg --no-tx
```

**6. Start the mission:**

```bash
python3 pi_sstv.py mission --config pi_sstv.cfg
```
=======
**3. Select your board:** Tools → Board → Arduino Uno / Arduino Nano.

**4. Upload the sketch.**

**5. Reconnect the DRA818 RXD wire, then power-cycle.**

**6. Observe the built-in LED** (see [LED Status Codes](#led-status-codes-arduino)).

Once programming is confirmed, the Arduino is no longer needed for flight. The Raspberry Pi takes over all radio control lines.
>>>>>>> cbbea99 (HAB SSTV)

---

## Configuration

<<<<<<< HEAD
### Generating a Config File

```bash
# Write to pi_sstv.cfg in the current directory:
python3 pi_sstv.py config generate

# Write to a specific path:
python3 pi_sstv.py config generate /home/pi-user/pi_sstv.cfg
```

The generated file is fully commented. Every setting is documented inline, and all values default to sensible flight-ready defaults. Uncomment and edit only what you need to change.

### Config File Search Order

When a verb is run without `--config`, the script searches for `pi_sstv.cfg` in the following locations, in order:

| Priority | Path |
|----------|------|
| 1 | `./pi_sstv.cfg` (current working directory) |
| 2 | `~/.pi_sstv/pi_sstv.cfg` |
| 3 | `/etc/pi_sstv.cfg` |
| 4 | `/home/pi-user/pi_sstv.cfg` |

The first file found is used. The `status` and `config validate` verbs report which file was auto-discovered.

An explicit `--config PATH` flag always takes precedence over the search order.

### Config Sections Reference

#### [paths]

File and binary path overrides. All are optional; the built-in defaults are shown.

| Key | Default | Description |
|-----|---------|-------------|
| `output_dir` | `/home/pi-user/Desktop/HAB` | Directory for captured images, WAV files, and the CSV log |
| `slowframe` | `/home/pi-user/Desktop/Slowframe/bin/slowframe` | Path to the SlowFrame binary |
| `test_image` | `/home/pi-user/pi-sstv/test.jpg` | Fallback image when the camera is unavailable |
| `data_csv` | `/home/pi-user/data.csv` | Rolling capture index CSV log |

#### [mission]

Controls the main capture-and-transmit loop.

| Key | Default | Description |
|-----|---------|-------------|
| `enabled` | `true` | Set to `false` to disable the mission. The script starts and exits cleanly without capturing or transmitting. |
| `schedule` | `standard` | Transmit schedule preset name. See [Transmit Schedule Profiles](#transmit-schedule-profiles). |
| `unavailable_mode_fallback` | `r36` | Global fallback mode when a scheduled mode is unavailable and its per-mode fallback chain is exhausted. |
| `total` | `500` | Total number of image captures before the mission ends. |
| `interval` | `10` | Seconds to wait between captures. |
| `callsign` | _(blank)_ | Station callsign printed in the image overlay (e.g. `W1AW-11`). Automatically enables the callsign overlay when set. |
| `min_captures_between_transmissions` | `0` | Minimum number of capture cycles that must elapse between any two transmissions, regardless of cooldown state. `0` disables this floor and relies entirely on the cooldown and duty-cycle system. |
| `no_tx` | `false` | Skip all radio transmission. Images are captured and encoded but never played back. |

#### [test_panels]

Controls the `--test-panels` workflow for transmitting static images rather than live captures. All keys are optional.

| Key | Default | Description |
|-----|---------|-------------|
| `source` | _(blank)_ | Path to a single image file or a directory of images. Required when `--test-panels` is invoked. |
| `selection` | `sequential` | How to pick images from a directory: `sequential` or `random`. |
| `count` | `1` | Number of images to transmit when source is a directory. |
| `mode` | `pd50` | SSTV mode to use for test-panel transmissions. |
| `include_callsign_overlay` | `true` | Include the callsign overlay on test-panel images. |
| `include_timestamp_overlay` | `false` | Include the timestamp overlay on test-panel images. |
| `allow_tx_without_callsign` | `false` | Allow transmission without a callsign configured. The default `false` enforces the FCC Part 97 station identification requirement. |

#### [radio]

Duty-cycle, cooldown, and TX timing controls.

| Key | Default | Description |
|-----|---------|-------------|
| `band` | `vhf` | Active radio band: `vhf`, `uhf`, or `both`. Determines which PTT pin is driven. |
| `tx_power_level` | `low` | DRA818 output power: `low` (0.5 W, H/L pin LOW) or `high` (1 W, H/L pin HIGH). |
| `pd_idle_mode` | `release` | DRA818 PD pin behavior between transmissions: `release` (INPUT float, Feather-owned) or `sleep` (OUTPUT LOW). |
| `rolling_duty_cycle_window_seconds` | `3600` | Rolling window duration in seconds for duty-cycle accounting. |
| `max_transmit_duty_cycle` | `0.25` | Maximum fraction of the rolling window that may be used for transmission. `0.25` = 25% = 900 s per hour. |
| `cooldown_scale_factor` | `1.0` | Multiplier applied to all per-mode cooldown durations. `0.75` is more aggressive; `1.5` is more conservative. |
| `radio_wake_delay_seconds` | `1.5` | Seconds to wait after asserting PD=HIGH before keying PTT, allowing the DRA818 time to power up. |
| `ptt_key_delay_seconds` | `0.25` | Seconds to wait after asserting PTT=LOW before audio playback begins. |
| `post_playback_delay_seconds` | `0.5` | Seconds to wait after audio playback ends before releasing PTT. |

#### [capture]

Camera and image acquisition settings for `rpicam-still`.

| Key | Default | Description |
|-----|---------|-------------|
| `quality` | `93` | JPEG compression quality, 1-100. 93 is visually lossless. |
| `metering` | `matrix` | AE metering: `matrix` (multi-zone, recommended for HAB), `average`, or `spot`. |
| `exposure` | `sport` | Exposure profile: `sport` (faster shutter, reduces motion blur), `normal`, or `long`. |
| `awb` | `auto` | Auto white balance mode. Common values: `auto`, `daylight`, `cloudy`, `indoor`. |
| `capture_file_timeout` | `8` | Seconds to wait for `rpicam-still` to produce an output file before treating the capture as failed. |

#### [encode]

SlowFrame encoding settings.

| Key | Default | Description |
|-----|---------|-------------|
| `format` | `wav` | Audio container: `wav` (required for direct aplay playback), `aiff`, or `ogg`. |
| `sample_rate` | `22050` | Audio sample rate in Hz. 22050 is the SSTV standard. Do not reduce. |
| `aspect` | `center` | Image aspect-ratio handling: `center` (centered with black borders, recommended), `pad`, or `stretch`. |
| `verbose` | `false` | Enable verbose SlowFrame output. Logs full encoder diagnostics. |
| `sstv_conversion_settle_seconds` | `0.5` | Seconds to wait after SlowFrame finishes before aplay opens the WAV file. |

#### [overlay]

Controls timestamp and callsign text overlays baked into each transmitted image by SlowFrame.

| Key | Default | Description |
|-----|---------|-------------|
| `enable_timestamp` | `true` | Print the UTC capture timestamp on every image. |
| `timestamp_size` | `11` | Timestamp font size in points. |
| `timestamp_position` | `top-left` | Position: `top-left`, `top-right`, `bottom-left`, `bottom-right`, `top`, `bottom`, `left`, `right`, `center`. |
| `timestamp_color` | `white` | Text color: named CSS color or `#RRGGBB` hex. |
| `timestamp_background_color` | `black` | Background color behind the timestamp text. |
| `timestamp_background_opacity` | `70` | Background opacity, 0 (transparent) to 100 (opaque). |
| `enable_callsign` | `false` | Print the station callsign. Automatically enabled when `[mission] callsign` is set. |
| `callsign_size` | `13` | Callsign font size in points. |
| `callsign_position` | `top-right` | Position (same values as `timestamp_position`). |
| `callsign_color` | `white` | Callsign text color. |
| `callsign_background_color` | `black` | Background color behind the callsign text. |
| `custom_text` | _(blank)_ | When set, replaces the entire overlay with this free-form string. Suppresses both the timestamp and callsign overlays. Leave blank to use auto-generated overlay text. |

#### [mmsstv]

Controls the MMSSTV encoder library.

| Key | Default | Description |
|-----|---------|-------------|
| `lib_path` | _(blank)_ | Explicit path to the MMSSTV shared library. Leave blank to let SlowFrame auto-detect via `$MMSSTV_LIB_PATH`, pkg-config, or standard library paths. Use the unversioned symlink (e.g. `libsstv_encoder.so`, not `libsstv_encoder.so.1.0.0`). |
| `disable` | `false` | Force native-only SlowFrame modes regardless of library availability. Equivalent to `export SLOWFRAME_NO_MMSSTV=1`. |

#### [logging]

Log verbosity and output destination.

| Key | Default | Description |
|-----|---------|-------------|
| `debug` | `false` | Enable DEBUG-level output: full subprocess commands, GPIO state transitions, fallback chain steps, duty-cycle arithmetic. |
| `log_file` | _(blank)_ | Write log output to this file in addition to stdout. Leave blank to log to stdout only. |

#### [alsa]

ALSA audio output device.

| Key | Default | Description |
|-----|---------|-------------|
| `playback` | `plughw:CARD=Headphones,DEV=0` | aplay device string. This default matches `dtoverlay=audremap,enable_jack=on`. Change to `plughw:CARD=Headphones,DEV=0` with `pins_18_19` variant if using GPIO18/GPIO19. |

#### [status_led]

GPIO-driven LED that signals operational state to the operator.

| Key | Default | Description |
|-----|---------|-------------|
| `enabled` | `true` | Enable the status LED. |
| `pin` | `21` | BCM GPIO pin number. |
| `active_high` | `true` | `true` = GPIO HIGH turns the LED on (common-cathode wiring). `false` = GPIO LOW turns the LED on. |
| `pwm_freq` | `120` | PWM frequency in Hz for breathing/pulsing effects. |
| `max_pct` | `100` | Maximum brightness percentage, 0-100. Reduce if the LED is very bright. |

#### [schedule_profile NAME]

Define a custom transmit schedule. Any section named `[schedule_profile NAME]` becomes a selectable preset. See [Custom Schedule Profiles](#custom-schedule-profiles).

---

## CLI Reference

### Verbs

`pi_sstv.py` uses a verb-first command structure. The general form is:

```
python3 pi_sstv.py VERB [OPTIONS]
```

Running the script with no arguments prints a startup banner showing the detected Python version, SlowFrame status, MMSSTV library status, and auto-discovered config file, followed by a quick-reference list of common commands.

---

### mission

Run the full continuous HAB mission: capture images, encode to SSTV audio, and transmit on a rotating schedule until the configured total number of captures is reached.

```
python3 pi_sstv.py mission [OPTIONS]
```

| Option | Description |
|--------|-------------|
| `-c`, `--config PATH` | Load settings from a config file. |
| `-s`, `--schedule PRESET` | Transmit schedule preset. Overrides the config file setting. |
| `-n`, `--total N` | Total number of captures before the mission ends. |
| `-i`, `--interval SECS` | Seconds between captures. |
| `--callsign CALL` | Station callsign printed in the image overlay. |
| `--no-tx` | Capture and encode but never transmit. Safe for bench testing. |
| `--band vhf|uhf|both` | Radio band to key. |
| `--power low|high` | Transmit power level. |
| `--cooldown-scale FACTOR` | Multiply all mode cooldown durations by this factor. |
| `--duty-cycle FRACTION` | Maximum rolling TX fraction (0.0-1.0). |
| `--min-captures N` | Minimum captures between any two transmissions. |
| `--output-dir PATH` | Directory for images, WAV files, and CSV log. |
| `--debug` | Enable DEBUG-level logging. |
| `--log-file PATH` | Log to file in addition to stdout. |
| `--quiet-log-file PATH` | Log to file only; suppress stdout. |
| `--mmsstv-lib PATH` | Path to MMSSTV shared library. |
| `--no-mmsstv` | Force native SlowFrame modes only. |

**Example -- dry run with custom schedule:**

```bash
python3 pi_sstv.py mission --config pi_sstv.cfg --schedule native-balanced --no-tx
```

**Example -- flight with logging to file:**

```bash
python3 pi_sstv.py mission --config pi_sstv.cfg --log-file /home/pi-user/Desktop/HAB/flight.log
```

---

### run

Execute a single SSTV pipeline pass: capture (or use a supplied image), encode, and transmit. Exits after one image. Does not use the schedule, cooldown, or duty-cycle system.

```
python3 pi_sstv.py run [OPTIONS]
```

| Option | Description |
|--------|-------------|
| `-m`, `--mode MODE` | SSTV mode to encode. Default: `r36`. |
| `-c`, `--config PATH` | Load settings from a config file. |
| `--image PATH` | Use this image file instead of capturing from the camera. |
| `--callsign CALL` | Callsign overlay. |
| `--no-callsign` | Suppress the callsign overlay even if set in config. |
| `--no-timestamp` | Suppress the timestamp overlay. |
| `--no-tx` | Encode but skip radio transmission. |
| `--band vhf|uhf|both` | Radio band to key. |
| `--power low|high` | Transmit power level. |
| `--keep-wav` | Keep the encoded WAV file after the run. |
| `--output-dir PATH` | Directory for output artifacts. |
| `--debug` | Enable DEBUG-level logging. |
| `--mmsstv-lib PATH` | Path to MMSSTV shared library. |
| `--no-mmsstv` | Force native SlowFrame modes only. |

**Example -- bench test without radio:**

```bash
python3 pi_sstv.py run --mode pd90 --no-tx
```

**Example -- transmit a specific image:**

```bash
python3 pi_sstv.py run --mode m2 --image /home/pi-user/test.jpg
```

---

### schedules

List all available transmit schedule presets, including custom profiles defined in the config file.

```
python3 pi_sstv.py schedules [--config PATH]
```

The output shows each profile name, its mode rotation, a description, and the fallback policy active for that profile. Custom profiles loaded from a config file are labelled `[custom]`.

---

### config

Config file utilities.

```
python3 pi_sstv.py config generate [PATH]
python3 pi_sstv.py config validate PATH
python3 pi_sstv.py config show [PATH]
```

| Subcommand | Description |
|------------|-------------|
| `generate [PATH]` | Write a fully-commented default config file. Defaults to `./pi_sstv.cfg` if no path is given. |
| `validate PATH` | Parse and validate the config file. Reports errors, warnings, and unknown sections. |
| `show [PATH]` | Print the resolved config values that would be applied if the file were loaded, with sources (default vs overridden). Auto-discovers the config if no path is given. |

**Example:**

```bash
python3 pi_sstv.py config generate /home/pi-user/pi_sstv.cfg
python3 pi_sstv.py config validate /home/pi-user/pi_sstv.cfg
```

---

### status

Run a pre-flight system readiness check. Checks Python version, GPIO library, `rpicam-still`, SlowFrame binary, MMSSTV library detection, `aplay`, audio device, output directory, config file validity, and systemd service state.

```
python3 pi_sstv.py status [--config PATH] [--verbose]
```

Also displays a GPIO pin assignment summary showing configured band, power level, PD idle mode, and status LED settings with their config-derived effects.

---

### service

Manage the `pi-sstv.service` systemd unit.

```
python3 pi_sstv.py service install [--config PATH] [--user USER]
python3 pi_sstv.py service uninstall
python3 pi_sstv.py service start
python3 pi_sstv.py service stop
python3 pi_sstv.py service status
python3 pi_sstv.py service logs [--lines N]
```

| Subcommand | Description |
|------------|-------------|
| `install` | Write and enable `pi-sstv.service`. Prompts before overwriting an existing unit file. |
| `uninstall` | Stop, disable, and remove the service unit file. |
| `start` | Start the service immediately. |
| `stop` | Stop the running service. |
| `status` | Show the systemd service status. |
| `logs` | Tail the service journal. `--lines N` controls how many lines to show (default: 50). |

---

### diag

Run hardware diagnostic routines without starting a full mission.

```
python3 pi_sstv.py diag SUBCOMMAND [OPTIONS]
```

| Subcommand | Description |
|------------|-------------|
| `ptt SECONDS` | Assert the PTT pin for the given number of seconds, then release. Useful for verifying the RF path with a dummy load and SWR meter. |
| `led SECONDS` | Run a status LED blink test for the given duration. |
| `gps` | Read and print NMEA sentences from the GPS serial port. |
| `gpio` | Print the current GPIO pin states for all mission-critical pins. |
| `audio` | Play a short test tone through the audio path. |

**Example:**

```bash
python3 pi_sstv.py diag ptt 2.0
python3 pi_sstv.py diag led 3.0
```

---

### help

Print inline documentation for verbs and pipeline topics.

```
python3 pi_sstv.py help
python3 pi_sstv.py help SUBJECT
```

Available subjects: `mission`, `run`, `schedules`, `config`, `service`, `status`, `diag`, `modes`, `gpio`, `capture`, `encode`, `overlay`, `mmsstv`, `schedule`, `tx`, `logging`.

---

### Legacy Flag Interface

The original flag-based interface is fully supported and will continue to work. It is equivalent to the `mission` verb with per-flag overrides.

```bash
python3 pi_sstv.py --config pi_sstv.cfg
python3 pi_sstv.py --test r36 --no-tx
python3 pi_sstv.py --help
=======
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
>>>>>>> cbbea99 (HAB SSTV)
```

---

## SSTV Modes

<<<<<<< HEAD
All modes are addressed by their canonical name. Common aliases (e.g. `martin1`, `scottie2`, `robot36`) are also accepted anywhere a mode name is expected.

### Mode Aliases

| Alias | Canonical Name |
|-------|---------------|
| `bw8` | `robot8bw` |
| `bw12` | `robot12bw` |
| `robot24` | `bw24` |
| `robot36` | `r36` |
| `robot72` | `r72` |
| `martin1` | `m1` |
| `martin2` | `m2` |
| `scottie1` | `s1` |
| `scottie2` | `s2` |
| `scottiedx` | `sdx` |

### Mode Reference

| Mode | Duration | Image Size | MMSSTV Required | Native Fallback | Description |
|------|----------|------------|-----------------|-----------------|-------------|
| `robot8bw` | 8 s | 160x120 | Yes | `bw24` | Ultra-fast monochrome status frame. Minimizes airtime. |
| `robot12bw` | 12 s | 160x120 | Yes | `bw24` | Very fast monochrome. Good for high-frequency status updates. |
| `bw24` | 24 s | 320x120 | No | -- | Fast native monochrome. No library dependency. |
| `r36` | 36 s | 320x240 | No | -- | Fast native color. Recommended default for general use. |
| `m2` | 58 s | 320x256 | No | -- | Balanced native color with broad receiver compatibility. |
| `s2` | 71 s | 320x256 | No | -- | Native Scottie 2. Moderate airtime, strong compatibility. |
| `r72` | 72 s | 320x240 | No | -- | Higher-quality native Robot color. |
| `pd50` | 50 s | 320x256 | Yes | `m2` | Fast MMSSTV PD color. Efficient when library is available. |
| `pd90` | 90 s | 320x256 | Yes | `r36` | Popular fast MMSSTV color mode. |
| `m1` | 114 s | 320x256 | No | -- | High-quality native Martin 1. Less frequent transmissions. |
| `s1` | 110 s | 320x256 | No | -- | High-quality native Scottie 1. Periodic detail shots. |
| `sdx` | 269 s | 320x256 | No | -- | Native Scottie DX. Very high detail; long airtime. |
| `pd120` | 120 s | 640x496 | Yes | `m1` | MMSSTV quality mode with larger image canvas. |
| `pd160` | 160 s | 512x400 | Yes | `m1` | Slower MMSSTV quality mode for longer detail passes. |
| `pd180` | 180 s | 640x496 | Yes | `m1` | High-detail MMSSTV mode for occasional mission snapshots. |
| `fax480` | 180 s | 512x480 | Yes | `m1` | High-detail MMSSTV mode. Best reserved for test windows. |
| `pd240` | 240 s | 640x496 | Yes | `m1` | Very high quality. Reserve for science windows. |
| `pd290` | 290 s | 800x616 | Yes | `pd180` | Highest quality. Occasional best-shot captures at peak altitude. |

Cooldown duration equals transmission duration for all modes (1:1 ratio). This ensures the minimum inter-transmission gap is at least as long as the transmission itself, which together with the duty-cycle system prevents regulatory overruns.
=======
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
>>>>>>> cbbea99 (HAB SSTV)

---

## Transmit Schedule Profiles

<<<<<<< HEAD
A transmit schedule is an ordered list of SSTV modes that the mission rotates through. After each successful transmission, the schedule advances to the next mode. Individual mode cooldowns and the global duty-cycle budget apply independently and may cause one or more schedule steps to be skipped on any given cycle.

### Built-in Presets

| Profile | Modes | Description |
|---------|-------|-------------|
| `mono` | `robot12bw`, `bw24` | Monochrome-only. Maximizes update frequency. Best for tight duty-cycle or low-power operation. |
| `rapid` | `robot12bw`, `r36` | Fast color plus status framing. Good balance of cadence and color detail. |
| `standard` | `r36`, `pd50`, `pd90` | Balanced quality and frequency. **Default profile.** Recommended for general HAB operation. |
| `quality` | `pd50`, `pd120` | Quality-focused color imaging. Use when image detail matters more than update rate. |
| `high-res` | `pd120`, `pd290` | Maximum detail. Use when high-resolution frames are the primary goal. |
| `native-rapid` | `bw24`, `r36` | Native-only rapid imaging. Use when MMSSTV is unavailable and frequent updates are needed. |
| `native-balanced` | `r36`, `m2`, `s2` | Native-only balanced imaging. Good default when color quality is desired without MMSSTV dependency. |
| `native-detail` | `r72`, `m1`, `s1` | Native-only higher-detail imaging. Prioritizes image quality over cadence. |

Select a profile in your config file:

```ini
[mission]
schedule = native-balanced
```

Or override at launch:

```bash
python3 pi_sstv.py mission --config pi_sstv.cfg --schedule quality
```

### Custom Schedule Profiles

Define a custom profile by adding a `[schedule_profile NAME]` section to your config file:

```ini
[schedule_profile my-profile]
modes = r36, pd90, m1
description = Ascending-altitude mixed cadence profile.
```

The profile name is normalized to lowercase when loaded. Modes may be separated by commas, whitespace, or both. The profile becomes available as `--schedule my-profile` or `schedule = my-profile` in `[mission]`.

To set a fallback mode specific to this profile (used when a scheduled mode is unavailable):

```ini
[schedule_profile my-profile]
modes = pd50, pd120
unavailable_mode_fallback = r36
```

### Fallback Mode Resolution

When a scheduled mode is unavailable (e.g. MMSSTV library not detected), the script resolves a fallback in the following order:

1. The mode's own `fallback_mode` defined in its `ModeProfile` (e.g. `pd90` falls back to `r36`).
2. The profile-level `unavailable_mode_fallback` defined in `[schedule_profile NAME]`.
3. The global `unavailable_mode_fallback` setting in `[mission]` (default: `r36`).

If the resolved fallback is also unavailable, the step is skipped entirely and the schedule advances.

---

## Duty-Cycle and Cooldown System

### Per-Mode Cooldown

Each mode has a cooldown period equal to its transmission duration. After a mode transmits, that mode cannot transmit again until the cooldown has elapsed. Cooldowns can be scaled globally:

```ini
[radio]
cooldown_scale_factor = 1.25   # 25% longer cooldowns
```

Or at launch:

```bash
python3 pi_sstv.py mission --config pi_sstv.cfg --cooldown-scale 0.75
```

### Rolling Duty-Cycle Budget

A rolling window (default: 1 hour) tracks total transmit time. The mission will not transmit if doing so would exceed the configured fraction of the window:

```ini
[radio]
rolling_duty_cycle_window_seconds = 3600
max_transmit_duty_cycle = 0.25   # 25% = 900 seconds per hour maximum
```

The rolling window is evaluated at the moment of each potential transmission. Old transmit records older than the window are expired automatically.

### Minimum Capture Floor

An optional capture counter floor prevents transmission if fewer than N capture cycles have elapsed since the last transmission:

```ini
[mission]
min_captures_between_transmissions = 3
```

The default is `0`, which disables this floor and relies entirely on the cooldown and duty-cycle system.

---

## Overlay System

The overlay system bakes text directly into each image before encoding. SlowFrame renders the overlay at encode time, so it appears in the received image as part of the SSTV frame.

### Default Behavior

When a callsign is configured and the timestamp overlay is enabled, the two are merged into a single overlay string: `CALLSIGN  YYYY-MM-DD HH:MM:SS UTC`. This avoids the SlowFrame limitation where multiple overlay arguments at the same position overwrite each other.

### Custom Overlay Text

To replace the entire overlay with a fixed string for every image:

```ini
[overlay]
custom_text = KW9D-11 HAB FLIGHT 2026
```

When `custom_text` is set, both the timestamp and callsign overlays are suppressed. Only the custom string is rendered.

### Overlay Position and Style

Position names accepted by SlowFrame:

```
top-left    top    top-right
left        center right
bottom-left bottom bottom-right
```

Colors accept named CSS colors (`white`, `black`, `yellow`, `red`, etc.) or hex values (`#RRGGBB`).

---

## Running as a systemd Service

The recommended way to run the payload during flight is as a systemd service. This ensures the mission restarts automatically after a power brownout or unexpected exit.

**Install the service:**

```bash
python3 pi_sstv.py service install --config /home/pi-user/Desktop/pi_sstv/pi_sstv.cfg
```

The script writes the unit file, reloads systemd, and enables the service. It will prompt before overwriting an existing unit.

**Start, stop, and check status:**

```bash
python3 pi_sstv.py service start
python3 pi_sstv.py service stop
python3 pi_sstv.py service status
```

**View live logs:**

```bash
python3 pi_sstv.py service logs
python3 pi_sstv.py service logs --lines 100
```

**Uninstall:**

```bash
python3 pi_sstv.py service uninstall
=======
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
>>>>>>> cbbea99 (HAB SSTV)
```

---

## GPIO Pin Reference

<<<<<<< HEAD
All pins use BCM numbering (`GPIO.BCM`). All pin numbers are fixed in hardware and defined as constants in the script.

| BCM Pin | Physical Pin | Signal | Active State | Notes |
|---------|-------------|--------|--------------|-------|
| GPIO4 | 7 | DRA818 PD (power-down) | HIGH = module active | Idle behavior configurable via `pd_idle_mode` |
| GPIO27 | 13 | PTT VHF (active-LOW) | LOW = transmitting | Used when `band = vhf` or `band = both` |
| GPIO17 | 11 | PTT UHF (active-LOW) | LOW = transmitting | Used when `band = uhf` or `band = both` |
| GPIO22 | 15 | H/L power level | LOW = 0.5 W; HIGH = 1 W | Controlled by `tx_power_level` setting |
| GPIO12 | 32 | PWM audio left | Output | Routed through RC filter to DRA818 MIC |
| GPIO13 | 33 | PWM audio right | Output | Routed through RC filter to DRA818 MIC |
| GPIO21 | 40 | Status LED | Configurable | Active-HIGH by default; see `[status_led]` |

The `status` verb displays the current GPIO configuration, including which PTT pin is active based on the loaded config, the effective power level, and the PD idle mode in effect.
=======
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
>>>>>>> cbbea99 (HAB SSTV)

---

## LED Status Codes (Arduino)

<<<<<<< HEAD
After the DRA818 programming sequence completes in `setup()`, the Arduino blinks its built-in LED indefinitely to report the result:

| Blink Pattern | Meaning |
|---------------|---------|
| Slow blink (500 ms on / 500 ms off) | All AT commands acknowledged. Programming completed successfully. |
| Fast blink (100 ms on / 100 ms off) | One or more AT commands failed or timed out. |

If you see fast blinking:

- Verify the DRA818 RXD wire is reconnected after the sketch was uploaded.
- Check that TX and RX wires are not swapped.
- Confirm the baud rate in the sketch matches the DRA818 default (`DRA818_BAUD = 9600`).
- Verify the DRA818 is powered and the PD pin is pulled HIGH before the sketch issues AT commands.

---

## Troubleshooting

**`rpicam-still` not found**

Ensure you are running Raspberry Pi OS Bookworm or later, which replaces `raspistill` with `rpicam-still`. On Bullseye, install `rpicam-apps`:

```bash
sudo apt install rpicam-apps
```

**No audio output / aplay device not found**

Confirm the overlay is in `/boot/firmware/config.txt`:

```
dtoverlay=audremap,enable_jack=on
```

Then verify it loaded:

```bash
aplay -L | grep Headphones
```

If using GPIO18/GPIO19 instead of GPIO12/GPIO13, use:

```
dtoverlay=audremap,pins_18_19,enable_jack=on
```

And update the config:

```ini
[alsa]
playback = plughw:CARD=Headphones,DEV=0
```

**MMSSTV modes falling back to native**

The MMSSTV library is not detected. Check:

```bash
python3 pi_sstv.py status
```

If SlowFrame is found but MMSSTV is not, set the library path:

```ini
[mmsstv]
lib_path = /path/to/libsstv_encoder.so
```

**Mission exits immediately**

Check that `[mission] enabled = true` in your config file. If `enabled = false`, the script starts, logs a notice, and exits cleanly without capturing or transmitting.

**Config file not found**

Run `python3 pi_sstv.py status` to see which paths are searched and which was auto-discovered. Generate a new config at the default location:

```bash
python3 pi_sstv.py config generate
```

**Permission denied on GPIO**

Add your user to the `gpio` group:

```bash
sudo usermod -aG gpio $USER
```

Then log out and back in.
=======
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
>>>>>>> cbbea99 (HAB SSTV)

---

## Contributing

1. Fork the repository and create a feature branch.
2. Test changes on hardware before submitting a pull request.
<<<<<<< HEAD
3. Follow existing code style; add comments for any hardware-specific behavior.
4. For bug reports, include: Raspberry Pi OS version, Python version (`python3 pi_sstv.py status`), and the relevant log output from `--debug` mode.

<<<<<<< HEAD
For questions, open a GitHub Issue or reach out via the amateur radio community forums.
>>>>>>> cbbea99 (HAB SSTV)
=======
For questions, open a GitHub Issue or reach out through the amateur radio community forums.
>>>>>>> 5a362d4 (Latest updates and bug fixes.  Added GPS support for altitude and grid sq location)
=======
3. Follow existing code style and include comments for any hardware-specific behavior.
4. For bug reports, include your Raspberry Pi OS version, Python version (`python3 --version`), and the relevant log output.

For questions, open a GitHub Issue or reach out via the amateur radio community forums.
>>>>>>> cbbea99 (HAB SSTV)
