#!/usr/bin/env python3

import argparse
import configparser
import logging
import sys
from dataclasses import dataclass, field
import time
from subprocess import run
from datetime import datetime, timezone
import csv
import os
import re
from typing import List, Optional, Set, Tuple
import RPi.GPIO as GPIO

logger = logging.getLogger("pi_sstv")

# Paths and constants
BASE_DIR = "/home/pi-user"
DEFAULT_CONFIG_PATH = os.path.join(BASE_DIR, "pi_sstv.cfg")
TIMESTAMPED_DIR = os.path.join(BASE_DIR, "Desktop/HAB")
PI_SSTV_BIN = os.path.join(BASE_DIR, "pi-sstv", "pi-sstv")

# Alternate/updated converter: Slowframe
SLOWFRAME_BIN = "/home/pi-user/Desktop/Slowframe/bin/slowframe"
TEST_IMAGE = os.path.join(BASE_DIR, "pi-sstv", "test.jpg")
SSTV_WAV = os.path.join(TIMESTAMPED_DIR, "HAB-SSTV.wav")
DATA_CSV = os.path.join(BASE_DIR, "data.csv")
RPICAM_BIN = "/usr/bin/rpicam-still"
RPICAM_QUALITY = 93
RPICAM_METERING = "matrix"   # multi-zone; handles split sky/ground HAB scenes better than average
RPICAM_EXPOSURE = "sport"    # prefer faster shutter to reduce motion blur from balloon movement
RPICAM_AWB = "auto"          # adapts from ground-launch color temperature to stratospheric daylight
SLOWFRAME_LIST_TIMEOUT_SECONDS = 15
MMSSTV_LIBRARY_ENV_VAR = "MMSSTV_LIB_PATH"
MMSSTV_DISABLE_ENV_VAR = "SLOWFRAME_NO_MMSSTV"

# SlowFrame output settings
SLOWFRAME_AUDIO_FORMAT = "wav"
SLOWFRAME_SAMPLE_RATE = 22050
SLOWFRAME_ASPECT_MODE = "center"
SLOWFRAME_VERBOSE = False

# SlowFrame text overlay settings
# Positions: top-left, top-right, bottom-left, bottom-right, top, bottom, left, right, center
# Colors: named (white, black, yellow, ...) or hex (#RRGGBB)
# Opacity: 0-100 integer
SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY = True
SLOWFRAME_TIMESTAMP_OVERLAY_SIZE = 11
SLOWFRAME_TIMESTAMP_OVERLAY_POSITION = "top-left"
SLOWFRAME_TIMESTAMP_OVERLAY_COLOR = "white"
SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR = "black"
SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY = 70

# Optional station ID overlay / CW ID
STATION_CALLSIGN = ""
SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = False
SLOWFRAME_CALLSIGN_OVERLAY_SIZE = 13
SLOWFRAME_CALLSIGN_OVERLAY_POSITION = "top-right"
SLOWFRAME_CALLSIGN_OVERLAY_COLOR = "white"
SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR = "black"

# Uncomment and adjust if you want a different default mode or output behavior.
# Common modes: m1, m2, s1, s2, sdx, r36, r72, bw24, pd90, pd120
# Formats: wav, aiff, ogg
# Aspect modes: center, pad, stretch

PIC_INTERVAL = 10
PIC_TOTAL = 500
MIN_CAPTURES_BETWEEN_TRANSMISSIONS = 12
ROLLING_DUTY_CYCLE_WINDOW_SECONDS = 3600
MAX_TRANSMIT_DUTY_CYCLE = 0.25

# Scale all per-mode cooldowns globally. 1.0 = tuned defaults, 0.75 = aggressive, 1.5 = conservative.
COOLDOWN_SCALE_FACTOR = 1.0
CAPTURE_FILE_TIMEOUT = 8
SSTV_CONVERSION_SETTLE_SECONDS = 0.5
RADIO_WAKE_DELAY_SECONDS = 1
PTT_KEY_DELAY_SECONDS = 0.1
POST_PLAYBACK_DELAY_SECONDS = 0.5

CSV_HEADERS = ["Index", "Time"]
GPIO_PIN_MODE = GPIO.BCM

# Wiring reference (BCM numbering)
# DRA818 control: PD=GPIO4 (pin 7), PTT=GPIO27 (pin 13), HL=GPIO22 (pin 15)
# Audio out with dtoverlay=audremap,enable_jack=on: GPIO12/GPIO13 (pins 32/33)
# Route both PWM audio pins into the LPF input, then feed LPF output to the DRA818 mic input.
# Optional peripherals below are documented only and are not used by this script.

# DRA818 GPIO pin definitions
DRA818_PTT_PIN = 27
DRA818_POWER_DOWN_PIN = 4
DRA818_POWER_LEVEL_PIN = 22

# PWM audio routing used by aplay via the Raspberry Pi audio overlay
AUDIO_LEFT_PWM_PIN = 12
AUDIO_RIGHT_PWM_PIN = 13
AUDIO_OVERLAY = "dtoverlay=audremap,enable_jack=on"

# Alternate audio pair if config.txt uses dtoverlay=audremap,pins_18_19,enable_jack=on
# AUDIO_LEFT_PWM_PIN = 18
# AUDIO_RIGHT_PWM_PIN = 19

# Optional GPS UART pins (not used by this script)
# GPS_UART_TX_PIN = 14  # physical pin 8
# GPS_UART_RX_PIN = 15  # physical pin 10

# Optional I2C sensor pins (not used by this script)
# SENSOR_I2C_SDA_PIN = 2  # physical pin 3
# SENSOR_I2C_SCL_PIN = 3  # physical pin 5


@dataclass(frozen=True)
class ModeProfile:
    name: str
    duration_seconds: int
    cooldown_seconds: int
    image_width: int = 320
    requires_mmsstv: bool = False
    fallback_mode: Optional[str] = None
    description: str = ""


@dataclass
class RuntimeState:
    available_modes: Set[str] = field(default_factory=set)
    mmsstv_library_detected: bool = False
    mmsstv_library_path: Optional[str] = None
    schedule_index: int = 0
    last_transmit_capture_number: int = 0
    last_transmit_end_monotonic: float = 0.0
    transmit_history: List[Tuple[float, float]] = field(default_factory=list)


MODE_PROFILES = {
    "robot8bw": ModeProfile(
        name="robot8bw",
        duration_seconds=8,
        cooldown_seconds=90,
        image_width=160,
        requires_mmsstv=True,
        fallback_mode="bw24",
        description="Ultra-fast MMSSTV monochrome status frame for tight duty-cycle budgets.",
    ),
    "robot12bw": ModeProfile(
        name="robot12bw",
        duration_seconds=12,
        cooldown_seconds=90,
        image_width=160,
        requires_mmsstv=True,
        fallback_mode="bw24",
        description="Very fast MMSSTV monochrome mode for rapid update windows.",
    ),
    "bw24": ModeProfile(
        name="bw24",
        duration_seconds=24,
        cooldown_seconds=120,
        description="Fast monochrome native mode for low duty-cycle updates.",
    ),
    "m4": ModeProfile(
        name="m4",
        duration_seconds=29,
        cooldown_seconds=135,
        description="Fast native Martin color mode; half the airtime of M2; ideal for rapid-update ascent phases.",
    ),
    "r36": ModeProfile(
        name="r36",
        duration_seconds=36,
        cooldown_seconds=150,
        description="Fast native color mode for regular balloon image updates.",
    ),
    "m2": ModeProfile(
        name="m2",
        duration_seconds=58,
        cooldown_seconds=240,
        description="Balanced native mode with good compatibility.",
    ),
    "s2": ModeProfile(
        name="s2",
        duration_seconds=71,
        cooldown_seconds=300,
        description="Native Scottie mode with strong compatibility and moderate airtime.",
    ),
    "r72": ModeProfile(
        name="r72",
        duration_seconds=72,
        cooldown_seconds=300,
        description="Higher-quality native Robot mode.",
    ),
    "pd50": ModeProfile(
        name="pd50",
        duration_seconds=50,
        cooldown_seconds=240,
        requires_mmsstv=True,
        fallback_mode="m2",
        description="Fast MMSSTV PD mode for efficient color updates.",
    ),
    "pd90": ModeProfile(
        name="pd90",
        duration_seconds=90,
        cooldown_seconds=360,
        requires_mmsstv=True,
        fallback_mode="r36",
        description="Popular MMSSTV fast color mode when the encoder library is available.",
    ),
    "m1": ModeProfile(
        name="m1",
        duration_seconds=114,
        cooldown_seconds=480,
        description="High-quality native Martin mode for less frequent transmissions.",
    ),
    "s1": ModeProfile(
        name="s1",
        duration_seconds=110,
        cooldown_seconds=480,
        description="Native Scottie high-quality mode for periodic detail shots.",
    ),
    "pd120": ModeProfile(
        name="pd120",
        duration_seconds=120,
        cooldown_seconds=540,
        image_width=640,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Higher-quality MMSSTV mode with a larger cooldown budget.",
    ),
    "pd160": ModeProfile(
        name="pd160",
        duration_seconds=160,
        cooldown_seconds=660,
        image_width=512,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Slower MMSSTV quality mode for longer detail passes.",
    ),
    "pd180": ModeProfile(
        name="pd180",
        duration_seconds=180,
        cooldown_seconds=720,
        image_width=640,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="High-detail MMSSTV mode, best for occasional mission snapshots.",
    ),
    "fax480": ModeProfile(
        name="fax480",
        duration_seconds=180,
        cooldown_seconds=720,
        image_width=512,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="High-detail MMSSTV mode best reserved for test windows.",
    ),
    "pd240": ModeProfile(
        name="pd240",
        duration_seconds=240,
        cooldown_seconds=900,
        image_width=640,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Very high quality MMSSTV PD mode for science windows and horizon detail passes.",
    ),
    "pd290": ModeProfile(
        name="pd290",
        duration_seconds=290,
        cooldown_seconds=1080,
        image_width=800,
        requires_mmsstv=True,
        fallback_mode="pd180",
        description="Highest quality MMSSTV mode; reserve for occasional best-shot captures at peak altitude.",
    ),
}

TRANSMIT_SCHEDULE_PROFILE = "hab_cruise"
TRANSMIT_SCHEDULE_PROFILES = {
    # hab_climb: absolute maximum update rate, mono-heavy — steepest part of the climb.
    "hab_climb": (
        "robot8bw",
        "robot12bw",
        "bw24",
        "m4",
        "robot12bw",
        "r36",
    ),
    # hab_rapid: fast color rotation with short cooldowns — upper ascent / release phase.
    "hab_rapid": (
        "robot12bw",
        "m4",
        "r36",
        "robot12bw",
        "m4",
        "pd50",
    ),
    # hab_cruise: balanced default — mixes status frames and quality shots across the full flight.
    "hab_cruise": (
        "robot12bw",
        "r36",
        "m2",
        "pd90",
        "s2",
        "r72",
        "m1",
    ),
    # hab_float: quality-first — anchored by PD modes for float altitude and science windows.
    "hab_float": (
        "r36",
        "pd90",
        "robot12bw",
        "pd120",
        "robot12bw",
        "pd180",
        "r36",
        "pd240",
    ),
}
TRANSMIT_SCHEDULE_DESCRIPTIONS = {
    "hab_climb":  "Maximum update-rate profile. Monochrome frames dominate to minimise cooldown "
                  "gaps during the steepest part of the climb.",
    "hab_rapid":  "Short bursts + one PD color shot per rotation. Best during rapid ascent "
                  "when update rate matters more than image quality.",
    "hab_cruise": "Status frames mixed with progressive quality shots. Default mission profile "
                  "for the full flight envelope.",
    "hab_float":  "Quality-first rotation anchored by PD modes. Suited for float altitude or "
                  "slow-drift windows where airtime budget is relaxed.",
}

TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES.get(
    TRANSMIT_SCHEDULE_PROFILE,
    TRANSMIT_SCHEDULE_PROFILES["hab_cruise"],
)


def generate_default_config(path: str):
    """Write a fully-commented default configuration file to *path*."""
    schedules = ", ".join(TRANSMIT_SCHEDULE_PROFILES.keys())
    content = f"""\
# =============================================================================
# pi_sstv.cfg  —  HamWing SSTV HAB payload controller configuration
# =============================================================================
# This file controls every configurable setting in the pi_sstv pipeline.
# All values shown are the built-in defaults.  Uncomment and edit any line
# to override it.  Command-line flags always take precedence over this file.
#
# Generate a fresh copy of this file at any time:
#   python3 pi_sstv.py --generate-config [PATH]
#
# Load this file at runtime:
#   python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg
#
# For detailed documentation on any section, run:
#   python3 pi_sstv.py --explain <topic>
#   Topics: capture  encode  overlay  mmsstv  modes  schedule  tx  gpio  logging
# =============================================================================


# -----------------------------------------------------------------------------
# [paths]  File and binary path overrides
# -----------------------------------------------------------------------------
# Use this section to relocate output files or point to alternate binaries.
# For --explain reference: python3 pi_sstv.py --explain encode
[paths]

# Directory where captured images, WAV files, and the CSV log are written.
# output_dir = {TIMESTAMPED_DIR}

# Path to the SlowFrame SSTV encoder binary.
# slowframe = {SLOWFRAME_BIN}

# Fallback image used when the camera is unavailable (bench testing).
# test_image = {TEST_IMAGE}

# Path to the rolling CSV capture index log.
# data_csv = {DATA_CSV}


# -----------------------------------------------------------------------------
# [mission]  Main capture-and-transmit loop
# -----------------------------------------------------------------------------
# For --explain reference: python3 pi_sstv.py --explain schedule
[mission]

# Transmit schedule preset.  Controls which SSTV modes are used in rotation.
# Available presets: {schedules}
#   hab_climb  - Mono-heavy maximum update rate; for the steepest climb phase.
#   hab_rapid  - Fast color bursts; best for upper ascent and release.
#   hab_cruise - Balanced default; works across the full flight envelope.
#   hab_float  - Quality-first PD modes; suited for float altitude / science windows.
schedule = {TRANSMIT_SCHEDULE_PROFILE}

# Total number of image captures before the mission ends.
total = {PIC_TOTAL}

# Seconds to wait between captures.
interval = {PIC_INTERVAL}

# Station callsign printed in the image overlay (e.g. W1AW-11).
# Leave blank to disable the callsign overlay.
callsign = {STATION_CALLSIGN}

# Minimum number of capture cycles that must elapse between any two
# transmissions, regardless of cooldown state.
min_captures_between_transmissions = {MIN_CAPTURES_BETWEEN_TRANSMISSIONS}

# Skip all radio transmission.  Images are captured and encoded but never
# played back.  Useful for bench testing without a radio connected.
# no_tx = false


# -----------------------------------------------------------------------------
# [radio]  Duty-cycle, cooldown, and TX timing
# -----------------------------------------------------------------------------
# For --explain reference: python3 pi_sstv.py --explain tx
[radio]

# Rolling window duration in seconds used for duty-cycle accounting.
rolling_duty_cycle_window_seconds = {ROLLING_DUTY_CYCLE_WINDOW_SECONDS}

# Maximum transmit fraction of the rolling window (0.0 – 1.0).
# 0.25 = 25 % = 900 s per hour.
max_transmit_duty_cycle = {MAX_TRANSMIT_DUTY_CYCLE}

# Multiply all per-mode cooldown durations by this factor.
# 1.0 = nominal defaults  |  0.75 = more aggressive  |  1.5 = conservative
cooldown_scale_factor = {COOLDOWN_SCALE_FACTOR}

# Seconds to wait after asserting PD=HIGH before keying PTT, allowing the
# DRA818 module time to power up and stabilise.
radio_wake_delay_seconds = {RADIO_WAKE_DELAY_SECONDS}

# Seconds to wait after asserting PTT=LOW before audio playback begins.
ptt_key_delay_seconds = {PTT_KEY_DELAY_SECONDS}

# Seconds to wait after audio playback ends before releasing PTT.
post_playback_delay_seconds = {POST_PLAYBACK_DELAY_SECONDS}


# -----------------------------------------------------------------------------
# [capture]  Camera and image acquisition  (OV5647 / rpicam-still)
# -----------------------------------------------------------------------------
# For --explain reference: python3 pi_sstv.py --explain capture
[capture]

# JPEG compression quality (1 – 100).  93 is visually lossless.
quality = {RPICAM_QUALITY}

# AE metering mode.
#   matrix  - Multi-zone evaluative; recommended for HAB sky/ground contrast.
#   average - Whole-frame average; can overexpose when sky fills the frame.
#   spot    - Centre-spot only; not suitable for wide HAB scenes.
metering = {RPICAM_METERING}

# Exposure profile.
#   sport   - Favours shorter shutter times; reduces gondola-motion blur.
#   normal  - Balanced shutter/gain trade-off.
#   long    - Allows very long shutter times; not recommended at altitude.
exposure = {RPICAM_EXPOSURE}

# Auto white-balance mode.
#   auto    - Adapts per-frame across the full flight temperature range.
#   daylight, cloudy, indoor, fluorescent, incandescent, flash, horizon, greyworld
awb = {RPICAM_AWB}

# Seconds to wait for the captured file to appear on disk before falling back
# to the test image.
capture_file_timeout = {CAPTURE_FILE_TIMEOUT}


# -----------------------------------------------------------------------------
# [encode]  SlowFrame audio encoding
# -----------------------------------------------------------------------------
# For --explain reference: python3 pi_sstv.py --explain encode
[encode]

# Audio container format.
#   wav   - Required for direct aplay playback.  (recommended)
#   aiff  - Apple/offline use.
#   ogg   - Compressed; for archival only.
format = {SLOWFRAME_AUDIO_FORMAT}

# Audio sample rate in Hz.  22050 is the SSTV standard; do not reduce.
sample_rate = {SLOWFRAME_SAMPLE_RATE}

# Image aspect-ratio handling when the camera frame differs from the mode canvas.
#   center  - Centred with black borders; no distortion.  (recommended)
#   pad     - Letterbox / pillarbox padding.
#   stretch - Stretch to fill; introduces distortion.
aspect = {SLOWFRAME_ASPECT_MODE}

# Enable verbose SlowFrame output (logs full encoder diagnostics).
verbose = {'true' if SLOWFRAME_VERBOSE else 'false'}

# Seconds to wait after SlowFrame finishes before proceeding; allows the
# file system to flush the WAV before aplay opens it.
sstv_conversion_settle_seconds = {SSTV_CONVERSION_SETTLE_SECONDS}


# -----------------------------------------------------------------------------
# [overlay]  Timestamp and callsign text overlays
# -----------------------------------------------------------------------------
# Overlays are baked into the image by SlowFrame at encode time.
# For --explain reference: python3 pi_sstv.py --explain overlay
#
# Positions: top-left  top-right  bottom-left  bottom-right
#            top  bottom  left  right  center
# Colors:    named CSS color (white, black, yellow, red, …) or hex (#RRGGBB)
# Opacity:   integer 0 (transparent) – 100 (opaque)
[overlay]

# --- Timestamp overlay ---
# Prints the UTC capture timestamp on every transmitted image.
enable_timestamp = {'true' if SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY else 'false'}
timestamp_size = {SLOWFRAME_TIMESTAMP_OVERLAY_SIZE}
timestamp_position = {SLOWFRAME_TIMESTAMP_OVERLAY_POSITION}
timestamp_color = {SLOWFRAME_TIMESTAMP_OVERLAY_COLOR}
timestamp_background_color = {SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR}
timestamp_background_opacity = {SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY}

# --- Callsign overlay ---
# Prints the station callsign.  Automatically enabled when [mission] callsign
# is set; can also be enabled independently here.
enable_callsign = {'true' if SLOWFRAME_ENABLE_CALLSIGN_OVERLAY else 'false'}
callsign_size = {SLOWFRAME_CALLSIGN_OVERLAY_SIZE}
callsign_position = {SLOWFRAME_CALLSIGN_OVERLAY_POSITION}
callsign_color = {SLOWFRAME_CALLSIGN_OVERLAY_COLOR}
callsign_background_color = {SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR}


# -----------------------------------------------------------------------------
# [mmsstv]  MMSSTV encoder library
# -----------------------------------------------------------------------------
# For --explain reference: python3 pi_sstv.py --explain mmsstv
[mmsstv]

# Explicit path to the MMSSTV shared library.  Leave blank to let SlowFrame
# auto-detect via $MMSSTV_LIB_PATH, pkg-config, or standard library paths.
# Use the unversioned symlink, not the versioned filename:
#   Correct:   /opt/mmsstv/lib/libsstv_encoder.so
#   Incorrect: /opt/mmsstv/lib/libsstv_encoder.so.1.0.0
lib_path =

# Set to true to force native-only SlowFrame modes regardless of whether the
# library is installed.  Equivalent to: export SLOWFRAME_NO_MMSSTV=1
disable = false


# -----------------------------------------------------------------------------
# [logging]  Log verbosity and output destination
# -----------------------------------------------------------------------------
# For --explain reference: python3 pi_sstv.py --explain logging
[logging]

# Enable DEBUG-level output.  Prints full subprocess commands, GPIO state
# transitions, fallback chain steps, and duty-cycle arithmetic.
debug = false

# Write log output to this file in addition to stdout.
# Leave blank to log to stdout only.
log_file =

# Write log output only to this file; suppress stdout entirely.
# Cannot be set at the same time as log_file.
quiet_log_file =
"""
    with open(path, "w") as fh:
        fh.write(content)
    print(f"Default configuration written to: {path}")
    print("Edit the file, then load it with:  python3 pi_sstv.py --config " + path)


def load_config(path: str):
    """Read *path* and apply recognised settings to module-level globals.

    Settings in the config file are applied before CLI argument overrides, so
    any explicit CLI flag always wins.  Unknown section names and keys are
    silently ignored to allow forward-compatible config files.
    """
    global TRANSMIT_SCHEDULE_PROFILE, TRANSMIT_SCHEDULE
    global PIC_TOTAL, PIC_INTERVAL, STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global MIN_CAPTURES_BETWEEN_TRANSMISSIONS
    global ROLLING_DUTY_CYCLE_WINDOW_SECONDS, MAX_TRANSMIT_DUTY_CYCLE, COOLDOWN_SCALE_FACTOR
    global RADIO_WAKE_DELAY_SECONDS, PTT_KEY_DELAY_SECONDS, POST_PLAYBACK_DELAY_SECONDS
    global RPICAM_QUALITY, RPICAM_METERING, RPICAM_EXPOSURE, RPICAM_AWB, CAPTURE_FILE_TIMEOUT
    global SLOWFRAME_AUDIO_FORMAT, SLOWFRAME_SAMPLE_RATE, SLOWFRAME_ASPECT_MODE
    global SLOWFRAME_VERBOSE, SSTV_CONVERSION_SETTLE_SECONDS
    global SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY
    global SLOWFRAME_TIMESTAMP_OVERLAY_SIZE, SLOWFRAME_TIMESTAMP_OVERLAY_POSITION
    global SLOWFRAME_TIMESTAMP_OVERLAY_COLOR, SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR
    global SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY
    global SLOWFRAME_CALLSIGN_OVERLAY_SIZE, SLOWFRAME_CALLSIGN_OVERLAY_POSITION
    global SLOWFRAME_CALLSIGN_OVERLAY_COLOR, SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR
    global TIMESTAMPED_DIR, SLOWFRAME_BIN, TEST_IMAGE, DATA_CSV, SSTV_WAV

    if not os.path.isfile(path):
        print(f"Config file not found: {path}", file=sys.stderr)
        sys.exit(1)

    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(path)

    def _str(section, key, default):
        return cfg.get(section, key, fallback=default)

    def _int(section, key, default):
        try:
            return cfg.getint(section, key, fallback=default)
        except ValueError as exc:
            print(f"Config [{section}] {key}: invalid integer — {exc}", file=sys.stderr)
            sys.exit(1)

    def _float(section, key, default):
        try:
            return cfg.getfloat(section, key, fallback=default)
        except ValueError as exc:
            print(f"Config [{section}] {key}: invalid float — {exc}", file=sys.stderr)
            sys.exit(1)

    def _bool(section, key, default):
        try:
            return cfg.getboolean(section, key, fallback=default)
        except ValueError as exc:
            print(f"Config [{section}] {key}: invalid boolean (use true/false) — {exc}", file=sys.stderr)
            sys.exit(1)

    # [paths]
    TIMESTAMPED_DIR   = _str("paths", "output_dir",  TIMESTAMPED_DIR)
    SLOWFRAME_BIN     = _str("paths", "slowframe",    SLOWFRAME_BIN)
    TEST_IMAGE        = _str("paths", "test_image",   TEST_IMAGE)
    DATA_CSV          = _str("paths", "data_csv",     DATA_CSV)
    SSTV_WAV          = os.path.join(TIMESTAMPED_DIR, "HAB-SSTV.wav")

    # [mission]
    schedule = _str("mission", "schedule", TRANSMIT_SCHEDULE_PROFILE)
    if schedule not in TRANSMIT_SCHEDULE_PROFILES:
        print(f"Config [mission] schedule: unknown preset '{schedule}'. "
              f"Valid: {', '.join(TRANSMIT_SCHEDULE_PROFILES)}", file=sys.stderr)
        sys.exit(1)
    TRANSMIT_SCHEDULE_PROFILE = schedule
    TRANSMIT_SCHEDULE         = TRANSMIT_SCHEDULE_PROFILES[schedule]
    PIC_TOTAL                        = _int  ("mission", "total",                            PIC_TOTAL)
    PIC_INTERVAL                     = _float("mission", "interval",                         PIC_INTERVAL)
    callsign                         = _str  ("mission", "callsign",                         STATION_CALLSIGN).strip()
    MIN_CAPTURES_BETWEEN_TRANSMISSIONS = _int("mission", "min_captures_between_transmissions", MIN_CAPTURES_BETWEEN_TRANSMISSIONS)
    if callsign:
        STATION_CALLSIGN             = callsign
        SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = True

    # [radio]
    ROLLING_DUTY_CYCLE_WINDOW_SECONDS = _int  ("radio", "rolling_duty_cycle_window_seconds", ROLLING_DUTY_CYCLE_WINDOW_SECONDS)
    MAX_TRANSMIT_DUTY_CYCLE           = _float("radio", "max_transmit_duty_cycle",           MAX_TRANSMIT_DUTY_CYCLE)
    COOLDOWN_SCALE_FACTOR             = _float("radio", "cooldown_scale_factor",             COOLDOWN_SCALE_FACTOR)
    RADIO_WAKE_DELAY_SECONDS          = _float("radio", "radio_wake_delay_seconds",          RADIO_WAKE_DELAY_SECONDS)
    PTT_KEY_DELAY_SECONDS             = _float("radio", "ptt_key_delay_seconds",             PTT_KEY_DELAY_SECONDS)
    POST_PLAYBACK_DELAY_SECONDS       = _float("radio", "post_playback_delay_seconds",       POST_PLAYBACK_DELAY_SECONDS)

    # [capture]
    RPICAM_QUALITY         = _int  ("capture", "quality",               RPICAM_QUALITY)
    RPICAM_METERING        = _str  ("capture", "metering",              RPICAM_METERING)
    RPICAM_EXPOSURE        = _str  ("capture", "exposure",              RPICAM_EXPOSURE)
    RPICAM_AWB             = _str  ("capture", "awb",                   RPICAM_AWB)
    CAPTURE_FILE_TIMEOUT   = _int  ("capture", "capture_file_timeout",  CAPTURE_FILE_TIMEOUT)

    # [encode]
    fmt = _str("encode", "format", SLOWFRAME_AUDIO_FORMAT)
    if fmt not in ("wav", "aiff", "ogg"):
        print(f"Config [encode] format: invalid value '{fmt}'. Valid: wav, aiff, ogg", file=sys.stderr)
        sys.exit(1)
    SLOWFRAME_AUDIO_FORMAT          = fmt
    SLOWFRAME_SAMPLE_RATE           = _int  ("encode", "sample_rate",                   SLOWFRAME_SAMPLE_RATE)
    aspect = _str("encode", "aspect", SLOWFRAME_ASPECT_MODE)
    if aspect not in ("center", "pad", "stretch"):
        print(f"Config [encode] aspect: invalid value '{aspect}'. Valid: center, pad, stretch", file=sys.stderr)
        sys.exit(1)
    SLOWFRAME_ASPECT_MODE           = aspect
    SLOWFRAME_VERBOSE               = _bool ("encode", "verbose",                       SLOWFRAME_VERBOSE)
    SSTV_CONVERSION_SETTLE_SECONDS  = _float("encode", "sstv_conversion_settle_seconds", SSTV_CONVERSION_SETTLE_SECONDS)

    # [overlay]
    SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY          = _bool ("overlay", "enable_timestamp",           SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY)
    SLOWFRAME_TIMESTAMP_OVERLAY_SIZE            = _int  ("overlay", "timestamp_size",             SLOWFRAME_TIMESTAMP_OVERLAY_SIZE)
    SLOWFRAME_TIMESTAMP_OVERLAY_POSITION        = _str  ("overlay", "timestamp_position",         SLOWFRAME_TIMESTAMP_OVERLAY_POSITION)
    SLOWFRAME_TIMESTAMP_OVERLAY_COLOR           = _str  ("overlay", "timestamp_color",            SLOWFRAME_TIMESTAMP_OVERLAY_COLOR)
    SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR= _str  ("overlay", "timestamp_background_color", SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR)
    SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY = _int("overlay", "timestamp_background_opacity", SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY)
    SLOWFRAME_ENABLE_CALLSIGN_OVERLAY           = _bool ("overlay", "enable_callsign",            SLOWFRAME_ENABLE_CALLSIGN_OVERLAY)
    SLOWFRAME_CALLSIGN_OVERLAY_SIZE             = _int  ("overlay", "callsign_size",              SLOWFRAME_CALLSIGN_OVERLAY_SIZE)
    SLOWFRAME_CALLSIGN_OVERLAY_POSITION         = _str  ("overlay", "callsign_position",          SLOWFRAME_CALLSIGN_OVERLAY_POSITION)
    SLOWFRAME_CALLSIGN_OVERLAY_COLOR            = _str  ("overlay", "callsign_color",             SLOWFRAME_CALLSIGN_OVERLAY_COLOR)
    SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR = _str  ("overlay", "callsign_background_color",  SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR)

    # [mmsstv]
    lib_path = _str("mmsstv", "lib_path", "").strip()
    if lib_path:
        os.environ[MMSSTV_LIBRARY_ENV_VAR] = lib_path
    if _bool("mmsstv", "disable", False):
        os.environ[MMSSTV_DISABLE_ENV_VAR] = "1"

    # [logging] — debug/log_file/quiet_log_file are handled by CLI args only;
    # reading them here would require reconfiguring the logging system after
    # it has already been set up.  Document them in the file but skip loading.


def configure_logging(debug: bool = False, log_file: str = None, quiet_stdout: bool = False):
    level = logging.DEBUG if debug else logging.INFO
    fmt = "[%(asctime)s] %(message)s"
    datefmt = "%Y-%m-%d %H:%M:%S"
    handlers = []
    if not quiet_stdout:
        handlers.append(logging.StreamHandler(sys.stdout))
    if log_file:
        handlers.append(logging.FileHandler(log_file))
    if not handlers:
        handlers.append(logging.NullHandler())
    logging.basicConfig(level=level, format=fmt, datefmt=datefmt, handlers=handlers)


def log(message: str):
    logger.info(message)


def log_debug(message: str):
    logger.debug(message)


def log_section(title: str, width: int = 56):
    bar = "=" * width
    log(bar)
    log(f"  {title}")
    log(bar)


# ---------------------------------------------------------------------------
# Contextual help topics  (--explain TOPIC)
# ---------------------------------------------------------------------------

HELP_TOPICS = {
    "capture": """\
TOPIC: capture
==============
Controls how images are taken with rpicam-still and the OV5647 camera module.

The OV5647 is a fixed-focus sensor.  At altitude the subject distance is
effectively infinite, so the lens is locked at the infinity position and no
autofocus is attempted.  Exposure and white-balance settings are chosen to
handle the challenging lighting conditions of a HAB flight:

  --metering  (default: matrix)
        matrix  - Multi-zone evaluative metering.  Weights each zone of the
                  frame independently, handling the stark contrast between a
                  bright ground scene and a black sky at altitude.  Recommended
                  for HAB missions.
        average - Average of the whole frame.  The large dark sky area pulls
                  the exposure high and can overexpose the ground/cloud scene.
        spot    - Meters only the centre spot.  Not suitable for wide HAB shots.

  --exposure  (default: sport)
        sport   - Biases the AE algorithm toward shorter shutter times to
                  reduce motion blur from gondola rotation and pendulum swing,
                  which is most severe when crossing the jet stream.
        normal  - Balanced shutter/gain trade-off.  Can produce blur on an
                  active gondola.
        long    - Allows very long shutter times.  Not recommended at altitude.

  --awb  (default: auto)
        Adapts per-frame from the warm color temperature at launch to the
        cold blue-shifted daylight of the stratosphere.  Auto is the correct
        choice for a full-flight mission.

  lens-position  (fixed: 0)
        Always set to 0 (infinity focus).  The OV5647 has a fixed plastic lens
        and rpicam-still will not move it, but explicitly passing 0 prevents
        any platform default from interfering.

  --quality  (default: 93)
        JPEG compression quality (1-100).  93 gives visually lossless images
        while keeping file sizes manageable for SD write speed.

Current defaults are defined at the top of the script:
  RPICAM_METERING  = "{metering}"
  RPICAM_EXPOSURE  = "{exposure}"
  RPICAM_AWB       = "{awb}"
  RPICAM_QUALITY   = {quality}

EXAMPLES
  Override metering for a bench test using an existing image:
    python3 pi_sstv.py --test r36 --test-image photo.jpg --no-tx

  Run a full test capture with the camera:
    python3 pi_sstv.py --test r36 --no-tx

EXTERNAL COMMANDS
  rpicam-still
    The libcamera-based still image capture tool used on all modern
    Raspberry Pi OS releases.
      rpicam-still --help
      man rpicam-still
    Online reference:
      https://www.raspberrypi.com/documentation/computers/camera_software.html

SEE ALSO
  --explain encode   Image-to-audio SSTV encoding pipeline
  --explain overlay  Timestamp and callsign text overlay options
""",

    "encode": """\
TOPIC: encode
=============
Controls how a captured image is converted to SSTV audio by SlowFrame.

The pipeline is:  capture (.jpg)  ->  SlowFrame  ->  audio file  ->  aplay -> TX

  --format   (default: wav)
        Audio container format.  wav is the most compatible and is required
        by aplay for direct playback.  aiff and ogg are provided for
        offline use or archival.
        Choices: wav, aiff, ogg

  --sample-rate  (default: 22050)
        Audio sample rate in Hz passed to SlowFrame.  22050 Hz (half of CD
        rate) is the standard for SSTV audio and is accepted by all DRA818
        module audio inputs without resampling artifacts.
        Reducing this below 22050 is not recommended; the SSTV subcarrier
        frequencies will be reproduced inaccurately.

  --aspect  (default: center)
        How SlowFrame handles the camera's native 4:3 aspect ratio when
        the SSTV mode requires a different pixel geometry:
        center  - Places the image in the centre of the frame with black
                  borders.  No distortion.  Recommended.
        pad     - Adds letterbox/pillarbox padding to fill the canvas.
        stretch - Stretches the image to fill.  Introduces distortion.

SSTV mode selection
  The mode determines the transmitted image resolution, duration, and whether
  the MMSSTV encoder library is required.  See --explain modes for the full
  table.  Key parameters:
        image_width      - Pixel width SlowFrame will scale the input to.
        duration_seconds - Approximate over-the-air TX time.
        cooldown_seconds - Minimum wait enforced after this mode transmits.

EXAMPLES
  Encode a test image to r36 WAV without transmitting:
    python3 pi_sstv.py --test r36 --test-image /home/pi-user/photo.jpg --no-tx

  Encode with ogg output for archival (bench only):
    python3 pi_sstv.py --test m1 --format ogg --no-tx

  Check SSTV mode list (durations, cooldowns, MMSSTV requirements):
    python3 pi_sstv.py --list-modes

EXTERNAL COMMANDS
  slowframe
    The SSTV audio encoder used by this script.  Run these commands to
    explore its full capability independently of this script:
      slowframe --help              General usage and flag reference
      slowframe -L                  List all available SSTV modes
      slowframe -L -v               Verbose mode list with format details
      slowframe -M                  Check MMSSTV library detection status
    Direct encode example (bypassing this script):
      slowframe -i photo.jpg -o out.wav -p r36 -f wav -r 22050

  aplay
    ALSA command-line audio player used to play the encoded WAV to the DRA818.
      aplay --help
      man aplay
    List available playback hardware:
      aplay -l

SEE ALSO
  --explain capture  Camera and image acquisition settings
  --explain overlay  Text overlays baked into the image before encoding
  --explain mmsstv   MMSSTV library modes (pd50, pd90, pd120, pd160, pd180, fax480)
  --explain modes    Full SSTV mode reference table
""",

    "overlay": """\
TOPIC: overlay
==============
SlowFrame can bake text overlays directly into the transmitted image before
encoding.  The overlay is rendered at encode time — it appears in the received
image on-air and in the saved WAV artifact.

Timestamp overlay (SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY)
  Prints the UTC capture time in the top-left corner of every image.
  Enabled by default.  Format: YYYY.MM.DD - HH:MM:SS [TEST] in test mode.

  Configuration constants (edit at top of script):
    SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY        = True/False
    SLOWFRAME_TIMESTAMP_OVERLAY_SIZE          = font size (default: 11)
    SLOWFRAME_TIMESTAMP_OVERLAY_POSITION      = top-left (default)
    SLOWFRAME_TIMESTAMP_OVERLAY_COLOR         = white (default)
    SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR    = black (default)
    SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY  = 0-100 (default: 70)

  Valid positions:
    top-left, top-right, bottom-left, bottom-right, top, bottom, left, right, center

  Colors: named CSS colors (white, black, yellow, red, ...) or hex (#RRGGBB)

Callsign overlay (SLOWFRAME_ENABLE_CALLSIGN_OVERLAY)
  Prints the station callsign, typically in the top-right corner.
  Disabled by default; enabled automatically when --callsign is passed.

  Configuration constants:
    STATION_CALLSIGN                          = "" (set via --callsign)
    SLOWFRAME_ENABLE_CALLSIGN_OVERLAY         = False (auto-enabled with --callsign)
    SLOWFRAME_CALLSIGN_OVERLAY_SIZE           = font size (default: 13)
    SLOWFRAME_CALLSIGN_OVERLAY_POSITION       = top-right (default)
    SLOWFRAME_CALLSIGN_OVERLAY_COLOR          = white (default)
    SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR = black (default)

Merging behaviour
  SlowFrame places all -T overlays at the same rendered position regardless of
  separate pos= values.  When both timestamp and callsign overlays are enabled,
  the script merges them into a single overlay string:
    "W1AW-11  2026.04.12 - 00:25:05 UTC"
  This prevents the two strings from stacking on top of each other.

Font size scaling
  Font sizes are scaled proportionally to the mode's image_width, anchored at
  320 px.  A size=11 at 320 px becomes size=22 at 640 px, keeping the overlay
  legible at both low- and high-resolution modes.

EXAMPLES
  Test with callsign overlay:
    python3 pi_sstv.py --test r36 --callsign W1AW-11 --no-tx

  Test high-resolution mode with callsign (overlay scales up automatically):
    python3 pi_sstv.py --test pd120 --callsign W1AW-11 --no-tx

EXTERNAL COMMANDS
  slowframe  (overlay flags)
    The -T flag accepts a pipe-delimited overlay descriptor string.  You can
    experiment with overlay rendering directly:
      slowframe -i photo.jpg -o out.wav -p r36 \
        -T "W1AW-11  2026.04.12|size=11|pos=top-left|color=white|bg=black|opacity=70"
      slowframe --help              Full -T syntax reference
      slowframe -L -v               Verbose mode list (shows resolution hints)

SEE ALSO
  --explain encode   Audio encoding settings
  --explain capture  Camera and image acquisition settings
""",

    "mmsstv": """\
TOPIC: mmsstv
=============
Several SSTV modes require the MMSSTV encoder library (libsstv_encoder.so).
These modes are not built into SlowFrame and will only work when the shared
library is present and detectable.

Modes requiring MMSSTV:
  robot8bw   robot12bw   pd50   pd90   pd120   pd160   pd180   fax480

Detection
  At startup, the script runs 'slowframe -M' which probes for the library.
  It checks, in order:
    1. $MMSSTV_LIB_PATH environment variable (or --mmsstv-lib flag)
    2. pkg-config --variable=libdir mmsstv-portable
    3. Standard paths: /usr/local/lib, /usr/lib, /opt/mmsstv/lib

  If detection fails, all MMSSTV modes fall back to their native equivalents
  (see fallback table below) and a clear warning is printed.

Providing the library path
  Set the environment variable before running:
    export MMSSTV_LIB_PATH=/path/to/libsstv_encoder.so

  Or pass it directly on the command line:
    python3 pi_sstv.py --mmsstv-lib /path/to/libsstv_encoder.so

  Important: use the unversioned .so symlink, not the versioned filename.
    Correct:   libsstv_encoder.so
    Incorrect: libsstv_encoder.so.1.0.0

Disabling MMSSTV
  Force native-only operation regardless of whether the library is present:
    python3 pi_sstv.py --no-mmsstv
  or:
    export SLOWFRAME_NO_MMSSTV=1

Fallback chain
  When the MMSSTV library is unavailable, each MMSSTV mode falls back to:
    robot8bw  -> bw24     robot12bw -> bw24
    pd50      -> m2       pd90      -> r36
    pd120     -> m1       pd160     -> m1
    pd180     -> m1       fax480    -> m1

EXAMPLES
  Verify the library is detected and pd90 encodes correctly:
    MMSSTV_LIB_PATH=/opt/mmsstv/lib/libsstv_encoder.so \\
      python3 pi_sstv.py --test pd90 --no-tx

  Force native fallback for the same test:
    python3 pi_sstv.py --test pd90 --no-mmsstv --no-tx

  Check what modes are available on this installation:
    python3 pi_sstv.py --list-modes

EXTERNAL COMMANDS
  slowframe  (MMSSTV library probe)
    Run the library status check directly to see full SlowFrame diagnostic
    output independent of this script:
      slowframe -M
      MMSSTV_LIB_PATH=/path/to/libsstv_encoder.so slowframe -M
      slowframe --help

  pkg-config  (library path lookup)
    If the mmsstv-portable package was installed via a package manager,
    pkg-config can reveal where the library landed:
      pkg-config --variable=libdir mmsstv-portable
      pkg-config --modversion mmsstv-portable

  ldconfig  (shared library cache)
    Confirm the library is visible to the dynamic linker:
      ldconfig -p | grep libsstv

SEE ALSO
  --explain modes    Full mode reference table
  --explain encode   Encoding pipeline and format options
  --explain schedule Transmit schedule presets and duty-cycle behaviour
""",

    "modes": """\
TOPIC: modes
============
SSTV mode reference.  All durations are approximate over-the-air TX times.

Native modes (no MMSSTV library required):
  Name         TX (s)   Cooldown (s)   Width   Description
  bw24             24            120     320   Fast monochrome, low duty-cycle updates
  r36              36            150     320   Fast native color, regular updates
  m2               58            240     320   Balanced, strong compatibility
  s2               71            300     320   Scottie 2, good compatibility
  r72              72            300     320   Higher-quality Robot color
  s1              110            480     320   Scottie 1, best native quality
  m1              114            480     320   Martin 1, high-quality, less frequent

MMSSTV library modes (require libsstv_encoder.so):
  Name         TX (s)   Cooldown (s)   Width   Fallback   Description
  robot8bw          8             90     160   bw24       Ultra-fast monochrome status frame
  robot12bw        12             90     160   bw24       Very fast monochrome
  pd50             50            240     320   m2         Fast PD color
  pd90             90            360     320   r36        Popular fast color
  pd120           120            540     640   m1         Higher-quality, larger image
  pd160           160            660     512   m1         Slower quality mode
  pd180           180            720     640   m1         High-detail mission snapshot
  fax480          180            720     512   m1         High-detail, test windows

Mode image_width
  Each mode encodes at a specific pixel width.  SlowFrame scales the captured
  image to this width before encoding.  The aspect handling (--aspect) controls
  how the camera's 4:3 frame is fitted to the mode canvas.

Cooldown scaling
  All cooldown values are multiplied by --cooldown-scale at runtime.
  Default scale is 1.0.  Use 1.5 for conservative thermal budgets or 0.75
  for more aggressive scheduling.

EXAMPLES
  List modes with live fallback resolution for the current environment:
    python3 pi_sstv.py --list-modes

  Test a specific mode end-to-end:
    python3 pi_sstv.py --test pd90 --no-tx

EXTERNAL COMMANDS
  slowframe  (mode listing)
    List all modes available on the current installation, including MMSSTV
    modes if the library is loaded:
      slowframe -L
      slowframe -L -v               Verbose output with resolution and format detail
      slowframe -M                  MMSSTV library detection report
      slowframe --help

SEE ALSO
  --explain schedule  How modes are sequenced during a mission
  --explain mmsstv    Library setup for MMSSTV modes
""",

    "schedule": """\
TOPIC: schedule
===============
During a HAB mission the script transmits on a rotating schedule of SSTV modes
rather than always using the same mode.  This varies image quality, duty cycle,
and receiver-side interest across the flight.

Presets
  hab_climb    robot8bw -> robot12bw -> bw24 -> m4 -> robot12bw -> r36
               Maximum update rate.  Monochrome-heavy to keep cooldowns short
               during the steepest part of the climb.

  hab_rapid    robot12bw -> m4 -> r36 -> robot12bw -> m4 -> pd50
               Fast color bursts.  Best for upper ascent and the release phase
               where update rate still matters but color shots are welcome.

  hab_cruise   robot12bw -> r36 -> m2 -> pd90 -> s2 -> r72 -> m1  (default)
               Mixes fast status frames with progressively higher-quality
               images.  Good all-round choice for the full flight envelope.

  hab_float    r36 -> pd90 -> robot12bw -> pd120 -> robot12bw -> pd180 -> r36 -> pd240
               Quality-first.  Anchored by PD modes for float altitude or
               slow-drift science windows.  Requires MMSSTV library; falls
               back gracefully if unavailable.

Selecting a preset
  python3 pi_sstv.py --schedule hab_rapid
  python3 pi_sstv.py --schedule hab_float --mmsstv-lib /path/to/libsstv_encoder.so

Transmission gating
  A transmission only proceeds when ALL of the following are satisfied:
    1. At least --min-captures capture cycles have elapsed since the last TX
       (default: {min_captures}).
    2. The per-mode cooldown for the resolved mode has expired, multiplied by
       --cooldown-scale (default: {cooldown_scale}x).
    3. The rolling duty-cycle budget has not been exceeded.
       Budget = --duty-cycle * {window}s window (default: {duty_pct}% = {budget}s per hour).

Duty-cycle protection
  --duty-cycle FRACTION   Maximum TX fraction of the rolling window (0.0-1.0).
                          Default: {duty_pct_raw} ({duty_pct}%).
  --cooldown-scale FACTOR Multiply every mode's cooldown by this factor.
                          1.0 = nominal, 0.75 = aggressive, 1.5 = conservative.
  --min-captures N        Hard minimum captures between any two transmissions.
                          Default: {min_captures}.

EXAMPLES
  Fast-color schedule, 200 captures, 8-second intervals:
    python3 pi_sstv.py --schedule hab_rapid --total 200 --interval 8

  Float schedule with conservative thermal protection:
    python3 pi_sstv.py --schedule hab_float --cooldown-scale 1.5

  Inspect all presets and their mode sequences:
    python3 pi_sstv.py --list-schedules

  Inspect fallback resolution for selected schedule:
    python3 pi_sstv.py --list-modes

EXTERNAL COMMANDS
  slowframe  (mode and library status)
    Verify which modes will be available for the active schedule before
    committing to a flight:
      slowframe -L                  List all available modes
      slowframe -M                  MMSSTV library detection status
      slowframe --help

SEE ALSO
  --explain modes    Mode durations, cooldowns, and MMSSTV requirements
  --explain mmsstv   Library setup for modes that need it
  --explain tx       Radio transmission timing and GPIO control
""",

    "tx": """\
TOPIC: tx
=========
Controls the radio transmission step: GPIO sequencing, audio playback,
and timing constants for the DRA818 module on the HamWing carrier board.

GPIO pin assignments (BCM numbering):
  DRA818_PTT_PIN         = {ptt}   (physical pin 13)
  DRA818_POWER_DOWN_PIN  = {pd}    (physical pin 7)
  DRA818_POWER_LEVEL_PIN = {hl}   (physical pin 15)
  Audio PWM output pins  = GPIO{al} (left) / GPIO{ar} (right)

Transmission sequence
  1. PD -> HIGH    Bring the DRA818 out of power-down (radio wake)
     wait  RADIO_WAKE_DELAY_SECONDS ({wake}s) for the module to stabilise
  2. PTT -> LOW    Key the transmitter
     wait  PTT_KEY_DELAY_SECONDS ({ptt_delay}s) before audio starts
  3. aplay         Stream the WAV file to the audio device
  4. wait          POST_PLAYBACK_DELAY_SECONDS ({post}s) after playback ends
  5. PTT -> HIGH   Unkey the transmitter
  6. PD -> LOW     Return the DRA818 to power-down (radio off)

PTT polarity
  The DRA818 PTT input is active-LOW.  Idle state is GPIO HIGH (line unkeyed).
  Pulling GPIO LOW keys the transmitter.  Never leave PTT LOW unattended.

Audio routing
  Audio is output via the Raspberry Pi PWM pins using the audremap overlay.
  Both pins feed a low-pass filter (LPF) before the DRA818 microphone input.
  Required config.txt entry:
    dtoverlay=audremap,enable_jack=on
  Alternate pin pair (pins 18/19):
    dtoverlay=audremap,pins_18_19,enable_jack=on

Flags
  --no-tx        Skip the entire TX stage.  Images are captured and encoded
                 but aplay is never called and PTT is never keyed.  Safe for
                 bench testing without a radio connected.
  --ptt-test     Key PTT for a short duration (default 1.0s) without audio,
                 to verify the GPIO control path end-to-end.
  --ptt-test N   Key PTT for N seconds.

EXAMPLES
  Safe encode test on a bench (no radio needed):
    python3 pi_sstv.py --test r36 --no-tx

  Verify PTT GPIO keying for 0.5 seconds:
    python3 pi_sstv.py --ptt-test 0.5

  Full pipeline test including radio TX:
    python3 pi_sstv.py --test r36

EXTERNAL COMMANDS
  aplay  (ALSA audio playback)
    The script calls aplay to stream the encoded WAV to the audio device.
    Useful commands for verifying the audio path before a flight:
      aplay -l                      List all playback hardware devices
      aplay -L                      List PCM device names (look for audremap)
      aplay HAB-SSTV.wav            Manual playback test
      aplay --help
      man aplay

  raspi-gpio  (GPIO inspection)
    Inspect or drive GPIO lines directly without running the full script:
      raspi-gpio get {ptt}           Read PTT pin state
      raspi-gpio get {pd}            Read power-down pin state
      raspi-gpio set {ptt} op dh     Drive PTT HIGH (idle/safe)
      raspi-gpio help

  gpio  (RPi.GPIO / WiringPi fallback)
    Alternative GPIO inspection tool if raspi-gpio is unavailable:
      gpio readall                   Print full pin state table

SEE ALSO
  --explain capture   Camera image acquisition
  --explain encode    SSTV audio encoding pipeline
  --explain schedule  Duty-cycle and cooldown protection
""",

    "gpio": """\
TOPIC: gpio
===========
Pin assignments and wiring reference for the HamWing carrier board.
All pin numbers use BCM (Broadcom) numbering unless noted.

DRA818 control lines:
  Signal   BCM GPIO   Physical Pin   Idle State   Active State
  PD       GPIO 4     Pin 7          LOW (off)    HIGH (on)
  PTT      GPIO 27    Pin 13         HIGH (idle)  LOW (keyed)
  HL       GPIO 22    Pin 15         LOW (low-pwr) HIGH (high-pwr)

Audio output (PWM via audremap overlay):
  Channel  BCM GPIO   Physical Pin
  Left     GPIO 12    Pin 32
  Right    GPIO 13    Pin 33
  (Alternate: GPIO 18 pin 12 / GPIO 19 pin 35 with pins_18_19 overlay)

Both PWM audio pins should be routed through a low-pass filter (LPF) before
reaching the DRA818 microphone input to remove PWM switching noise above the
SSTV audio band (~300 Hz - 3 kHz).

Optional peripherals (not used by this script, documented for reference):
  GPS UART:    TX=GPIO14 (Pin 8)   RX=GPIO15 (Pin 10)
  I2C sensor:  SDA=GPIO2 (Pin 3)   SCL=GPIO3 (Pin 5)

config.txt audio overlay
  Standard pair (GPIO 12/13):
    dtoverlay=audremap,enable_jack=on
  Alternate pair (GPIO 18/19):
    dtoverlay=audremap,pins_18_19,enable_jack=on

EXAMPLES
  Verify PTT and PD GPIO control without audio:
    python3 pi_sstv.py --ptt-test

EXTERNAL COMMANDS
  raspi-gpio  (read and set GPIO lines from the command line)
    Inspect or manually drive the DRA818 control pins to verify wiring
    without running the full script:
      raspi-gpio get                 Dump state of all GPIO pins
      raspi-gpio get {ptt}           Read PTT pin (should be 1 = HIGH when idle)
      raspi-gpio get {pd}            Read power-down pin (should be 0 = LOW at rest)
      raspi-gpio set {ptt} op dh     Drive PTT HIGH safely
      raspi-gpio set {pd} op dl      Drive PD LOW (radio off)
      raspi-gpio help

  gpio  (WiringPi fallback)
    Print a full BCM/physical/WiringPi pin state table:
      gpio readall

  pinout  (Raspberry Pi physical pin diagram)
    Print an ASCII pinout diagram of the board in the terminal:
      pinout

SEE ALSO
  --explain tx   Transmission timing and GPIO sequence
""",

    "logging": """\
TOPIC: logging
==============
Controls log verbosity and output destination.

Log levels
  INFO  (default)  Normal operational messages: mode selection, capture results,
                   encode configuration, GPIO transitions, TX timing, warnings.
  DEBUG            Everything in INFO plus full subprocess command lines,
                   SlowFrame raw output, fallback chain detail, duty-cycle
                   arithmetic, and file-wait polling.

Flags
  --debug                    Enable DEBUG-level output to stdout (and log file
                             if --log-file is also set).
  --log-file PATH            Write log output to PATH in addition to stdout.
  --quiet-log-file PATH      Write log output only to PATH; suppress stdout.
                             Cannot be combined with --log-file.

Log format
  [YYYY-MM-DD HH:MM:SS] message

Section headings
  Major pipeline phases are separated by titled section headers:
    ========================================================
      SlowFrame Capability Discovery
    ========================================================
  Individual capture cycles are headed:
    --- Capture #N  YYYY-MM-DD HH:MM:SS ---

EXAMPLES
  Run a mission with full debug logging to file and stdout:
    python3 pi_sstv.py --debug --log-file /home/pi-user/mission.log

  Silent mission — logs to file only:
    python3 pi_sstv.py --quiet-log-file /home/pi-user/mission.log

  Debug a single encode test to stdout only:
    python3 pi_sstv.py --test pd90 --no-tx --debug

EXTERNAL COMMANDS
  journalctl  (systemd log access)
    If pi_sstv.py is run as a systemd service, logs can be queried with:
      journalctl -u pi-sstv.service -f          Follow live output
      journalctl -u pi-sstv.service --since today
      journalctl --help

  grep  (filter log files)
    Filter saved log files for specific pipeline stages or failures:
      grep 'FAIL\\|ERROR\\|WARNING' /home/pi-user/mission.log
      grep 'Encode:' /home/pi-user/mission.log
      grep 'TX:' /home/pi-user/mission.log
      grep 'MMSSTV' /home/pi-user/mission.log

  tail / less
    Monitor or page through a mission log:
      tail -f /home/pi-user/mission.log
      less +F /home/pi-user/mission.log

SEE ALSO
  --explain capture   Capture stage status messages
  --explain encode    Encode stage status messages
  --explain tx        TX stage status messages
""",
}

HELP_TOPIC_ALIASES = {
    "cam":      "capture",
    "camera":   "capture",
    "image":    "capture",
    "audio":    "encode",
    "encoding": "encode",
    "wav":      "encode",
    "text":     "overlay",
    "callsign": "overlay",
    "timestamp":"overlay",
    "lib":      "mmsstv",
    "library":  "mmsstv",
    "mode":     "modes",
    "radio":    "tx",
    "transmit": "tx",
    "ptt":      "tx",
    "pins":     "gpio",
    "wiring":   "gpio",
    "log":      "logging",
    "debug":    "logging",
    "duty":     "schedule",
    "cooldown": "schedule",
}


def print_explain(topic: str):
    canonical = HELP_TOPIC_ALIASES.get(topic.lower(), topic.lower())
    text = HELP_TOPICS.get(canonical)
    if text is None:
        valid = sorted(set(HELP_TOPICS.keys()) | set(HELP_TOPIC_ALIASES.keys()))
        print(f"Unknown topic: '{topic}'")
        print(f"Available topics: {', '.join(valid)}")
        sys.exit(1)
    # Substitute live script constants into topics that reference them.
    text = text.format(
        metering=RPICAM_METERING,
        exposure=RPICAM_EXPOSURE,
        awb=RPICAM_AWB,
        quality=RPICAM_QUALITY,
        min_captures=MIN_CAPTURES_BETWEEN_TRANSMISSIONS,
        cooldown_scale=COOLDOWN_SCALE_FACTOR,
        window=ROLLING_DUTY_CYCLE_WINDOW_SECONDS,
        duty_pct=int(MAX_TRANSMIT_DUTY_CYCLE * 100),
        duty_pct_raw=MAX_TRANSMIT_DUTY_CYCLE,
        budget=int(ROLLING_DUTY_CYCLE_WINDOW_SECONDS * MAX_TRANSMIT_DUTY_CYCLE),
        ptt=DRA818_PTT_PIN,
        pd=DRA818_POWER_DOWN_PIN,
        hl=DRA818_POWER_LEVEL_PIN,
        al=AUDIO_LEFT_PWM_PIN,
        ar=AUDIO_RIGHT_PWM_PIN,
        wake=RADIO_WAKE_DELAY_SECONDS,
        ptt_delay=PTT_KEY_DELAY_SECONDS,
        post=POST_PLAYBACK_DELAY_SECONDS,
    )
    print(text)


def parse_args():
    description = (
        "pi_sstv.py  —  HamWing SSTV HAB payload controller\n"
        + "=" * 56 + "\n\n"
        "Captures images with rpicam-still (OV5647), encodes them to SSTV\n"
        "audio using SlowFrame, and transmits over a DRA818 VHF module on\n"
        "the HamWing carrier board.\n\n"
        "Pipeline stages:\n"
        "  1. capture   rpicam-still  ->  JPEG image\n"
        "  2. encode    SlowFrame     ->  WAV audio (SSTV format)\n"
        "  3. transmit  aplay + GPIO  ->  DRA818 over-the-air TX\n\n"
        "Operating modes:\n"
        "  mission      Continuous capture-and-transmit loop (default when\n"
        "               --config or mission flags are supplied).\n"
        "  --test MODE  Single-shot pipeline validation for one SSTV mode.\n"
        "  --ptt-test   GPIO PTT/PD keying verification without audio.\n\n"
        "Configuration:\n"
        "  Settings can be provided via a .cfg file (--config), CLI flags,\n"
        "  or a mix of both.  CLI flags always override the config file.\n"
        "  Generate a documented template:  python3 pi_sstv.py --generate-config\n\n"
        "Reference documentation:\n"
        "  python3 pi_sstv.py --explain <topic>\n"
        "  Topics: capture  encode  overlay  mmsstv  modes  schedule  tx  gpio  logging"
    )
    epilog = (
        "QUICK START\n"
        "  Generate a config file and edit it for your callsign and schedule:\n"
        "    python3 pi_sstv.py --generate-config\n"
        "    nano /home/pi-user/pi_sstv.cfg\n"
        "    python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n\n"
        "MISSION EXAMPLES\n"
        "  Normal HAB mission — balanced schedule, default settings:\n"
        "    python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n\n"
        "  Mission with callsign overlay, rapid schedule, 200 captures at 8s intervals:\n"
        "    python3 pi_sstv.py --schedule hab_rapid --callsign W1AW-11 --total 200 --interval 8\n\n"
        "  Float schedule with MMSSTV library enabled:\n"
        "    python3 pi_sstv.py --schedule hab_float --mmsstv-lib /opt/mmsstv/lib/libsstv_encoder.so\n\n"
        "  Conservative thermal profile (1.5x cooldowns, 100 captures):\n"
        "    python3 pi_sstv.py --total 100 --cooldown-scale 1.5\n\n"
        "  Mission logging to file only (suppress stdout):\n"
        "    python3 pi_sstv.py --quiet-log-file /home/pi-user/mission.log\n\n"
        "PIPELINE TEST EXAMPLES\n"
        "  Test r36 encode + TX (full pipeline, radio connected):\n"
        "    python3 pi_sstv.py --test r36\n\n"
        "  Test pd90 encode only — no TX, safe bench test:\n"
        "    python3 pi_sstv.py --test pd90 --no-tx\n\n"
        "  Test with an existing image instead of the camera:\n"
        "    python3 pi_sstv.py --test m1 --test-image /home/pi-user/photo.jpg --no-tx\n\n"
        "  Test with callsign overlay and debug logging:\n"
        "    python3 pi_sstv.py --test r36 --callsign W1AW-11 --no-tx --debug\n\n"
        "GPIO / RADIO TESTS\n"
        "  Key PTT for 1 second to verify DRA818 GPIO wiring:\n"
        "    python3 pi_sstv.py --ptt-test\n\n"
        "  Key PTT for a custom duration:\n"
        "    python3 pi_sstv.py --ptt-test 0.5\n\n"
        "REFERENCE / INSPECTION\n"
        "  List all SSTV mode profiles (duration, cooldown, MMSSTV requirements):\n"
        "    python3 pi_sstv.py --list-modes\n\n"
        "  List all transmit schedule presets and mode sequences:\n"
        "    python3 pi_sstv.py --list-schedules\n\n"
        "  Show detailed documentation for a pipeline topic:\n"
        "    python3 pi_sstv.py --explain capture\n"
        "    python3 pi_sstv.py --explain mmsstv\n"
        "    python3 pi_sstv.py --explain schedule\n"
        "    python3 pi_sstv.py --explain tx\n\n"
        "MMSSTV LIBRARY\n"
        "  Enable MMSSTV modes (pd50, pd90, pd120, pd160, pd180, fax480, robot8bw, robot12bw):\n"
        "    export MMSSTV_LIB_PATH=/path/to/libsstv_encoder.so\n"
        "    python3 pi_sstv.py --test pd90 --no-tx\n\n"
        "  Or pass it directly:\n"
        "    python3 pi_sstv.py --mmsstv-lib /path/to/libsstv_encoder.so --test pd90 --no-tx\n\n"
        "  Disable MMSSTV (native SlowFrame modes only):\n"
        "    python3 pi_sstv.py --no-mmsstv\n\n"
        "CONFIGURATION FILE\n"
        "  Generate a fully-documented default config template:\n"
        "    python3 pi_sstv.py --generate-config\n"
        "    python3 pi_sstv.py --generate-config /path/to/custom.cfg\n\n"
        "  Load the config file at runtime:\n"
        "    python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n\n"
        "  CLI flags always override config file values:\n"
        "    python3 pi_sstv.py --config pi_sstv.cfg --schedule hab_rapid --callsign W1AW-11\n"
    )

    parser = argparse.ArgumentParser(
        prog="pi_sstv.py",
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog,
    )

    # --- Pipeline mode flags ---
    mode_group = parser.add_argument_group(
        "pipeline mode",
        "Select an alternative operating mode. Omit all four for a normal mission.",
    )
    mode_group.add_argument(
        "--explain",
        metavar="TOPIC",
        default=None,
        help=(
            "Print detailed reference documentation for a pipeline topic and exit. "
            "Topics: capture, encode, overlay, mmsstv, modes, schedule, tx, gpio, logging. "
            "Aliases are accepted (e.g. camera, radio, lib, wiring). "
            "Example: python3 pi_sstv.py --explain mmsstv"
        ),
    )
    mode_group.add_argument(
        "--test",
        metavar="MODE",
        default=None,
        help=(
            "Run a single-shot pipeline test for MODE (e.g. r36, pd90, m1). "
            "Captures one image, encodes it to SSTV audio, and optionally transmits. "
            "Output files use a consistent TEST-YYYYMMDD-HHMMSS prefix so every "
            "stage of the pipeline (capture, WAV) can be individually validated. "
            "Combine with --no-tx to skip radio transmit. "
            "Use --test-image to supply an existing image instead of calling the camera."
        ),
    )
    mode_group.add_argument(
        "--list-modes",
        action="store_true",
        help="Print all known SSTV mode profiles with duration, cooldown, and MMSSTV requirements, then exit.",
    )
    mode_group.add_argument(
        "--list-schedules",
        action="store_true",
        help="Print all schedule presets and their mode sequences, then exit.",
    )
    mode_group.add_argument(
        "--ptt-test",
        nargs="?",
        const=1.0,
        type=float,
        default=None,
        metavar="SECONDS",
        help=(
            "Run a brief GPIO-only PTT keying test and exit. "
            "If SECONDS is omitted, keys PTT for 1.0s. "
            "This verifies PD/PTT control transitions without requiring SlowFrame encoding."
        ),
    )

    # --- Mission settings ---
    mission = parser.add_argument_group(
        "mission",
        "Control the main capture-and-transmit mission loop.",
    )
    mission.add_argument(
        "--schedule",
        metavar="PRESET",
        choices=list(TRANSMIT_SCHEDULE_PROFILES.keys()),
        default=TRANSMIT_SCHEDULE_PROFILE,
        help=(
            f"Transmit schedule preset. Choices: {', '.join(TRANSMIT_SCHEDULE_PROFILES)}. "
            f"Default: {TRANSMIT_SCHEDULE_PROFILE}."
        ),
    )
    mission.add_argument(
        "--total",
        metavar="N",
        type=int,
        default=PIC_TOTAL,
        help=f"Total number of captures to take before the mission ends. Default: {PIC_TOTAL}.",
    )
    mission.add_argument(
        "--interval",
        metavar="SECS",
        type=float,
        default=PIC_INTERVAL,
        help=f"Seconds to wait between captures. Default: {PIC_INTERVAL}.",
    )
    mission.add_argument(
        "--callsign",
        metavar="CALL",
        default=STATION_CALLSIGN or None,
        help=(
            "Station callsign to overlay on transmitted images (e.g. W1AW-11). "
            "Enables the callsign overlay when provided. Default: disabled."
        ),
    )
    mission.add_argument(
        "--no-tx",
        action="store_true",
        help=(
            "Skip all radio transmission. Images are captured and SSTV audio is encoded "
            "but never played back or keyed over PTT. Useful for bench testing the "
            "capture and encode stages without a radio connected."
        ),
    )

    # --- Radio protection ---
    radio = parser.add_argument_group(
        "radio protection",
        "Tune duty-cycle and thermal protection parameters.",
    )
    radio.add_argument(
        "--cooldown-scale",
        metavar="FACTOR",
        type=float,
        default=COOLDOWN_SCALE_FACTOR,
        help=(
            "Multiply all per-mode cooldown durations by this factor. "
            "1.0 = tuned defaults, 0.75 = more aggressive, 1.5 = more conservative. "
            f"Default: {COOLDOWN_SCALE_FACTOR}."
        ),
    )
    radio.add_argument(
        "--duty-cycle",
        metavar="FRACTION",
        type=float,
        default=MAX_TRANSMIT_DUTY_CYCLE,
        help=(
            "Maximum rolling transmit duty-cycle as a fraction (0.0–1.0) over a "
            f"{ROLLING_DUTY_CYCLE_WINDOW_SECONDS}s window. "
            f"Default: {MAX_TRANSMIT_DUTY_CYCLE} ({int(MAX_TRANSMIT_DUTY_CYCLE * 100)}%%)."
        ),
    )
    radio.add_argument(
        "--min-captures",
        metavar="N",
        type=int,
        default=MIN_CAPTURES_BETWEEN_TRANSMISSIONS,
        help=(
            "Minimum number of capture cycles that must elapse between transmissions. "
            f"Default: {MIN_CAPTURES_BETWEEN_TRANSMISSIONS}."
        ),
    )

    # --- Encoding ---
    encode = parser.add_argument_group(
        "encoding",
        "SlowFrame audio encoding parameters.",
    )
    encode.add_argument(
        "--format",
        metavar="FMT",
        choices=["wav", "aiff", "ogg"],
        default=SLOWFRAME_AUDIO_FORMAT,
        help=f"Audio container format passed to SlowFrame. Default: {SLOWFRAME_AUDIO_FORMAT}.",
    )
    encode.add_argument(
        "--sample-rate",
        metavar="HZ",
        type=int,
        default=SLOWFRAME_SAMPLE_RATE,
        help=f"Audio sample rate in Hz. Default: {SLOWFRAME_SAMPLE_RATE}.",
    )
    encode.add_argument(
        "--aspect",
        metavar="MODE",
        choices=["center", "pad", "stretch"],
        default=SLOWFRAME_ASPECT_MODE,
        help=f"Image aspect-ratio handling. Default: {SLOWFRAME_ASPECT_MODE}.",
    )

    # --- Paths ---
    paths = parser.add_argument_group(
        "paths",
        "Override default file and binary paths.",
    )
    paths.add_argument(
        "--output-dir",
        metavar="PATH",
        default=TIMESTAMPED_DIR,
        help=f"Directory where captured images, WAV files, and CSV logs are written. Default: {TIMESTAMPED_DIR}.",
    )
    paths.add_argument(
        "--slowframe",
        metavar="PATH",
        default=SLOWFRAME_BIN,
        help=f"Path to the slowframe binary. Default: {SLOWFRAME_BIN}.",
    )
    paths.add_argument(
        "--test-image",
        metavar="PATH",
        default=None,
        help=(
            "Image path for use with --test. When provided, the camera is skipped "
            "and this image is encoded directly. When omitted, the camera is attempted "
            f"first and falls back to the default test image ({TEST_IMAGE}) on failure."
        ),
    )

    # --- MMSSTV ---
    mmsstv = parser.add_argument_group(
        "MMSSTV",
        "Control MMSSTV encoder library detection and usage.",
    )
    mmsstv.add_argument(
        "--no-mmsstv",
        action="store_true",
        help=(
            "Disable MMSSTV library support. Only native SlowFrame modes are used "
            "(bw24, r36, r72, m1, m2, s1, s2). Equivalent to setting "
            f"SLOWFRAME_NO_MMSSTV=1 in the environment."
        ),
    )
    mmsstv.add_argument(
        "--mmsstv-lib",
        metavar="PATH",
        default=None,
        help=(
            "Explicit path to the MMSSTV shared library (libsstv_encoder.so.1.0.0). "
            f"Overrides the {MMSSTV_LIBRARY_ENV_VAR} environment variable."
        ),
    )

    # --- Logging ---
    log_group = parser.add_argument_group(
        "logging",
        "Control log verbosity and output destination.",
    )
    log_group.add_argument(
        "--debug",
        action="store_true",
        help=(
            "Enable DEBUG-level logging. Prints full subprocess commands, "
            "GPIO state transitions, fallback chain steps, and timing details."
        ),
    )
    log_group.add_argument(
        "--log-file",
        metavar="PATH",
        default=None,
        help="Write log output to this file in addition to stdout.",
    )
    log_group.add_argument(
        "--quiet-log-file",
        metavar="PATH",
        default=None,
        help="Write log output only to this file and suppress stdout log output.",
    )

    # --- Configuration file ---
    cfg_group = parser.add_argument_group(
        "configuration file",
        "Load pipeline settings from a .cfg file.  CLI flags always override config file values.",
    )
    cfg_group.add_argument(
        "--config",
        metavar="PATH",
        default=None,
        help=(
            "Load pipeline settings from PATH before applying any CLI overrides. "
            "Use --generate-config to create a fully-documented template. "
            f"Default search path (if not specified): {DEFAULT_CONFIG_PATH}"
        ),
    )
    cfg_group.add_argument(
        "--generate-config",
        metavar="PATH",
        nargs="?",
        const=DEFAULT_CONFIG_PATH,
        default=None,
        help=(
            "Write a fully-documented default configuration file to PATH and exit. "
            f"If PATH is omitted, writes to {DEFAULT_CONFIG_PATH}. "
            "Edit the file then run:  python3 pi_sstv.py --config PATH"
        ),
    )

    args = parser.parse_args()

    if args.test and args.test not in MODE_PROFILES:
        parser.error(
            f"--test: unknown mode '{args.test}'. "
            f"Valid modes: {', '.join(sorted(MODE_PROFILES))}. "
            "Run --list-modes for details."
        )

    if args.test and args.ptt_test is not None:
        parser.error("--test and --ptt-test cannot be used together")

    if args.ptt_test is not None and args.ptt_test <= 0:
        parser.error("--ptt-test duration must be > 0 seconds")

    if args.log_file and args.quiet_log_file:
        parser.error("--log-file and --quiet-log-file cannot be used together")

    return args


def ensure_runtime_paths():
    os.makedirs(TIMESTAMPED_DIR, exist_ok=True)

    if not os.path.exists(DATA_CSV):
        with open(DATA_CSV, 'a', newline='') as file_handle:
            writer = csv.writer(file_handle)
            writer.writerow(CSV_HEADERS)


def write_csv(index, string_time):
    row = [index, string_time]
    with open(DATA_CSV, 'a', newline='') as file_handle:
        writer = csv.writer(file_handle)
        writer.writerow(row)


def list_modes():
    native = get_native_modes()
    header = f"{'Name':<12}  {'TX (s)':>6}  {'Cooldown (s)':>12}  {'MMSSTV':>6}  {'Fallback':<10}  Description"
    print(header)
    print("-" * len(header))
    for name, profile in sorted(MODE_PROFILES.items(), key=lambda x: x[1].duration_seconds):
        mmsstv_flag = "yes" if profile.requires_mmsstv else "no"
        fallback = profile.fallback_mode or "-"
        print(
            f"{profile.name:<12}  {profile.duration_seconds:>6}  {profile.cooldown_seconds:>12}"
            f"  {mmsstv_flag:>6}  {fallback:<10}  {profile.description}"
        )
    native_names = ", ".join(sorted(native))
    print(f"\nNative modes (no MMSSTV library required): {native_names}")


def list_schedules():
    WIDE  = "═" * 66
    THIN  = "─" * 66
    print("Schedule presets\n")
    for preset_name, modes in TRANSMIT_SCHEDULE_PROFILES.items():
        is_active    = preset_name == TRANSMIT_SCHEDULE_PROFILE
        active_tag   = "  ◀ active" if is_active else ""
        description  = TRANSMIT_SCHEDULE_DESCRIPTIONS.get(preset_name, "")
        profiles     = [MODE_PROFILES[m] for m in modes if m in MODE_PROFILES]

        total_tx    = sum(p.duration_seconds for p in profiles)
        total_cool  = sum(p.cooldown_seconds  for p in profiles)
        total_cycle = total_tx + total_cool
        duty        = 100.0 * total_tx / total_cycle if total_cycle else 0.0
        cycle_min   = total_cycle / 60.0
        mmsstv_steps = [p for p in profiles if p.requires_mmsstv]

        # Deduplicate fallback pairs for the summary (preserve insertion order)
        seen_fb: dict = {}
        for p in mmsstv_steps:
            key = f"{p.name} -> {p.fallback_mode}"
            seen_fb[key] = True
        unique_fb = list(seen_fb.keys())

        # ── header ────────────────────────────────────────────────────────────
        print(f"  {WIDE}")
        print(f"  {preset_name.upper()}{active_tag}")
        if description:
            # wrap description to ~62 chars
            words, line = description.split(), ""
            for w in words:
                if len(line) + len(w) + 1 > 62:
                    print(f"  {line}")
                    line = w
                else:
                    line = (line + " " + w).lstrip()
            if line:
                print(f"  {line}")
        print(f"  {THIN}")

        # ── step table ────────────────────────────────────────────────────────
        print(f"  {'#':>2}  {'Mode':<12}  {'TX (s)':>6}  {'Cool (s)':>8}  "
              f"{'Gap (s)':>7}  {'MMSSTV':>6}  Fallback")
        print(f"  {'':>2}  {'':12}  {'------':>6}  {'--------':>8}  "
              f"{'-------':>7}  {'------':>6}")
        for i, p in enumerate(profiles, 1):
            gap        = p.duration_seconds + p.cooldown_seconds
            mmsstv_str = "yes" if p.requires_mmsstv else "-"
            fb_str     = p.fallback_mode or "-"
            print(f"  {i:>2}  {p.name:<12}  {p.duration_seconds:>6}  "
                  f"{p.cooldown_seconds:>8}  {gap:>7}  {mmsstv_str:>6}  {fb_str}")
        print(f"  {THIN}")

        # ── summary block ─────────────────────────────────────────────────────
        print(f"  {'Steps':22}: {len(profiles)}")
        print(f"  {'Total airtime':22}: {total_tx} s")
        print(f"  {'Total cooldown':22}: {total_cool} s")
        print(f"  {'Min rotation time':22}: {total_cycle} s  ({cycle_min:.1f} min)")
        print(f"  {'Max duty cycle':22}: {duty:.1f}%")
        if mmsstv_steps:
            print(f"  {'MMSSTV library needed':22}: yes")
            print(f"  {'Fallbacks (unique)':22}: {',  '.join(unique_fb)}")
        else:
            print(f"  {'MMSSTV library needed':22}: no")
        print()


def wait_for_file(path, timeout=5):
    """Wait until file exists and has non-zero size or timeout (seconds)."""
    start = time.time()
    while time.time() - start < timeout:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            return True
        time.sleep(0.1)
    return False


def build_text_overlay(text, size, position, color, background_color=None, background_opacity=None):
    overlay_parts = [
        text,
        f"size={size}",
        f"pos={position}",
        f"color={color}",
    ]

    if background_color:
        overlay_parts.append(f"bg={background_color}")
        if background_opacity is not None:
            overlay_parts.append(f"opacity={background_opacity}")

    return "|".join(overlay_parts)


def get_native_modes():
    return {name for name, profile in MODE_PROFILES.items() if not profile.requires_mmsstv}


def discover_slowframe_capabilities():
    state = RuntimeState(available_modes=set(get_native_modes()))
    log_section("SlowFrame Capability Discovery")

    if os.environ.get(MMSSTV_DISABLE_ENV_VAR):
        log("MMSSTV support disabled by environment; using native modes only.")
        return state

    # --- MMSSTV library detection via -M ---
    lib_path_requested = os.environ.get(MMSSTV_LIBRARY_ENV_VAR)
    mmsstv_cmd = [SLOWFRAME_BIN, "-M"]
    log(f"MMSSTV check: probing SlowFrame with '{' '.join(mmsstv_cmd)}'")
    if lib_path_requested:
        log(f"MMSSTV check: {MMSSTV_LIBRARY_ENV_VAR}={lib_path_requested}")
        if not os.path.isfile(lib_path_requested):
            log(f"MMSSTV check: WARNING — library path does not exist on disk: {lib_path_requested}")
    else:
        log(f"MMSSTV check: {MMSSTV_LIBRARY_ENV_VAR} not set; SlowFrame will search default locations")

    try:
        mmsstv_result = run(
            mmsstv_cmd,
            capture_output=True,
            text=True,
            timeout=SLOWFRAME_LIST_TIMEOUT_SECONDS,
        )
        mmsstv_output = "\n".join(filter(None, [mmsstv_result.stdout, mmsstv_result.stderr]))
        # Log every non-blank line from -M so the full SlowFrame report is visible.
        for line in mmsstv_output.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            log(f"MMSSTV check: {stripped}")
            # Match: "Library Status:      ✓ DETECTED"
            stripped_lower = stripped.lower()
            if stripped_lower.startswith("library status:") and "detected" in stripped_lower and "not detected" not in stripped_lower:
                state.mmsstv_library_detected = True
            # Match: "Library Path:        /path/to/libsstv_encoder.so"
            if stripped_lower.startswith("library path:"):
                state.mmsstv_library_path = stripped.split(":", 1)[1].strip()
        if state.mmsstv_library_detected:
            log(f"MMSSTV check: PASS — library loaded: {state.mmsstv_library_path or '(path not reported)'}")
        else:
            mmsstv_mode_names = ", ".join(
                sorted(n for n, p in MODE_PROFILES.items() if p.requires_mmsstv)
            )
            fallback_pairs = ", ".join(
                f"{n} -> {p.fallback_mode}"
                for n, p in sorted(MODE_PROFILES.items())
                if p.requires_mmsstv and p.fallback_mode
            )
            log("")
            log("  !! MMSSTV LIBRARY NOT AVAILABLE !!")
            log("  The following modes require the MMSSTV encoder library and cannot be used:")
            log(f"    {mmsstv_mode_names}")
            log("  These modes will automatically fall back to native equivalents:")
            log(f"    {fallback_pairs}")
            if lib_path_requested:
                log(f"  {MMSSTV_LIBRARY_ENV_VAR} was set to '{lib_path_requested}'")
                log("  but SlowFrame rejected it. Use the unversioned .so symlink path, e.g.:")
                log("    export MMSSTV_LIB_PATH=/path/to/libsstv_encoder.so")
            else:
                log(f"  To enable MMSSTV modes, set {MMSSTV_LIBRARY_ENV_VAR} or use --mmsstv-lib, e.g.:")
                log("    export MMSSTV_LIB_PATH=/path/to/libsstv_encoder.so")
            log("")
    except Exception as error:
        log(f"MMSSTV check: probe command failed: {error}")

    # --- Mode list via -L ---
    list_cmd = [SLOWFRAME_BIN, "-L"]
    if SLOWFRAME_VERBOSE:
        list_cmd.insert(1, "-v")
    log_debug(f"SlowFrame mode list command: {' '.join(list_cmd)}")
    try:
        result = run(
            list_cmd,
            capture_output=True,
            text=True,
            check=True,
            timeout=SLOWFRAME_LIST_TIMEOUT_SECONDS,
        )
    except Exception as error:
        log(f"SlowFrame mode list failed; using native modes only: {error}")
        return state

    output = "\n".join(filter(None, [result.stdout, result.stderr]))
    log_debug(f"SlowFrame -L raw output:\n{output}")

    for line in output.splitlines():
        mode_match = re.match(r"^([a-z0-9_]+)\s+-", line.strip().lower())
        if mode_match:
            state.available_modes.add(mode_match.group(1))

    mmsstv_status = "MMSSTV enabled" if state.mmsstv_library_detected else "native-only"
    native_modes = sorted(get_native_modes())

    # If the MMSSTV library was detected, add all known MMSSTV mode profiles to
    # available_modes directly.  The -L listing does not enumerate MMSSTV modes
    # in a reliably parseable format, so we populate from MODE_PROFILES instead.
    if state.mmsstv_library_detected:
        for name, profile in MODE_PROFILES.items():
            if profile.requires_mmsstv:
                state.available_modes.add(name)

    mmsstv_modes = sorted(m for m in state.available_modes if m not in native_modes)
    log(f"SlowFrame discovery: {len(state.available_modes)} modes available ({mmsstv_status})")
    log(f"  native : {', '.join(native_modes)}")
    if mmsstv_modes:
        log(f"  mmsstv : {', '.join(mmsstv_modes)}")
        log("  MMSSTV mode details:")
        for mode_name in mmsstv_modes:
            profile = MODE_PROFILES.get(mode_name)
            if profile:
                log(f"    {profile.name:<12}  {profile.duration_seconds:>4}s TX  {profile.description}")
    else:
        log("  mmsstv : none (library not loaded)")

    return state


def resolve_mode_name(requested_mode, available_modes):
    current_mode = requested_mode
    visited_modes = set()

    while current_mode and current_mode not in visited_modes:
        visited_modes.add(current_mode)
        if current_mode in available_modes:
            if current_mode != requested_mode:
                log(f"Mode resolution: {requested_mode} → {current_mode} (fallback)")
            return current_mode

        current_profile = MODE_PROFILES.get(current_mode)

        # Emit a specific warning when an MMSSTV mode is unavailable so the
        # operator knows exactly why the fallback is happening.
        if current_profile and current_profile.requires_mmsstv:
            log(
                f"Mode resolution: {current_mode} requires MMSSTV library but it is not available"
                + (f"; use --mmsstv-lib to provide the library path" if current_mode == requested_mode else "")
            )
        else:
            log_debug(f"Mode resolution: {current_mode} not available, trying fallback")

        next_mode = current_profile.fallback_mode if current_profile else None
        if next_mode:
            log_debug(f"Mode resolution: trying fallback: {next_mode}")
        current_mode = next_mode

    if "r36" in available_modes:
        log(f"Mode resolution: fallback chain exhausted for {requested_mode}, using r36")
        return "r36"

    fallback = next(iter(sorted(available_modes)))
    log(f"Mode resolution: r36 unavailable, using first available mode: {fallback}")
    return fallback


def get_scheduled_mode_name(runtime_state):
    return TRANSMIT_SCHEDULE[runtime_state.schedule_index % len(TRANSMIT_SCHEDULE)]


def prune_transmit_history(runtime_state, now_monotonic):
    runtime_state.transmit_history = [
        history_entry
        for history_entry in runtime_state.transmit_history
        if now_monotonic - history_entry[0] <= ROLLING_DUTY_CYCLE_WINDOW_SECONDS
    ]


def get_rolling_transmit_seconds(runtime_state, now_monotonic):
    prune_transmit_history(runtime_state, now_monotonic)
    return sum(duration_seconds for _, duration_seconds in runtime_state.transmit_history)


def should_attempt_transmit(capture_number, runtime_state):
    if capture_number == 1:
        return True

    captures_since_last_transmit = capture_number - runtime_state.last_transmit_capture_number
    return captures_since_last_transmit >= MIN_CAPTURES_BETWEEN_TRANSMISSIONS


def can_transmit_mode(capture_number, mode_profile, runtime_state, now_monotonic):
    if runtime_state.last_transmit_end_monotonic:
        elapsed_since_last_transmit = now_monotonic - runtime_state.last_transmit_end_monotonic
        effective_cooldown = mode_profile.cooldown_seconds * COOLDOWN_SCALE_FACTOR
        log_debug(
            f"Cooldown check: {elapsed_since_last_transmit:.0f}s elapsed, "
            f"{effective_cooldown:.0f}s required for {mode_profile.name}"
        )
        if elapsed_since_last_transmit < effective_cooldown:
            remaining_cooldown = int(effective_cooldown - elapsed_since_last_transmit)
            return False, f"cooldown active ({remaining_cooldown}s remaining)"

    rolling_transmit_seconds = get_rolling_transmit_seconds(runtime_state, now_monotonic)
    transmit_budget_seconds = ROLLING_DUTY_CYCLE_WINDOW_SECONDS * MAX_TRANSMIT_DUTY_CYCLE
    log_debug(
        f"Duty-cycle check: {rolling_transmit_seconds:.0f}s used + "
        f"{mode_profile.duration_seconds}s needed <= {transmit_budget_seconds:.0f}s budget"
    )
    if rolling_transmit_seconds + mode_profile.duration_seconds > transmit_budget_seconds:
        return False, "rolling duty-cycle budget exceeded"

    return True, "ready"


def select_mode_profile(runtime_state):
    requested_mode = get_scheduled_mode_name(runtime_state)
    resolved_mode = resolve_mode_name(requested_mode, runtime_state.available_modes)
    return requested_mode, MODE_PROFILES[resolved_mode]


def _scaled_overlay_size(image_width: int, base_size: int) -> int:
    """Scale a font size proportionally to image width, anchored at 320px."""
    return max(8, round(base_size * image_width / 320))


def build_slowframe_command(input_path, output_path, timestamp_message, mode_name):
    command = [
        SLOWFRAME_BIN,
        "-i", input_path,
        "-o", output_path,
        "-p", mode_name,
        "-f", SLOWFRAME_AUDIO_FORMAT,
        "-r", str(SLOWFRAME_SAMPLE_RATE),
        "-a", SLOWFRAME_ASPECT_MODE,
    ]

    if SLOWFRAME_VERBOSE:
        command.append("-v")

    mode_width = MODE_PROFILES[mode_name].image_width if mode_name in MODE_PROFILES else 320
    if SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY:
        # When a callsign overlay is also enabled, merge it into the timestamp overlay
        # as a prefix.  SlowFrame renders all -T overlays at the same position
        # regardless of the pos= value on secondary overlays, so separate overlay
        # arguments for callsign and timestamp end up on top of each other.
        if SLOWFRAME_ENABLE_CALLSIGN_OVERLAY and STATION_CALLSIGN:
            overlay_text = f"{STATION_CALLSIGN}  {timestamp_message}"
        else:
            overlay_text = timestamp_message
        command.extend([
            "-T",
            build_text_overlay(
                text=overlay_text,
                size=_scaled_overlay_size(mode_width, SLOWFRAME_TIMESTAMP_OVERLAY_SIZE),
                position=SLOWFRAME_TIMESTAMP_OVERLAY_POSITION,
                color=SLOWFRAME_TIMESTAMP_OVERLAY_COLOR,
                background_color=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR,
                background_opacity=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY,
            ),
        ])
    elif SLOWFRAME_ENABLE_CALLSIGN_OVERLAY and STATION_CALLSIGN:
        # Timestamp overlay off but callsign overlay on — emit callsign alone.
        command.extend([
            "-T",
            build_text_overlay(
                text=STATION_CALLSIGN,
                size=_scaled_overlay_size(mode_width, SLOWFRAME_CALLSIGN_OVERLAY_SIZE),
                position=SLOWFRAME_CALLSIGN_OVERLAY_POSITION,
                color=SLOWFRAME_CALLSIGN_OVERLAY_COLOR,
                background_color=SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR,
            ),
        ])

    return command


def setup_gpio():
    log_debug(f"GPIO setup: mode=BCM, PTT={DRA818_PTT_PIN}, PD={DRA818_POWER_DOWN_PIN}, HL={DRA818_POWER_LEVEL_PIN}")
    GPIO.cleanup()                                                 # clear any stale state from a previous run
    GPIO.setmode(GPIO_PIN_MODE)
    GPIO.setup(DRA818_PTT_PIN, GPIO.OUT, initial=GPIO.HIGH)       # PTT idle = HIGH (unkeyed)
    GPIO.setup(DRA818_POWER_LEVEL_PIN, GPIO.OUT, initial=GPIO.LOW) # HL LOW = low power
    GPIO.setup(DRA818_POWER_DOWN_PIN, GPIO.OUT, initial=GPIO.LOW)  # PD LOW = radio off
    log_debug("GPIO setup complete: PTT=HIGH(idle), HL=LOW(low-power), PD=LOW(off)")


def capture_image(output_path):
    cmd = [
        RPICAM_BIN,
        "--nopreview",
        "-o", output_path,
        "--quality", str(RPICAM_QUALITY),
        "--metering", RPICAM_METERING,
        "--exposure", RPICAM_EXPOSURE,
        "--awb", RPICAM_AWB,
        "--lens-position", "0",
    ]
    log(f"Capture: metering={RPICAM_METERING}  exposure={RPICAM_EXPOSURE}  awb={RPICAM_AWB}  quality={RPICAM_QUALITY}  lens=fixed-infinity")
    log(f"Capture: output -> {output_path}")
    log_debug(f"rpicam-still command: {' '.join(cmd)}")
    try:
        run(cmd, check=True)
        size = os.path.getsize(output_path)
        log(f"Capture: OK  {size:,} bytes")
        return output_path
    except Exception as error:
        log(f"Capture: FAILED ({error}); falling back to test image: {TEST_IMAGE}")
        return TEST_IMAGE


def get_capture_path(capture_time):
    capture_stamp = capture_time.strftime("%Y.%m.%d-%H%M%S")
    capture_name = f"{capture_stamp}.jpg"
    return capture_stamp, os.path.join(TIMESTAMPED_DIR, capture_name)


def resolve_transmit_image(image_path):
    if wait_for_file(image_path, timeout=CAPTURE_FILE_TIMEOUT):
        return image_path

    log("Captured file missing or empty; falling back to test image.")
    return TEST_IMAGE


def generate_sstv_audio(image_path, timestamp_message, mode_name, wav_path=None):
    output = wav_path or SSTV_WAV
    profile = MODE_PROFILES.get(mode_name)
    mode_width = profile.image_width if profile else 320
    expected_duration = profile.duration_seconds if profile else "?"

    # Describe overlay configuration
    if SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY and SLOWFRAME_ENABLE_CALLSIGN_OVERLAY and STATION_CALLSIGN:
        overlay_desc = f"timestamp+callsign  text='{STATION_CALLSIGN}  {timestamp_message}'"
    elif SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY:
        overlay_desc = f"timestamp  text='{timestamp_message}'"
    elif SLOWFRAME_ENABLE_CALLSIGN_OVERLAY and STATION_CALLSIGN:
        overlay_desc = f"callsign  text='{STATION_CALLSIGN}'"
    else:
        overlay_desc = "none"

    log(f"Encode: mode={mode_name}  image={mode_width}px wide  ~{expected_duration}s TX")
    log(f"Encode: format={SLOWFRAME_AUDIO_FORMAT}  rate={SLOWFRAME_SAMPLE_RATE}Hz  aspect={SLOWFRAME_ASPECT_MODE}")
    log(f"Encode: overlay={overlay_desc}")
    log(f"Encode: output -> {output}")

    cmd = build_slowframe_command(image_path, output, timestamp_message, mode_name)
    log_debug(f"SlowFrame command: {' '.join(cmd)}")
    run(cmd, check=True)
    size = os.path.getsize(output)
    log(f"Encode: OK  {size:,} bytes")
    time.sleep(SSTV_CONVERSION_SETTLE_SECONDS)


def transmit_sstv_audio(wav_path=None):
    audio_path = wav_path or SSTV_WAV
    log(f"TX: file={audio_path}")
    log(f"TX: wake_delay={RADIO_WAKE_DELAY_SECONDS}s  ptt_key_delay={PTT_KEY_DELAY_SECONDS}s  post_playback_delay={POST_PLAYBACK_DELAY_SECONDS}s")
    log("TX: PD -> HIGH (radio wake)")
    transmit_started_at = time.monotonic()
    GPIO.output(DRA818_POWER_DOWN_PIN, GPIO.HIGH)
    time.sleep(RADIO_WAKE_DELAY_SECONDS)
    log("TX: PTT -> LOW (keyed)")
    GPIO.output(DRA818_PTT_PIN, GPIO.LOW)
    time.sleep(PTT_KEY_DELAY_SECONDS)

    try:
        run(["aplay", audio_path], check=True)
    finally:
        time.sleep(POST_PLAYBACK_DELAY_SECONDS)
        log("TX: PTT -> HIGH (unkeyed)")
        GPIO.output(DRA818_PTT_PIN, GPIO.HIGH)
        log("TX: PD -> LOW (radio off)")
        GPIO.output(DRA818_POWER_DOWN_PIN, GPIO.LOW)

    elapsed = time.monotonic() - transmit_started_at
    log(f"TX: complete  {elapsed:.1f}s elapsed")
    return elapsed


def run_ptt_test(key_seconds: float):
    """Key the radio briefly to verify GPIO PD/PTT control path."""
    log(f"PTT test: starting (key duration {key_seconds:.2f}s)")
    started_at = time.monotonic()

    # Bring radio out of power-down and then key PTT.
    GPIO.output(DRA818_POWER_DOWN_PIN, GPIO.HIGH)
    log_debug("PTT test: PD->HIGH (radio wake)")
    time.sleep(RADIO_WAKE_DELAY_SECONDS)

    GPIO.output(DRA818_PTT_PIN, GPIO.LOW)
    log_debug("PTT test: PTT->LOW (keyed)")
    time.sleep(key_seconds)

    GPIO.output(DRA818_PTT_PIN, GPIO.HIGH)
    log_debug("PTT test: PTT->HIGH (unkeyed)")
    GPIO.output(DRA818_POWER_DOWN_PIN, GPIO.LOW)
    log_debug("PTT test: PD->LOW (radio off)")

    elapsed = time.monotonic() - started_at
    log(f"PTT test: complete (elapsed {elapsed:.2f}s)")


def process_capture(index):
    current_time = datetime.now()
    picture_time, capture_path = get_capture_path(current_time)
    log(f"--- Capture #{index + 1}  {current_time.strftime('%Y-%m-%d %H:%M:%S')} ---")
    captured_image_path = capture_image(capture_path)
    timestamp_message = current_time.strftime("%Y.%m.%d - %H:%M:%S")

    write_csv(index, picture_time)

    return captured_image_path, timestamp_message


def run_test_pipeline(mode_name: str, args, runtime_state: RuntimeState):
    """Execute a single-shot pipeline test.

    Validates each stage — camera capture, SSTV encode, and optionally radio TX —
    using a consistent file prefix so every output artifact can be identified and
    inspected.  Exits with code 0 on full success, 1 on any stage failure.
    """
    test_id = datetime.now(timezone.utc).strftime("TEST-%Y%m%d-%H%M%S")
    output_dir = args.output_dir
    resolved_mode = resolve_mode_name(mode_name, runtime_state.available_modes)
    wav_path = os.path.join(output_dir, f"{test_id}-{resolved_mode}.wav")

    # Resolve the image source before printing the header so the logged path is accurate.
    if args.test_image is not None:
        capture_path = args.test_image
        capture_source = "supplied"
    else:
        capture_path = os.path.join(output_dir, f"{test_id}-capture.jpg")
        capture_source = "camera"

    separator = "=" * 56
    log_section("Test Pipeline")
    log(f"  run-id  : {test_id}")
    log(f"  mode    : {resolved_mode}" + (f"  (fallback from {mode_name})" if resolved_mode != mode_name else ""))
    log(f"  capture : {capture_path}  [{capture_source}]")
    log(f"  wav     : {wav_path}")
    log(f"  no-tx   : {args.no_tx}")
    log(separator)

    profile = MODE_PROFILES.get(resolved_mode)
    if not profile:
        log(f"ERROR: resolved mode '{resolved_mode}' not found in MODE_PROFILES")
        sys.exit(1)

    # --- Stage 1: Image capture ---
    log("Stage 1/3  Image capture")
    if args.test_image is not None:
        # User explicitly supplied an image — skip the camera entirely.
        if not os.path.isfile(capture_path):
            log(f"  FAIL  --test-image path not found: {capture_path}")
            sys.exit(1)
        size = os.path.getsize(capture_path)
        log(f"  PASS  {capture_path}  ({size:,} bytes)")
    else:
        # Try the camera; fall back to the default test image on failure.
        captured = capture_image(capture_path)
        if captured != capture_path:
            if captured and os.path.isfile(captured):
                capture_path = captured
                size = os.path.getsize(capture_path)
                log(f"  NOTE  camera unavailable; using test image: {capture_path}  ({size:,} bytes)")
            else:
                log(f"  FAIL  camera unavailable and no test image found: {TEST_IMAGE}")
                sys.exit(1)
        else:
            size = os.path.getsize(capture_path)
            log(f"  PASS  {capture_path}  ({size:,} bytes)")

    # --- Stage 2: SSTV encode ---
    log("Stage 2/3  SSTV encode")
    timestamp_message = datetime.now(timezone.utc).strftime("%Y.%m.%d - %H:%M:%S UTC [TEST]")
    try:
        generate_sstv_audio(capture_path, timestamp_message, resolved_mode, wav_path=wav_path)
        size = os.path.getsize(wav_path)
        duration_est = profile.duration_seconds
        log(f"  PASS  {wav_path}  ({size:,} bytes, ~{duration_est}s expected TX)")
    except Exception as error:
        log(f"  FAIL  Encoding failed: {error}")
        sys.exit(1)

    # --- Stage 3: Radio TX ---
    if args.no_tx:
        log("Stage 3/3  Radio TX  SKIPPED (--no-tx)")
        log(separator)
        log("Test result: PASS (encode-only)")
        log(separator)
        return

    log("Stage 3/3  Radio TX")
    try:
        duration = transmit_sstv_audio(wav_path=wav_path)
        log(f"  PASS  transmitted {duration:.1f}s")
    except Exception as error:
        log(f"  FAIL  Transmission failed: {error}")
        sys.exit(1)

    log(separator)
    log("Test result: PASS (full pipeline)")
    log(separator)


def print_mission_summary(runtime_state):
    budget_seconds = int(ROLLING_DUTY_CYCLE_WINDOW_SECONDS * MAX_TRANSMIT_DUTY_CYCLE)
    mmsstv_status = "enabled" if runtime_state.mmsstv_library_detected else "disabled (native modes only)"

    log_section("Mission")
    log(f"Schedule: {TRANSMIT_SCHEDULE_PROFILE}")
    log(f"MMSSTV: {mmsstv_status}")
    log(f"Duty-cycle budget: {budget_seconds}s / {ROLLING_DUTY_CYCLE_WINDOW_SECONDS}s window ({int(MAX_TRANSMIT_DUTY_CYCLE * 100)}% max)")
    log(f"Cooldown scale factor: {COOLDOWN_SCALE_FACTOR}x")
    log(f"Capture interval: {PIC_INTERVAL}s, min {MIN_CAPTURES_BETWEEN_TRANSMISSIONS} between TX")
    log("Scheduled modes:")

    seen = set()
    for mode_name in TRANSMIT_SCHEDULE:
        if mode_name in seen:
            continue
        seen.add(mode_name)
        resolved = resolve_mode_name(mode_name, runtime_state.available_modes)
        profile = MODE_PROFILES[resolved]
        effective_cooldown = int(profile.cooldown_seconds * COOLDOWN_SCALE_FACTOR)
        fallback_note = f" [fallback from {mode_name}]" if resolved != mode_name else ""
        log(f"  {resolved:<12} {profile.duration_seconds:>4}s TX  {effective_cooldown:>5}s cooldown{fallback_note}")

    log("=" * 56)


def main():
    # Show usage hint when called with no arguments at all.
    if len(sys.argv) == 1:
        print(
            "pi_sstv.py  —  HamWing SSTV HAB payload controller\n"
            "\n"
            "No arguments provided.  Common starting points:\n"
            "\n"
            "  Generate a config file (recommended first step):\n"
            f"    python3 pi_sstv.py --generate-config\n"
            "\n"
            "  Run a bench encode test (no radio required):\n"
            "    python3 pi_sstv.py --test r36 --no-tx\n"
            "\n"
            "  Start a live HAB mission with a config file:\n"
            "    python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n"
            "\n"
            "  Show full help and all options:\n"
            "    python3 pi_sstv.py --help\n"
            "\n"
            "  Show detailed documentation for a pipeline topic:\n"
            "    python3 pi_sstv.py --explain capture\n"
            "    python3 pi_sstv.py --explain mmsstv\n"
            "    python3 pi_sstv.py --explain schedule\n"
        )
        sys.exit(0)

    args = parse_args()
    selected_log_file = args.quiet_log_file or args.log_file
    quiet_stdout = args.quiet_log_file is not None
    configure_logging(debug=args.debug, log_file=selected_log_file, quiet_stdout=quiet_stdout)

    # Info-only modes — no GPIO, no paths, no subprocess needed
    if args.list_modes:
        list_modes()
        return
    if args.list_schedules:
        list_schedules()
        return
    if args.explain:
        print_explain(args.explain)
        return
    if args.generate_config is not None:
        generate_default_config(args.generate_config)
        return

    # Apply config file settings (before CLI overrides so CLI always wins).
    if args.config:
        load_config(args.config)

    # Apply CLI overrides to module-level configuration
    global TRANSMIT_SCHEDULE, TRANSMIT_SCHEDULE_PROFILE
    global PIC_TOTAL, PIC_INTERVAL
    global STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global COOLDOWN_SCALE_FACTOR, MAX_TRANSMIT_DUTY_CYCLE, MIN_CAPTURES_BETWEEN_TRANSMISSIONS
    global SLOWFRAME_AUDIO_FORMAT, SLOWFRAME_SAMPLE_RATE, SLOWFRAME_ASPECT_MODE
    global TIMESTAMPED_DIR, SLOWFRAME_BIN, TEST_IMAGE, SSTV_WAV

    TRANSMIT_SCHEDULE_PROFILE = args.schedule
    TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES.get(args.schedule, TRANSMIT_SCHEDULE_PROFILES["hab_cruise"])
    PIC_TOTAL = args.total
    PIC_INTERVAL = args.interval
    if args.callsign:
        STATION_CALLSIGN = args.callsign
        SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = True
    COOLDOWN_SCALE_FACTOR = args.cooldown_scale
    MAX_TRANSMIT_DUTY_CYCLE = args.duty_cycle
    MIN_CAPTURES_BETWEEN_TRANSMISSIONS = args.min_captures
    SLOWFRAME_AUDIO_FORMAT = args.format
    SLOWFRAME_SAMPLE_RATE = args.sample_rate
    SLOWFRAME_ASPECT_MODE = args.aspect
    TIMESTAMPED_DIR = args.output_dir
    SLOWFRAME_BIN = args.slowframe
    if args.test_image is not None:
        TEST_IMAGE = args.test_image
    SSTV_WAV = os.path.join(TIMESTAMPED_DIR, "HAB-SSTV.wav")

    if args.no_mmsstv:
        os.environ[MMSSTV_DISABLE_ENV_VAR] = "1"
    if args.mmsstv_lib:
        os.environ[MMSSTV_LIBRARY_ENV_VAR] = args.mmsstv_lib

    ensure_runtime_paths()

    # Determine whether GPIO is needed
    needs_gpio = not (args.test and args.no_tx)
    if needs_gpio:
        setup_gpio()

    if args.ptt_test is not None:
        run_ptt_test(args.ptt_test)
        return

    runtime_state = discover_slowframe_capabilities()

    try:
        if args.test:
            run_test_pipeline(args.test, args, runtime_state)
            return

        # --- Normal HAB mission ---
        print_mission_summary(runtime_state)

        for index in range(PIC_TOTAL):
            capture_number = index + 1
            image_path, timestamp_message = process_capture(index)

            time.sleep(PIC_INTERVAL)

            if not should_attempt_transmit(capture_number, runtime_state):
                log_debug(f"[#{capture_number}] Skipping transmit check: capture interval not reached")
                continue

            requested_mode, mode_profile = select_mode_profile(runtime_state)
            now_monotonic = time.monotonic()
            can_transmit, reason = can_transmit_mode(
                capture_number,
                mode_profile,
                runtime_state,
                now_monotonic,
            )

            if not can_transmit:
                log(f"[#{capture_number}] Skipping transmit ({requested_mode}): {reason}")
                continue

            if requested_mode != mode_profile.name:
                log(f"[#{capture_number}] Fallback: {requested_mode} \u2192 {mode_profile.name}")
            else:
                log(f"[#{capture_number}] Transmitting: {mode_profile.name} ({mode_profile.duration_seconds}s)")

            image_path = resolve_transmit_image(image_path)

            if args.no_tx:
                log(f"[#{capture_number}] TX skipped (--no-tx): would have transmitted {mode_profile.name}")
                continue

            try:
                generate_sstv_audio(image_path, timestamp_message, mode_profile.name)
            except Exception as error:
                log(f"Slowframe conversion failed: {error}")
                continue

            try:
                actual_transmit_duration = transmit_sstv_audio()
            except Exception as error:
                log(f"Playback failed: {error}")
                continue

            runtime_state.last_transmit_capture_number = capture_number
            runtime_state.last_transmit_end_monotonic = time.monotonic()
            runtime_state.transmit_history.append(
                (runtime_state.last_transmit_end_monotonic, actual_transmit_duration)
            )
            runtime_state.schedule_index += 1
            rolling_used = get_rolling_transmit_seconds(runtime_state, runtime_state.last_transmit_end_monotonic)
            rolling_pct = 100.0 * rolling_used / ROLLING_DUTY_CYCLE_WINDOW_SECONDS
            log(f"[#{capture_number}] TX done: {mode_profile.name}, {actual_transmit_duration:.1f}s, rolling duty={rolling_pct:.1f}%")
    finally:
        if needs_gpio:
            GPIO.cleanup()


if __name__ == '__main__':
    main()
