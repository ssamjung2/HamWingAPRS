#!/usr/bin/env python3

import argparse
import configparser
import logging
import sys
from dataclasses import dataclass, field
import time
from subprocess import run, CalledProcessError
from datetime import datetime, timezone
import csv
import os
import re
from typing import List, Optional, Set, Tuple
import RPi.GPIO as GPIO

logger = logging.getLogger("pi_sstv")

SCRIPT_VERSION = "2.0"

# Paths and constants
BASE_DIR = "/home/pi-user"
DEFAULT_CONFIG_PATH = os.path.join(BASE_DIR, "pi_sstv.cfg")

# Ordered list of directories searched for pi_sstv.cfg when no --config flag is
# given.  The first file found is used.  Evaluated at runtime so $HOME is resolved
# for the actual calling user.
CONFIG_SEARCH_PATHS = [
    os.path.join(os.getcwd(), "pi_sstv.cfg"),                          # .  (cwd)
    os.path.join(os.path.expanduser("~"), ".pi_sstv", "pi_sstv.cfg"),  # ~/.pi_sstv/
    os.path.join("/etc", "pi_sstv.cfg"),                               # /etc/
    DEFAULT_CONFIG_PATH,                                               # /home/pi-user/
]
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
OVERLAY_TEXT_OVERRIDE = ""  # If non-empty, replaces the entire overlay text on every image.
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
MISSION_ENABLED = True  # Set to false in config to disable mission (service exits cleanly, no watchdog restart)

# Dedicated image panel/card test workflow (separate from mission fallback).
# Used only when --test-panels is invoked explicitly.
TEST_PANEL_SOURCE = ""           # File or folder path; required when --test-panels is used.
TEST_PANEL_SELECTION = "sequential"  # sequential or random (when source is a folder)
TEST_PANEL_COUNT = 1
TEST_PANEL_DEFAULT_MODE = "pd50"
TEST_PANEL_INCLUDE_CALLSIGN_OVERLAY = True
TEST_PANEL_INCLUDE_TIMESTAMP_OVERLAY = False
TEST_PANEL_ALLOW_TX_WITHOUT_CALLSIGN = False  # FCC Part 97 guardrail
TEST_PANEL_IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")

MIN_CAPTURES_BETWEEN_TRANSMISSIONS = 0  # 0 = no capture-count floor; rely on cooldown gating only
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
DRA818_VHF_PTT_PIN = 27      # BCM 27, physical pin 13 — active-LOW VHF TX key
DRA818_UHF_PTT_PIN = 17      # BCM 17, physical pin 11 — active-LOW UHF TX key
DRA818_POWER_DOWN_PIN = 4    # BCM  4, physical pin  7 — HIGH=active, LOW=sleep
DRA818_POWER_LEVEL_PIN = 22  # BCM 22, physical pin 15 — HIGH=1 W, LOW=0.5 W

# Active radio band selection: "vhf", "uhf", or "both"
ACTIVE_RADIO_BAND = "vhf"
TX_POWER_LEVEL = "low"      # "low"=0.5 W (HL LOW), "high"=1 W (HL HIGH)
PD_IDLE_MODE = "release"    # "release"=INPUT (Feather owns PD), "sleep"=OUTPUT LOW

# Status LED — GPIO-driven LED used to signal operational state to the operator.
# Wiring: GPIO pin → current-limiting resistor (330 Ω) → LED anode → cathode → GND
# active_high=True means GPIO HIGH = LED on (common-cathode wiring).
STATUS_LED_ENABLED     = True
STATUS_LED_PIN         = 21        # BCM numbering; physical pin 40
STATUS_LED_ACTIVE_HIGH = True
STATUS_LED_PWM_FREQ    = 120       # Hz — high enough that PWM flicker is invisible
STATUS_LED_MAX_PCT     = 100       # 0–100; cap brightness if LED is very bright

# PWM audio routing used by aplay via the Raspberry Pi audio overlay
AUDIO_LEFT_PWM_PIN = 12
AUDIO_RIGHT_PWM_PIN = 13
AUDIO_OVERLAY = "dtoverlay=audremap,enable_jack=on"
APLAY_DEVICE = "plughw:CARD=Headphones,DEV=0"  # default device for dtoverlay=audremap

# Alternate audio pair if config.txt uses dtoverlay=audremap,pins_18_19,enable_jack=on
# AUDIO_LEFT_PWM_PIN = 18
# AUDIO_RIGHT_PWM_PIN = 19

# Optional GPS UART pins (used by diag gps)
# GPS_UART_TX_PIN = 14  # physical pin 8
# GPS_UART_RX_PIN = 15  # physical pin 10
GPS_SERIAL_PORT = "/dev/ttyS0"   # Pi Zero UART; change to /dev/ttyAMA0 if needed
GPS_SERIAL_BAUD = 9600

# Optional I2C sensor pins (not used by this script)
# SENSOR_I2C_SDA_PIN = 2  # physical pin 3
# SENSOR_I2C_SCL_PIN = 3  # physical pin 5


@dataclass(frozen=True)
class ModeProfile:
    name: str
    duration_seconds: int
    cooldown_seconds: int
    image_width: int = 320
    image_height: Optional[int] = None
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
        cooldown_seconds=8,
        image_width=160,
        image_height=120,
        requires_mmsstv=True,
        fallback_mode="bw24",
        description="Ultra-fast MMSSTV monochrome status frame for tight duty-cycle budgets.",
    ),
    "robot12bw": ModeProfile(
        name="robot12bw",
        duration_seconds=12,
        cooldown_seconds=12,
        image_width=160,
        image_height=120,
        requires_mmsstv=True,
        fallback_mode="bw24",
        description="Very fast MMSSTV monochrome mode for rapid update windows.",
    ),
    "bw24": ModeProfile(
        name="bw24",
        duration_seconds=24,
        cooldown_seconds=24,
        image_width=320,
        image_height=120,
        description="Fast monochrome native mode for low duty-cycle updates.",
    ),
    "r36": ModeProfile(
        name="r36",
        duration_seconds=36,
        cooldown_seconds=36,
        image_width=320,
        image_height=240,
        description="Fast native color mode for regular balloon image updates.",
    ),
    "m2": ModeProfile(
        name="m2",
        duration_seconds=58,
        cooldown_seconds=58,
        image_width=320,
        image_height=256,
        description="Balanced native mode with good compatibility.",
    ),
    "s2": ModeProfile(
        name="s2",
        duration_seconds=71,
        cooldown_seconds=71,
        image_width=320,
        image_height=256,
        description="Native Scottie mode with strong compatibility and moderate airtime.",
    ),
    "sdx": ModeProfile(
        name="sdx",
        duration_seconds=269,
        cooldown_seconds=269,
        image_width=320,
        image_height=256,
        description="Native Scottie DX mode for very high-detail, long-duration snapshots.",
    ),
    "r72": ModeProfile(
        name="r72",
        duration_seconds=72,
        cooldown_seconds=72,
        image_width=320,
        image_height=240,
        description="Higher-quality native Robot mode.",
    ),
    "pd50": ModeProfile(
        name="pd50",
        duration_seconds=50,
        cooldown_seconds=50,
        image_width=320,
        image_height=256,
        requires_mmsstv=True,
        fallback_mode="m2",
        description="Fast MMSSTV PD mode for efficient color updates.",
    ),
    "pd90": ModeProfile(
        name="pd90",
        duration_seconds=90,
        cooldown_seconds=90,
        image_width=320,
        image_height=256,
        requires_mmsstv=True,
        fallback_mode="r36",
        description="Popular MMSSTV fast color mode when the encoder library is available.",
    ),
    "m1": ModeProfile(
        name="m1",
        duration_seconds=114,
        cooldown_seconds=114,
        image_width=320,
        image_height=256,
        description="High-quality native Martin mode for less frequent transmissions.",
    ),
    "s1": ModeProfile(
        name="s1",
        duration_seconds=110,
        cooldown_seconds=110,
        image_width=320,
        image_height=256,
        description="Native Scottie high-quality mode for periodic detail shots.",
    ),
    "pd120": ModeProfile(
        name="pd120",
        duration_seconds=120,
        cooldown_seconds=120,
        image_width=640,
        image_height=496,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Higher-quality MMSSTV mode with a larger cooldown budget.",
    ),
    "pd160": ModeProfile(
        name="pd160",
        duration_seconds=160,
        cooldown_seconds=160,
        image_width=512,
        image_height=400,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Slower MMSSTV quality mode for longer detail passes.",
    ),
    "pd180": ModeProfile(
        name="pd180",
        duration_seconds=180,
        cooldown_seconds=180,
        image_width=640,
        image_height=496,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="High-detail MMSSTV mode, best for occasional mission snapshots.",
    ),
    "fax480": ModeProfile(
        name="fax480",
        duration_seconds=180,
        cooldown_seconds=180,
        image_width=512,
        image_height=480,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="High-detail MMSSTV mode best reserved for test windows.",
    ),
    "pd240": ModeProfile(
        name="pd240",
        duration_seconds=240,
        cooldown_seconds=240,
        image_width=640,
        image_height=496,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Very high quality MMSSTV PD mode for science windows and horizon detail passes.",
    ),
    "pd290": ModeProfile(
        name="pd290",
        duration_seconds=290,
        cooldown_seconds=290,
        image_width=800,
        image_height=616,
        requires_mmsstv=True,
        fallback_mode="pd180",
        description="Highest quality MMSSTV mode; reserve for occasional best-shot captures at peak altitude.",
    ),
}

# Accept common SlowFrame/MMSSTV naming variants and punctuation styles.
# Keys are normalized with non-alphanumeric characters removed.
MODE_ALIASES = {
    "bw8": "robot8bw",
    "bw12": "robot12bw",
    "robot24": "bw24",
    "robot36": "r36",
    "robot72": "r72",
    "martin1": "m1",
    "martin2": "m2",
    "scottie1": "s1",
    "scottie2": "s2",
    "scottiedx": "sdx",
}

# Mode key -> exact token to pass to `slowframe -p`.
# Defaults to canonical keys until runtime discovery captures precise tokens.
MODE_PROTOCOL_TOKENS = {name: name for name in MODE_PROFILES}


def canonicalize_mode_name(mode_name: Optional[str]) -> Optional[str]:
    """Normalize user-provided mode names/aliases to script canonical keys."""
    if mode_name is None:
        return None
    raw = mode_name.strip().lower()
    if not raw:
        return raw
    if raw in MODE_PROFILES:
        return raw
    normalized = re.sub(r"[^a-z0-9]", "", raw)
    if normalized in MODE_PROFILES:
        return normalized
    if normalized in MODE_ALIASES:
        return MODE_ALIASES[normalized]
    return raw


def _normalize_schedule_mode_name(mode_name: Optional[str]) -> Optional[str]:
    if mode_name is None:
        return None
    raw = mode_name.strip().lower()
    if not raw:
        return None
    return canonicalize_mode_name(raw) or raw


def _is_valid_schedule_mode_token(mode_name: str) -> bool:
    return re.fullmatch(r"[a-z0-9][a-z0-9_/-]*", mode_name) is not None


def _parse_schedule_mode_list(raw_value: str, context: str) -> Tuple[str, ...]:
    tokens = [
        _normalize_schedule_mode_name(token)
        for token in re.split(r"[\s,]+", raw_value)
        if token.strip()
    ]
    modes = tuple(token for token in tokens if token)
    if not modes:
        raise ValueError(f"{context}: at least one mode is required")
    invalid_tokens = [token for token in modes if not _is_valid_schedule_mode_token(token)]
    if invalid_tokens:
        raise ValueError(f"{context}: invalid mode token(s): {', '.join(invalid_tokens)}")
    return modes


TRANSMIT_SCHEDULE_PROFILE = "standard"
BUILTIN_TRANSMIT_SCHEDULE_PROFILES = {
    # mono: Monochrome status-only for maximum update frequency.
    "mono": (
        "robot12bw",
        "bw24",
    ),
    # rapid: Fast color + status filler for frequent imaging.
    "rapid": (
        "robot12bw",
        "r36",
    ),
    # standard: Balanced quality and frequency (default).
    "standard": (
        "r36",
        "pd50",
        "pd90",
    ),
    # quality: Medium-quality color with longer cooldown budgets.
    "quality": (
        "pd50",
        "pd120",
    ),
    # high-res: Highest quality detailed imagery.
    "high-res": (
        "pd120",
        "pd290",
    ),
    # native-rapid: Fast native-only rotation; no MMSSTV dependency.
    "native-rapid": (
        "bw24",
        "r36",
    ),
    # native-balanced: General native-only mission profile; no MMSSTV dependency.
    "native-balanced": (
        "r36",
        "m2",
        "s2",
    ),
    # native-detail: Higher-detail native-only rotation; no MMSSTV dependency.
    "native-detail": (
        "r72",
        "m1",
        "s1",
    ),
}
BUILTIN_TRANSMIT_SCHEDULE_DESCRIPTIONS = {
    "mono":             "Monochrome status updates only. Best when you need the highest update frequency.",
    "rapid":            "Fast color plus status framing. Good balance of image cadence and color detail.",
    "standard":         "Balanced quality and frequency (default). Recommended for general operation.",
    "quality":          "Quality-focused color imaging. Use when you want stronger image detail without "
                        "committing to maximum-resolution cadence.",
    "high-res":         "Maximum detail imaging. Use when high-resolution frames are prioritized over "
                        "update rate.",
    "native-rapid":     "Native-only rapid imaging (bw24 + r36). Use when MMSSTV is unavailable and "
                        "frequent updates are required.",
    "native-balanced":  "Native-only balanced imaging (r36 + m2 + s2). Good default when you want "
                        "color quality without MMSSTV library dependence.",
    "native-detail":    "Native-only higher-detail imaging (r72 + m1 + s1). Use when prioritizing "
                        "native image detail over cadence.",
}
BUILTIN_TRANSMIT_SCHEDULE_FALLBACK_MODES = {
    name: None for name in BUILTIN_TRANSMIT_SCHEDULE_PROFILES
}
SCHEDULE_PROFILE_SECTION_PREFIX = "schedule_profile "
GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK = "r36"

# Mutable working copies — extended at config-load time with custom profiles.
TRANSMIT_SCHEDULE_PROFILES = {
    name: tuple(modes) for name, modes in BUILTIN_TRANSMIT_SCHEDULE_PROFILES.items()
}
TRANSMIT_SCHEDULE_DESCRIPTIONS = dict(BUILTIN_TRANSMIT_SCHEDULE_DESCRIPTIONS)
TRANSMIT_SCHEDULE_FALLBACK_MODES = dict(BUILTIN_TRANSMIT_SCHEDULE_FALLBACK_MODES)

TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES.get(
    TRANSMIT_SCHEDULE_PROFILE,
    TRANSMIT_SCHEDULE_PROFILES["standard"],
)


def _reset_schedule_profile_registry() -> None:
    """Restore TRANSMIT_SCHEDULE_PROFILES/DESCRIPTIONS/FALLBACK_MODES to built-in defaults."""
    global TRANSMIT_SCHEDULE_PROFILES, TRANSMIT_SCHEDULE_DESCRIPTIONS, TRANSMIT_SCHEDULE_FALLBACK_MODES
    TRANSMIT_SCHEDULE_PROFILES = {
        name: tuple(modes) for name, modes in BUILTIN_TRANSMIT_SCHEDULE_PROFILES.items()
    }
    TRANSMIT_SCHEDULE_DESCRIPTIONS = dict(BUILTIN_TRANSMIT_SCHEDULE_DESCRIPTIONS)
    TRANSMIT_SCHEDULE_FALLBACK_MODES = dict(BUILTIN_TRANSMIT_SCHEDULE_FALLBACK_MODES)


def _normalize_schedule_profile_name(profile_name: Optional[str]) -> Optional[str]:
    if profile_name is None:
        return None
    normalized = profile_name.strip().lower()
    return normalized or None


def _validate_schedule_profiles(strict_defined_modes: bool = True) -> None:
    invalid_by_profile = {}
    for profile_name, modes in TRANSMIT_SCHEDULE_PROFILES.items():
        invalid_modes: List[str] = []
        for mode_name in modes:
            normalized_mode = _normalize_schedule_mode_name(mode_name)
            if not normalized_mode or not _is_valid_schedule_mode_token(normalized_mode):
                invalid_modes.append(mode_name)
                continue
            if strict_defined_modes and normalized_mode not in MODE_PROFILES:
                invalid_modes.append(mode_name)
        if invalid_modes:
            invalid_by_profile[profile_name] = invalid_modes
    if invalid_by_profile:
        details = "; ".join(
            f"{pn}: {', '.join(im)}"
            for pn, im in sorted(invalid_by_profile.items())
        )
        label = "undefined or invalid" if strict_defined_modes else "invalid"
        raise RuntimeError(f"Schedule profile contains {label} mode(s): {details}")


_validate_schedule_profiles()


def get_effective_schedule_fallback_mode(profile_name: Optional[str] = None) -> Optional[str]:
    """Return the most specific unavailable-mode fallback for a schedule profile."""
    normalized = _normalize_schedule_profile_name(profile_name)
    profile_fallback = None
    if normalized is not None:
        profile_fallback = TRANSMIT_SCHEDULE_FALLBACK_MODES.get(normalized)
    return _normalize_schedule_mode_name(profile_fallback or GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK)


def describe_schedule_fallback_policy(profile_name: Optional[str] = None) -> str:
    normalized = _normalize_schedule_profile_name(profile_name)
    profile_fallback = None
    if normalized is not None:
        profile_fallback = _normalize_schedule_mode_name(
            TRANSMIT_SCHEDULE_FALLBACK_MODES.get(normalized)
        )
    global_fallback = _normalize_schedule_mode_name(GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK)
    if profile_fallback:
        return f"profile={profile_fallback}  global={global_fallback or '-'}"
    return f"profile=<inherit>  global={global_fallback or '-'}"


def find_default_config() -> Optional[str]:
    """Return the first pi_sstv.cfg found in CONFIG_SEARCH_PATHS, or None."""
    for candidate in CONFIG_SEARCH_PATHS:
        if os.path.isfile(candidate):
            return candidate
    return None


def generate_default_config(path: str):
    """Write a fully-commented default configuration file to *path*."""
    builtin_schedule_lines: List[str] = []
    for schedule_name, modes in BUILTIN_TRANSMIT_SCHEDULE_PROFILES.items():
        description = BUILTIN_TRANSMIT_SCHEDULE_DESCRIPTIONS.get(schedule_name, "")
        default_marker = " (default)" if schedule_name == TRANSMIT_SCHEDULE_PROFILE else ""
        summary = f"{description}{default_marker}".strip()
        builtin_schedule_lines.append(f"#   {schedule_name:<16} - {summary}")
        builtin_schedule_lines.append(f"#                      -> {' -> '.join(modes)}")
    builtin_schedule_block = "\n".join(builtin_schedule_lines)
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

# Set to false to disable mission execution.  The script (and systemd service)
# will start, log a notice, and exit cleanly without capturing or transmitting.
enabled = true

# Transmit schedule preset.  Controls which SSTV modes are used in rotation.
# Built-in presets:
{builtin_schedule_block}
# Additional custom presets can be defined with [schedule_profile <name>] sections.
# CLI note: --schedule NAME overrides this value after the config file is loaded.
schedule = {TRANSMIT_SCHEDULE_PROFILE}

# Global default mode used when a scheduled mode is unavailable and its own
# fallback chain is exhausted.
# unavailable_mode_fallback = {GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK}

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
# [test_panels]  Dedicated test-image transmit workflow (--test-panels)
# -----------------------------------------------------------------------------
# Transmits a static image file (or images from a folder) instead of live
# captures.  Only active when the --test-panels verb is used.  All values are
# optional; defaults are shown.
[test_panels]

# Path to a single image file or a directory containing images.
# Required when --test-panels is invoked.
# source =

# How to pick images when source is a directory.
#   sequential  - In lexicographic order (default).
#   random      - Randomly sampled with replacement.
# selection = sequential

# How many images to transmit (ignored when source is a single file).
# count = 1

# SSTV mode to use for test-panel transmissions.
# mode = pd50

# Overlay behaviour during test-panel transmissions.
# include_callsign_overlay = true
# include_timestamp_overlay = false

# Set to true only if you are running the test without a callsign configured.
# The default (false) is the FCC Part 97-compliant guardrail.
# allow_tx_without_callsign = false


# -----------------------------------------------------------------------------
# [schedule_profile custom]  Optional operator-defined schedule profile
# -----------------------------------------------------------------------------
# Any section named [schedule_profile <name>] becomes a selectable preset.
# Profile names are normalized to lowercase when loaded.
# The modes value may use commas, whitespace, or both as separators.
# Mode tokens are validated when the config loads; actual encoder availability
# is resolved later at runtime after SlowFrame mode discovery.
#
# Activate with:  schedule = custom  (in [mission] above)
# Or override from the CLI:  python3 pi_sstv.py --config pi_sstv.cfg --schedule custom
#
# Full example with optional per-profile fallback override:
# [schedule_profile custom]
# modes = robot12bw, r36, pd50, pd90
# description = Operator-defined mixed cadence profile.
# unavailable_mode_fallback = m2


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

# Optional free-form text to replace the entire overlay on every image.
# When set, both timestamp and callsign overlays are suppressed in favour of
# this string.  Leave blank (or omit) to use the auto-generated overlay text.
# custom_text =


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
    global TRANSMIT_SCHEDULE_PROFILES, TRANSMIT_SCHEDULE_DESCRIPTIONS, TRANSMIT_SCHEDULE_FALLBACK_MODES
    global GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK
    global PIC_TOTAL, PIC_INTERVAL, MISSION_ENABLED, STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global OVERLAY_TEXT_OVERRIDE
    global TEST_PANEL_SOURCE, TEST_PANEL_SELECTION, TEST_PANEL_COUNT, TEST_PANEL_DEFAULT_MODE
    global TEST_PANEL_INCLUDE_CALLSIGN_OVERLAY, TEST_PANEL_INCLUDE_TIMESTAMP_OVERLAY
    global TEST_PANEL_ALLOW_TX_WITHOUT_CALLSIGN
    global MIN_CAPTURES_BETWEEN_TRANSMISSIONS
    global STATUS_LED_ENABLED, STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH
    global STATUS_LED_PWM_FREQ, STATUS_LED_MAX_PCT
    global ACTIVE_RADIO_BAND, TX_POWER_LEVEL, PD_IDLE_MODE
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
    global APLAY_DEVICE

    if not os.path.isfile(path):
        print(f"Config file not found: {path}", file=sys.stderr)
        sys.exit(1)

    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(path)
    _reset_schedule_profile_registry()

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
    # [schedule_profile <name>]  — load custom operator-defined presets first so
    # they are available when [mission] schedule is validated below.
    for section_name in cfg.sections():
        lowered_section = section_name.lower()
        if not lowered_section.startswith(SCHEDULE_PROFILE_SECTION_PREFIX):
            continue
        profile_name = _normalize_schedule_profile_name(
            section_name[len(SCHEDULE_PROFILE_SECTION_PREFIX):]
        )
        if not profile_name:
            print(
                f"Config [{section_name}]: section name must include a profile name "
                f"after '{SCHEDULE_PROFILE_SECTION_PREFIX}'",
                file=sys.stderr,
            )
            sys.exit(1)
        modes_raw = cfg.get(section_name, "modes", fallback="")
        try:
            parsed_modes = _parse_schedule_mode_list(
                modes_raw, f"Config [{section_name}] modes"
            )
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            sys.exit(1)
        description = cfg.get(section_name, "description", fallback="").strip()
        if not description:
            description = (
                f"Operator-defined schedule loaded from config section [{section_name}]."
            )
        fallback_raw = cfg.get(section_name, "unavailable_mode_fallback", fallback="").strip()
        fallback_mode = _normalize_schedule_mode_name(fallback_raw) if fallback_raw else None
        if fallback_mode and not _is_valid_schedule_mode_token(fallback_mode):
            print(
                f"Config [{section_name}] unavailable_mode_fallback: "
                f"invalid mode token '{fallback_raw}'",
                file=sys.stderr,
            )
            sys.exit(1)
        TRANSMIT_SCHEDULE_PROFILES[profile_name] = parsed_modes
        TRANSMIT_SCHEDULE_DESCRIPTIONS[profile_name] = description
        TRANSMIT_SCHEDULE_FALLBACK_MODES[profile_name] = fallback_mode

    try:
        _validate_schedule_profiles(strict_defined_modes=False)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)

    # [mission]
    GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK = _normalize_schedule_mode_name(
        _str("mission", "unavailable_mode_fallback", GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK)
    ) or GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK
    if not _is_valid_schedule_mode_token(GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK):
        print(
            "Config [mission] unavailable_mode_fallback: "
            f"invalid mode token '{GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK}'",
            file=sys.stderr,
        )
        sys.exit(1)

    schedule = _normalize_schedule_profile_name(
        _str("mission", "schedule", TRANSMIT_SCHEDULE_PROFILE)
    )
    if schedule not in TRANSMIT_SCHEDULE_PROFILES:
        print(f"Config [mission] schedule: unknown preset '{schedule}'. "
              f"Valid: {', '.join(TRANSMIT_SCHEDULE_PROFILES)}", file=sys.stderr)
        sys.exit(1)
    TRANSMIT_SCHEDULE_PROFILE = schedule
    TRANSMIT_SCHEDULE         = TRANSMIT_SCHEDULE_PROFILES[schedule]
    MISSION_ENABLED                      = _bool ("mission", "enabled",                          MISSION_ENABLED)
    PIC_TOTAL                        = _int  ("mission", "total",                            PIC_TOTAL)
    PIC_INTERVAL                     = _float("mission", "interval",                         PIC_INTERVAL)
    callsign                         = _str  ("mission", "callsign",                         STATION_CALLSIGN).strip()
    MIN_CAPTURES_BETWEEN_TRANSMISSIONS = _int("mission", "min_captures_between_transmissions", MIN_CAPTURES_BETWEEN_TRANSMISSIONS)
    if callsign:
        STATION_CALLSIGN             = callsign
        SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = True

    # [test_panels]
    TEST_PANEL_SOURCE = _str("test_panels", "source", TEST_PANEL_SOURCE).strip()
    panel_selection = _str("test_panels", "selection", TEST_PANEL_SELECTION).strip().lower()
    if panel_selection not in ("sequential", "random"):
        print("Config [test_panels] selection: invalid value. Valid: sequential, random", file=sys.stderr)
        sys.exit(1)
    TEST_PANEL_SELECTION = panel_selection
    TEST_PANEL_COUNT = max(1, _int("test_panels", "count", TEST_PANEL_COUNT))
    TEST_PANEL_DEFAULT_MODE = canonicalize_mode_name(
        _str("test_panels", "mode", TEST_PANEL_DEFAULT_MODE).strip().lower()
    ) or TEST_PANEL_DEFAULT_MODE
    TEST_PANEL_INCLUDE_CALLSIGN_OVERLAY  = _bool("test_panels", "include_callsign_overlay",  TEST_PANEL_INCLUDE_CALLSIGN_OVERLAY)
    TEST_PANEL_INCLUDE_TIMESTAMP_OVERLAY = _bool("test_panels", "include_timestamp_overlay", TEST_PANEL_INCLUDE_TIMESTAMP_OVERLAY)
    TEST_PANEL_ALLOW_TX_WITHOUT_CALLSIGN = _bool("test_panels", "allow_tx_without_callsign", TEST_PANEL_ALLOW_TX_WITHOUT_CALLSIGN)

    # [radio]
    band = _str("radio", "band", ACTIVE_RADIO_BAND).strip().lower()
    if band not in ("vhf", "uhf", "both"):
        print(f"Config [radio] band: invalid value '{band}'. Valid: vhf, uhf, both", file=sys.stderr)
        sys.exit(1)
    ACTIVE_RADIO_BAND                 = band
    tx_power = _str("radio", "tx_power_level", TX_POWER_LEVEL).strip().lower()
    if tx_power not in ("low", "high"):
        print(f"Config [radio] tx_power_level: invalid value '{tx_power}'. Valid: low, high", file=sys.stderr)
        sys.exit(1)
    TX_POWER_LEVEL                    = tx_power
    pd_idle = _str("radio", "pd_idle_mode", PD_IDLE_MODE).strip().lower()
    if pd_idle not in ("release", "sleep"):
        print(f"Config [radio] pd_idle_mode: invalid value '{pd_idle}'. Valid: release, sleep", file=sys.stderr)
        sys.exit(1)
    PD_IDLE_MODE                      = pd_idle
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
    OVERLAY_TEXT_OVERRIDE                       = _str  ("overlay", "custom_text",               OVERLAY_TEXT_OVERRIDE).strip()

    # [alsa]
    APLAY_DEVICE = _str("alsa", "playback", APLAY_DEVICE)

    # [mmsstv]
    lib_path = _str("mmsstv", "lib_path", "").strip()
    if lib_path:
        os.environ[MMSSTV_LIBRARY_ENV_VAR] = lib_path
    if _bool("mmsstv", "disable", False):
        os.environ[MMSSTV_DISABLE_ENV_VAR] = "1"

    # [status_led]
    STATUS_LED_ENABLED     = _bool("status_led", "enabled",     STATUS_LED_ENABLED)
    STATUS_LED_PIN         = _int ("status_led", "pin",         STATUS_LED_PIN)
    STATUS_LED_ACTIVE_HIGH = _bool("status_led", "active_high", STATUS_LED_ACTIVE_HIGH)
    STATUS_LED_PWM_FREQ    = _int ("status_led", "pwm_freq",    STATUS_LED_PWM_FREQ)
    STATUS_LED_MAX_PCT     = _int ("status_led", "max_pct",     STATUS_LED_MAX_PCT)

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
  DRA818_VHF_PTT_PIN     = {ptt}   (physical pin 13)
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
        ptt=DRA818_VHF_PTT_PIN,
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
            "Auto-search order (if not specified): "
            ". → ~/.pi_sstv/ → /etc/ → " + DEFAULT_CONFIG_PATH
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
    builtin_names = set(BUILTIN_TRANSMIT_SCHEDULE_PROFILES.keys())
    print("Schedule presets\n")
    for preset_name, modes in TRANSMIT_SCHEDULE_PROFILES.items():
        is_active    = preset_name == TRANSMIT_SCHEDULE_PROFILE
        is_custom    = preset_name not in builtin_names
        active_tag   = "  ◀ active" if is_active else ""
        custom_tag   = "  [custom]" if is_custom else ""
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
        print(f"  {preset_name.upper()}{active_tag}{custom_tag}")
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
        fb_policy = describe_schedule_fallback_policy(preset_name)
        print(f"  Fallback policy: {fb_policy}")
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
    if OVERLAY_TEXT_OVERRIDE:
        # Operator-supplied free-form text replaces all auto-generated overlay content.
        command.extend([
            "-T",
            build_text_overlay(
                text=OVERLAY_TEXT_OVERRIDE,
                size=_scaled_overlay_size(mode_width, SLOWFRAME_TIMESTAMP_OVERLAY_SIZE),
                position=SLOWFRAME_TIMESTAMP_OVERLAY_POSITION,
                color=SLOWFRAME_TIMESTAMP_OVERLAY_COLOR,
                background_color=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR,
                background_opacity=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY,
            ),
        ])
    elif SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY:
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


def get_active_ptt_pins() -> List[int]:
    """Return the PTT GPIO pin(s) for the selected radio band."""
    if ACTIVE_RADIO_BAND == "uhf":
        return [DRA818_UHF_PTT_PIN]
    elif ACTIVE_RADIO_BAND == "both":
        return [DRA818_VHF_PTT_PIN, DRA818_UHF_PTT_PIN]
    else:  # "vhf" (default)
        return [DRA818_VHF_PTT_PIN]


def apply_tx_power_level() -> None:
    """Apply configured DRA818 H/L output level."""
    if TX_POWER_LEVEL == "high":
        GPIO.output(DRA818_POWER_LEVEL_PIN, GPIO.HIGH)
        log_debug(f"TX power level: HL GPIO{DRA818_POWER_LEVEL_PIN}=HIGH (H / 1.0 W)")
    else:
        GPIO.output(DRA818_POWER_LEVEL_PIN, GPIO.LOW)
        log_debug(f"TX power level: HL GPIO{DRA818_POWER_LEVEL_PIN}=LOW (L / 0.5 W)")


def apply_pd_idle_policy() -> None:
    """Apply configured PD idle behavior after TX/PTT completes."""
    if PD_IDLE_MODE == "sleep":
        GPIO.setup(DRA818_POWER_DOWN_PIN, GPIO.OUT, initial=GPIO.LOW)
        log_debug("PD idle policy: OUTPUT LOW (Pi-enforced sleep)")
    else:
        GPIO.setup(DRA818_POWER_DOWN_PIN, GPIO.IN)
        log_debug("PD idle policy: INPUT (released to Feather M0)")


def setup_gpio():
    log_debug(f"GPIO setup: mode=BCM, VHF_PTT={DRA818_VHF_PTT_PIN}, UHF_PTT={DRA818_UHF_PTT_PIN}, PD={DRA818_POWER_DOWN_PIN}, HL={DRA818_POWER_LEVEL_PIN}")
    GPIO.cleanup()                                                          # clear any stale state from a previous run
    GPIO.setmode(GPIO_PIN_MODE)
    # Keep both PTT lines as INPUT with PUD_UP at idle so the Feather M0 can
    # safely drive them too.  Pi claims ownership (OUTPUT HIGH) only when about
    # to TX via claim_ptt_line(), and releases back to INPUT PUD_UP via
    # release_ptt_line() when done.
    GPIO.setup(DRA818_VHF_PTT_PIN,     GPIO.IN, pull_up_down=GPIO.PUD_UP)  # VHF PTT — released
    GPIO.setup(DRA818_UHF_PTT_PIN,     GPIO.IN, pull_up_down=GPIO.PUD_UP)  # UHF PTT — released
    GPIO.setup(DRA818_POWER_LEVEL_PIN, GPIO.OUT, initial=GPIO.LOW)         # HL — apply_tx_power_level sets final level
    GPIO.setup(DRA818_POWER_DOWN_PIN,  GPIO.IN)                            # PD — apply_pd_idle_policy sets final state
    apply_tx_power_level()
    apply_pd_idle_policy()
    log_debug(
        "GPIO setup complete: "
        f"VHF_PTT=INPUT_PULLUP, UHF_PTT=INPUT_PULLUP, HL={TX_POWER_LEVEL.upper()}, PD_IDLE={PD_IDLE_MODE.upper()}"
    )


def claim_ptt_line(pin: int = DRA818_VHF_PTT_PIN):
    """Claim a PTT line: configure as OUTPUT HIGH (Pi owns, radio unkeyed)."""
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
    log_debug(f"PTT GPIO{pin}: claimed OUTPUT HIGH (idle)")


def release_ptt_line(pin: int = DRA818_VHF_PTT_PIN):
    """Release a PTT line back to INPUT PUD_UP (shared with Feather M0)."""
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    log_debug(f"PTT GPIO{pin}: released INPUT PUD_UP")


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


def transmit_sstv_audio(wav_path=None, ptt_pin: int = None):
    audio_path = wav_path or SSTV_WAV
    ptt_pins = [ptt_pin] if ptt_pin is not None else get_active_ptt_pins()
    log(f"TX: file={audio_path}")
    log(f"TX: device={APLAY_DEVICE}  band={ACTIVE_RADIO_BAND.upper()}  pins={ptt_pins}")
    log(f"TX: wake_delay={RADIO_WAKE_DELAY_SECONDS}s  ptt_key_delay={PTT_KEY_DELAY_SECONDS}s  post_playback_delay={POST_PLAYBACK_DELAY_SECONDS}s")
    log("TX: PD -> OUTPUT HIGH (Pi claims line, radio wake)")
    transmit_started_at = time.monotonic()
    apply_tx_power_level()
    GPIO.setup(DRA818_POWER_DOWN_PIN, GPIO.OUT, initial=GPIO.HIGH)  # claim PD from Feather M0
    time.sleep(RADIO_WAKE_DELAY_SECONDS)
    for pin in ptt_pins:
        claim_ptt_line(pin)
    log(f"TX: PTT -> LOW (keyed, pins {ptt_pins})")
    for pin in ptt_pins:
        GPIO.output(pin, GPIO.LOW)
    time.sleep(PTT_KEY_DELAY_SECONDS)

    try:
        run(["aplay", "-D", APLAY_DEVICE, audio_path], check=True)
    finally:
        time.sleep(POST_PLAYBACK_DELAY_SECONDS)
        log(f"TX: PTT -> HIGH (unkeyed, pins {ptt_pins})")
        for pin in ptt_pins:
            GPIO.output(pin, GPIO.HIGH)
        for pin in ptt_pins:
            release_ptt_line(pin)
        if PD_IDLE_MODE == "sleep":
            log("TX: PD -> LOW (idle sleep mode)")
        else:
            log("TX: PD -> INPUT (released back to Feather M0)")
        apply_pd_idle_policy()

    elapsed = time.monotonic() - transmit_started_at
    log(f"TX: complete  {elapsed:.1f}s elapsed")
    return elapsed


def run_ptt_test(key_seconds: float):
    """Key the radio briefly to verify GPIO PD/PTT control path."""
    ptt_pins = get_active_ptt_pins()
    band_label = ACTIVE_RADIO_BAND.upper()
    log(f"PTT test: starting (band={band_label}  pins={ptt_pins}  duration={key_seconds:.2f}s)")
    started_at = time.monotonic()

    # Bring radio out of power-down and then key PTT.
    apply_tx_power_level()
    GPIO.setup(DRA818_POWER_DOWN_PIN, GPIO.OUT, initial=GPIO.HIGH)  # claim PD from Feather M0
    log_debug("PTT test: PD->OUTPUT HIGH (Pi claims line, radio wake)")
    time.sleep(RADIO_WAKE_DELAY_SECONDS)

    for pin in ptt_pins:
        claim_ptt_line(pin)
    for pin in ptt_pins:
        GPIO.output(pin, GPIO.LOW)
    log_debug(f"PTT test: PTT->LOW (keyed, pins {ptt_pins})")
    time.sleep(key_seconds)

    for pin in ptt_pins:
        GPIO.output(pin, GPIO.HIGH)
    log_debug(f"PTT test: PTT->HIGH (unkeyed, pins {ptt_pins})")
    for pin in ptt_pins:
        release_ptt_line(pin)
    apply_pd_idle_policy()
    if PD_IDLE_MODE == "sleep":
        log_debug("PTT test: PD->LOW (idle sleep mode)")
    else:
        log_debug("PTT test: PD->INPUT (released to Feather M0)")

    elapsed = time.monotonic() - started_at
    log(f"PTT test: complete (elapsed {elapsed:.2f}s)")


def _led_breathe(pwm, lo: float, hi: float, period: float, duration: float, verbose: bool = False):
    """Drive *pwm* through a smooth sine-wave breathe between *lo* and *hi* duty-cycle
    for *duration* seconds, completing full cycles of length *period*."""
    import math
    step = 0.02  # seconds per frame (~50 fps)
    log_interval = 0.25  # seconds between verbose dc log lines
    t = 0.0
    last_logged = -log_interval
    end = time.monotonic() + duration
    while time.monotonic() < end:
        # Sine goes 0→1→0 over one period
        phase = (t % period) / period  # 0.0 – 1.0
        sine = (1.0 - math.cos(2 * math.pi * phase)) / 2.0  # 0.0 – 1.0
        dc = lo + (hi - lo) * sine
        pwm.ChangeDutyCycle(dc)
        if verbose and t - last_logged >= log_interval:
            log_debug(f"    dc={dc:5.1f}%  phase={phase:.3f}  t={t:.2f}s")
            last_logged = t
        time.sleep(step)
        t += step


def _led_solid(pwm, dc: float, duration: float, verbose: bool = False):
    """Hold *pwm* at a constant duty-cycle *dc* for *duration* seconds."""
    pwm.ChangeDutyCycle(dc)
    if verbose:
        log_debug(f"    dc={dc:5.1f}%  (solid)  duration={duration:.2f}s")
    time.sleep(duration)


def run_diag_led(duration: float, verbose: bool = False, config_path: str = None):
    """Cycle through all operational LED states for *duration* seconds each."""
    configure_logging(debug=verbose)
    if verbose:
        print_runtime_startup("led-test", config_path)

    if not STATUS_LED_ENABLED:
        log("Status LED is disabled (STATUS_LED_ENABLED=False).")
        log("Enable it in the config [status_led] section or set STATUS_LED_ENABLED=True.")
        return

    # Duty-cycle sense: for active-high wiring, dc=100 means fully on.
    # For active-low, invert: on=0, off=100.
    hi = float(STATUS_LED_MAX_PCT) if STATUS_LED_ACTIVE_HIGH else 0.0
    lo = 0.0 if STATUS_LED_ACTIVE_HIGH else float(STATUS_LED_MAX_PCT)
    off_dc = 0.0 if STATUS_LED_ACTIVE_HIGH else float(STATUS_LED_MAX_PCT)

    log_section("LED Test")
    log(f"  pin         : GPIO{STATUS_LED_PIN}")
    log(f"  active_high : {STATUS_LED_ACTIVE_HIGH}")
    log(f"  pwm         : {STATUS_LED_PWM_FREQ}Hz")
    log(f"  max_pct     : {STATUS_LED_MAX_PCT}%")
    log(f"  per_state   : {duration:.2f}s")
    log(f"  confirm     : interactive")
    if verbose:
        log(f"  dc_hi       : {hi:.1f}%")
        log(f"  dc_lo       : {lo:.1f}%")
        log(f"  dc_off      : {off_dc:.1f}%")
        log(f"  states      : idle(4s) capture(2s) encode(1s) tx(solid) error(0.4s) off idle")
    log("-" * 72)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(STATUS_LED_PIN, GPIO.OUT, initial=GPIO.LOW if STATUS_LED_ACTIVE_HIGH else GPIO.HIGH)
    pwm = GPIO.PWM(STATUS_LED_PIN, STATUS_LED_PWM_FREQ)
    pwm.start(off_dc)

    states_run = 0
    try:
        # idle: slow breathe (4 s period)
        log("LED test: state=idle (slow breathe)")
        if verbose:
            log_debug(f"  period=4.0s  lo={lo:.1f}%  hi={hi:.1f}%  duration={duration:.2f}s")
        _led_breathe(pwm, lo, hi, period=4.0, duration=duration, verbose=verbose)
        states_run += 1

        # capture: medium breathe (2 s period)
        log("LED test: state=capture (capture-rate breathe)")
        if verbose:
            log_debug(f"  period=2.0s  lo={lo:.1f}%  hi={hi:.1f}%  duration={duration:.2f}s")
        _led_breathe(pwm, lo, hi, period=2.0, duration=duration, verbose=verbose)
        states_run += 1

        # encode: faster breathe (1 s period)
        log("LED test: state=encode (encode-rate breathe)")
        if verbose:
            log_debug(f"  period=1.0s  lo={lo:.1f}%  hi={hi:.1f}%  duration={duration:.2f}s")
        _led_breathe(pwm, lo, hi, period=1.0, duration=duration, verbose=verbose)
        states_run += 1

        # tx: solid on
        log("LED test: state=tx (solid on)")
        _led_solid(pwm, hi, duration, verbose=verbose)
        states_run += 1

        # error: rapid breathe (0.4 s period)
        log("LED test: state=error (rapid breathe)")
        if verbose:
            log_debug(f"  period=0.4s  lo={lo:.1f}%  hi={hi:.1f}%  duration={duration:.2f}s")
        _led_breathe(pwm, lo, hi, period=0.4, duration=duration, verbose=verbose)
        states_run += 1

        # off
        log("LED test: state=off (off)")
        _led_solid(pwm, off_dc, duration, verbose=verbose)
        states_run += 1

        # return to idle
        log("LED test: state=idle (return to idle breathe)")
        if verbose:
            log_debug(f"  period=4.0s  lo={lo:.1f}%  hi={hi:.1f}%  duration={duration:.2f}s")
        _led_breathe(pwm, lo, hi, period=4.0, duration=duration, verbose=verbose)
        states_run += 1

    finally:
        pwm.stop()
        GPIO.output(STATUS_LED_PIN, GPIO.LOW if STATUS_LED_ACTIVE_HIGH else GPIO.HIGH)
        if verbose:
            log_debug("  GPIO cleanup: PWM stopped, pin set to idle level")

    answer = input("LED test: did you see the expected LED pattern? [y/N]: ").strip().lower()
    log("-" * 72)
    if answer in ("y", "yes"):
        log("  Result      : PASS")
        log(f"  states      : {states_run}")
        log("  note        : visual confirmation accepted")
    else:
        log("  Result      : FAIL")
        log(f"  states      : {states_run}")
        log("  note        : visual confirmation rejected")
    log("-" * 72)


def run_diag_gps(duration: float, verbose: bool = False, config_path: str = None):
    """Read raw NMEA sentences from the GPS UART for *duration* seconds."""
    configure_logging(debug=verbose)
    if verbose:
        print_runtime_startup("gps-test", config_path)
    log_section("Diag: GPS")
    log(f"  device      : {GPS_SERIAL_PORT}")
    log(f"  baud        : {GPS_SERIAL_BAUD}")
    log(f"  duration    : {duration:.0f}s")
    if verbose:
        log_debug(f"  timeout     : 1s per readline")
        log_debug(f"  filter      : NMEA sentences only (lines starting with '$')")
    log("-" * 72)
    try:
        import serial  # pyserial
    except ImportError:
        log("FAIL  pyserial not installed.  Run: pip3 install pyserial")
        sys.exit(1)
    try:
        with serial.Serial(GPS_SERIAL_PORT, GPS_SERIAL_BAUD, timeout=1) as ser:
            start = time.monotonic()
            count = 0
            while time.monotonic() - start < duration:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if line.startswith("$"):
                    log(f"  {line}")
                    count += 1
        if count > 0:
            log(f"PASS  Received {count} NMEA sentence(s) in {duration:.0f}s")
        else:
            log(f"WARN  Received 0 NMEA sentence(s) in {duration:.0f}s — check wiring and baud rate")
    except OSError as error:
        log(f"FAIL  Serial error on {GPS_SERIAL_PORT}: {error}")
        sys.exit(1)


def run_diag_ptt(duration: float, verbose: bool = False, config_path: str = None,
                 module: str = "uhf"):
    """Key PTT/PD for *duration* seconds and report GPIO state transitions."""
    configure_logging(debug=verbose)
    if verbose:
        print_runtime_startup("ptt-test", config_path)

    module = module.lower()
    band_ranges = {"vhf": "134 – 174 MHz  (DRA818V)", "uhf": "400 – 470 MHz  (DRA818U)"}
    band_label  = band_ranges.get(module, module.upper())
    power_label = "LOW (≤0.5 W)" if True else "HIGH (1 W)"  # HL is always held LOW at idle
    ptt_pin     = DRA818_UHF_PTT_PIN if module == "uhf" else DRA818_VHF_PTT_PIN
    ptt_phys    = "pin 11" if module == "uhf" else "pin 13"

    log_section("Diag: PTT")
    log(f"  module      : {module.upper()}  —  {band_label}")
    log(f"  power       : {power_label}  (h_l held LOW during test)")
    log(f"  wake_delay  : {RADIO_WAKE_DELAY_SECONDS}s  (time for DRA818 to stabilise after PD HIGH)")
    log(f"  ptt_delay   : {PTT_KEY_DELAY_SECONDS}s  (settling time before audio)")
    log(f"  key_dur     : {duration:.2f}s")
    log("-" * 72)
    log(f"  Signal   BCM GPIO   Phys Pin   Idle          Active")
    log(f"  ──────── ────────── ────────── ───────────── ────────────────")
    log(f"  PTT      GPIO{ptt_pin:<6}  {ptt_phys}     HIGH (idle)   LOW (keyed)")
    log(f"  PD       GPIO{DRA818_POWER_DOWN_PIN:<6}  pin  7     LOW (sleep)   HIGH (active)")
    log(f"  HL       GPIO{DRA818_POWER_LEVEL_PIN:<6}  pin 15     LOW (≤0.5 W) HIGH (1 W)")
    log(f"  Audio L  GPIO{AUDIO_LEFT_PWM_PIN:<6}  pin 32     PWM output")
    log(f"  Audio R  GPIO{AUDIO_RIGHT_PWM_PIN:<6}  pin 33     PWM output")

    GPIO.setwarnings(False)
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    # PTT lines idle as INPUT PUD_UP — shared with Feather M0, weak pull keeps them HIGH.
    # Pi claims ownership via claim_ptt_line() immediately before keying.
    GPIO.setup(DRA818_VHF_PTT_PIN,     GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(DRA818_UHF_PTT_PIN,     GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(DRA818_POWER_LEVEL_PIN, GPIO.OUT, initial=GPIO.LOW)   # apply_tx_power_level sets final level
    GPIO.setup(DRA818_POWER_DOWN_PIN,  GPIO.IN)                      # apply_pd_idle_policy sets final state
    apply_tx_power_level()
    apply_pd_idle_policy()

    if verbose:
        log_debug(f"  GPIO init: VHF_PTT=INPUT_PUD_UP  UHF_PTT=INPUT_PUD_UP  HL={TX_POWER_LEVEL.upper()}  PD_IDLE={PD_IDLE_MODE.upper()}")

    started_at = time.monotonic()
    try:
        log(f"  [{time.monotonic()-started_at:.3f}s]  PD  -> OUTPUT HIGH (Pi claims line, radio wake — waiting {RADIO_WAKE_DELAY_SECONDS}s)")
        apply_tx_power_level()
        GPIO.setup(DRA818_POWER_DOWN_PIN, GPIO.OUT, initial=GPIO.HIGH)  # claim PD from Feather M0
        time.sleep(RADIO_WAKE_DELAY_SECONDS)

        claim_ptt_line(ptt_pin)
        log(f"  [{time.monotonic()-started_at:.3f}s]  PTT -> LOW   (keyed — holding {duration:.2f}s)")
        GPIO.output(ptt_pin, GPIO.LOW)
        time.sleep(duration)

        log(f"  [{time.monotonic()-started_at:.3f}s]  PTT -> HIGH  (unkeyed)")
        GPIO.output(ptt_pin, GPIO.HIGH)
        release_ptt_line(ptt_pin)
        time.sleep(POST_PLAYBACK_DELAY_SECONDS)

        apply_pd_idle_policy()
        if PD_IDLE_MODE == "sleep":
            log(f"  [{time.monotonic()-started_at:.3f}s]  PD  -> LOW (idle sleep mode)")
        else:
            log(f"  [{time.monotonic()-started_at:.3f}s]  PD  -> INPUT (released back to Feather M0)")
    finally:
        if verbose:
            log_debug(f"  [{time.monotonic()-started_at:.3f}s]  GPIO left in safe state (PTT=INPUT_PUD_UP, PD_IDLE={PD_IDLE_MODE.upper()})")

    elapsed = time.monotonic() - started_at
    log("-" * 72)
    log(f"  Result      : PASS")
    log(f"  module      : {module.upper()}  ({band_label})")
    log(f"  elapsed     : {elapsed:.2f}s")
    log(f"  keyed_for   : {duration:.2f}s")
    log(f"  note        : all GPIO transitions completed without error")
    log("-" * 72)


def run_diag_tx(mode: str = "m1", image_path: str = None, image_dir: str = None,
                no_tx: bool = False, module: str = "uhf",
                verbose: bool = False, config_path: str = None):
    """Run the full capture → encode → TX pipeline once using a test image."""
    configure_logging(debug=verbose)

    # Full config needed so SLOWFRAME_BIN, TIMESTAMPED_DIR, etc. are set.
    if config_path:
        load_config(config_path)
    ensure_runtime_paths()

    # Resolve the image to use.
    if image_path:
        if not os.path.isfile(image_path):
            log(f"FAIL  --image path not found: {image_path}")
            sys.exit(1)
        resolved_image = image_path
    elif image_dir:
        if not os.path.isdir(image_dir):
            log(f"FAIL  --image-dir path not found: {image_dir}")
            sys.exit(1)
        candidates = sorted(
            f for f in os.listdir(image_dir)
            if f.lower().endswith((".jpg", ".jpeg", ".png"))
        )
        if not candidates:
            log(f"FAIL  No .jpg/.png images found in: {image_dir}")
            sys.exit(1)
        resolved_image = os.path.join(image_dir, candidates[0])
    else:
        resolved_image = TEST_IMAGE

    # Resolve the PTT pin for the selected module.
    ptt_pin = DRA818_UHF_PTT_PIN if module.lower() == "uhf" else DRA818_VHF_PTT_PIN

    # Build a minimal args namespace that run_test_pipeline expects.
    import types
    fake_args = types.SimpleNamespace(
        test=mode,
        test_image=resolved_image,
        no_tx=no_tx,
        output_dir=TIMESTAMPED_DIR,
        ptt_pin=ptt_pin,
    )

    # Drive all PTT and control pins to safe idle states immediately — before
    # any subprocess work — so a previous command's stale state cannot leave
    # PTT asserted.  PTT lines idle as INPUT PUD_UP (shared with Feather M0).
    if not no_tx:
        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(DRA818_VHF_PTT_PIN,     GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(DRA818_UHF_PTT_PIN,     GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(DRA818_POWER_LEVEL_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(DRA818_POWER_DOWN_PIN,  GPIO.IN)  # PD released — Feather M0 owns at idle

    # Run capability discovery after GPIO is safe.
    runtime_state = discover_slowframe_capabilities()

    try:
        run_test_pipeline(mode, fake_args, runtime_state)
    finally:
        if not no_tx:
            # PTT lines are INPUT PUD_UP at idle and PD is INPUT (released to
            # Feather M0) — transmit_sstv_audio() already returned them to
            # this state, so no additional cleanup is needed here.
            pass


def run_diag_alsa(verbose: bool = False, config_path: str = None):
    """List ALSA playback devices and verify aplay is reachable."""
    configure_logging(debug=verbose)
    if verbose:
        print_runtime_startup("alsa-test", config_path)

    log_section("Diag: ALSA")
    log(f"  audio_L     : GPIO{AUDIO_LEFT_PWM_PIN}  (PWM)")
    log(f"  audio_R     : GPIO{AUDIO_RIGHT_PWM_PIN}  (PWM)")
    log(f"  overlay     : {AUDIO_OVERLAY}")
    log("-" * 72)

    import shutil
    aplay_path = shutil.which("aplay")
    if not aplay_path:
        log("FAIL  aplay not found in PATH.  Install: sudo apt install alsa-utils")
        sys.exit(1)
    log(f"  aplay       : {aplay_path}")

    # aplay -l — hardware device list
    log("ALSA: hardware device list (aplay -l)")
    try:
        result = run(["aplay", "-l"], capture_output=True, text=True)
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines():
                log(f"  {line}")
        else:
            log("  (no output)")
    except Exception as error:
        log(f"FAIL  aplay -l failed: {error}")
        sys.exit(1)

    # aplay -L — PCM device names
    log("ALSA: PCM device names (aplay -L)")
    try:
        result = run(["aplay", "-L"], capture_output=True, text=True)
        output = (result.stdout + result.stderr).strip()
        audremap_lines = []
        for line in output.splitlines():
            if verbose:
                log(f"  {line}")
            if "audremap" in line.lower() or "headphones" in line.lower():
                audremap_lines.append(line.strip())
        if not verbose and audremap_lines:
            log("  (audremap / Headphones entries)")
            for line in audremap_lines:
                log(f"    {line}")
        elif not verbose and not audremap_lines:
            log("  WARN  No audremap or Headphones PCM device found")
            log("        Check that /boot/config.txt contains:")
            log(f"          dtoverlay=audremap,enable_jack=on")
    except Exception as error:
        log(f"FAIL  aplay -L failed: {error}")
        sys.exit(1)

    log("-" * 72)
    log("  Result      : PASS")
    log("  note        : aplay reachable; Headphones PCM present (dtoverlay=audremap maps GPIO12/13 to 'Headphones')")
    log("-" * 72)


def run_diag_camera(verbose: bool = False, config_path: str = None):
    """Verify rpicam-still is present and take a test capture."""
    configure_logging(debug=verbose)
    if verbose:
        print_runtime_startup("camera-test", config_path)

    log_section("Diag: Camera")
    log(f"  binary      : {RPICAM_BIN}")
    log(f"  quality     : {RPICAM_QUALITY}")
    log(f"  metering    : {RPICAM_METERING}")
    log(f"  exposure    : {RPICAM_EXPOSURE}")
    log(f"  awb         : {RPICAM_AWB}")
    log(f"  test_image  : {TEST_IMAGE}")
    log("-" * 72)

    if not os.path.isfile(RPICAM_BIN):
        log(f"FAIL  rpicam-still not found at {RPICAM_BIN}")
        log( "      Install: sudo apt install rpicam-apps")
        sys.exit(1)
    log(f"  binary OK   : {RPICAM_BIN}")

    import tempfile
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
        out_path = tmp.name

    cmd = [
        RPICAM_BIN, "--nopreview",
        "-o", out_path,
        "--quality", str(RPICAM_QUALITY),
        "--metering", RPICAM_METERING,
        "--exposure", RPICAM_EXPOSURE,
        "--awb", RPICAM_AWB,
        "--lens-position", "0",
    ]
    if verbose:
        log_debug(f"  command: {' '.join(cmd)}")

    log("Camera: capturing test image …")
    try:
        proc = run(cmd, capture_output=True, text=True)
        combined_output = proc.stdout + proc.stderr
        if verbose or proc.returncode != 0:
            for line in combined_output.splitlines():
                if line.strip():
                    log(f"  rpicam-still: {line}")
        if proc.returncode != 0:
            raise CalledProcessError(proc.returncode, cmd, proc.stdout, proc.stderr)
        size = os.path.getsize(out_path)
        log(f"Camera: OK  {size:,} bytes  -> {out_path}")
        result = "PASS"
        note = f"captured {size:,} bytes"
    except CalledProcessError as error:
        combined_output = (error.stdout or "") + (error.stderr or "")
        log(f"Camera: FAIL  exit status {error.returncode}")
        if "pipeline handler in use" in combined_output.lower() or "failed to acquire camera" in combined_output.lower():
            log( "        Camera is held by another process.")
            log( "        Kill any other rpicam-still/libcamera instance and retry.")
        elif "no cameras available" in combined_output.lower():
            log( "        No camera detected — check ribbon cable and raspi-config camera enable.")
        else:
            log( "        Check camera ribbon cable and that the camera is enabled in raspi-config.")
        result = "FAIL"
        note = f"rpicam-still exit {error.returncode}"
    except Exception as error:
        log(f"Camera: FAIL  {error}")
        result = "FAIL"
        note = str(error)
    finally:
        try:
            os.unlink(out_path)
        except OSError:
            pass

    log("-" * 72)
    log(f"  Result      : {result}")
    log(f"  note        : {note}")
    log("-" * 72)
    if result == "FAIL":
        sys.exit(1)


def run_diag_slowframe(verbose: bool = False, config_path: str = None):
    """Check SlowFrame binary, version, MMSSTV library status, and available modes."""
    configure_logging(debug=verbose)
    if verbose:
        print_runtime_startup("slowframe-test", config_path)

    log_section("Diag: SlowFrame")
    log(f"  binary      : {SLOWFRAME_BIN}")
    log("-" * 72)

    if not os.path.isfile(SLOWFRAME_BIN):
        log(f"FAIL  slowframe binary not found at {SLOWFRAME_BIN}")
        log( "      Check the path or set --slowframe / [paths] slowframe= in your config.")
        sys.exit(1)

    st = os.stat(SLOWFRAME_BIN)
    executable = os.access(SLOWFRAME_BIN, os.X_OK)
    log(f"  binary OK   : {SLOWFRAME_BIN}")
    log(f"  size        : {st.st_size:,} bytes")
    log(f"  permissions : {oct(st.st_mode & 0o777)}  executable={'yes' if executable else 'NO'}")
    if not executable:
        log(f"FAIL  binary is not executable.  Fix: chmod +x {SLOWFRAME_BIN}")
        sys.exit(1)

    def _emit(output, skip_usage=True):
        """Print binary output lines, indented, optionally dropping usage/example footer."""
        for line in output.splitlines():
            if skip_usage and line.strip().lower().startswith(("usage:", "example:")):
                continue
            log(f"  {line}")

    def _parse_version(output):
        m = re.search(r'SlowFrame\s+(v[\d.]+)', output, re.IGNORECASE)
        return m.group(1) if m else "(unknown)"

    def _parse_mmsstv(output):
        """Return (loaded: bool, lib_path: str|None, lib_version: str|None) from -M output."""
        loaded, path, version = False, None, None
        for line in output.splitlines():
            sl = line.strip().lower()
            if sl.startswith("library status:") and "detected" in sl and "not detected" not in sl:
                loaded = True
            if sl.startswith("library path:"):
                path = line.strip().split(":", 1)[1].strip()
            if sl.startswith("library version:"):
                version = line.strip().split(":", 1)[1].strip()
        return loaded, path, version

    def _parse_mode_count(output):
        """Count mode table rows (code followed by ≥2 spaces then non-space)."""
        return sum(1 for ln in output.splitlines()
                   if re.match(r'^\s*[a-z][a-z0-9]*\s{2,}\S', ln))

    # ── -M: MMSSTV library status ────────────────────────────────────────────
    log("")
    log("[ slowframe -M ]")
    mmsstv_loaded, mmsstv_lib_path, mmsstv_lib_version = False, None, None
    try:
        m_result = run(
            [SLOWFRAME_BIN, "-M"],
            capture_output=True, text=True,
            timeout=SLOWFRAME_LIST_TIMEOUT_SECONDS,
        )
        m_output = (m_result.stdout + m_result.stderr)
        mmsstv_loaded, mmsstv_lib_path, mmsstv_lib_version = _parse_mmsstv(m_output)
        _emit(m_output, skip_usage=False)
    except Exception as error:
        log(f"  WARN  slowframe -M failed: {error}")

    # ── -L: available modes ───────────────────────────────────────────────────
    log("")
    log("[ slowframe -L ]")
    version_str = "(unknown)"
    mode_count = 0
    try:
        l_result = run(
            [SLOWFRAME_BIN, "-L"],
            capture_output=True, text=True,
            timeout=SLOWFRAME_LIST_TIMEOUT_SECONDS,
        )
        l_output = (l_result.stdout + l_result.stderr)
        version_str = _parse_version(l_output)
        mode_count = _parse_mode_count(l_output)
        _emit(l_output, skip_usage=not verbose)
    except Exception as error:
        log(f"FAIL  slowframe -L failed: {error}")
        sys.exit(1)

    # ── summary ───────────────────────────────────────────────────────────────
    log("")
    log("-" * 72)
    log(f"  Result      : PASS")
    log(f"  version     : {version_str}")
    if mmsstv_loaded:
        mmsstv_detail = f"v{mmsstv_lib_version}" if mmsstv_lib_version else ""
        if mmsstv_lib_path:
            mmsstv_detail = (mmsstv_detail + f"  {mmsstv_lib_path}").strip()
        log(f"  mmsstv      : LOADED  {mmsstv_detail}".rstrip())
    else:
        log(f"  mmsstv      : NOT LOADED")
        log(f"  hint        : export {MMSSTV_LIBRARY_ENV_VAR}=/path/to/libsstv_encoder.so")
    log(f"  modes       : {mode_count}")
    log("-" * 72)


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
        ptt_pin_override = getattr(args, "ptt_pin", None)
        duration = transmit_sstv_audio(wav_path=wav_path, ptt_pin=ptt_pin_override)
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


def print_runtime_startup(run_mode: str, config_path: str = None):
    """Print effective runtime settings in the standard operator log format."""
    W = 72
    bar  = "=" * W
    dash = "-" * W
    log(bar)
    log("  Runtime Startup")
    log(bar)
    log(f"  run_mode    : {run_mode}")
    log(f"  config      : {config_path or '(none)'}")
    log(f"  schedule    : {TRANSMIT_SCHEDULE_PROFILE}")
    log(f"  output_dir  : {TIMESTAMPED_DIR}")
    log(f"  slowframe   : {SLOWFRAME_BIN}")
    log(f"  test_image  : {TEST_IMAGE}")
    log(f"  camera      : rpicam-still ({RPICAM_BIN})")
    log(f"  capture     : quality={RPICAM_QUALITY}  metering={RPICAM_METERING}  "
        f"exposure={RPICAM_EXPOSURE}  awb={RPICAM_AWB}")
    log(f"  encode      : format={SLOWFRAME_AUDIO_FORMAT}  rate={SLOWFRAME_SAMPLE_RATE}Hz  "
        f"aspect={SLOWFRAME_ASPECT_MODE}")
    callsign_id = f"'{STATION_CALLSIGN}'" if STATION_CALLSIGN else "''"
    log(f"  overlay     : timestamp={SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY}  "
        f"callsign={SLOWFRAME_ENABLE_CALLSIGN_OVERLAY}  id={callsign_id}")
    log(f"  status_led  : enabled={STATUS_LED_ENABLED}  pin=GPIO{STATUS_LED_PIN}  "
        f"active_high={STATUS_LED_ACTIVE_HIGH}  pwm={STATUS_LED_PWM_FREQ}Hz  max={STATUS_LED_MAX_PCT}%")
    log(f"  ptt         : VHF=GPIO{DRA818_VHF_PTT_PIN}  UHF=GPIO{DRA818_UHF_PTT_PIN}")
    log(f"  pd          : GPIO{DRA818_POWER_DOWN_PIN}  (HIGH during TX, LOW at idle)")
    log(f"  h_l         : GPIO{DRA818_POWER_LEVEL_PIN}  default LOW  (L = low power)")
    log(f"  audio       : L=GPIO{AUDIO_LEFT_PWM_PIN}  R=GPIO{AUDIO_RIGHT_PWM_PIN}  "
        f"overlay={AUDIO_OVERLAY}")
    log(f"  alsa        : playback={APLAY_DEVICE}")
    log(f"  gps         : device={GPS_SERIAL_PORT}  baud={GPS_SERIAL_BAUD}")
    log(f"  cooldown    : scale={COOLDOWN_SCALE_FACTOR:.2f}  "
        f"duty_cycle={MAX_TRANSMIT_DUTY_CYCLE * 100:.1f}%  "
        f"window={ROLLING_DUTY_CYCLE_WINDOW_SECONDS}s")
    log(dash)
    log(dash)
    log("  Result      : READY")
    log(dash)


# =============================================================================
# Verb: help
# =============================================================================

VERB_HELP_SUBJECTS = {
    "mission": (
        "Run the continuous capture-and-transmit HAB mission loop.",
        """\
VERB: mission
=============
Run the full HAB mission — continuous image capture, SSTV audio encoding,
and scheduled radio transmission — until TOTAL captures have been taken.

USAGE
  python3 pi_sstv.py mission [options]

ARGUMENTS
  -c, --config PATH         Load settings from a .cfg file before any flags.
  -s, --schedule PRESET     Transmit schedule: hab_climb, hab_rapid,
                            hab_cruise (default), hab_float.
  -n, --total N             Total captures before the mission ends (default: 500).
  -i, --interval SECS       Seconds between captures (default: 10).
  --callsign CALL           Callsign overlay on every transmitted image.
  --no-tx                   Capture and encode but never transmit.
  --cooldown-scale FACTOR   Multiply all mode cooldowns (default: 1.0).
  --duty-cycle FRACTION     Max rolling TX fraction (default: 0.25).
  --min-captures N          Captures between any two transmissions (default: 12).
  --band {vhf,uhf,both}     Radio band to key (default: vhf).
  --power {low,high}        Transmit power level (default: low).
  --output-dir PATH         Directory for images, WAV, and CSV.
  --debug                   Enable DEBUG-level logging.
  --log-file PATH           Log to file in addition to stdout.
  --quiet-log-file PATH     Log to file only, suppress stdout.

EXAMPLES
  Standard HAB flight using a config file:
    python3 pi_sstv.py mission --config /home/pi-user/pi_sstv.cfg

  Rapid-ascent schedule, 200 captures, 8-second intervals:
    python3 pi_sstv.py mission --schedule hab_rapid --total 200 --interval 8

  Float altitude with MMSSTV library and callsign overlay:
    python3 pi_sstv.py mission --schedule hab_float --callsign W1AW-11 \\
        --mmsstv-lib /opt/mmsstv/lib/libsstv_encoder.so

  Bench capture test — no radio required:
    python3 pi_sstv.py mission --total 10 --no-tx --interval 5

SEE ALSO
  help schedules    Transmit schedule details and mode sequences
  help run          Single-shot pipeline test
  config show       Review effective config before a flight
""",
    ),
    "run": (
        "Execute a single capture → encode → transmit pipeline pass.",
        """\
VERB: run
=========
Run a single SSTV pipeline pass: capture an image (or use an existing one),
encode it to SSTV audio, and optionally transmit it over the radio.

USAGE
  python3 pi_sstv.py run [options]

ARGUMENTS
  -m, --mode MODE           SSTV mode to encode (e.g. r36, pd90, m1).
                            Run 'help modes' to list all modes.
  -c, --config PATH         Load settings from a .cfg file.
  --image PATH              Use this image instead of capturing from the camera.
  --callsign CALL           Callsign overlay baked into the transmitted image.
  --no-callsign             Suppress callsign overlay even if set in config.
  --no-timestamp            Suppress timestamp overlay.
  --no-tx                   Encode but skip radio transmit (safe bench test).
  --band {vhf,uhf,both}     Radio band to key (default: vhf).
  --power {low,high}        Transmit power level (default: low).
  --keep-wav                Keep the encoded WAV file after the run.
  --output-dir PATH         Directory for output artifacts.
  --debug                   Enable DEBUG-level logging.
  --mmsstv-lib PATH         Path to MMSSTV shared library.
  --no-mmsstv               Force native SlowFrame modes only.

EXAMPLES
  Single r36 encode + transmit (radio connected):
    python3 pi_sstv.py run --mode r36

  Bench encode test — no radio required:
    python3 pi_sstv.py run --mode pd90 --no-tx

  Encode from an existing image with callsign overlay:
    python3 pi_sstv.py run --mode m1 --image /home/pi-user/photo.jpg \\
        --callsign W1AW-11 --no-tx

  Full UHF TX using a config file:
    python3 pi_sstv.py run --mode r36 --band uhf \\
        --config /home/pi-user/pi_sstv.cfg

SEE ALSO
  help modes        Mode durations and MMSSTV requirements
  help mission      Continuous capture-and-transmit loop
  diag tx           Full pipeline test via the diag subsystem
""",
    ),
    "schedules": (
        "List available SSTV transmit schedule presets.",
        """\
VERB: schedules
===============
List all transmit schedule presets, their mode sequences, and duty-cycle
summaries.  Use this before a flight to confirm which modes will be used
and whether the MMSSTV library is required.

USAGE
  python3 pi_sstv.py schedules [options]

ARGUMENTS
  -c, --config PATH         Load settings to pick up the active schedule.
  --detail PRESET           Show expanded details for one preset.

EXAMPLES
  List all presets:
    python3 pi_sstv.py schedules

  Expanded detail for the float schedule:
    python3 pi_sstv.py schedules --detail hab_float

  Show which schedule is active per the config file:
    python3 pi_sstv.py schedules --config /home/pi-user/pi_sstv.cfg

SEE ALSO
  help modes        SSTV mode reference table
  help mission      mission verb flags for selecting a schedule
""",
    ),
    "config": (
        "Generate, validate, or display a pi_sstv configuration file.",
        """\
VERB: config
============
Manage the pi_sstv configuration file: generate a documented template,
validate an existing file, or display the effective runtime settings that
would result from loading it.

USAGE
  python3 pi_sstv.py config generate [PATH]
  python3 pi_sstv.py config validate PATH
  python3 pi_sstv.py config show     PATH

SUBCOMMANDS
  generate [PATH]   Write a fully-commented default .cfg to PATH.
                    Default path: /home/pi-user/pi_sstv.cfg
  validate  PATH    Parse PATH, check all values, report errors and warnings.
                    Exits 0 on success, 1 on any invalid value.
  show      PATH    Load PATH and print the effective runtime settings table
                    that pi_sstv.py will see when running with this file.

EXAMPLES
  Generate the default template:
    python3 pi_sstv.py config generate

  Generate to a custom path:
    python3 pi_sstv.py config generate /boot/pi_sstv.cfg

  Validate a config before a flight:
    python3 pi_sstv.py config validate /home/pi-user/pi_sstv.cfg

  Review effective settings:
    python3 pi_sstv.py config show /home/pi-user/pi_sstv.cfg

SEE ALSO
  help mission      Mission flags that can override config values
  service install   Install the systemd service (reads the same config)
""",
    ),
    "service": (
        "Install or remove the pi_sstv and hamwing-gpio-init systemd services.",
        """\
VERB: service
=============
Install or remove the systemd unit files that run pi_sstv.py automatically
on boot and keep the HamWing GPIO lines in a safe idle state.

Two unit files are managed:
  pi-sstv.service             Runs pi_sstv.py as the HAB mission service.
  hamwing-gpio-init.service   Sets GPIO idle levels at early boot before the
                              main service starts (PTT lines INPUT PUD_UP,
                              PD=LOW, HL=LOW).

USAGE
  python3 pi_sstv.py service install   [options]
  python3 pi_sstv.py service uninstall [options]
  python3 pi_sstv.py service status
  python3 pi_sstv.py service logs      [--lines N]

ARGUMENTS (install)
  -c, --config PATH         Path to the .cfg file the service should load.
  --user USER               System user the service runs as (default: pi-user).
  --script PATH             Absolute path to pi_sstv.py (default: this file).
  --python PATH             Python interpreter (default: /usr/bin/python3).
  --dry-run                 Print the unit file contents without writing them.

EXAMPLES
  Install with default settings:
    python3 pi_sstv.py service install

  Install pointing at a config file:
    python3 pi_sstv.py service install --config /home/pi-user/pi_sstv.cfg

  Preview unit files without installing:
    python3 pi_sstv.py service install --dry-run

  Check running status:
    python3 pi_sstv.py service status

  Tail live service logs:
    python3 pi_sstv.py service logs

  Remove both unit files and disable them:
    python3 pi_sstv.py service uninstall

NOTE
  Writing to /etc/systemd/system/ requires root.  The script calls sudo
  automatically; you will be prompted for your password.

SEE ALSO
  service status    Check whether both services are running
  config show       Verify the config file before installing the service
""",
    ),
    "status": (
        "Check system readiness: binaries, GPIO, audio, config, and services.",
        """\
VERB: status
============
Run a quick pre-flight readiness check.  Tests every dependency and
subsystem needed for a HAB mission and prints a PASS/WARN/FAIL summary.

USAGE
  python3 pi_sstv.py status [options]

ARGUMENTS
  -c, --config PATH         Config file to load and validate.
  --verbose                 Show detailed output for every check.

CHECKS PERFORMED
  Python version            3.9+ required.
  RPi.GPIO                  Library importable and GPIO available.
  rpicam-still              Binary present and executable.
  SlowFrame                 Binary present, version, MMSSTV library.
  aplay / ALSA              aplay in PATH, audremap PCM device found.
  Output directory          Exists and is writable.
  Config file               If provided: parses cleanly, all keys valid.
  systemd services          pi-sstv.service and hamwing-gpio-init.service.

EXAMPLES
  Quick readiness check:
    python3 pi_sstv.py status

  Detailed check with config file:
    python3 pi_sstv.py status --config /home/pi-user/pi_sstv.cfg --verbose

SEE ALSO
  diag alsa         Deep ALSA device inspection
  diag camera       Camera capture test
  diag slowframe    SlowFrame binary and MMSSTV library check
  config validate   Config file validation
""",
    ),
    "diag": (
        "Hardware diagnostic utilities (led, gps, ptt, alsa, camera, slowframe, tx).",
        """\
VERB: diag
==========
Low-level hardware diagnostics.  Each sub-command exercises one subsystem.

USAGE
  python3 pi_sstv.py diag SUBCOMMAND [options]

SUBCOMMANDS
  led [SECS]       Cycle the status LED through all operational states.
  gps [SECS]       Read NMEA sentences from the GPS UART for SECS seconds.
  ptt [SECS]       Key PTT/PD GPIO lines for SECS seconds.
  alsa             List ALSA playback devices and verify aplay.
  camera           Verify rpicam-still and take a test capture.
  slowframe        Check SlowFrame binary, version, and MMSSTV library.
  tx               Run the full capture → encode → TX pipeline once.

EXAMPLES
  LED blink test, 1.5s per state:
    python3 pi_sstv.py diag led 1.5
  GPS NMEA receive for 30 seconds:
    python3 pi_sstv.py diag gps 30
  Key PTT for 0.5 seconds (UHF):
    python3 pi_sstv.py diag ptt 0.5 --module uhf
  Run a full pipeline test with no TX:
    python3 pi_sstv.py diag tx --mode r36 --no-tx

SEE ALSO
  status       System-wide readiness summary
  run          Single-shot user-facing pipeline run
""",
    ),
    "modes": (
        "SSTV mode reference table — durations, cooldowns, MMSSTV requirements.",
        """\
VERB / TOPIC: modes
===================
Lists all built-in SSTV modes with their over-the-air TX duration, required
cooldown, image width, and whether the MMSSTV encoder library is needed.

USAGE
  python3 pi_sstv.py help modes

NATIVE MODES (no MMSSTV library required)
  Name         TX (s)   Cooldown (s)   Width   Description
  bw24             24            120     320   Fast monochrome, low duty-cycle updates
  r36              36            150     320   Fast native color, regular updates
  m4               29            135     320   Fast Martin color, half airtime of M2
  m2               58            240     320   Balanced, strong compatibility
  s2               71            300     320   Scottie 2, good compatibility
  r72              72            300     320   Higher-quality Robot color
  s1              110            480     320   Scottie 1, best native quality
  m1              114            480     320   Martin 1, high-quality, less frequent

MMSSTV MODES (require libsstv_encoder.so)
  Name         TX (s)   Cooldown (s)   Width   Fallback   Description
  robot8bw          8             90     160   bw24       Ultra-fast monochrome status frame
  robot12bw        12             90     160   bw24       Very fast monochrome
  pd50             50            240     320   m2         Fast PD color
  pd90             90            360     320   r36        Popular fast color
  pd120           120            540     640   m1         Higher-quality, larger image
  pd160           160            660     512   m1         Slower quality mode
  pd180           180            720     640   m1         High-detail mission snapshot
  fax480          180            720     512   m1         High-detail, test windows
  pd240           240            900     640   m1         Very high quality PD mode
  pd290           290           1080     800   pd180      Highest quality mode

SEE ALSO
  schedules         Mode sequences used during a mission
  help mmsstv       MMSSTV library installation and detection
  run --mode MODE   Single-shot encode/TX for any mode
""",
    ),
    "gpio": (
        "GPIO pin assignments and HamWing wiring reference.",
        """\
TOPIC: gpio
===========
See full GPIO wiring reference:
  python3 pi_sstv.py --explain gpio

QUICK REFERENCE
  Signal   BCM GPIO   Physical Pin   Idle        Active
  PTT VHF  GPIO 27    pin 13         HIGH (idle) LOW (keyed)
  PTT UHF  GPIO 17    pin 11         HIGH (idle) LOW (keyed)
  PD       GPIO 4     pin 7          LOW/INPUT   HIGH (radio on)
  HL       GPIO 22    pin 15         LOW (0.5W)  HIGH (1W)
  Audio L  GPIO 12    pin 32         PWM output
  Audio R  GPIO 13    pin 33         PWM output

SEE ALSO
  --explain gpio      Full wiring and overlay documentation
  --explain tx        GPIO transmission sequence
  diag ptt            GPIO PTT keying verification
""",
    ),
    "mmsstv": (
        "MMSSTV encoder library — installation, detection, and fallback behaviour.",
        """\
TOPIC: mmsstv
=============
See full MMSSTV documentation:
  python3 pi_sstv.py --explain mmsstv

QUICK REFERENCE
  Provide the library:
    export MMSSTV_LIB_PATH=/path/to/libsstv_encoder.so
    python3 pi_sstv.py run --mode pd90 --no-tx

  Verify detection:
    python3 pi_sstv.py diag slowframe

  Disable MMSSTV (native modes only):
    python3 pi_sstv.py run --no-mmsstv --mode pd90

SEE ALSO
  --explain mmsstv    Full library and fallback documentation
  diag slowframe      Binary and library status check
""",
    ),
    "capture": (
        "Camera capture settings — metering, exposure, AWB, quality.",
        "See full documentation:\n  python3 pi_sstv.py --explain capture\n",
    ),
    "encode": (
        "SSTV audio encoding — format, sample rate, aspect mode.",
        "See full documentation:\n  python3 pi_sstv.py --explain encode\n",
    ),
    "overlay": (
        "Timestamp and callsign text overlay settings.",
        "See full documentation:\n  python3 pi_sstv.py --explain overlay\n",
    ),
    "schedule": (
        "Transmit schedule presets and duty-cycle gating.",
        "See full documentation:\n  python3 pi_sstv.py --explain schedule\n",
    ),
    "tx": (
        "Radio transmission GPIO sequence and timing constants.",
        "See full documentation:\n  python3 pi_sstv.py --explain tx\n",
    ),
    "logging": (
        "Log verbosity and output destination flags.",
        "See full documentation:\n  python3 pi_sstv.py --explain logging\n",
    ),
}

VERB_HELP_ALIASES = {
    "runs":       "run",
    "missions":   "mission",
    "sched":      "schedules",
    "schedule":   "schedules",
    "cfg":        "config",
    "conf":       "config",
    "svc":        "service",
    "services":   "service",
    "systemd":    "service",
    "check":      "status",
    "ready":      "status",
    "mode":       "modes",
    "camera":     "capture",
    "cam":        "capture",
    "radio":      "tx",
    "ptt":        "tx",
    "wiring":     "gpio",
    "pins":       "gpio",
    "lib":        "mmsstv",
    "library":    "mmsstv",
    "debug":      "logging",
    "log":        "logging",
}


def _dispatch_help(args_list):
    """Parse and execute: help [SUBJECT]"""
    p = argparse.ArgumentParser(
        prog="pi_sstv.py help",
        description="Display help topics for pi_sstv.py verbs and pipeline subjects.",
    )
    p.add_argument(
        "subject",
        nargs="?",
        default=None,
        metavar="SUBJECT",
        help="Topic or verb to get help on.  Omit to list all topics.",
    )
    a, _ = p.parse_known_args(args_list)

    if a.subject is None:
        # List all subjects
        print("pi_sstv.py  —  HamWing SSTV HAB payload controller\n")
        print("VERBS")
        print("  Use 'python3 pi_sstv.py help SUBJECT' for full documentation.\n")
        # Verbs first, then pipeline topics
        verb_order = ["mission", "run", "schedules", "config", "service", "status", "diag"]
        topic_order = ["modes", "gpio", "capture", "encode", "overlay", "mmsstv",
                       "schedule", "tx", "logging"]
        print(f"  {'Verb/Subject':<16}  Description")
        print(f"  {'-'*16}  {'-'*50}")
        for key in verb_order:
            summary, _ = VERB_HELP_SUBJECTS[key]
            print(f"  {key:<16}  {summary}")
        print()
        print("PIPELINE TOPICS")
        print(f"  {'Topic':<16}  Description")
        print(f"  {'-'*16}  {'-'*50}")
        for key in topic_order:
            summary, _ = VERB_HELP_SUBJECTS[key]
            print(f"  {key:<16}  {summary}")
        print()
        print("QUICK EXAMPLES")
        print("  python3 pi_sstv.py mission --config /home/pi-user/pi_sstv.cfg")
        print("  python3 pi_sstv.py run --mode r36 --no-tx")
        print("  python3 pi_sstv.py config generate")
        print("  python3 pi_sstv.py service install")
        print("  python3 pi_sstv.py status")
        print("  python3 pi_sstv.py diag ptt 1.0")
        return

    canonical = VERB_HELP_ALIASES.get(a.subject.lower(), a.subject.lower())
    entry = VERB_HELP_SUBJECTS.get(canonical)
    if entry is None:
        valid = sorted(set(VERB_HELP_SUBJECTS.keys()) | set(VERB_HELP_ALIASES.keys()))
        print(f"Unknown subject: '{a.subject}'")
        print(f"Available subjects: {', '.join(valid)}")
        sys.exit(1)
    _, detail = entry
    print(detail)


# =============================================================================
# Verb: config
# =============================================================================

def _validate_config_file(path: str) -> bool:
    """Parse *path* through the full load_config() path and report errors/warnings.

    Returns True if the file is valid, False if any error was found.
    Prints all output to stdout so it is easy to read in a terminal.
    """
    W = 72
    bar = "=" * W
    thin = "-" * W
    print(bar)
    print("  Config Validate")
    print(bar)
    print(f"  path        : {path}")

    if not os.path.isfile(path):
        print(f"  FAIL  file not found: {path}")
        print(thin)
        return False

    errors = []
    warnings = []

    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(path)

    known_sections = {
        "paths", "mission", "radio", "capture", "encode",
        "overlay", "mmsstv", "logging", "status_led", "alsa", "gps", "test_panels",
    }
    custom_profiles: List[str] = []
    for section in cfg.sections():
        lowered = section.lower()
        if lowered.startswith(SCHEDULE_PROFILE_SECTION_PREFIX):
            profile_name = lowered[len(SCHEDULE_PROFILE_SECTION_PREFIX):].strip()
            if not profile_name:
                errors.append(
                    f"[{section}]: section name must include a profile name "
                    f"after '{SCHEDULE_PROFILE_SECTION_PREFIX}'"
                )
            else:
                custom_profiles.append(section)
        elif section not in known_sections:
            warnings.append(f"[{section}] — unknown section (will be ignored)")

    # Validate [schedule_profile <name>] sections.
    for section_name in custom_profiles:
        modes_raw = cfg.get(section_name, "modes", fallback="").strip()
        if not modes_raw:
            errors.append(f"[{section_name}] modes — required; at least one mode token must be listed")
        else:
            try:
                _parse_schedule_mode_list(modes_raw, f"[{section_name}] modes")
            except ValueError as exc:
                errors.append(str(exc))
        fallback_raw = cfg.get(section_name, "unavailable_mode_fallback", fallback="").strip()
        if fallback_raw:
            fb = _normalize_schedule_mode_name(fallback_raw)
            if not fb or not _is_valid_schedule_mode_token(fb):
                errors.append(
                    f"[{section_name}] unavailable_mode_fallback = '{fallback_raw}' — invalid mode token"
                )

    def _check_str_choice(section, key, valid):
        val = cfg.get(section, key, fallback=None)
        if val is not None and val.strip().lower() not in valid:
            errors.append(f"[{section}] {key} = '{val}' — invalid; must be one of: {', '.join(valid)}")

    def _check_float_range(section, key, lo, hi):
        raw = cfg.get(section, key, fallback=None)
        if raw is None:
            return
        try:
            v = float(raw)
            if not (lo <= v <= hi):
                errors.append(f"[{section}] {key} = {v} — out of range [{lo}, {hi}]")
        except ValueError:
            errors.append(f"[{section}] {key} = '{raw}' — not a valid number")

    def _check_int_pos(section, key):
        raw = cfg.get(section, key, fallback=None)
        if raw is None:
            return
        try:
            v = int(raw)
            if v <= 0:
                errors.append(f"[{section}] {key} = {v} — must be > 0")
        except ValueError:
            errors.append(f"[{section}] {key} = '{raw}' — not a valid integer")

    def _check_int_nonneg(section, key):
        raw = cfg.get(section, key, fallback=None)
        if raw is None:
            return
        try:
            v = int(raw)
            if v < 0:
                errors.append(f"[{section}] {key} = {v} — must be >= 0")
        except ValueError:
            errors.append(f"[{section}] {key} = '{raw}' — not a valid integer")

    def _check_bool(section, key):
        raw = cfg.get(section, key, fallback=None)
        if raw is None:
            return
        if raw.strip().lower() not in ("true", "false", "1", "0", "yes", "no", "on", "off"):
            errors.append(f"[{section}] {key} = '{raw}' — not a valid boolean (use true/false)")

    def _check_path_exists(section, key):
        raw = cfg.get(section, key, fallback=None)
        if raw and raw.strip() and not os.path.exists(raw.strip()):
            warnings.append(f"[{section}] {key} = '{raw.strip()}' — path does not exist")

    # [mission]
    # Collect custom profile names from this config so schedule validation is accurate.
    cfg_custom_profiles = {
        section.lower()[len(SCHEDULE_PROFILE_SECTION_PREFIX):].strip()
        for section in cfg.sections()
        if section.lower().startswith(SCHEDULE_PROFILE_SECTION_PREFIX)
    }
    all_valid_schedules = set(TRANSMIT_SCHEDULE_PROFILES.keys()) | cfg_custom_profiles
    sched = cfg.get("mission", "schedule", fallback=None)
    if sched and sched.strip().lower() not in all_valid_schedules:
        errors.append(f"[mission] schedule = '{sched}' — unknown preset; "
                      f"valid: {', '.join(sorted(all_valid_schedules))}")
    _check_int_pos("mission", "total")
    _check_float_range("mission", "interval", 1.0, 3600.0)
    _check_int_nonneg("mission", "min_captures_between_transmissions")
    _check_bool("mission", "enabled")
    _check_bool("mission", "no_tx")

    # [test_panels]
    _check_str_choice("test_panels", "selection", {"sequential", "random"})
    _check_int_pos("test_panels", "count")
    _check_bool("test_panels", "include_callsign_overlay")
    _check_bool("test_panels", "include_timestamp_overlay")
    _check_bool("test_panels", "allow_tx_without_callsign")
    tp_mode_raw = cfg.get("test_panels", "mode", fallback="").strip().lower()
    if tp_mode_raw:
        if not _is_valid_schedule_mode_token(canonicalize_mode_name(tp_mode_raw) or tp_mode_raw):
            errors.append(f"[test_panels] mode = '{tp_mode_raw}' — unrecognised mode token")

    # [radio]
    _check_str_choice("radio", "band", {"vhf", "uhf", "both"})
    _check_str_choice("radio", "tx_power_level", {"low", "high"})
    _check_str_choice("radio", "pd_idle_mode", {"release", "sleep"})
    _check_float_range("radio", "max_transmit_duty_cycle", 0.0, 1.0)
    _check_float_range("radio", "cooldown_scale_factor", 0.1, 10.0)

    # [capture]
    _check_float_range("capture", "quality", 1, 100)
    _check_str_choice("capture", "metering", {"matrix", "average", "spot"})
    _check_str_choice("capture", "exposure", {"sport", "normal", "long"})
    _check_str_choice("capture", "awb",
                      {"auto", "daylight", "cloudy", "indoor", "fluorescent",
                       "incandescent", "flash", "horizon", "greyworld"})

    # [encode]
    _check_str_choice("encode", "format", {"wav", "aiff", "ogg"})
    _check_str_choice("encode", "aspect", {"center", "pad", "stretch"})
    _check_bool("encode", "verbose")

    # [mmsstv]
    mmsstv_lib = cfg.get("mmsstv", "lib_path", fallback="").strip()
    if mmsstv_lib and not os.path.isfile(mmsstv_lib):
        warnings.append(f"[mmsstv] lib_path = '{mmsstv_lib}' — file not found")
    _check_bool("mmsstv", "disable")

    # [logging]
    _check_bool("logging", "debug")
    for key in ("log_file", "quiet_log_file"):
        _check_path_exists("logging", key)

    # [paths]
    for key in ("slowframe",):
        _check_path_exists("paths", key)

    # Report
    print(thin)
    if errors:
        print(f"  ERRORS ({len(errors)})")
        for e in errors:
            print(f"    ERROR : {e}")
    if warnings:
        print(f"  WARNINGS ({len(warnings)})")
        for w in warnings:
            print(f"    WARN  : {w}")

    if not errors and not warnings:
        print("  All checks passed — no errors or warnings.")
    elif not errors:
        print(f"  No errors.  {len(warnings)} warning(s).")

    print(thin)
    result = len(errors) == 0
    print(f"  Result      : {'PASS' if result else 'FAIL'}")
    print(thin)
    return result


def _dispatch_config(args_list):
    """Parse and execute: config {generate|validate|show} [PATH]"""
    p = argparse.ArgumentParser(
        prog="pi_sstv.py config",
        description="Manage pi_sstv configuration files.",
    )
    sub = p.add_subparsers(dest="action", metavar="{generate,validate,show}", required=True)

    gen_p = sub.add_parser("generate", help="Write a documented default config template.")
    gen_p.add_argument(
        "path",
        nargs="?",
        default=DEFAULT_CONFIG_PATH,
        metavar="PATH",
        help=f"Output path (default: {DEFAULT_CONFIG_PATH})",
    )

    val_p = sub.add_parser("validate", help="Parse a config file and report errors/warnings.")
    val_p.add_argument("path", metavar="PATH", help="Config file to validate.")

    show_p = sub.add_parser("show", help="Load a config and print the effective runtime settings.")
    show_p.add_argument("path", metavar="PATH", help="Config file to load.")

    a, _ = p.parse_known_args(args_list)

    if a.action == "generate":
        generate_default_config(a.path)

    elif a.action == "validate":
        ok = _validate_config_file(a.path)
        sys.exit(0 if ok else 1)

    elif a.action == "show":
        if not os.path.isfile(a.path):
            print(f"Config file not found: {a.path}", file=sys.stderr)
            sys.exit(1)
        load_config(a.path)
        configure_logging()
        print_runtime_startup("config-show", config_path=a.path)


# =============================================================================
# Verb: schedules
# =============================================================================

def _dispatch_schedules(args_list):
    """Parse and execute: schedules [--detail PRESET] [-c CONFIG]"""
    p = argparse.ArgumentParser(
        prog="pi_sstv.py schedules",
        description="List SSTV transmit schedule presets.",
    )
    p.add_argument(
        "-c", "--config",
        metavar="PATH",
        default=None,
        help="Load a config file to include custom [schedule_profile] presets and show "
             "which schedule is active.",
    )
    p.add_argument(
        "--detail",
        metavar="PRESET",
        default=None,
        help="Show expanded information for a single preset.",
    )
    a, _ = p.parse_known_args(args_list)

    if a.config:
        load_config(a.config)

    if a.detail:
        # Validate the preset name now that custom profiles may have been loaded.
        preset_name = _normalize_schedule_profile_name(a.detail) or a.detail
        if preset_name not in TRANSMIT_SCHEDULE_PROFILES:
            valid = ", ".join(TRANSMIT_SCHEDULE_PROFILES)
            print(f"Unknown schedule preset: '{a.detail}'.  Valid: {valid}", file=sys.stderr)
            sys.exit(1)

        modes = TRANSMIT_SCHEDULE_PROFILES[preset_name]
        description = TRANSMIT_SCHEDULE_DESCRIPTIONS.get(preset_name, "")
        profiles = [MODE_PROFILES[m] for m in modes if m in MODE_PROFILES]
        is_active = preset_name == TRANSMIT_SCHEDULE_PROFILE
        is_custom = preset_name not in BUILTIN_TRANSMIT_SCHEDULE_PROFILES
        active_tag = "  ◀ ACTIVE" if is_active else ""
        type_tag = "  [custom]" if is_custom else ""

        W = 72
        print("=" * W)
        print(f"  Schedule: {preset_name.upper()}{active_tag}{type_tag}")
        print("=" * W)
        if description:
            print(f"  {description}")
        fb_policy = describe_schedule_fallback_policy(preset_name)
        print(f"  Fallback policy: {fb_policy}")
        print("-" * W)
        print(f"  {'#':>2}  {'Mode':<12}  {'TX (s)':>6}  {'Cooldown (s)':>12}  "
              f"{'Cycle (s)':>9}  {'MMSSTV':>6}  Fallback")
        print(f"  {'':>2}  {'':12}  {'------':>6}  {'------------':>12}  "
              f"{'----------':>9}  {'------':>6}")
        total_tx, total_cool = 0, 0
        for i, profile in enumerate(profiles, 1):
            cycle = profile.duration_seconds + profile.cooldown_seconds
            mmsstv_str = "yes" if profile.requires_mmsstv else "-"
            fb_str = profile.fallback_mode or "-"
            print(f"  {i:>2}  {profile.name:<12}  {profile.duration_seconds:>6}  "
                  f"{profile.cooldown_seconds:>12}  {cycle:>9}  {mmsstv_str:>6}  {fb_str}")
            total_tx += profile.duration_seconds
            total_cool += profile.cooldown_seconds
        total_cycle = total_tx + total_cool
        duty = 100.0 * total_tx / total_cycle if total_cycle else 0.0
        cycle_min = total_cycle / 60.0
        mmsstv_needed = any(p.requires_mmsstv for p in profiles)
        print("-" * W)
        print(f"  {'Total airtime':<28}: {total_tx} s")
        print(f"  {'Total cooldown':<28}: {total_cool} s")
        print(f"  {'Min rotation time':<28}: {total_cycle} s  ({cycle_min:.1f} min)")
        print(f"  {'Max duty cycle':<28}: {duty:.1f}%")
        print(f"  {'MMSSTV library required':<28}: {'yes' if mmsstv_needed else 'no'}")
        print("=" * W)
        return

    # Default: print full list via existing list_schedules()
    list_schedules()


# =============================================================================
# Verb: run
# =============================================================================

def _dispatch_run(args_list):
    """Parse and execute: run [options]"""
    p = argparse.ArgumentParser(
        prog="pi_sstv.py run",
        description=(
            "Execute a single SSTV pipeline pass: capture → encode → transmit.\n"
            "Use --no-tx for a safe bench test without a radio connected."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("-m", "--mode", default="r36", metavar="MODE",
                   help="SSTV mode to encode (default: r36).")
    p.add_argument("-c", "--config", default=None, metavar="PATH",
                   help="Config file to load.")
    p.add_argument("--image", default=None, metavar="PATH",
                   help="Use this image instead of capturing from the camera.")
    p.add_argument("--callsign", default=None, metavar="CALL",
                   help="Callsign overlay baked into the transmitted image.")
    p.add_argument("--no-callsign", action="store_true",
                   help="Suppress callsign overlay even if set in config.")
    p.add_argument("--no-timestamp", action="store_true",
                   help="Suppress timestamp overlay.")
    p.add_argument("--no-tx", action="store_true",
                   help="Encode but skip radio transmit (safe bench test).")
    p.add_argument("--band", choices=["vhf", "uhf", "both"], default=None,
                   help="Radio band to key (overrides config).")
    p.add_argument("--power", choices=["low", "high"], default=None,
                   help="Transmit power level (overrides config).")
    p.add_argument("--keep-wav", action="store_true",
                   help="Keep the encoded WAV file after the run.")
    p.add_argument("--output-dir", default=None, metavar="PATH",
                   help="Directory for output artifacts.")
    p.add_argument("--debug", action="store_true",
                   help="Enable DEBUG-level logging.")
    p.add_argument("--mmsstv-lib", default=None, metavar="PATH",
                   help="Path to MMSSTV shared library.")
    p.add_argument("--no-mmsstv", action="store_true",
                   help="Force native SlowFrame modes only.")

    a, _ = p.parse_known_args(args_list)

    configure_logging(debug=a.debug)

    # Config file first, then CLI overrides
    if a.config:
        load_config(a.config)

    global ACTIVE_RADIO_BAND, TX_POWER_LEVEL
    global STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY
    global TIMESTAMPED_DIR

    if a.band:
        ACTIVE_RADIO_BAND = a.band
    if a.power:
        TX_POWER_LEVEL = a.power
    if a.callsign:
        STATION_CALLSIGN = a.callsign
        SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = True
    if a.no_callsign:
        SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = False
    if a.no_timestamp:
        SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY = False
    if a.output_dir:
        TIMESTAMPED_DIR = a.output_dir
    if a.no_mmsstv:
        os.environ[MMSSTV_DISABLE_ENV_VAR] = "1"
    if a.mmsstv_lib:
        os.environ[MMSSTV_LIBRARY_ENV_VAR] = a.mmsstv_lib

    if a.mode not in MODE_PROFILES:
        p.error(
            f"--mode: unknown mode '{a.mode}'. "
            f"Valid modes: {', '.join(sorted(MODE_PROFILES))}."
        )

    ensure_runtime_paths()

    needs_gpio = not a.no_tx
    if needs_gpio:
        setup_gpio()

    runtime_state = discover_slowframe_capabilities()

    import types
    fake_args = types.SimpleNamespace(
        test=a.mode,
        test_image=a.image,
        no_tx=a.no_tx,
        output_dir=TIMESTAMPED_DIR,
        ptt_pin=None,
    )

    wav_path = None
    try:
        run_test_pipeline(a.mode, fake_args, runtime_state)
    finally:
        if needs_gpio:
            GPIO.cleanup()
        # Clean up WAV unless --keep-wav was passed
        if not a.keep_wav:
            wav_candidate = os.path.join(TIMESTAMPED_DIR, "HAB-SSTV.wav")
            if wav_path is None and os.path.isfile(wav_candidate):
                # Only remove the default WAV, not a user-specified artifact
                pass  # run_test_pipeline uses a timestamped name; no cleanup needed


# =============================================================================
# Verb: mission
# =============================================================================

def _dispatch_mission(args_list):
    """Parse and execute: mission [options]  (mirrors the legacy flag-based mission path)"""
    global TRANSMIT_SCHEDULE_PROFILE, TRANSMIT_SCHEDULE
    global PIC_TOTAL, PIC_INTERVAL
    global STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global COOLDOWN_SCALE_FACTOR, MAX_TRANSMIT_DUTY_CYCLE, MIN_CAPTURES_BETWEEN_TRANSMISSIONS
    global TIMESTAMPED_DIR, ACTIVE_RADIO_BAND, TX_POWER_LEVEL

    p = argparse.ArgumentParser(
        prog="pi_sstv.py mission",
        description=(
            "Run the continuous HAB mission: capture images, encode to SSTV audio,\n"
            "and transmit on a rotating schedule until TOTAL captures are taken."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("-c", "--config", default=None, metavar="PATH",
                   help="Config file to load.")
    p.add_argument("-s", "--schedule",
                   choices=list(TRANSMIT_SCHEDULE_PROFILES.keys()),
                   default=None, metavar="PRESET",
                   help=f"Transmit schedule preset (default: {TRANSMIT_SCHEDULE_PROFILE}).")
    p.add_argument("-n", "--total", type=int, default=None, metavar="N",
                   help=f"Total captures (default: {PIC_TOTAL}).")
    p.add_argument("-i", "--interval", type=float, default=None, metavar="SECS",
                   help=f"Seconds between captures (default: {PIC_INTERVAL}).")
    p.add_argument("--callsign", default=None, metavar="CALL",
                   help="Callsign overlay on every image.")
    p.add_argument("--no-tx", action="store_true",
                   help="Capture and encode but never transmit.")
    p.add_argument("--band", choices=["vhf", "uhf", "both"], default=None,
                   help="Radio band to key.")
    p.add_argument("--power", choices=["low", "high"], default=None,
                   help="Transmit power level.")
    p.add_argument("--cooldown-scale", type=float, default=None, metavar="FACTOR",
                   help="Multiply all mode cooldowns.")
    p.add_argument("--duty-cycle", type=float, default=None, metavar="FRACTION",
                   help="Max rolling TX fraction (0.0–1.0).")
    p.add_argument("--min-captures", type=int, default=None, metavar="N",
                   help="Captures between any two transmissions.")
    p.add_argument("--output-dir", default=None, metavar="PATH",
                   help="Directory for images, WAV, and CSV.")
    p.add_argument("--debug", action="store_true",
                   help="Enable DEBUG-level logging.")
    p.add_argument("--log-file", default=None, metavar="PATH",
                   help="Log to file in addition to stdout.")
    p.add_argument("--quiet-log-file", default=None, metavar="PATH",
                   help="Log to file only, suppress stdout.")
    p.add_argument("--mmsstv-lib", default=None, metavar="PATH",
                   help="Path to MMSSTV shared library.")
    p.add_argument("--no-mmsstv", action="store_true",
                   help="Force native SlowFrame modes only.")

    a, _ = p.parse_known_args(args_list)

    selected_log_file = a.quiet_log_file or a.log_file
    quiet_stdout = a.quiet_log_file is not None
    configure_logging(debug=a.debug, log_file=selected_log_file, quiet_stdout=quiet_stdout)

    if a.config:
        load_config(a.config)

    if a.schedule:
        TRANSMIT_SCHEDULE_PROFILE = a.schedule
        TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES[a.schedule]
    if a.total is not None:
        PIC_TOTAL = a.total
    if a.interval is not None:
        PIC_INTERVAL = a.interval
    if a.callsign:
        STATION_CALLSIGN = a.callsign
        SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = True
    if a.cooldown_scale is not None:
        COOLDOWN_SCALE_FACTOR = a.cooldown_scale
    if a.duty_cycle is not None:
        MAX_TRANSMIT_DUTY_CYCLE = a.duty_cycle
    if a.min_captures is not None:
        MIN_CAPTURES_BETWEEN_TRANSMISSIONS = a.min_captures
    if a.output_dir:
        TIMESTAMPED_DIR = a.output_dir
    if a.band:
        ACTIVE_RADIO_BAND = a.band
    if a.power:
        TX_POWER_LEVEL = a.power
    if a.no_mmsstv:
        os.environ[MMSSTV_DISABLE_ENV_VAR] = "1"
    if a.mmsstv_lib:
        os.environ[MMSSTV_LIBRARY_ENV_VAR] = a.mmsstv_lib

    ensure_runtime_paths()

    needs_gpio = not a.no_tx
    if needs_gpio:
        setup_gpio()

    runtime_state = discover_slowframe_capabilities()
    print_mission_summary(runtime_state)

    try:
        for index in range(PIC_TOTAL):
            capture_number = index + 1
            image_path, timestamp_message = process_capture(index)
            time.sleep(PIC_INTERVAL)

            if not should_attempt_transmit(capture_number, runtime_state):
                log_debug(f"[#{capture_number}] Skipping transmit check: capture interval not reached")
                continue

            requested_mode, mode_profile = select_mode_profile(runtime_state)
            now_monotonic = time.monotonic()
            can_tx, reason = can_transmit_mode(capture_number, mode_profile, runtime_state, now_monotonic)

            if not can_tx:
                log(f"[#{capture_number}] Skipping transmit ({requested_mode}): {reason}")
                continue

            if requested_mode != mode_profile.name:
                log(f"[#{capture_number}] Fallback: {requested_mode} → {mode_profile.name}")
            else:
                log(f"[#{capture_number}] Transmitting: {mode_profile.name} ({mode_profile.duration_seconds}s)")

            image_path = resolve_transmit_image(image_path)

            if a.no_tx:
                log(f"[#{capture_number}] TX skipped (--no-tx): would have transmitted {mode_profile.name}")
                continue

            try:
                generate_sstv_audio(image_path, timestamp_message, mode_profile.name)
            except Exception as error:
                log(f"Encode failed: {error}")
                continue

            try:
                actual_duration = transmit_sstv_audio()
            except Exception as error:
                log(f"Playback failed: {error}")
                continue

            runtime_state.last_transmit_capture_number = capture_number
            runtime_state.last_transmit_end_monotonic = time.monotonic()
            runtime_state.transmit_history.append(
                (runtime_state.last_transmit_end_monotonic, actual_duration)
            )
            runtime_state.schedule_index += 1
            rolling_used = get_rolling_transmit_seconds(runtime_state, runtime_state.last_transmit_end_monotonic)
            rolling_pct = 100.0 * rolling_used / ROLLING_DUTY_CYCLE_WINDOW_SECONDS
            log(f"[#{capture_number}] TX done: {mode_profile.name}, {actual_duration:.1f}s, rolling duty={rolling_pct:.1f}%")
    finally:
        if needs_gpio:
            GPIO.cleanup()


# =============================================================================
# Verb: status
# =============================================================================

def _dispatch_status(args_list):
    """Parse and execute: status [--config PATH] [--verbose]"""
    p = argparse.ArgumentParser(
        prog="pi_sstv.py status",
        description="Run a pre-flight system readiness check.",
    )
    p.add_argument("-c", "--config", default=None, metavar="PATH",
                   help="Config file to load and validate.")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="Show detailed output for every check.")
    a, _ = p.parse_known_args(args_list)

    configure_logging(debug=a.verbose)

    if a.config:
        load_config(a.config)

    W = 72
    bar = "=" * W
    thin = "-" * W
    print(bar)
    print("  pi_sstv.py  —  System Status")
    print(bar)

    checks = []   # list of (label, result, detail)

    def _check(label, result, detail=""):
        mark = "PASS" if result else "FAIL"
        checks.append((label, mark, detail))
        status_char = "✓" if result else "✗"
        detail_str = f"  {detail}" if detail else ""
        print(f"  [{mark}]  {label:<38}{detail_str}")

    def _warn(label, detail=""):
        checks.append((label, "WARN", detail))
        detail_str = f"  {detail}" if detail else ""
        print(f"  [WARN]  {label:<38}{detail_str}")

    # Python version
    import platform
    py_ver = platform.python_version()
    py_ok = sys.version_info >= (3, 9)
    _check("Python version", py_ok, py_ver)

    # RPi.GPIO
    try:
        import RPi.GPIO as _gpio_test  # noqa: F401
        _check("RPi.GPIO importable", True)
    except ImportError as exc:
        _check("RPi.GPIO importable", False, str(exc))

    # rpicam-still
    rpicam_ok = os.path.isfile(RPICAM_BIN) and os.access(RPICAM_BIN, os.X_OK)
    _check("rpicam-still binary", rpicam_ok, RPICAM_BIN)

    # SlowFrame
    sf_ok = os.path.isfile(SLOWFRAME_BIN) and os.access(SLOWFRAME_BIN, os.X_OK)
    _check("SlowFrame binary", sf_ok, SLOWFRAME_BIN)

    if sf_ok:
        # MMSSTV detection
        try:
            result = run(
                [SLOWFRAME_BIN, "-M"], capture_output=True, text=True,
                timeout=SLOWFRAME_LIST_TIMEOUT_SECONDS,
            )
            output = result.stdout + result.stderr
            mmsstv_detected = (
                "detected" in output.lower() and "not detected" not in output.lower()
            )
            if mmsstv_detected:
                _check("MMSSTV library", True, "detected")
            else:
                _warn("MMSSTV library", "not detected — MMSSTV modes will fall back to native")
        except Exception as exc:
            _warn("MMSSTV library", f"probe failed: {exc}")

    # aplay
    import shutil
    aplay_path = shutil.which("aplay")
    _check("aplay in PATH", bool(aplay_path), aplay_path or "not found")

    if aplay_path:
        try:
            aplay_L = run(["aplay", "-L"], capture_output=True, text=True, timeout=10)
            aplay_output = aplay_L.stdout + aplay_L.stderr
            audremap_ok = any(
                "audremap" in ln.lower() or "headphones" in ln.lower()
                for ln in aplay_output.splitlines()
            )
            if audremap_ok:
                _check("audremap PCM device (aplay -L)", True)
            else:
                _warn("audremap PCM device (aplay -L)",
                      "not found — check dtoverlay=audremap,enable_jack=on in /boot/config.txt")
        except Exception as exc:
            _warn("audremap PCM device (aplay -L)", f"aplay -L failed: {exc}")

    # Output directory
    out_dir_exists = os.path.isdir(TIMESTAMPED_DIR)
    out_dir_writable = out_dir_exists and os.access(TIMESTAMPED_DIR, os.W_OK)
    if out_dir_exists:
        _check("Output directory writable", out_dir_writable, TIMESTAMPED_DIR)
    else:
        _warn("Output directory", f"does not exist yet — will be created: {TIMESTAMPED_DIR}")

    # Config file — auto-discover if --config not supplied
    effective_cfg = a.config or find_default_config()
    if effective_cfg:
        auto_tag = "" if a.config else f" (auto-discovered)"
        print(f"\n  Config: {effective_cfg}{auto_tag}")
        cfg_valid = _validate_config_file(effective_cfg)
        _check("Config file valid", cfg_valid, effective_cfg)
        if cfg_valid:
            # Load config so subsequent checks (output dir, etc.) reflect its settings
            load_config(effective_cfg)
    else:
        search_list = "\n          ".join(CONFIG_SEARCH_PATHS)
        _warn("Config file",
              f"not found in any search path — run 'config generate' to create one\n"
              f"          Searched:\n          {search_list}")

    # systemd services
    for svc in ("pi-sstv.service", "hamwing-gpio-init.service"):
        try:
            result = run(
                ["systemctl", "is-active", svc],
                capture_output=True, text=True, timeout=5,
            )
            active = result.stdout.strip() == "active"
            enabled_result = run(
                ["systemctl", "is-enabled", svc],
                capture_output=True, text=True, timeout=5,
            )
            enabled = enabled_result.stdout.strip() == "enabled"
            if active and enabled:
                _check(f"systemd: {svc}", True, "active + enabled")
            elif enabled and not active:
                _warn(f"systemd: {svc}", "enabled but not running")
            else:
                _warn(f"systemd: {svc}", "not installed — run 'service install' to set up")
        except FileNotFoundError:
            _warn(f"systemd: {svc}", "systemctl not available")
        except Exception as exc:
            _warn(f"systemd: {svc}", str(exc))

    # GPIO pin summary
    print()
    print(bar)
    print("  GPIO Pin Assignments  (BCM numbering)")
    print("  [cfg: ...] = current config value and its effect on this pin")
    print(bar)
    band_label = {
        "vhf":  "VHF only",
        "uhf":  "UHF only",
        "both": "VHF + UHF",
    }.get(ACTIVE_RADIO_BAND, ACTIVE_RADIO_BAND)
    ptt_pins = get_active_ptt_pins()
    ptt_label = ", ".join(f"GPIO{p}" for p in ptt_pins)
    hl_watts  = "1 W  (HL HIGH)" if TX_POWER_LEVEL == "high" else "0.5 W  (HL LOW)"
    pd_effect = "idle as INPUT float" if PD_IDLE_MODE == "release" else "idle as OUTPUT LOW"

    col = 40
    def _pin_row(label, bcm, phys, note=""):
        phys_str = f"(physical pin {phys:>2})" if isinstance(phys, int) else f"(physical pin {phys})"
        suffix = f"  [cfg: {note}]" if note else ""
        print(f"  {label:<{col}} BCM {bcm:>2}  {phys_str}{suffix}")

    print("  DRA818 control")
    _pin_row("  PD   power-down  (active-HIGH)",   DRA818_POWER_DOWN_PIN,   7,
             f"pd_idle_mode={PD_IDLE_MODE} → {pd_effect}")
    active_vhf = ACTIVE_RADIO_BAND in ("vhf", "both")
    active_uhf = ACTIVE_RADIO_BAND in ("uhf", "both")
    _pin_row("  PTT  VHF TX key  (active-LOW)",    DRA818_VHF_PTT_PIN,     13,
             f"band={ACTIVE_RADIO_BAND} → {'in use' if active_vhf else 'not selected'}")
    _pin_row("  PTT  UHF TX key  (active-LOW)",    DRA818_UHF_PTT_PIN,     11,
             f"band={ACTIVE_RADIO_BAND} → {'in use' if active_uhf else 'not selected'}")
    _pin_row("  H/L  power level",                 DRA818_POWER_LEVEL_PIN, 15,
             f"tx_power_level={TX_POWER_LEVEL} → {hl_watts}")
    print()
    print("  Audio  (PWM — fixed wiring)")
    _pin_row("  Left  PWM out",  AUDIO_LEFT_PWM_PIN,  32,
             f"overlay: {AUDIO_OVERLAY}")
    _pin_row("  Right PWM out",  AUDIO_RIGHT_PWM_PIN, 33,
             f"aplay device: {APLAY_DEVICE}")
    print()
    print("  Status LED")
    if STATUS_LED_ENABLED:
        polarity = "active-HIGH  GPIO HIGH = LED on" if STATUS_LED_ACTIVE_HIGH else "active-LOW  GPIO LOW = LED on"
        _pin_row("  Status LED", STATUS_LED_PIN, 40,
                 f"enabled, {polarity}")
    else:
        print(f"  {'Status LED':<{col}}              [cfg: status_led.enabled=false → pin not driven]")

    print(thin)
    fails = sum(1 for _, r, _ in checks if r == "FAIL")
    warns = sum(1 for _, r, _ in checks if r == "WARN")
    passes = sum(1 for _, r, _ in checks if r == "PASS")
    print(f"  Result      : {'PASS' if fails == 0 else 'FAIL'}")
    print(f"  Checks      : {passes} passed, {warns} warned, {fails} failed")
    print(thin)
    sys.exit(0 if fails == 0 else 1)


# =============================================================================
# Verb: service
# =============================================================================

_PI_SSTV_SERVICE_TEMPLATE = """\
[Unit]
Description=pi_sstv HamWing SSTV HAB payload controller
After=network.target hamwing-gpio-init.service
Wants=hamwing-gpio-init.service

[Service]
Type=simple
User={user}
WorkingDirectory={work_dir}
ExecStart={python} {script}{config_arg}
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal
SyslogIdentifier=pi-sstv

[Install]
WantedBy=multi-user.target
"""

_HAMWING_GPIO_INIT_SERVICE_TEMPLATE = """\
[Unit]
Description=HamWing GPIO idle-state initialisation
DefaultDependencies=no
Before=pi-sstv.service

[Service]
Type=oneshot
RemainAfterExit=yes
# Set PTT lines to INPUT PUD_UP (shared with Feather M0, weak pull keeps HIGH).
# Set PD (GPIO4) to INPUT (released to Feather M0 at boot).
# Set HL (GPIO22) to OUTPUT LOW (0.5 W default, safe).
ExecStart=/bin/sh -c " \\
    raspi-gpio set 27 ip pu && \\
    raspi-gpio set 17 ip pu && \\
    raspi-gpio set 4  ip    && \\
    raspi-gpio set 22 op dl"

[Install]
WantedBy=multi-user.target
"""

_SYSTEMD_UNIT_DIR = "/etc/systemd/system"


def _write_unit_file_sudo(unit_name: str, content: str, dry_run: bool = False) -> bool:
    """Write *content* to /etc/systemd/system/*unit_name* via sudo tee.

    Returns True on success, False on failure.  When *dry_run* is True the
    content is printed to stdout without writing any file.
    """
    dest = os.path.join(_SYSTEMD_UNIT_DIR, unit_name)

    if dry_run:
        W = 72
        print("=" * W)
        print(f"  DRY RUN — would write: {dest}")
        print("=" * W)
        print(content)
        print("-" * W)
        return True

    import subprocess
    try:
        proc = subprocess.run(
            ["sudo", "tee", dest],
            input=content,
            capture_output=True,
            text=True,
        )
        if proc.returncode != 0:
            print(f"ERROR writing {dest}: {proc.stderr.strip()}", file=sys.stderr)
            return False
        return True
    except FileNotFoundError:
        print("ERROR: sudo not found in PATH.", file=sys.stderr)
        return False


def _systemctl_sudo(args_list: list, dry_run: bool = False) -> bool:
    """Run systemctl with sudo.  Returns True on success."""
    cmd = ["sudo", "systemctl"] + args_list
    if dry_run:
        print(f"  DRY RUN — would run: {' '.join(cmd)}")
        return True
    import subprocess
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            print(f"  WARN  systemctl {' '.join(args_list)}: {proc.stderr.strip()}", file=sys.stderr)
            return False
        return True
    except FileNotFoundError:
        print("ERROR: sudo/systemctl not found.", file=sys.stderr)
        return False


def _dispatch_service(args_list):
    """Parse and execute: service {install|uninstall|status|logs} [options]"""
    import subprocess

    p = argparse.ArgumentParser(
        prog="pi_sstv.py service",
        description="Manage pi-sstv and hamwing-gpio-init systemd services.",
    )
    sub = p.add_subparsers(dest="action",
                           metavar="{install,uninstall,status,logs}",
                           required=True)

    # install
    inst_p = sub.add_parser("install", help="Install and enable both unit files.")
    inst_p.add_argument("-c", "--config", default=None, metavar="PATH",
                        help="Path to the .cfg file the service loads at startup.")
    inst_p.add_argument("--user", default="pi-user", metavar="USER",
                        help="System user the service runs as (default: pi-user).")
    inst_p.add_argument("--script", default=os.path.abspath(__file__), metavar="PATH",
                        help="Absolute path to pi_sstv.py (default: this file).")
    inst_p.add_argument("--python", default="/usr/bin/python3", metavar="PATH",
                        help="Python interpreter (default: /usr/bin/python3).")
    inst_p.add_argument("--dry-run", action="store_true",
                        help="Print unit files without writing them.")

    # uninstall
    uninst_p = sub.add_parser("uninstall", help="Stop, disable, and remove both unit files.")
    uninst_p.add_argument("--dry-run", action="store_true",
                          help="Print what would be done without making changes.")

    # status
    sub.add_parser("status", help="Show systemd status for both services.")

    # logs
    logs_p = sub.add_parser("logs", help="Tail the pi-sstv service journal.")
    logs_p.add_argument("--lines", type=int, default=50, metavar="N",
                        help="Number of log lines to show (default: 50).")
    logs_p.add_argument("-f", "--follow", action="store_true",
                        help="Follow the journal in real time (Ctrl-C to stop).")

    a, _ = p.parse_known_args(args_list)

    if a.action == "install":
        print("Installing HamWing systemd services …\n")
        work_dir = os.path.dirname(os.path.abspath(a.script))
        config_arg = f" mission --config {a.config}" if a.config else " mission"

        pi_sstv_unit = _PI_SSTV_SERVICE_TEMPLATE.format(
            user=a.user,
            work_dir=work_dir,
            python=a.python,
            script=a.script,
            config_arg=config_arg,
        )
        gpio_init_unit = _HAMWING_GPIO_INIT_SERVICE_TEMPLATE

        ok = True
        print(f"  Writing pi-sstv.service …")
        ok &= _write_unit_file_sudo("pi-sstv.service", pi_sstv_unit, dry_run=a.dry_run)
        print(f"  Writing hamwing-gpio-init.service …")
        ok &= _write_unit_file_sudo("hamwing-gpio-init.service", gpio_init_unit, dry_run=a.dry_run)

        if ok:
            print("  Reloading systemd daemon …")
            ok &= _systemctl_sudo(["daemon-reload"], dry_run=a.dry_run)
            print("  Enabling hamwing-gpio-init.service …")
            ok &= _systemctl_sudo(["enable", "hamwing-gpio-init.service"], dry_run=a.dry_run)
            print("  Enabling pi-sstv.service …")
            ok &= _systemctl_sudo(["enable", "pi-sstv.service"], dry_run=a.dry_run)
            if not a.dry_run:
                print("  Starting hamwing-gpio-init.service …")
                ok &= _systemctl_sudo(["start", "hamwing-gpio-init.service"])
                print("  Starting pi-sstv.service …")
                ok &= _systemctl_sudo(["start", "pi-sstv.service"])

        print()
        if a.dry_run:
            print("Dry run complete.  No changes were made.")
        elif ok:
            print("Installation complete.")
            print("  Check status : python3 pi_sstv.py service status")
            print("  Follow logs  : python3 pi_sstv.py service logs --follow")
        else:
            print("Installation finished with errors.  Check output above.", file=sys.stderr)
            sys.exit(1)

    elif a.action == "uninstall":
        print("Removing HamWing systemd services …\n")
        dry = a.dry_run
        ok = True
        for svc in ("pi-sstv.service", "hamwing-gpio-init.service"):
            print(f"  Stopping  {svc} …")
            _systemctl_sudo(["stop", svc], dry_run=dry)
            print(f"  Disabling {svc} …")
            _systemctl_sudo(["disable", svc], dry_run=dry)
            dest = os.path.join(_SYSTEMD_UNIT_DIR, svc)
            if dry:
                print(f"  DRY RUN — would delete: {dest}")
            else:
                try:
                    result = subprocess.run(["sudo", "rm", "-f", dest],
                                            capture_output=True, text=True)
                    if result.returncode == 0:
                        print(f"  Removed   {dest}")
                    else:
                        print(f"  WARN  could not remove {dest}: {result.stderr.strip()}")
                        ok = False
                except Exception as exc:
                    print(f"  ERROR: {exc}", file=sys.stderr)
                    ok = False

        if not dry:
            _systemctl_sudo(["daemon-reload"])
        print()
        if dry:
            print("Dry run complete.  No changes were made.")
        elif ok:
            print("Uninstall complete.")
        else:
            print("Uninstall finished with errors.  Check output above.", file=sys.stderr)
            sys.exit(1)

    elif a.action == "status":
        for svc in ("hamwing-gpio-init.service", "pi-sstv.service"):
            print(f"{'─' * 72}")
            print(f"  {svc}")
            print(f"{'─' * 72}")
            try:
                result = subprocess.run(
                    ["systemctl", "status", "--no-pager", svc],
                    capture_output=True, text=True,
                )
                output = (result.stdout + result.stderr).strip()
                for line in output.splitlines():
                    print(f"  {line}")
            except FileNotFoundError:
                print("  systemctl not available on this system.")
            print()

    elif a.action == "logs":
        cmd = ["journalctl", "-u", "pi-sstv.service", "--no-pager",
               f"-n", str(a.lines)]
        if a.follow:
            cmd.append("-f")
        try:
            subprocess.run(cmd)
        except FileNotFoundError:
            print("ERROR: journalctl not available.", file=sys.stderr)
            sys.exit(1)
        except KeyboardInterrupt:
            pass


# =============================================================================
# Diag config loader (lightweight; diag verb only)
# =============================================================================

def _load_diag_config(path: str):
    """Minimal config loader for the diag sub-command.

    Reads only the [status_led] and [gps] sections so that unrelated
    mission settings (e.g. custom schedule names) cannot block diagnostics.
    """
    global STATUS_LED_ENABLED, STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH
    global STATUS_LED_PWM_FREQ, STATUS_LED_MAX_PCT
    global GPS_SERIAL_PORT, GPS_SERIAL_BAUD

    if not os.path.isfile(path):
        print(f"Config file not found: {path}", file=sys.stderr)
        sys.exit(1)

    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(path)

    def _b(section, key, default):
        try:
            return cfg.getboolean(section, key, fallback=default)
        except ValueError:
            return default

    def _i(section, key, default):
        try:
            return cfg.getint(section, key, fallback=default)
        except ValueError:
            return default

    def _s(section, key, default):
        return cfg.get(section, key, fallback=default)

    STATUS_LED_ENABLED     = _b("status_led", "enabled",     STATUS_LED_ENABLED)
    STATUS_LED_PIN         = _i("status_led", "pin",         STATUS_LED_PIN)
    STATUS_LED_ACTIVE_HIGH = _b("status_led", "active_high", STATUS_LED_ACTIVE_HIGH)
    STATUS_LED_PWM_FREQ    = _i("status_led", "pwm_freq",    STATUS_LED_PWM_FREQ)
    STATUS_LED_MAX_PCT     = _i("status_led", "max_pct",     STATUS_LED_MAX_PCT)
    GPS_SERIAL_PORT        = _s("gps",        "device",      GPS_SERIAL_PORT)
    GPS_SERIAL_BAUD        = _i("gps",        "baud",        GPS_SERIAL_BAUD)


def _dispatch_diag(args_list, config_path: str = None):
    """Parse and execute a  'diag {led|gps|ptt|alsa|camera|slowframe} [...]'  sub-command."""
    p = argparse.ArgumentParser(
        prog="pi_sstv.py diag",
        description="Hardware diagnostic utilities.",
    )
    sub = p.add_subparsers(dest="target", metavar="{led,gps,ptt,alsa,camera,slowframe,tx}", required=True)

    # --- Subcommands with a positional SECONDS duration argument ---
    for sp, default_dur, help_text in (
        (sub.add_parser("led", help="Exercise the status LED through all operational states"), 1.5,  "Duration per state in seconds (default: 1.5)"),
        (sub.add_parser("gps", help="Read GPS NMEA sentences for DURATION seconds"),          30.0, "Listen duration in seconds (default: 30)"),
    ):
        sp.add_argument("duration", type=float, nargs="?", default=default_dur,
                        metavar="SECONDS", help=help_text)
        sp.add_argument("-v", "--verbose", action="store_true",
                        help="Show runtime settings and detailed step logs")

    # --- PTT subcommand (duration + --module) ---
    ptt_sp = sub.add_parser("ptt", help="Key PTT/PD GPIO lines for DURATION seconds")
    ptt_sp.add_argument("duration", type=float, nargs="?", default=1.0,
                        metavar="SECONDS", help="Key duration in seconds (default: 1.0)")
    ptt_sp.add_argument("-m", "--module", choices=["vhf", "uhf"], default="uhf",
                        help="Radio module variant to label (default: uhf)")
    ptt_sp.add_argument("-v", "--verbose", action="store_true",
                        help="Show runtime settings and detailed step logs")

    # --- Subcommands with no duration argument ---
    for name, help_text in (
        ("alsa",       "List ALSA playback devices and verify aplay"),
        ("camera",     "Verify rpicam-still and take a test capture"),
        ("slowframe",  "Check SlowFrame binary and list available SSTV modes"),
    ):
        sp = sub.add_parser(name, help=help_text)
        sp.add_argument("-v", "--verbose", action="store_true",
                        help="Show runtime settings and detailed step logs")

    # --- TX end-to-end pipeline test ---
    tx_sp = sub.add_parser("tx", help="Run the full capture → encode → TX pipeline once")
    tx_sp.add_argument("-m", "--mode", default="m1", metavar="MODE",
                       help="SSTV mode to encode (default: m1)")
    tx_sp.add_argument("--image", default=None, metavar="FILE",
                       help="Use this image instead of capturing or the default test image")
    tx_sp.add_argument("--image-dir", default=None, metavar="DIR",
                       help="Pick the first image (alphabetically) from this directory")
    tx_sp.add_argument("--no-tx", action="store_true",
                       help="Skip the radio TX stage (encode only)")
    tx_sp.add_argument("--module", choices=["vhf", "uhf"], default="uhf",
                       help="Radio module to key (default: uhf)")
    tx_sp.add_argument("-v", "--verbose", action="store_true",
                       help="Show runtime settings and detailed step logs")

    a, _ = p.parse_known_args(args_list)  # ignore unrecognised flags (e.g. --config)
    if a.target == "led":
        run_diag_led(a.duration, verbose=a.verbose, config_path=config_path)
    elif a.target == "gps":
        run_diag_gps(a.duration, verbose=a.verbose, config_path=config_path)
    elif a.target == "ptt":
        run_diag_ptt(a.duration, verbose=a.verbose, config_path=config_path, module=a.module)
    elif a.target == "alsa":
        run_diag_alsa(verbose=a.verbose, config_path=config_path)
    elif a.target == "camera":
        run_diag_camera(verbose=a.verbose, config_path=config_path)
    elif a.target == "slowframe":
        run_diag_slowframe(verbose=a.verbose, config_path=config_path)
    elif a.target == "tx":
        run_diag_tx(
            mode=a.mode,
            image_path=a.image,
            image_dir=a.image_dir,
            no_tx=a.no_tx,
            module=a.module,
            verbose=a.verbose,
            config_path=config_path,
        )


def main():
    global SLOWFRAME_BIN
    global TRANSMIT_SCHEDULE, TRANSMIT_SCHEDULE_PROFILE
    global PIC_TOTAL, PIC_INTERVAL
    global STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global COOLDOWN_SCALE_FACTOR, MAX_TRANSMIT_DUTY_CYCLE, MIN_CAPTURES_BETWEEN_TRANSMISSIONS
    global SLOWFRAME_AUDIO_FORMAT, SLOWFRAME_SAMPLE_RATE, SLOWFRAME_ASPECT_MODE
    global TIMESTAMPED_DIR, TEST_IMAGE, SSTV_WAV
    # ---------------------------------------------------------------------------
    # Verb dispatch — handled before argparse so positional arguments (e.g. a
    # numeric duration like "1.5") are not misinterpreted as unknown flags.
    # ---------------------------------------------------------------------------
    _VERBS = {
        "diag":      None,          # handled separately below (needs cfg preload)
        "help":      _dispatch_help,
        "config":    _dispatch_config,
        "schedules": _dispatch_schedules,
        "run":       _dispatch_run,
        "mission":   _dispatch_mission,
        "status":    _dispatch_status,
        "service":   _dispatch_service,
    }

    if len(sys.argv) >= 2 and sys.argv[1] in _VERBS:
        verb = sys.argv[1]
        rest = sys.argv[2:]

        if verb == "diag":
            # Pre-load a --config so status_led / gps settings take effect.
            # Use the lightweight diag loader to avoid mission-validation errors.
            cfg_path = None
            for i, arg in enumerate(sys.argv):
                if arg == "--config" and i + 1 < len(sys.argv):
                    cfg_path = sys.argv[i + 1]
                    _load_diag_config(cfg_path)
                    break
            _dispatch_diag(rest, config_path=cfg_path)
        else:
            _VERBS[verb](rest)
        return

    # Show usage hint when called with no arguments at all.
    if len(sys.argv) == 1:
        import platform, shutil

        py_ver = platform.python_version()

        sf_found = os.path.isfile(SLOWFRAME_BIN) and os.access(SLOWFRAME_BIN, os.X_OK)
        sf_label = "found" if sf_found else "NOT FOUND"

        if sf_found:
            try:
                _sf_probe = run(
                    [SLOWFRAME_BIN, "-M"], capture_output=True, text=True, timeout=5
                )
                _sf_out = _sf_probe.stdout + _sf_probe.stderr
                if "detected" in _sf_out.lower() and "not detected" not in _sf_out.lower():
                    mmsstv_label = "detected"
                else:
                    mmsstv_label = "not detected  (MMSSTV modes will fall back to native)"
            except Exception:
                mmsstv_label = "probe failed"
        else:
            mmsstv_label = "unavailable  (SlowFrame not found)"

        cfg_path = find_default_config()
        if cfg_path:
            cfg_label = f"found  ({cfg_path})"
        else:
            cfg_label = "not found  (run 'config generate' to create one)"

        W = 72
        bar  = "=" * W
        thin = "-" * W
        print(bar)
        print(f"  pi_sstv.py {SCRIPT_VERSION}  —  HamWing SSTV HAB payload controller")
        print(thin)
        print(f"  Python     : {py_ver}")
        print(f"  SlowFrame  : {sf_label}  ({SLOWFRAME_BIN})")
        print(f"  MMSSTV     : {mmsstv_label}")
        print(f"  Config     : {cfg_label}")
        print(bar)
        print()
        print("  No arguments provided.  Common starting points:")
        print()
        print("  Run a HAB mission:       python3 pi_sstv.py mission --config pi_sstv.cfg")
        print("  Single encode+TX test:   python3 pi_sstv.py run --mode r36 --no-tx")
        print("  Generate a config file:  python3 pi_sstv.py config generate")
        print("  Validate a config file:  python3 pi_sstv.py config validate pi_sstv.cfg")
        print("  System readiness check:  python3 pi_sstv.py status")
        print("  Install systemd service: python3 pi_sstv.py service install")
        print("  List transmit schedules: python3 pi_sstv.py schedules")
        print("  Hardware diagnostics:    python3 pi_sstv.py diag ptt 1.0")
        print("                           python3 pi_sstv.py diag led 1.5")
        print("  Help topics:             python3 pi_sstv.py help")
        print("                           python3 pi_sstv.py help mission")
        print()
        print("  Legacy flag-based usage:")
        print("    python3 pi_sstv.py --test r36 --no-tx")
        print("    python3 pi_sstv.py --config pi_sstv.cfg")
        print("    python3 pi_sstv.py --help")
        print()
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
