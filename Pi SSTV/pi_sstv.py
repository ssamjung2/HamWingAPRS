#!/usr/bin/env python3

import argparse
import configparser
import logging
import sys
import math
from dataclasses import dataclass, field
import time
from subprocess import CalledProcessError, run
from datetime import datetime, timezone, timedelta
import csv
import os
import re
import wave
from typing import List, Optional, Set, Tuple
import RPi.GPIO as GPIO

try:
    import serial as _serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _serial = None
    _SERIAL_AVAILABLE = False

logger = logging.getLogger("pi_sstv")

# Paths and constants
BASE_DIR = "/home/pi-user"
DEFAULT_CONFIG_PATH = os.path.join(BASE_DIR, "pi_sstv.cfg")
GENERATE_CONFIG_USE_CONFIG_PATH = "__use_config_path__"
TIMESTAMPED_DIR = os.path.join(BASE_DIR, "Desktop/HAB")
PI_SSTV_BIN = os.path.join(BASE_DIR, "pi-sstv", "pi-sstv")

# Alternate/updated converter: SlowFrame
SLOWFRAME_BIN = "/home/pi-user/Desktop/Slowframe/bin/slowframe"
TEST_IMAGE = os.path.join(BASE_DIR, "pi-sstv", "test.jpg")
SSTV_WAV = os.path.join(TIMESTAMPED_DIR, "HAB-SSTV.wav")
DATA_CSV = os.path.join(BASE_DIR, "data.csv")
RPICAM_BIN = "/usr/bin/rpicam-still"
CAMERA_NAME = "Raspberry Pi Camera Module"
CAMERA_MODEL = "OV5647"
CAMERA_SENSOR_CLASS = "5 MP fixed-focus CSI sensor"
CAMERA_NATIVE_RESOLUTION = "2592x1944"
CAMERA_CAPTURE_RESOLUTION = "auto (rpicam-still default; no explicit width/height override)"
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
SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY = 50

# Optional station ID overlay / CW ID
STATION_CALLSIGN = ""
OVERLAY_TEXT_OVERRIDE = ""
SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = False
SLOWFRAME_CALLSIGN_OVERLAY_SIZE = 14
SLOWFRAME_CALLSIGN_OVERLAY_POSITION = "top-right"
SLOWFRAME_CALLSIGN_OVERLAY_COLOR = "white"
SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR = "black"
SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_OPACITY = 50

# Uncomment and adjust if you want a different default mode or output behavior.
# Common modes:
#   Native: bw24, m1, m2, r36, r72, s1, s2, sdx
#   Common curated MMSSTV: robot8bw, robot12bw, pd50, pd90, pd120, pd160, pd180, pd240, pd290, fax480
# Formats: wav, aiff, ogg
# Aspect modes: center, pad, stretch

PIC_INTERVAL = 10
PIC_TOTAL = 500
# Keep the default gate time-based; operators can still add a hard capture-count
# floor with --min-captures or the config file when they need it.
MIN_CAPTURES_BETWEEN_TRANSMISSIONS = 0
MAX_TRANSMIT_DUTY_CYCLE = 0.35
TX_COOLDOWN_METHOD = "adaptive_dutycycle"
FIXED_TX_COOLDOWN_SECONDS = 30.0
COOLDOWN_SCALE_FACTOR = 1.0

# Method choices:
#   fixed                   : static cooldown in seconds between TX events
#   adaptive_dutycycle      : cooldown derived from last TX duration and duty target
#   adaptive_avg_dutycycle  : cooldown distributed by each mode's share of the schedule block
#   estimated               : duty-derived cooldown corrected by simplified thermal/altitude model
TX_COOLDOWN_METHOD_CHOICES = (
    "fixed",
    "adaptive_dutycycle",
    "adaptive_avg_dutycycle",
    "estimated",
)

# Deprecated config keys that should emit operator-visible warnings when present.
DEPRECATED_CONFIG_KEYS = {
    "radio": {
        "rolling_duty_cycle_window_seconds": (
            "deprecated and ignored; rolling-window duty gating was removed. "
            "Use cooldown_method + max_transmit_duty_cycle + cooldown_scale_factor instead."
        ),
    },
}

# Estimated thermal model constants (DRA818 + HamWing PCB path).
ESTIMATED_PCB_HEAT_TRANSFER_COEFFICIENT = 0.4      # W/(m^2 K)
ESTIMATED_AIR_HEAT_TRANSFER_COEFFICIENT = 10.0     # W/(m^2 K) still air at sea level
ESTIMATED_MIN_AIR_DENSITY_FACTOR = 0.22            # fractional convective cooling at peak altitude
ESTIMATED_FLIGHT_DURATION_MINUTES = 120.0          # full mission duration estimate
ESTIMATED_FREEFALL_MINUTES = 30.0                  # final descent window returning toward denser air
ESTIMATED_EFFECTIVE_THERMAL_AREA_M2 = 0.0030       # module + PCB contact area estimate
ESTIMATED_TX_HEAT_POWER_LOW_W = 0.60               # approximate waste heat at low power TX
ESTIMATED_TX_HEAT_POWER_HIGH_W = 1.10              # approximate waste heat at high power TX
ESTIMATED_COOLDOWN_SAFETY_FACTOR = 1.10            # conservative margin for uncertainty

CAPTURE_FILE_TIMEOUT = 8
SSTV_CONVERSION_SETTLE_SECONDS = 0.5
RADIO_WAKE_DELAY_SECONDS = 1
PTT_KEY_DELAY_SECONDS = 0.1
POST_PLAYBACK_DELAY_SECONDS = 0.5

CSV_HEADERS = ["Index", "Time"]
GPIO_PIN_MODE = GPIO.BCM

# ---------------------------------------------------------------------------
# Wiring reference (BCM numbering unless noted)
#
# Power
#   GPS 3V3      <- Pi 3V3 (pin 1 or 17)   most GPS breakouts accept 3V3
#   Sensor 3V3   <- Pi 3V3 (pin 1 or 17)
#   All GND      <- Pi GND (pins 6/9/14/20/25/…)
#
# I2C sensor
#   Sensor SDA   -> Pi SDA  GPIO2  (pin 3)
#   Sensor SCL   -> Pi SCL  GPIO3  (pin 5)
#
# HamWing radio control
#   Pi GPIO4  (pin  7)  -> HamWing PD      (DRA818 power-down, active-LOW sleep)
#   Pi GPIO27 (pin 13)  -> HamWing VHF PTT (DRA818V active-LOW TX key)
#   Pi GPIO17 (pin 11)  -> HamWing UHF PTT (DRA818U active-LOW TX key)  ← ADD WIRE
#   Pi GPIO22 (pin 15)  -> HamWing HL      (HIGH=1 W, LOW=0.5 W)
#
# Audio path
#   Pi GPIO12 (pin 32)  \
#   Pi GPIO13 (pin 33)  /  -> LPF input  (both PWM pins bridged into LPF)
#   LPF output              -> DRA818V mic input  (or HamWing Ring2 jack)
#   Requires config.txt:  dtoverlay=audremap,enable_jack=on
#
# GPS module  (u-blox NEO-M8N or NEO-6M recommended)
#   GPS TX (data out)  -> Pi GPIO15 / RX  (pin 10)
#   GPS RX (optional)  -> Pi GPIO14 / TX  (pin  8)  only needed for GPS config
#   GPS VCC            -> Pi 3V3          (pin  1 or 17)
#   GPS GND            -> Pi GND          (any GND pin)
#   /dev/serial0 at 9600 baud, NMEA 0183 GGA sentences
#
# RF
#   VHF Antenna  -> DRA818V SMA
#   UHF Antenna  -> DRA818U SMA
#
# Camera
#   PiCam  -> Pi CSI ribbon connector
# ---------------------------------------------------------------------------

# DRA818 GPIO pin definitions
DRA818_PTT_PIN = 27          # BCM 27, physical pin 13 — active-LOW VHF TX key
DRA818_UHF_PTT_PIN = 17      # BCM 17, physical pin 11 — active-LOW UHF TX key  ← add second wire here
DRA818_POWER_DOWN_PIN = 4    # BCM  4, physical pin  7 — HIGH=active, LOW=sleep
DRA818_POWER_LEVEL_PIN = 22  # BCM 22, physical pin 15 — HIGH=1 W, LOW=0.5 W

# Active radio band selection: "vhf", "uhf", or "both"
ACTIVE_RADIO_BAND = "vhf"
TX_POWER_LEVEL = "low"      # "low"=0.5 W (HL LOW), "high"=1 W (HL HIGH)
PD_IDLE_MODE = "release"    # "release"=INPUT (Feather owns PD), "sleep"=OUTPUT LOW

# I2C sensor pin definitions (BCM numbering, hardware I2C1 / i2c-1)
SENSOR_I2C_SDA_PIN = 2  # physical pin 3
SENSOR_I2C_SCL_PIN = 3  # physical pin 5

# PWM audio routing used by aplay via the Raspberry Pi audio overlay
AUDIO_LEFT_PWM_PIN = 12
AUDIO_RIGHT_PWM_PIN = 13
AUDIO_OVERLAY = "dtoverlay=audremap,enable_jack=on"

# ALSA playback reliability settings
# PI_SSTV_ALSA_DEVICE: force one specific playback target (e.g. "plughw:Headphones,0").
# If empty, the script auto-selects from available devices each run.
ALSA_AUDIO_DEVICE = os.environ.get("PI_SSTV_ALSA_DEVICE", "").strip()
ALSA_DEVICE_CANDIDATES = [
    "plughw:CARD=Headphones,DEV=0",
    "plughw:Headphones,0",
    "default:CARD=Headphones",
    "sysdefault:CARD=Headphones",
    "default",
    "sysdefault",
]

# Audio level guardrails (help prevent overdrive/clipping into DRA818 mic input)
def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


# Leave mixer device empty by default so plain `amixer` (default card) is tried first.
ALSA_MIXER_DEVICE = os.environ.get("PI_SSTV_ALSA_MIXER_DEVICE", "").strip()
ALSA_MIXER_CONTROL = os.environ.get("PI_SSTV_ALSA_MIXER_CONTROL", "PCM").strip()
ALSA_MIXER_CONTROL_CANDIDATES = [
    ALSA_MIXER_CONTROL,
    "Headphone",
    "Digital",
    "Master",
    "PCM",
]
ALSA_TARGET_VOLUME_PERCENT = max(0, min(100, _env_int("PI_SSTV_ALSA_TARGET_VOLUME", 70)))
ALSA_MAX_SAFE_VOLUME_PERCENT = max(0, min(100, _env_int("PI_SSTV_ALSA_MAX_SAFE_VOLUME", 85)))
ALSA_ENFORCE_VOLUME = _env_bool("PI_SSTV_ALSA_ENFORCE_VOLUME", True)
APLAY_TIMEOUT_SECONDS = max(10, _env_int("PI_SSTV_APLAY_TIMEOUT_SECONDS", 360))
APLAY_TIMEOUT_MARGIN_SECONDS = max(10, _env_int("PI_SSTV_APLAY_TIMEOUT_MARGIN_SECONDS", 45))

# Cache the resolved mixer control to avoid repeated probe overhead and warning spam.
_ALSA_RESOLVED_MIXER_CONTROL: Optional[str] = None
_ALSA_MIXER_DISABLED: bool = False

# Alternate audio pair if config.txt uses dtoverlay=audremap,pins_18_19,enable_jack=on
# AUDIO_LEFT_PWM_PIN = 18
# AUDIO_RIGHT_PWM_PIN = 19

# ---------------------------------------------------------------------------
# GPS module settings (UART-connected, NMEA 0183)
# Recommended module: u-blox NEO-M8N or NEO-6M breakout (UART interface)
#   Wiring: GPS TX -> Pi GPIO15/RX (pin 10), GPS VCC -> 3.3 V (pin 1), GPS GND -> GND
#   Requires: pip3 install pyserial
#   Requires: disable Linux serial console via raspi-config -> Interface Options -> Serial Port
#             (disable login shell, keep hardware serial enabled)
# ---------------------------------------------------------------------------
GPS_ENABLED = False
GPS_DEVICE = "/dev/serial0"   # Pi Zero hardware UART (GPIO14/GPIO15)
GPS_BAUD = 9600
GPS_TIMEOUT_SECONDS = 5.0
GPS_ALTITUDE_UNITS = "m"      # "m" for metres, "ft" for feet


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
    last_transmit_mode_name: Optional[str] = None
    last_transmit_duration_seconds: float = 0.0


MODE_PROFILES = {
    "robot8bw": ModeProfile(
        name="robot8bw",
        duration_seconds=8,
        cooldown_seconds=75,
        image_width=160,
        image_height=120,
        requires_mmsstv=True,
        fallback_mode="bw24",
        description="Ultra-fast MMSSTV monochrome status frame for tight duty-cycle budgets.",
    ),
    "robot12bw": ModeProfile(
        name="robot12bw",
        duration_seconds=12,
        cooldown_seconds=75,
        image_width=160,
        image_height=120,
        requires_mmsstv=True,
        fallback_mode="bw24",
        description="Very fast MMSSTV monochrome mode for rapid update windows.",
    ),
    "bw24": ModeProfile(
        name="bw24",
        duration_seconds=24,
        cooldown_seconds=105,
        image_height=120,
        description="Fast monochrome native mode for low duty-cycle updates.",
    ),
    "r36": ModeProfile(
        name="r36",
        duration_seconds=36,
        cooldown_seconds=135,
        image_height=240,
        description="Fast native color mode for regular balloon image updates.",
    ),
    "m2": ModeProfile(
        name="m2",
        duration_seconds=58,
        cooldown_seconds=210,
        image_height=256,
        description="Balanced native mode with good compatibility.",
    ),
    "s2": ModeProfile(
        name="s2",
        duration_seconds=71,
        cooldown_seconds=255,
        image_height=256,
        description="Native Scottie mode with strong compatibility and moderate airtime.",
    ),
    "sdx": ModeProfile(
        name="sdx",
        duration_seconds=269,
        cooldown_seconds=780,
        image_height=256,
        description="Native Scottie DX mode for very high-detail, long-duration snapshots.",
    ),
    "r72": ModeProfile(
        name="r72",
        duration_seconds=72,
        cooldown_seconds=255,
        image_height=240,
        description="Higher-quality native Robot mode.",
    ),
    "pd50": ModeProfile(
        name="pd50",
        duration_seconds=50,
        cooldown_seconds=210,
        image_height=256,
        requires_mmsstv=True,
        fallback_mode="m2",
        description="Fast MMSSTV PD mode for efficient color updates.",
    ),
    "pd90": ModeProfile(
        name="pd90",
        duration_seconds=90,
        cooldown_seconds=300,
        image_height=256,
        requires_mmsstv=True,
        fallback_mode="r36",
        description="Popular MMSSTV fast color mode when the encoder library is available.",
    ),
    "m1": ModeProfile(
        name="m1",
        duration_seconds=114,
        cooldown_seconds=420,
        image_height=256,
        description="High-quality native Martin mode for less frequent transmissions.",
    ),
    "s1": ModeProfile(
        name="s1",
        duration_seconds=110,
        cooldown_seconds=420,
        image_height=256,
        description="Native Scottie high-quality mode for periodic detail shots.",
    ),
    "pd120": ModeProfile(
        name="pd120",
        duration_seconds=120,
        cooldown_seconds=420,
        image_width=640,
        image_height=496,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Higher-quality MMSSTV mode with a larger cooldown budget.",
    ),
    "pd160": ModeProfile(
        name="pd160",
        duration_seconds=160,
        cooldown_seconds=540,
        image_width=512,
        image_height=400,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Slower MMSSTV quality mode for longer detail passes.",
    ),
    "pd180": ModeProfile(
        name="pd180",
        duration_seconds=180,
        cooldown_seconds=600,
        image_width=640,
        image_height=496,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="High-detail MMSSTV mode, best for occasional mission snapshots.",
    ),
    "fax480": ModeProfile(
        name="fax480",
        duration_seconds=180,
        cooldown_seconds=600,
        image_width=512,
        image_height=480,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="High-detail MMSSTV mode best reserved for test windows.",
    ),
    "pd240": ModeProfile(
        name="pd240",
        duration_seconds=240,
        cooldown_seconds=720,
        image_width=640,
        image_height=496,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Very high quality MMSSTV PD mode for science windows and horizon detail passes.",
    ),
    "pd290": ModeProfile(
        name="pd290",
        duration_seconds=290,
        cooldown_seconds=840,
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


def get_effective_schedule_fallback_mode(profile_name: Optional[str] = None) -> Optional[str]:
    normalized_profile_name = _normalize_schedule_profile_name(profile_name)
    profile_fallback = None
    if normalized_profile_name is not None:
        profile_fallback = TRANSMIT_SCHEDULE_FALLBACK_MODES.get(normalized_profile_name)
    return _normalize_schedule_mode_name(profile_fallback or GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK)


def describe_schedule_fallback_policy(profile_name: Optional[str] = None) -> str:
    normalized_profile_name = _normalize_schedule_profile_name(profile_name)
    profile_fallback = None
    if normalized_profile_name is not None:
        profile_fallback = _normalize_schedule_mode_name(TRANSMIT_SCHEDULE_FALLBACK_MODES.get(normalized_profile_name))

    global_fallback = _normalize_schedule_mode_name(GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK)
    if profile_fallback:
        return f"profile={profile_fallback}  global={global_fallback or '-'}"
    return f"profile=<inherit>  global={global_fallback or '-'}"


def get_protocol_token_for_mode(mode_name: str) -> str:
    """Return exact SlowFrame protocol token for a canonical mode key."""
    return MODE_PROTOCOL_TOKENS.get(mode_name, mode_name)


def _candidate_protocol_tokens(mode_name: str) -> List[str]:
    """Return likely protocol tokens for retrying mode selection on strict SlowFrame builds."""
    candidates: List[str] = [get_protocol_token_for_mode(mode_name), mode_name]

    normalized_mode = re.sub(r"[^a-z0-9]", "", mode_name.lower())
    for alias_key, target in MODE_ALIASES.items():
        if target != mode_name:
            continue
        candidates.append(alias_key)
        if alias_key.startswith("bw") and alias_key[2:].isdigit():
            candidates.append(f"b/w{alias_key[2:]}")

    robot_match = re.match(r"^robot(\d+)bw$", normalized_mode)
    if robot_match:
        bw_suffix = robot_match.group(1)
        candidates.append(f"b/w{bw_suffix}")
        candidates.append(f"bw{bw_suffix}")

    deduped: List[str] = []
    seen: Set[str] = set()
    for token in candidates:
        if not token:
            continue
        token_l = token.lower()
        if token_l in seen:
            continue
        seen.add(token_l)
        deduped.append(token)
    return deduped


def _tuned_cooldown_for_duration(duration_seconds: int) -> int:
    """Return moderate cooldown defaults for dynamically discovered modes."""
    if duration_seconds <= 12:
        return 75
    if duration_seconds <= 24:
        return 105
    if duration_seconds <= 40:
        return 135
    if duration_seconds <= 60:
        return 210
    if duration_seconds <= 80:
        return 255
    if duration_seconds <= 100:
        return 300
    if duration_seconds <= 130:
        return 420
    if duration_seconds <= 180:
        return 600
    if duration_seconds <= 240:
        return 720
    if duration_seconds <= 300:
        return 840
    if duration_seconds <= 360:
        return 960
    return max(1080, int(round(duration_seconds * 2.8)))


def _default_fallback_mode(duration_seconds: int, is_mono: bool) -> str:
    """Select a native fallback for modes not explicitly curated in MODE_PROFILES."""
    if is_mono:
        return "bw24"
    if duration_seconds <= 45:
        return "r36"
    if duration_seconds <= 90:
        return "r72"
    return "m1"


def _augment_mode_profiles_from_slowframe_output(output: str) -> Tuple[Set[str], Set[str], int]:
    """Parse slowframe -L output, add missing profiles, and return (native, mmsstv, added_count)."""
    native_modes: Set[str] = set()
    mmsstv_modes: Set[str] = set()
    section: Optional[str] = None
    added_count = 0

    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        lowered = line.lower()
        if "native modes" in lowered:
            section = "native"
            continue
        if "mmsstv modes" in lowered:
            section = "mmsstv"
            continue

        if lowered.startswith("code") or lowered.startswith("total modes:") or lowered.startswith("usage:") or lowered.startswith("example:"):
            continue
        if all(ch in "═─- " for ch in line):
            continue

        code_token: Optional[str] = None
        columns = re.split(r"\s{2,}", line)
        if columns and re.fullmatch(r"[a-z0-9_\-/]+", columns[0].strip().lower()):
            code_token = columns[0].strip().lower()
        else:
            legacy = re.match(r"^([a-z0-9_\-/]+)\s+-", lowered)
            if legacy:
                code_token = legacy.group(1)

        if not code_token:
            continue

        canonical = canonicalize_mode_name(code_token) or code_token
        MODE_PROTOCOL_TOKENS[canonical] = code_token
        normalized = re.sub(r"[^a-z0-9]", "", code_token)
        if normalized and canonical != code_token and normalized not in MODE_ALIASES:
            MODE_ALIASES[normalized] = canonical

        existing_profile = MODE_PROFILES.get(canonical)
        is_mmsstv = existing_profile.requires_mmsstv if existing_profile else (section == "mmsstv")
        if is_mmsstv:
            mmsstv_modes.add(canonical)
        else:
            native_modes.add(canonical)

        # Keep curated profiles intact, only synthesize unknown ones.
        if existing_profile:
            continue

        duration_match = re.search(r"(\d+(?:\.\d+)?)s\b", lowered)
        if duration_match:
            duration_seconds = max(1, int(round(float(duration_match.group(1)))))
        else:
            duration_seconds = 60

        res_match = re.search(r"(\d+)x(\d+)", lowered)
        if res_match:
            image_width = int(res_match.group(1))
            image_height: Optional[int] = int(res_match.group(2))
        else:
            image_width = 320
            image_height = None

        is_mono = ("mono" in lowered) or ("b/w" in code_token)
        cooldown_seconds = _tuned_cooldown_for_duration(duration_seconds)
        fallback_mode = _default_fallback_mode(duration_seconds, is_mono)

        MODE_PROFILES[canonical] = ModeProfile(
            name=canonical,
            duration_seconds=duration_seconds,
            cooldown_seconds=cooldown_seconds,
            image_width=image_width,
            image_height=image_height,
            requires_mmsstv=is_mmsstv,
            fallback_mode=fallback_mode,
            description="Discovered from slowframe -L (auto-profiled).",
        )
        added_count += 1

    return native_modes, mmsstv_modes, added_count


def refresh_mode_profiles_from_slowframe() -> Tuple[Set[str], Set[str], int]:
    """Run slowframe -L and augment MODE_PROFILES with discovered mode entries."""
    list_cmd = [SLOWFRAME_BIN, "-L"]
    if SLOWFRAME_VERBOSE:
        list_cmd.insert(1, "-v")
    result = run(
        list_cmd,
        capture_output=True,
        text=True,
        check=True,
        timeout=SLOWFRAME_LIST_TIMEOUT_SECONDS,
    )
    output = "\n".join(filter(None, [result.stdout, result.stderr]))
    return _augment_mode_profiles_from_slowframe_output(output)

TRANSMIT_SCHEDULE_PROFILE = "hab_cruise"
BUILTIN_TRANSMIT_SCHEDULE_PROFILES = {
    # hab_climb: absolute maximum update rate, mono-heavy — steepest part of the climb.
    "hab_climb": (
        "robot8bw",
        "robot12bw",
        "bw24",
        "r36",
        "robot12bw",
        "r36",
    ),
    # hab_rapid: fast color rotation with short cooldowns — upper ascent / release phase.
    "hab_rapid": (
        "robot12bw",
        "r36",
        "robot12bw",
        "r36",
        "pd50",
    ),
    # hab_cruise: balanced default — mixes status frames and quality shots across the full flight.
    "hab_cruise": (
        "robot12bw",
        "r36",
        "m2",
        "pd120",
        "s2",
        "robot12bw",
        "m1",
        "pd120",
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
        "pd290",
    ),
}
BUILTIN_TRANSMIT_SCHEDULE_DESCRIPTIONS = {
    "hab_climb":  "Maximum update-rate profile. Monochrome frames dominate to minimise cooldown "
                  "gaps during the steepest part of the climb.",
    "hab_rapid":  "Short bursts + one PD color shot per rotation. Tuned for higher throughput "
                  "at low-power TX with improved thermal margins.",
    "hab_cruise": "Balanced default with periodic PD120 quality updates. Tuned for sustained "
                  "operations at 0.5 W with moderate thermal headroom.",
    "hab_float":  "Quality-first rotation including PD290 opportunities. Best with added "
                  "heatsinking and continuous thermal monitoring.",
}
BUILTIN_TRANSMIT_SCHEDULE_FALLBACK_MODES = {
    name: None for name in BUILTIN_TRANSMIT_SCHEDULE_PROFILES
}
SCHEDULE_PROFILE_SECTION_PREFIX = "schedule_profile "
GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK = "r36"

TRANSMIT_SCHEDULE_PROFILES = {
    name: tuple(modes) for name, modes in BUILTIN_TRANSMIT_SCHEDULE_PROFILES.items()
}
TRANSMIT_SCHEDULE_DESCRIPTIONS = dict(BUILTIN_TRANSMIT_SCHEDULE_DESCRIPTIONS)
TRANSMIT_SCHEDULE_FALLBACK_MODES = dict(BUILTIN_TRANSMIT_SCHEDULE_FALLBACK_MODES)

TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES.get(
    TRANSMIT_SCHEDULE_PROFILE,
    TRANSMIT_SCHEDULE_PROFILES["hab_cruise"],
)


def _reset_schedule_profile_registry() -> None:
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
            f"{profile_name}: {', '.join(invalid_modes)}"
            for profile_name, invalid_modes in sorted(invalid_by_profile.items())
        )
        label = "undefined or invalid" if strict_defined_modes else "invalid"
        raise RuntimeError(f"Schedule profile contains {label} mode(s): {details}")


_validate_schedule_profiles()


def generate_default_config(path: str):
    """Write a fully-commented default configuration file to *path*."""
    schedules = ", ".join(TRANSMIT_SCHEDULE_PROFILES.keys())
    builtin_schedule_lines: List[str] = []
    for schedule_name, modes in BUILTIN_TRANSMIT_SCHEDULE_PROFILES.items():
        description = BUILTIN_TRANSMIT_SCHEDULE_DESCRIPTIONS.get(schedule_name, "")
        builtin_schedule_lines.append(f"#   {schedule_name:<10} - {description}")
        builtin_schedule_lines.append(f"#               {' -> '.join(modes)}")
    builtin_schedule_block = "\n".join(builtin_schedule_lines)
    GPS_ENABLED_STR = "true" if GPS_ENABLED else "false"
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
# Built-in preset details:
{builtin_schedule_block}
# Additional custom presets may be added with [schedule_profile <name>] sections below.
# CLI note: --schedule NAME overrides this value after the config file is loaded.
schedule = {TRANSMIT_SCHEDULE_PROFILE}

# Global default mode used when a scheduled mode is unavailable and its own
# fallback chain is exhausted.
# Fallback order during scheduling is:
#   1. The requested mode's curated fallback chain.
#   2. The active profile's unavailable_mode_fallback, if set.
#   3. This global [mission] unavailable_mode_fallback.
#   4. A final safety fallback to r36, then the first available mode.
unavailable_mode_fallback = {GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK}

# Total number of image captures before the mission ends.
total = {PIC_TOTAL}

# Seconds to wait between captures.
interval = {PIC_INTERVAL}

# Station callsign printed in the image overlay (e.g. W1AW-11).
# Callsign is required for encode/test/mission workflows.
callsign = {STATION_CALLSIGN}

# Minimum number of capture cycles that must elapse between any two
# transmissions, regardless of cooldown state.
min_captures_between_transmissions = {MIN_CAPTURES_BETWEEN_TRANSMISSIONS}

# Skip all radio transmission.  Images are captured and encoded but never
# played back.  Useful for bench testing without a radio connected.
# no_tx = false


# -----------------------------------------------------------------------------
# [schedule_profile custom]  Optional operator-defined schedule profile
# -----------------------------------------------------------------------------
# Any section named [schedule_profile <name>] becomes a selectable schedule.
# Names are normalized to lowercase when loaded.
# The modes value may use commas, whitespace, or both as separators.
# Mode tokens are validated when the config loads; actual encoder availability
# is resolved later at runtime after SlowFrame mode discovery.
#
# Set [mission] schedule = <name> to activate it, or override from the CLI:
#   python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg --schedule custom
#
# Example activation using a global mission fallback:
#   [mission]
#   schedule = custom
#   unavailable_mode_fallback = r36
#
# Full example custom profile with its own default override:
# [schedule_profile custom]
# modes = robot12bw, r36, pd50, pd90
# description = Operator-defined mixed rapid schedule.
# unavailable_mode_fallback = m2


# -----------------------------------------------------------------------------
# [radio]  Radio band selection, duty-cycle, cooldown, and TX timing
# -----------------------------------------------------------------------------
# For --explain reference: python3 pi_sstv.py --explain tx
[radio]

# Active radio band for transmission.
#   vhf   - DRA818V only via GPIO27 (physical pin 13)   (default)
#   uhf   - DRA818U only via GPIO17 (physical pin 11)   add wire: Pi GPIO17 -> HamWing UHF PTT
#   both  - Key both PTT lines simultaneously for dual-band TX
band = {ACTIVE_RADIO_BAND}

# TX power level applied before each TX/PTT key operation.
#   low   - H/L LOW  (L, ~0.5 W)
#   high  - H/L HIGH (H, ~1.0 W)
tx_power_level = {TX_POWER_LEVEL}

# PD line behavior when idle (after TX/PTT completes).
#   release - set PD pin to INPUT (high-impedance) so Feather M0 controls final state (default)
#   sleep   - drive PD LOW from Pi for explicit radio sleep/power-save at idle
pd_idle_mode = {PD_IDLE_MODE}

# Cooldown model used to decide when the next TX may occur.
#   fixed
#   adaptive_dutycycle
#   adaptive_avg_dutycycle
#   estimated
cooldown_method = {TX_COOLDOWN_METHOD}

# Used by cooldown_method=fixed; static seconds between TX events.
fixed_tx_cooldown_seconds = {FIXED_TX_COOLDOWN_SECONDS}

# Target duty cycle fraction used by adaptive methods (0.0 – 1.0).
max_transmit_duty_cycle = {MAX_TRANSMIT_DUTY_CYCLE}

# Global multiplier applied to calculated cooldown time from the selected method.
# 1.0 = nominal  |  0.75 = more aggressive  |  1.5 = conservative
cooldown_scale_factor = {COOLDOWN_SCALE_FACTOR}

# Estimated thermal model settings.
# Flight profile estimate: ascent/float period + freefall period near mission end.
estimated_flight_duration_minutes = {ESTIMATED_FLIGHT_DURATION_MINUTES}
estimated_freefall_minutes = {ESTIMATED_FREEFALL_MINUTES}

# Heat transfer coefficients and thermal surface assumptions.
estimated_pcb_heat_transfer_coefficient = {ESTIMATED_PCB_HEAT_TRANSFER_COEFFICIENT}
estimated_air_heat_transfer_coefficient = {ESTIMATED_AIR_HEAT_TRANSFER_COEFFICIENT}
estimated_min_air_density_factor = {ESTIMATED_MIN_AIR_DENSITY_FACTOR}
estimated_effective_thermal_area_m2 = {ESTIMATED_EFFECTIVE_THERMAL_AREA_M2}

# Approximate RF PA waste heat (W) for low/high TX power policies.
estimated_tx_heat_power_low_w = {ESTIMATED_TX_HEAT_POWER_LOW_W}
estimated_tx_heat_power_high_w = {ESTIMATED_TX_HEAT_POWER_HIGH_W}

# Additional conservative multiplier for estimated cooling time.
estimated_cooldown_safety_factor = {ESTIMATED_COOLDOWN_SAFETY_FACTOR}

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
callsign_background_opacity = {SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_OPACITY}

# Optional replacement for default MODE DATE TIME body text.
# Callsign remains mandatory and is always prepended automatically.
custom_text = {OVERLAY_TEXT_OVERRIDE}


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
# [alsa]  Playback device, mixer guardrails, and timeout behavior
# -----------------------------------------------------------------------------
# Use this section to pin a playback target or tune mixer guardrails.
# Equivalent environment variables:
#   PI_SSTV_ALSA_DEVICE
#   PI_SSTV_ALSA_MIXER_DEVICE
#   PI_SSTV_ALSA_MIXER_CONTROL
#   PI_SSTV_ALSA_TARGET_VOLUME
#   PI_SSTV_ALSA_MAX_SAFE_VOLUME
#   PI_SSTV_ALSA_ENFORCE_VOLUME
#   PI_SSTV_APLAY_TIMEOUT_SECONDS
#   PI_SSTV_APLAY_TIMEOUT_MARGIN_SECONDS
[alsa]

# Force one specific ALSA playback device. Leave blank to auto-select.
# Example: plughw:Headphones,0
playback_device = {ALSA_AUDIO_DEVICE}

# Mixer target device/card used by amixer. Leave blank to probe the default card.
mixer_device = {ALSA_MIXER_DEVICE}

# Preferred mixer control name used for volume guardrails.
# Example: PCM, Headphone, Master
mixer_control = {ALSA_MIXER_CONTROL}

# Target mixer volume percentage applied before TX when guardrails are enabled.
target_volume_percent = {ALSA_TARGET_VOLUME_PERCENT}

# Warning threshold if mixer readback exceeds this percentage.
max_safe_volume_percent = {ALSA_MAX_SAFE_VOLUME_PERCENT}

# Enable ALSA mixer guardrails.
enforce_volume = {'true' if ALSA_ENFORCE_VOLUME else 'false'}

# Minimum aplay timeout in seconds.
# Keep this at 360s or higher for long TX modes (e.g., PD290).
aplay_timeout_seconds = {APLAY_TIMEOUT_SECONDS}

# Extra timeout margin added on top of WAV/mode duration.
aplay_timeout_margin_seconds = {APLAY_TIMEOUT_MARGIN_SECONDS}


# -----------------------------------------------------------------------------
# [gps]  GPS module for gridsquare and altitude overlay
# -----------------------------------------------------------------------------
# Recommended module: u-blox NEO-M8N or NEO-6M (UART, NMEA 0183)
# Wiring: GPS TX -> Pi GPIO15 / RX (physical pin 10)
#         GPS VCC -> Pi 3V3 (pin 1 or 17)
#         GPS GND -> Pi GND (any)
# Setup:  pip3 install pyserial
#         sudo raspi-config -> Interface Options -> Serial Port
#           -> Disable login shell, Enable serial hardware
#         reboot, then verify: ls /dev/serial0
[gps]

# Enable GPS polling and gridsquare/altitude image overlay.
enable = {GPS_ENABLED_STR}

# Serial device path (Pi Zero hardware UART via GPIO14/GPIO15).
device = {GPS_DEVICE}

# GPS serial baud rate.  Most u-blox modules default to 9600.
baud = {GPS_BAUD}

# Altitude display units.
#   m  - metres above MSL   (default)
#   ft - feet above MSL
units = {GPS_ALTITUDE_UNITS}


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
    global PIC_TOTAL, PIC_INTERVAL, STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global OVERLAY_TEXT_OVERRIDE
    global MIN_CAPTURES_BETWEEN_TRANSMISSIONS
    global MAX_TRANSMIT_DUTY_CYCLE, COOLDOWN_SCALE_FACTOR
    global TX_COOLDOWN_METHOD, FIXED_TX_COOLDOWN_SECONDS
    global ESTIMATED_FLIGHT_DURATION_MINUTES, ESTIMATED_FREEFALL_MINUTES
    global ESTIMATED_PCB_HEAT_TRANSFER_COEFFICIENT, ESTIMATED_AIR_HEAT_TRANSFER_COEFFICIENT
    global ESTIMATED_MIN_AIR_DENSITY_FACTOR, ESTIMATED_EFFECTIVE_THERMAL_AREA_M2
    global ESTIMATED_TX_HEAT_POWER_LOW_W, ESTIMATED_TX_HEAT_POWER_HIGH_W
    global ESTIMATED_COOLDOWN_SAFETY_FACTOR
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
    global SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_OPACITY
    global TIMESTAMPED_DIR, SLOWFRAME_BIN, TEST_IMAGE, DATA_CSV, SSTV_WAV
    global ACTIVE_RADIO_BAND, TX_POWER_LEVEL, PD_IDLE_MODE
    global GPS_ENABLED, GPS_DEVICE, GPS_BAUD, GPS_ALTITUDE_UNITS
    global ALSA_AUDIO_DEVICE, ALSA_MIXER_DEVICE, ALSA_MIXER_CONTROL
    global ALSA_TARGET_VOLUME_PERCENT, ALSA_MAX_SAFE_VOLUME_PERCENT, ALSA_ENFORCE_VOLUME
    global APLAY_TIMEOUT_SECONDS, APLAY_TIMEOUT_MARGIN_SECONDS

    if not os.path.isfile(path):
        print(f"Config file not found: {path}", file=sys.stderr)
        sys.exit(1)

    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(path)
    _reset_schedule_profile_registry()

    def _warn_deprecated_keys() -> None:
        for section_name, deprecated_map in DEPRECATED_CONFIG_KEYS.items():
            if not cfg.has_section(section_name):
                continue
            for key_name, message in deprecated_map.items():
                if cfg.has_option(section_name, key_name):
                    print(
                        f"WARNING: Config [{section_name}] {key_name} is {message}",
                        file=sys.stderr,
                    )

    _warn_deprecated_keys()

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

    # [schedule_profile <name>]
    for section_name in cfg.sections():
        lowered_section = section_name.lower()
        if not lowered_section.startswith(SCHEDULE_PROFILE_SECTION_PREFIX):
            continue

        profile_name = _normalize_schedule_profile_name(section_name[len(SCHEDULE_PROFILE_SECTION_PREFIX):])
        if not profile_name:
            print(
                f"Config [{section_name}]: section name must include a profile name after '{SCHEDULE_PROFILE_SECTION_PREFIX}'",
                file=sys.stderr,
            )
            sys.exit(1)

        modes_raw = cfg.get(section_name, "modes", fallback="")
        try:
            parsed_modes = _parse_schedule_mode_list(modes_raw, f"Config [{section_name}] modes")
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            sys.exit(1)

        description = cfg.get(section_name, "description", fallback="").strip()
        if not description:
            description = f"Operator-defined schedule loaded from config section [{section_name}]."

        fallback_raw = cfg.get(section_name, "unavailable_mode_fallback", fallback="").strip()
        fallback_mode = _normalize_schedule_mode_name(fallback_raw) if fallback_raw else None
        if fallback_mode and not _is_valid_schedule_mode_token(fallback_mode):
            print(
                f"Config [{section_name}] unavailable_mode_fallback: invalid mode token '{fallback_raw}'",
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

    schedule = _normalize_schedule_profile_name(_str("mission", "schedule", TRANSMIT_SCHEDULE_PROFILE))
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

    cooldown_method = _normalize_cooldown_method(_str("radio", "cooldown_method", TX_COOLDOWN_METHOD))
    if cooldown_method not in TX_COOLDOWN_METHOD_CHOICES:
        print(
            f"Config [radio] cooldown_method: invalid value '{cooldown_method}'. "
            f"Valid: {', '.join(TX_COOLDOWN_METHOD_CHOICES)}",
            file=sys.stderr,
        )
        sys.exit(1)
    TX_COOLDOWN_METHOD                = cooldown_method
    FIXED_TX_COOLDOWN_SECONDS         = max(0.0, _float("radio", "fixed_tx_cooldown_seconds", FIXED_TX_COOLDOWN_SECONDS))

    MAX_TRANSMIT_DUTY_CYCLE           = _float("radio", "max_transmit_duty_cycle",           MAX_TRANSMIT_DUTY_CYCLE)
    if not (0 < MAX_TRANSMIT_DUTY_CYCLE <= 1.0):
        print("Config [radio] max_transmit_duty_cycle: must be > 0 and <= 1.0", file=sys.stderr)
        sys.exit(1)
    COOLDOWN_SCALE_FACTOR             = _float("radio", "cooldown_scale_factor",             COOLDOWN_SCALE_FACTOR)
    ESTIMATED_FLIGHT_DURATION_MINUTES = max(1.0, _float("radio", "estimated_flight_duration_minutes", ESTIMATED_FLIGHT_DURATION_MINUTES))
    ESTIMATED_FREEFALL_MINUTES        = max(0.0, _float("radio", "estimated_freefall_minutes", ESTIMATED_FREEFALL_MINUTES))
    ESTIMATED_PCB_HEAT_TRANSFER_COEFFICIENT = max(0.01, _float("radio", "estimated_pcb_heat_transfer_coefficient", ESTIMATED_PCB_HEAT_TRANSFER_COEFFICIENT))
    ESTIMATED_AIR_HEAT_TRANSFER_COEFFICIENT = max(0.01, _float("radio", "estimated_air_heat_transfer_coefficient", ESTIMATED_AIR_HEAT_TRANSFER_COEFFICIENT))
    ESTIMATED_MIN_AIR_DENSITY_FACTOR = max(0.05, min(1.0, _float("radio", "estimated_min_air_density_factor", ESTIMATED_MIN_AIR_DENSITY_FACTOR)))
    ESTIMATED_EFFECTIVE_THERMAL_AREA_M2 = max(1e-5, _float("radio", "estimated_effective_thermal_area_m2", ESTIMATED_EFFECTIVE_THERMAL_AREA_M2))
    ESTIMATED_TX_HEAT_POWER_LOW_W     = max(0.05, _float("radio", "estimated_tx_heat_power_low_w", ESTIMATED_TX_HEAT_POWER_LOW_W))
    ESTIMATED_TX_HEAT_POWER_HIGH_W    = max(0.05, _float("radio", "estimated_tx_heat_power_high_w", ESTIMATED_TX_HEAT_POWER_HIGH_W))
    ESTIMATED_COOLDOWN_SAFETY_FACTOR  = max(0.1, _float("radio", "estimated_cooldown_safety_factor", ESTIMATED_COOLDOWN_SAFETY_FACTOR))
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
    SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_OPACITY = _int("overlay", "callsign_background_opacity", SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_OPACITY)
    OVERLAY_TEXT_OVERRIDE = _str("overlay", "custom_text", OVERLAY_TEXT_OVERRIDE).strip()

    # [mmsstv]
    lib_path = _str("mmsstv", "lib_path", "").strip()
    if lib_path:
        os.environ[MMSSTV_LIBRARY_ENV_VAR] = lib_path
    if _bool("mmsstv", "disable", False):
        os.environ[MMSSTV_DISABLE_ENV_VAR] = "1"

    # [alsa]
    ALSA_AUDIO_DEVICE             = _str ("alsa", "playback_device",         ALSA_AUDIO_DEVICE).strip()
    ALSA_MIXER_DEVICE             = _str ("alsa", "mixer_device",            ALSA_MIXER_DEVICE).strip()
    ALSA_MIXER_CONTROL            = _str ("alsa", "mixer_control",           ALSA_MIXER_CONTROL).strip()
    ALSA_TARGET_VOLUME_PERCENT    = max(0, min(100, _int ("alsa", "target_volume_percent",   ALSA_TARGET_VOLUME_PERCENT)))
    ALSA_MAX_SAFE_VOLUME_PERCENT  = max(0, min(100, _int ("alsa", "max_safe_volume_percent", ALSA_MAX_SAFE_VOLUME_PERCENT)))
    ALSA_ENFORCE_VOLUME           = _bool("alsa", "enforce_volume",          ALSA_ENFORCE_VOLUME)
    APLAY_TIMEOUT_SECONDS         = max(10, _int("alsa", "aplay_timeout_seconds",        APLAY_TIMEOUT_SECONDS))
    APLAY_TIMEOUT_MARGIN_SECONDS  = max(10, _int("alsa", "aplay_timeout_margin_seconds", APLAY_TIMEOUT_MARGIN_SECONDS))

    # [gps]
    GPS_ENABLED       = _bool("gps", "enable",  GPS_ENABLED)
    GPS_DEVICE        = _str ("gps", "device",  GPS_DEVICE)
    GPS_BAUD          = _int ("gps", "baud",    GPS_BAUD)
    units = _str("gps", "units", GPS_ALTITUDE_UNITS).strip().lower()
    if units not in ("m", "ft"):
        print(f"Config [gps] units: invalid value '{units}'. Valid: m, ft", file=sys.stderr)
        sys.exit(1)
    GPS_ALTITUDE_UNITS = units

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


# ---------------------------------------------------------------------------
# Radio band helpers
# ---------------------------------------------------------------------------

def get_active_ptt_pins() -> List[int]:
    """Return the PTT GPIO pin(s) for the selected radio band."""
    if ACTIVE_RADIO_BAND == "uhf":
        return [DRA818_UHF_PTT_PIN]
    elif ACTIVE_RADIO_BAND == "both":
        return [DRA818_PTT_PIN, DRA818_UHF_PTT_PIN]
    else:  # "vhf" (default)
        return [DRA818_PTT_PIN]


# ---------------------------------------------------------------------------
# GPS helpers  (requires pyserial: pip3 install pyserial)
# ---------------------------------------------------------------------------

def latlon_to_maidenhead(lat: float, lon: float, precision: int = 6) -> str:
    """Convert decimal lat/lon to Maidenhead grid locator (4 or 6 characters).

    Examples:
        latlon_to_maidenhead(44.97, -93.27)  ->  'EN34mb'   (Minneapolis area)
        latlon_to_maidenhead(41.88, -87.63)  ->  'EN61sl'   (Chicago area)
    """
    lon_adj = lon + 180.0
    lat_adj = lat + 90.0
    field_lon = int(lon_adj / 20)
    field_lat = int(lat_adj / 10)
    sq_lon = int((lon_adj % 20) / 2)
    sq_lat = int(lat_adj % 10)
    locator = f"{chr(65 + field_lon)}{chr(65 + field_lat)}{sq_lon}{sq_lat}"
    if precision >= 6:
        sub_lon = int((lon_adj % 2) * 12)
        sub_lat = int((lat_adj % 1) * 24)
        locator += f"{chr(97 + sub_lon)}{chr(97 + sub_lat)}"
    return locator


def read_gps_fix() -> Optional[Tuple[float, float, Optional[float]]]:
    """Poll the serial GPS device for one valid NMEA GGA fix.

    Returns (latitude, longitude, altitude_m) or None on failure/timeout.
    Parses $GPGGA / $GNGGA sentences (multi-constellation modules use GNGGA).
    Requires pyserial: pip3 install pyserial
    """
    if not GPS_ENABLED:
        return None
    if not _SERIAL_AVAILABLE:
        log("GPS: pyserial not available — install with: pip3 install pyserial")
        return None
    try:
        with _serial.Serial(GPS_DEVICE, GPS_BAUD, timeout=GPS_TIMEOUT_SECONDS) as port:
            deadline = time.monotonic() + GPS_TIMEOUT_SECONDS
            while time.monotonic() < deadline:
                try:
                    raw = port.readline()
                    line = raw.decode("ascii", errors="ignore").strip()
                except Exception:
                    continue
                if not line.startswith("$"):
                    continue
                # Strip checksum (*HH) before splitting
                parts = line.split("*")[0].split(",")
                sentence = parts[0]
                # GGA: fix data including altitude
                if sentence in ("$GPGGA", "$GNGGA") and len(parts) >= 10:
                    fix_quality = parts[6]
                    if fix_quality not in ("1", "2", "3", "4", "5", "6"):
                        continue  # no valid fix yet
                    try:
                        lat_raw = float(parts[2])
                        lat_hemi = parts[3]
                        lon_raw = float(parts[4])
                        lon_hemi = parts[5]
                        alt_str = parts[9]
                        # NMEA latitude is ddmm.mmmm — convert to decimal degrees
                        lat_deg = int(lat_raw / 100)
                        lat = lat_deg + (lat_raw - lat_deg * 100) / 60.0
                        if lat_hemi == "S":
                            lat = -lat
                        lon_deg = int(lon_raw / 100)
                        lon = lon_deg + (lon_raw - lon_deg * 100) / 60.0
                        if lon_hemi == "W":
                            lon = -lon
                        alt = float(alt_str) if alt_str else None
                        log_debug(f"GPS: fix lat={lat:.5f} lon={lon:.5f} alt={alt}m")
                        return lat, lon, alt
                    except (ValueError, IndexError):
                        continue
    except Exception as exc:
        log(f"GPS: serial read error ({GPS_DEVICE}): {exc}")
    return None


def build_gps_overlay_text() -> Optional[str]:
    """Return a compact GPS overlay string (grid + altitude), or None if no fix."""
    fix = read_gps_fix()
    if fix is None:
        return None
    lat, lon, alt = fix
    grid = latlon_to_maidenhead(lat, lon, precision=6)
    parts = [grid]
    if alt is not None:
        if GPS_ALTITUDE_UNITS == "ft":
            alt_display = f"{int(alt * 3.28084)}ft"
        else:
            alt_display = f"{int(alt)}m"
        parts.append(alt_display)
    return "  ".join(parts)


def log_debug(message: str):
    logger.debug(message)


def log_section(title: str, width: int = 56):
    bar = "=" * width
    log(bar)
    log(f"  {title}")
    log(bar)


LOG_FRAME_WIDTH = 72


def log_kv(label: str, value, label_width: int = 12):
    log(f"  {label:<{label_width}}: {value}")


def log_stage_header(title: str, details: Optional[List[Tuple[str, object]]] = None, width: int = LOG_FRAME_WIDTH):
    bar = "=" * width
    log(bar)
    log(f"  {title}")
    log(bar)
    if details:
        for label, value in details:
            if value is None:
                continue
            log_kv(label, value)
        log("-" * width)


def log_stage_footer(status: str, details: Optional[List[Tuple[str, object]]] = None, width: int = LOG_FRAME_WIDTH):
    bar = "-" * width
    log(bar)
    log(f"  Result      : {status}")
    if details:
        for label, value in details:
            if value is None:
                continue
            log_kv(label, value)
    log(bar)


def describe_alsa_guardrails() -> str:
    if not ALSA_ENFORCE_VOLUME:
        return "disabled"
    mixer_target = ALSA_MIXER_DEVICE or "<default>"
    control = ALSA_MIXER_CONTROL or "auto"
    return (
        f"{mixer_target}/{control}  target={ALSA_TARGET_VOLUME_PERCENT}%  "
        f"safe<={ALSA_MAX_SAFE_VOLUME_PERCENT}%"
    )


def describe_radio_selection() -> str:
    if ACTIVE_RADIO_BAND == "uhf":
        return f"UHF only  (DRA818U via GPIO{DRA818_UHF_PTT_PIN}, active-LOW)"
    if ACTIVE_RADIO_BAND == "both":
        return (
            f"VHF+UHF  (DRA818V via GPIO{DRA818_PTT_PIN}, "
            f"DRA818U via GPIO{DRA818_UHF_PTT_PIN}, active-LOW)"
        )
    return f"VHF only  (DRA818V via GPIO{DRA818_PTT_PIN}, active-LOW)"


def describe_radio_control_states() -> List[Tuple[str, str]]:
    if TX_POWER_LEVEL == "high":
        power_state = f"GPIO{DRA818_POWER_LEVEL_PIN}  default HIGH  (H = 1.0 W)"
    else:
        power_state = f"GPIO{DRA818_POWER_LEVEL_PIN}  default LOW   (L = 0.5 W low-power)"

    if PD_IDLE_MODE == "sleep":
        pd_state = (
            f"GPIO{DRA818_POWER_DOWN_PIN}  OUTPUT HIGH during TX; "
            "OUTPUT LOW after TX (Pi-enforced sleep)"
        )
    else:
        pd_state = (
            f"GPIO{DRA818_POWER_DOWN_PIN}  OUTPUT HIGH during TX; "
            "INPUT at idle (released to Feather M0)"
        )

    return [
        ("radio_sel", describe_radio_selection()),
        ("pd", pd_state),
        ("h_l", power_state),
        ("pd_mode", PD_IDLE_MODE),
        ("tx_power", TX_POWER_LEVEL),
    ]


def describe_camera_capture() -> List[Tuple[str, str]]:
    return [
        ("camera", f"{CAMERA_NAME}  ({CAMERA_MODEL})"),
        ("sensor", CAMERA_SENSOR_CLASS),
        ("native_res", CAMERA_NATIVE_RESOLUTION),
        ("capture_res", CAMERA_CAPTURE_RESOLUTION),
    ]


def describe_mode_geometry(profile: Optional[ModeProfile]) -> str:
    if profile is None:
        return "unknown"
    if profile.image_height is not None:
        return f"{profile.image_width}x{profile.image_height} px nominal raster"
    return f"{profile.image_width}px wide (height not tracked)"


def format_overlay_timestamp(capture_time: datetime, is_test: bool = False) -> str:
    utc_time = capture_time.astimezone(timezone.utc)
    timestamp = utc_time.strftime("%Y-%m-%d %H:%MZ")
    if is_test:
        return f"{timestamp} TEST"
    return timestamp


def build_overlay_text(mode_name: str, timestamp_message: Optional[str], gps_text: Optional[str] = None) -> str:
    custom_body = OVERLAY_TEXT_OVERRIDE.strip()
    if custom_body:
        return custom_body

    parts: List[str] = [mode_name.upper()]
    if timestamp_message:
        parts.append(timestamp_message)
    return " ".join(parts)


def describe_overlay(mode_name: str, timestamp_message: Optional[str], gps_text: Optional[str] = None) -> str:
    overlay_body = build_overlay_text(mode_name, timestamp_message, gps_text=gps_text)
    if STATION_CALLSIGN:
        if _use_compact_overlay_layout(mode_name):
            compact_size = max(8, _scaled_overlay_size(mode_name, SLOWFRAME_TIMESTAMP_OVERLAY_SIZE, minimum_size=9) - 2)
            budget = _estimate_overlay_char_budget(mode_name, compact_size)
            compact_text = _build_compact_overlay_text(mode_name, timestamp_message, gps_text, budget)
            return f"compact-merged  text='{compact_text}'"
        return f"merged  text='{STATION_CALLSIGN}  {overlay_body}'"
    return "none"


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
        image_width/height - Nominal raster geometry for the selected SSTV mode.
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
  --explain mmsstv   MMSSTV library modes (robot8bw, robot12bw, pd50, pd90, pd120, pd160, pd180, pd240, pd290, fax480)
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
    Enabled by default.  Format: YYYY-MM-DD HH:MM:SSZ (UTC), with "TEST"
    appended in test mode.

  Configuration constants (edit at top of script):
    SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY        = True/False
    SLOWFRAME_TIMESTAMP_OVERLAY_SIZE          = font size (default: 11)
    SLOWFRAME_TIMESTAMP_OVERLAY_POSITION      = top-left (default)
    SLOWFRAME_TIMESTAMP_OVERLAY_COLOR         = white (default)
    SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR    = black (default)
    SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY  = 0-100 (default: 50)

  Valid positions:
    top-left, top-right, bottom-left, bottom-right, top, bottom, left, right, center

  Colors: named CSS colors (white, black, yellow, red, ...) or hex (#RRGGBB)

Callsign overlay (SLOWFRAME_ENABLE_CALLSIGN_OVERLAY)
    Prints the station callsign prefix as part of the merged default overlay:
        CALLSIGN MODE DATE TIME
    Callsign is required for encode/test/mission workflows.

  Configuration constants:
    STATION_CALLSIGN                          = "" (set via --callsign / config)
    SLOWFRAME_ENABLE_CALLSIGN_OVERLAY         = True when callsign is present
    SLOWFRAME_CALLSIGN_OVERLAY_SIZE           = font size (default: 14)
    SLOWFRAME_CALLSIGN_OVERLAY_POSITION       = top-right (default)
    SLOWFRAME_CALLSIGN_OVERLAY_COLOR          = white (default)
    SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_COLOR = black (default)
    SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND_OPACITY = 0-100 (default: 50)

Merging behaviour
  SlowFrame places all -T overlays at the same rendered position regardless of
  separate pos= values.  When both timestamp and callsign overlays are enabled,
    the script merges them into a single overlay string that also includes the
    SSTV mode and optional GPS text:
        "W1AW-11  R36  2026-04-12 00:25:05Z  EN34mb 1234m"
  This prevents the two strings from stacking on top of each other.

Font size scaling
  Font sizes are scaled proportionally to the mode's image_width, anchored
  320 px.  A size=11 at 320 px becomes size=22 at 640 px, keeping the overlay
  legible at both low- and high-resolution modes.

EXAMPLES
  Test with callsign overlay:
    python3 pi_sstv.py --test r36 --callsign W1AW-11 --no-tx

  Test high-resolution mode with callsign (overlay scales up automatically):
    python3 pi_sstv.py --test pd120 --callsign W1AW-11 --no-tx

EXTERNAL COMMANDS
  slowframe  (overlay flags)
                                The -T flag accepts a pipe-delimited overlay descriptor string. You can
                experiment with overlay rendering directly:
      slowframe -i photo.jpg -o out.wav -p r36 \
                                -T "W1AW-11  R36  2026-04-12 00:25:05Z|size=11|color=white|bg=black|bgbar=true|bgbar-margin=4|pos=top-left"
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
    robot8bw   robot12bw   pd50   pd90   pd120   pd160   pd180   pd240   pd290   fax480

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
        pd180     -> m1       pd240     -> m1
        pd290     -> pd180    fax480    -> m1

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
    Name         TX (s)   Cooldown (s)   WxH       Description
    bw24             24            105   320x120   Fast monochrome, low duty-cycle updates
    r36              36            135   320x240   Fast native color, regular updates
    m2               58            210   320x256   Balanced, strong compatibility
    s2               71            255   320x256   Scottie 2, good compatibility
    r72              72            255   320x240   Higher-quality Robot color
    s1              110            420   320x256   Scottie 1, best native quality
    m1              114            420   320x256   Martin 1, high-quality, less frequent
    sdx             269            780   320x256   Scottie DX, long-duration high-detail native mode

Curated MMSSTV library modes (require libsstv_encoder.so; additional modes may be auto-profiled from slowframe -L):
    Name         TX (s)   Cooldown (s)   WxH       Fallback   Description
    robot8bw          8             75   160x120   bw24       Ultra-fast monochrome status frame
    robot12bw        12             75   160x120   bw24       Very fast monochrome
    pd50             50            210   320x256   m2         Fast PD color
    pd90             90            300   320x256   r36        Popular fast color
    pd120           120            420   640x496   m1         Higher-quality, larger image
    pd160           160            540   512x400   m1         Slower quality mode
    pd180           180            600   640x496   m1         High-detail mission snapshot
    fax480          180            600   512x480   m1         High-detail, test windows
    pd240           240            720   640x496   m1         Very high quality PD mode
    pd290           290            840   800x616   pd180      Highest quality PD mode

Mode geometry
    Each mode encodes at a nominal raster width and height. SlowFrame scales the
    captured image to this raster before encoding. The aspect handling (--aspect)
    controls how the camera's 4:3 frame is fitted to the mode canvas.

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
    hab_climb    robot8bw -> robot12bw -> bw24 -> r36 -> robot12bw -> r36
               Maximum update rate.  Monochrome-heavy to keep cooldowns short
               during the steepest part of the climb.

    hab_rapid    robot12bw -> r36 -> r36 -> robot12bw -> r36 -> pd50
               Fast color bursts.  Best for upper ascent and the release phase
               where update rate still matters but color shots are welcome.

    hab_cruise   robot12bw -> r36 -> m2 -> pd90 -> s2 -> r72 -> m1 -> pd120  (default)
               Mixes fast status frames with progressively higher-quality
               images.  Good all-round choice for the full flight envelope.

    hab_float    r36 -> pd90 -> robot12bw -> pd120 -> robot12bw -> pd180 -> pd240 -> pd290
               Quality-first.  Anchored by PD modes for float altitude or
               slow-drift science windows.  Requires MMSSTV library; falls
               back gracefully if unavailable.

Selecting a preset
  python3 pi_sstv.py --schedule hab_rapid
  python3 pi_sstv.py --schedule hab_float --mmsstv-lib /path/to/libsstv_encoder.so

Custom schedule profiles in config
    Config files may define additional profiles with sections named:
        [schedule_profile <name>]

    Example:
        [schedule_profile custom]
        modes = robot12bw, r36, pd50, pd90
        description = Operator-defined mixed rapid schedule.
        unavailable_mode_fallback = r36

    Then select it with:
        [mission]
        schedule = custom

Unavailable-mode fallback defaults
    If a scheduled mode is unavailable, the script first follows that mode's
    curated fallback chain. If that chain is exhausted, it tries:
        1. The active schedule profile's unavailable_mode_fallback, when set.
        2. The global [mission] unavailable_mode_fallback.
        3. A final built-in safety fallback to r36, then the first available mode.

Transmission gating
  A transmission only proceeds when ALL of the following are satisfied:
    1. At least --min-captures capture cycles have elapsed since the last TX
       (default: {min_captures}).
        2. The selected cooldown method has expired (fixed, adaptive_dutycycle,
             adaptive_avg_dutycycle, or estimated), multiplied by --cooldown-scale.

Cooldown methods
    --cooldown-method fixed
            Use --fixed-cooldown-seconds between every TX regardless of mode.

    --cooldown-method adaptive_dutycycle
            Uses the previous TX duration and --duty-cycle to compute cooldown:
            cooldown = tx_duration * ((1-duty)/duty)

    --cooldown-method adaptive_avg_dutycycle
            Uses each mode's TX-duration share of the current schedule block and
            --duty-cycle to distribute cooldown across the schedule profile.

    --cooldown-method estimated
            Starts with adaptive duty cooldown and applies a thermal multiplier based
            on TX power, PCB/air heat-transfer assumptions, and altitude phase.
            Altitude phase is estimated using --estimated-flight-minutes and
            --estimated-freefall-minutes.

Duty-cycle protection
    --duty-cycle FRACTION   Target TX fraction used by adaptive cooldown models.
                                                    Default: {duty_pct_raw} ({duty_pct}%).
    --cooldown-scale FACTOR Multiply calculated cooldown by this factor.
                          1.0 = nominal, 0.75 = aggressive, 1.5 = conservative.
    --fixed-cooldown-seconds SECS  Static cooldown for fixed method.
    --estimated-flight-minutes MIN Flight duration estimate for thermal method.
    --estimated-freefall-minutes MIN Final descent window for thermal method.
    --min-captures N        Optional hard minimum captures between any two
                                                    transmissions. Default: {min_captures}.

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
  DRA818_POWER_LEVEL_PIN = {hl}   (physical pin 15, default LOW = L / 0.5 W)
  Audio PWM output pins  = GPIO{al} (left) / GPIO{ar} (right)

Transmission sequence
  1. PD -> HIGH    Bring the DRA818 out of power-down (radio wake)
     wait  RADIO_WAKE_DELAY_SECONDS ({wake}s) for the module to stabilise
  2. PTT -> LOW    Key the transmitter
     wait  PTT_KEY_DELAY_SECONDS ({ptt_delay}s) before audio starts
  3. aplay         Stream the WAV file to the audio device
  4. wait          POST_PLAYBACK_DELAY_SECONDS ({post}s) after playback ends
  5. PTT -> HIGH   Unkey the transmitter
  6. PD idle       Apply configured idle policy:
                                     release -> INPUT (Feather M0 owns final state)
                                     sleep   -> OUTPUT LOW (Pi-enforced sleep)

PD idle behaviour
    Default policy is `release`: the Pi sets PD to INPUT at idle because PD is
    shared with the Feather M0, and the Feather determines final line state.
    Optional policy `sleep` drives PD LOW after TX/PTT for Pi-enforced idle
    power-save mode.

TX power and PD policy flags
    --tx-power low|high
            Set DRA818 H/L line policy. low = ~0.5 W (L), high = ~1.0 W (H).
            Applied during GPIO setup and re-applied before each TX/PTT key event.

    --pd-idle release|sleep
            Controls PD line behavior after TX/PTT:
                release = PD -> INPUT (shared control back to Feather M0)
                sleep   = PD -> OUTPUT LOW (Pi-enforced idle power-save)

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
  --tx-power     Choose TX power level policy (low/high).
  --pd-idle      Choose PD idle policy after TX/PTT (release/sleep).
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

    pinctrl  (GPIO inspection)
    Inspect or drive GPIO lines directly without running the full script:
            pinctrl get {ptt}              Read PTT pin state
            pinctrl get {pd}               Read power-down pin state
            pinctrl set {ptt} op dh        Drive PTT HIGH (idle/safe)
            pinctrl help

    pinctl  (GPIO inspection command)
        Alternative GPIO inspection command spelling:
            pinctl get                     Dump state of all GPIO pins

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
    Signal   BCM GPIO   Physical Pin   Pi-Driven State            Notes
    PD       GPIO 4     Pin 7          HIGH during TX, INPUT idle Shared with Feather M0
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
    pinctrl  (read and set GPIO lines from the command line)
    Inspect or manually drive the DRA818 control pins to verify wiring
    without running the full script:
            pinctrl get                    Dump state of all GPIO pins
            pinctrl get {ptt}              Read PTT pin (should be 1 = HIGH when idle)
            pinctrl get {pd}               Read power-down pin (depends on --pd-idle mode)
            pinctrl set {ptt} op dh        Drive PTT HIGH safely
            pinctrl set {pd} op dl         Drive PD LOW (radio off)
            pinctrl help

    pinctl  (GPIO inspection command)
        Dump GPIO pin states directly:
            pinctl get

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
    Major stages are emitted as framed sections with key/value summaries and
    explicit result footers. Typical headings include:
        Runtime Startup
        SlowFrame Capability Discovery
        Stage 1/3  Image Capture
        Stage 2/3  SSTV Encode
        Stage 3/3  Radio TX

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

HELP_TOPIC_ORDER = [
    "capture",
    "encode",
    "overlay",
    "mmsstv",
    "modes",
    "schedule",
    "tx",
    "gpio",
    "logging",
]

HELP_TOPIC_SUMMARIES = {
    "capture": "Camera acquisition tuning (metering, exposure, AWB, quality)",
    "encode": "SlowFrame encoding settings (mode, format, sample rate, aspect)",
    "overlay": "Timestamp/callsign/GPS overlay behavior and SlowFrame -T usage",
    "mmsstv": "Library detection, enable/disable, and fallback behavior",
    "modes": "Complete SSTV mode table (durations, cooldowns, geometry, fallback)",
    "schedule": "Mission presets, cooldown scaling, duty-cycle gating",
    "tx": "Radio GPIO sequence, timing, and aplay transmit path",
    "gpio": "Pin mapping and wiring reference",
    "logging": "Log levels, framed stage output, and log file options",
}

EXPLAIN_TOPIC_GUIDES = {
    "capture": {
        "when": "Tune camera behavior for changing light, motion blur, and image quality.",
        "flags": ["--test MODE", "--test-image PATH", "--no-tx"],
        "failures": [
            "rpicam-still unavailable or camera not detected",
            "captured file missing/empty (falls back to test image)",
        ],
        "checklist": [
            "Confirm camera ribbon/enablement",
            "Run --test r36 --no-tx",
            "Verify captured JPG size and clarity",
        ],
        "commands": [
            "python3 pi_sstv.py --test r36 --no-tx",
            "python3 pi_sstv.py --test r36 --test-image /home/pi-user/photo.jpg --no-tx",
        ],
    },
    "encode": {
        "when": "Adjust SlowFrame mode/format/aspect and validate generated audio artifacts.",
        "flags": ["--test MODE", "--format FMT", "--sample-rate HZ", "--aspect MODE"],
        "failures": [
            "slowframe binary missing or not executable",
            "invalid mode or unsupported output format",
        ],
        "checklist": [
            "Run encode-only test with --no-tx",
            "Confirm WAV/AIFF/OGG artifact exists",
            "Check mode duration and geometry in logs",
        ],
        "commands": [
            "python3 pi_sstv.py --test pd90 --no-tx",
            "python3 pi_sstv.py --test m1 --format ogg --no-tx",
        ],
    },
    "overlay": {
        "when": "Control timestamp/callsign/GPS text overlays baked into transmitted images.",
        "flags": ["--callsign CALL", "--test MODE", "--no-tx"],
        "failures": [
            "overly dense overlay text reduces readability",
            "unexpected overlap when assuming separate overlay positions",
        ],
        "checklist": [
            "Set callsign when needed",
            "Run a no-TX test and inspect resulting image",
            "Verify timestamp format and mode token visibility",
        ],
        "commands": [
            "python3 pi_sstv.py --test r36 --callsign W1AW-11 --no-tx",
            "python3 pi_sstv.py --test pd120 --callsign W1AW-11 --no-tx",
        ],
    },
    "mmsstv": {
        "when": "Enable high-detail MMSSTV modes and understand fallback behavior.",
        "flags": ["--mmsstv-lib PATH", "--no-mmsstv", "--list-modes"],
        "failures": [
            "library not detected by SlowFrame -M",
            "versioned .so path used instead of unversioned symlink",
        ],
        "checklist": [
            "Set MMSSTV_LIB_PATH or pass --mmsstv-lib",
            "Run --list-modes and confirm MMSSTV modes present",
            "Run a no-TX PD mode test",
        ],
        "commands": [
            "python3 pi_sstv.py --mmsstv-lib /path/to/libsstv_encoder.so --test pd90 --no-tx",
            "python3 pi_sstv.py --no-mmsstv --test pd90 --no-tx",
        ],
    },
    "modes": {
        "when": "Choose SSTV modes based on airtime, cooldown, and image geometry.",
        "flags": ["--list-modes", "--test MODE", "--no-mmsstv"],
        "failures": [
            "requested mode unavailable and falls back",
            "mode choice exceeds thermal/duty-cycle budget",
        ],
        "checklist": [
            "Inspect mode table",
            "Validate preferred mode with --test",
            "Confirm fallback chain is acceptable",
        ],
        "commands": [
            "python3 pi_sstv.py --list-modes",
            "python3 pi_sstv.py --test r36 --no-tx",
        ],
    },
    "schedule": {
        "when": "Select mission transmit cadence and airtime profile over the flight.",
        "flags": [
            "--schedule PRESET",
            "--cooldown-method METHOD",
            "--fixed-cooldown-seconds SECS",
            "--duty-cycle FRACTION",
            "--cooldown-scale FACTOR",
            "--estimated-flight-minutes MIN",
            "--estimated-freefall-minutes MIN",
            "--min-captures N",
        ],
        "failures": [
            "TX skipped due to cooldown model or min-captures gating",
            "schedule relies on MMSSTV modes when library is unavailable",
        ],
        "checklist": [
            "Run --list-schedules",
            "Pick preset for ascent/cruise/float phase",
            "Select cooldown method (fixed/adaptive/estimated)",
            "Tune duty-cycle and cooldown-scale before flight",
        ],
        "commands": [
            "python3 pi_sstv.py --list-schedules",
            "python3 pi_sstv.py --schedule hab_rapid --total 200 --interval 8",
        ],
    },
    "tx": {
        "when": "Validate radio keying/audio playback sequence and TX timing behavior.",
        "flags": ["--no-tx", "--ptt-test [SECONDS]", "--radio BAND"],
        "failures": [
            "aplay device failure or timeout",
            "GPIO wiring/polarity mismatch for PD/PTT",
        ],
        "checklist": [
            "Start with --ptt-test",
            "Run --test MODE --no-tx",
            "Run full --test MODE with radio connected",
        ],
        "commands": [
            "python3 pi_sstv.py --ptt-test 0.5",
            "python3 pi_sstv.py --test r36",
        ],
    },
    "gpio": {
        "when": "Reference pin mapping and verify physical wiring against script expectations.",
        "flags": ["--ptt-test", "--radio BAND"],
        "failures": [
            "wrong BCM-to-physical pin mapping",
            "shared PD/PTT line contention with other controllers",
        ],
        "checklist": [
            "Cross-check BCM and physical pins",
            "Validate idle vs keyed states",
            "Use pinctrl for line-level verification",
        ],
        "commands": [
            "python3 pi_sstv.py --ptt-test",
            "pinctrl get 27",
        ],
    },
    "logging": {
        "when": "Control verbosity and where operational evidence is recorded.",
        "flags": ["--debug", "--log-file PATH", "--quiet-log-file PATH"],
        "failures": [
            "insufficient detail when debug is off",
            "missing stdout output when quiet-log-file is enabled",
        ],
        "checklist": [
            "Choose stdout + file or file-only mode",
            "Enable --debug for troubleshooting runs",
            "Review framed stage PASS/FAIL sections",
        ],
        "commands": [
            "python3 pi_sstv.py --debug --log-file /home/pi-user/mission.log",
            "python3 pi_sstv.py --quiet-log-file /home/pi-user/mission.log",
        ],
    },
}


def _render_structured_explain_header(topic: str) -> str:
    guide = EXPLAIN_TOPIC_GUIDES.get(topic)
    if not guide:
        return ""

    lines = [
        "OPERATOR GUIDE",
        "==============",
        f"Topic      : {topic}",
        f"Summary    : {HELP_TOPIC_SUMMARIES.get(topic, '')}",
        f"When to use: {guide.get('when', '')}",
        "",
        "Key flags",
        "---------",
    ]

    for flag in guide.get("flags", []):
        lines.append(f"  - {flag}")

    lines.extend(["", "Failure modes", "-------------"])
    for failure in guide.get("failures", []):
        lines.append(f"  - {failure}")

    lines.extend(["", "Operator checklist", "------------------"])
    for step in guide.get("checklist", []):
        lines.append(f"  - {step}")

    lines.extend(["", "Quick commands", "--------------"])
    for cmd in guide.get("commands", []):
        lines.append(f"  {cmd}")

    lines.append("\n" + "=" * 72 + "\n")
    return "\n".join(lines)


def print_help_topics():
    print(
        "HELP TOPICS\n"
        "===========\n"
        "Use: python3 pi_sstv.py --explain <topic>\n"
    )
    for topic in HELP_TOPIC_ORDER:
        summary = HELP_TOPIC_SUMMARIES.get(topic, "")
        print(f"  {topic:<10} {summary}")

    alias_keys = sorted(HELP_TOPIC_ALIASES.keys())
    print("\nAliases:")
    print("  " + ", ".join(alias_keys))


def print_help_quick():
    print(
        "OPERATOR QUICK START\n"
        "====================\n"
        "1) Create config template\n"
        f"   python3 pi_sstv.py --generate-config\n"
        "\n"
        "2) Safe bench test (capture + encode, no RF TX)\n"
        "   python3 pi_sstv.py --test r36 --no-tx\n"
        "\n"
        "3) Start mission from config\n"
        "   python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n"
        "\n"
        "4) Explore documentation by topic\n"
        "   python3 pi_sstv.py --help-topics\n"
        "   python3 pi_sstv.py --explain schedule\n"
        "\n"
        "5) Inspect supported modes and schedules\n"
        "   python3 pi_sstv.py --list-modes\n"
        "   python3 pi_sstv.py --list-schedules\n"
    )


def print_help_examples():
    print(
        "EXAMPLE COOKBOOK\n"
        "================\n"
        "Mission runs (config + presets)\n"
        "  python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n"
        "  python3 pi_sstv.py --schedule hab_rapid --callsign W1AW-11 --total 200 --interval 8\n"
        "  python3 pi_sstv.py --schedule hab_float --mmsstv-lib /path/to/libsstv_encoder.so\n"
        "\n"
        "Radio-aware mission examples\n"
        "  # VHF only (default):\n"
        "  python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg --radio vhf\n"
        "  # UHF only:\n"
        "  python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg --radio uhf\n"
        "  # Dual-band keying (both PTT lines):\n"
        "  python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg --radio both\n"
        "\n"
        "Safety-first pipeline tests\n"
        "  python3 pi_sstv.py --test r36\n"
        "  python3 pi_sstv.py --test pd90 --no-tx\n"
        "  python3 pi_sstv.py --test m1 --test-image /home/pi-user/photo.jpg --no-tx\n"
        "  python3 pi_sstv.py --test r36 --callsign W1AW-11 --no-tx --debug\n"
        "\n"
        "Radio / GPIO checks\n"
        "  python3 pi_sstv.py --ptt-test\n"
        "  python3 pi_sstv.py --ptt-test 0.5\n"
        "  python3 pi_sstv.py --ptt-test 1.0 --radio both\n"
        "\n"
        "ALSA and playback path validation\n"
        "  python3 pi_sstv.py --alsa-volume-check\n"
        "  python3 pi_sstv.py --alsa-playback-device plughw:Headphones,0 --test r36 --no-tx\n"
        "\n"
        "Duty-cycle / thermal tuning examples\n"
        "  python3 pi_sstv.py --schedule hab_cruise --cooldown-scale 1.5 --duty-cycle 0.20\n"
        "  python3 pi_sstv.py --schedule hab_rapid --cooldown-scale 0.8 --min-captures 8\n"
        "\n"
        "GPS overlay examples\n"
        "  python3 pi_sstv.py --gps --gps-device /dev/serial0 --gps-baud 9600 --test r36 --no-tx\n"
        "  python3 pi_sstv.py --gps --gps-units ft --config /home/pi-user/pi_sstv.cfg\n"
        "\n"
        "Logging examples\n"
        "  python3 pi_sstv.py --debug --log-file /home/pi-user/mission.log --config /home/pi-user/pi_sstv.cfg\n"
        "  python3 pi_sstv.py --quiet-log-file /home/pi-user/mission.log --config /home/pi-user/pi_sstv.cfg\n"
        "\n"
        "Documentation\n"
        "  python3 pi_sstv.py --help-topics\n"
        "  python3 pi_sstv.py --help-flight\n"
        "  python3 pi_sstv.py --explain capture\n"
        "  python3 pi_sstv.py --explain mmsstv\n"
        "  python3 pi_sstv.py --explain schedule\n"
        "  python3 pi_sstv.py --explain tx\n"
    )


def print_help_flight():
    print(
        "FLIGHT PREFLIGHT CHECKLIST\n"
        "==========================\n"
        "Goal: verify hardware, software, and mission settings before arming TX.\n"
        "\n"
        "A) Hardware wiring\n"
        "  [ ] Camera ribbon seated and camera detected\n"
        "  [ ] Audio LPF path connected to DRA818 mic input\n"
        "  [ ] PTT/PD/HL GPIO wiring matches script pin map\n"
        "  [ ] Antennas attached (VHF/UHF as configured)\n"
        "\n"
        "B) Required binaries and files\n"
        f"  [ ] SlowFrame binary present: {SLOWFRAME_BIN}\n"
        f"  [ ] rpicam-still present: {RPICAM_BIN}\n"
        "  [ ] Config file generated and reviewed\n"
        "\n"
        "C) Safety-first bench validation (no RF TX)\n"
        "  [ ] Run encode-only test:\n"
        "      python3 pi_sstv.py --test r36 --no-tx\n"
        "  [ ] Inspect image + WAV artifacts in output directory\n"
        "  [ ] Verify overlay text (callsign/timestamp/GPS) is readable\n"
        "\n"
        "D) Radio control validation\n"
        "  [ ] Run GPIO key test:\n"
        "      python3 pi_sstv.py --ptt-test 0.5\n"
        "  [ ] Confirm expected key/unkey behavior on radio hardware\n"
        "\n"
        "E) Audio path validation\n"
        "  [ ] Run ALSA guardrail/mixer check:\n"
        "      python3 pi_sstv.py --alsa-volume-check\n"
        "  [ ] Confirm mixer target is sane (typically <= 85%)\n"
        "\n"
        "F) MMSSTV capability (optional but recommended)\n"
        "  [ ] If using PD/Robot modes, validate library detection:\n"
        "      python3 pi_sstv.py --list-modes\n"
        "  [ ] If needed, set library path with --mmsstv-lib or MMSSTV_LIB_PATH\n"
        "\n"
        "G) Mission profile review\n"
        f"  [ ] Schedule preset reviewed (current default: {TRANSMIT_SCHEDULE_PROFILE})\n"
        f"  [ ] Capture cadence reviewed (interval={PIC_INTERVAL}s, total={PIC_TOTAL})\n"
        f"  [ ] Duty/cooldown reviewed (duty={int(MAX_TRANSMIT_DUTY_CYCLE * 100)}%, cooldown-scale={COOLDOWN_SCALE_FACTOR}x)\n"
        f"  [ ] Minimum captures between TX reviewed (min-captures={MIN_CAPTURES_BETWEEN_TRANSMISSIONS})\n"
        "\n"
        "H) Ready-to-launch commands\n"
        "  1) Generate/update config template:\n"
        "     python3 pi_sstv.py --generate-config\n"
        "  2) Final mission start from config:\n"
        "     python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n"
        "\n"
        "Related guides:\n"
        "  python3 pi_sstv.py --help-quick\n"
        "  python3 pi_sstv.py --help-examples\n"
        "  python3 pi_sstv.py --help-topics\n"
    )


def print_help_cli_overview():
    print(
        "pi_sstv.py - HamWing SSTV HAB payload controller\n"
        "=================================================\n"
        "\n"
        "Use this short help for day-to-day operations.\n"
        "For the complete argparse flag reference:  python3 pi_sstv.py --help-all\n"
        "\n"
        "Core workflows\n"
        "  Mission run\n"
        "    python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n"
        "  Bench test (no RF TX)\n"
        "    python3 pi_sstv.py --test r36 --no-tx\n"
        "  GPIO key test\n"
        "    python3 pi_sstv.py --ptt-test 0.5\n"
        "  ALSA validation\n"
        "    python3 pi_sstv.py --alsa-volume-check\n"
        "\n"
        "Most-used options\n"
        "  Mission: --config, --schedule, --total, --interval, --callsign\n"
        "  Safety : --no-tx, --tx-power, --pd-idle, --cooldown-method, --fixed-cooldown-seconds, --cooldown-scale, --duty-cycle, --min-captures\n"
        "  Encode : --test, --format, --sample-rate, --aspect\n"
        "  Audio  : --alsa-playback-device, --alsa-mixer-control\n"
        "  GPS    : --gps, --gps-device, --gps-baud, --gps-units\n"
        "\n"
        "Guided help\n"
        "  python3 pi_sstv.py --help-quick\n"
        "  python3 pi_sstv.py --help-flight\n"
        "  python3 pi_sstv.py --help-examples\n"
        "  python3 pi_sstv.py --help-topics\n"
        "  python3 pi_sstv.py --explain <topic>\n"
        "\n"
        "Reference lists\n"
        "  python3 pi_sstv.py --list-modes\n"
        "  python3 pi_sstv.py --list-schedules\n"
    )


def print_explain(topic: str):
    canonical = HELP_TOPIC_ALIASES.get(topic.lower(), topic.lower())
    text = HELP_TOPICS.get(canonical)
    if text is None:
        valid = sorted(set(HELP_TOPICS.keys()) | set(HELP_TOPIC_ALIASES.keys()))
        print(f"Unknown topic: '{topic}'")
        print(f"Available topics: {', '.join(valid)}")
        print("Tip: run 'python3 pi_sstv.py --help-topics' for a structured topic index.")
        sys.exit(1)
    # Substitute live script constants into topics that reference them.
    text = text.format(
        metering=RPICAM_METERING,
        exposure=RPICAM_EXPOSURE,
        awb=RPICAM_AWB,
        quality=RPICAM_QUALITY,
        min_captures=MIN_CAPTURES_BETWEEN_TRANSMISSIONS,
        cooldown_scale=COOLDOWN_SCALE_FACTOR,
        duty_pct=int(MAX_TRANSMIT_DUTY_CYCLE * 100),
        duty_pct_raw=MAX_TRANSMIT_DUTY_CYCLE,
        ptt=DRA818_PTT_PIN,
        pd=DRA818_POWER_DOWN_PIN,
        hl=DRA818_POWER_LEVEL_PIN,
        al=AUDIO_LEFT_PWM_PIN,
        ar=AUDIO_RIGHT_PWM_PIN,
        wake=RADIO_WAKE_DELAY_SECONDS,
        ptt_delay=PTT_KEY_DELAY_SECONDS,
        post=POST_PLAYBACK_DELAY_SECONDS,
    )
    header = _render_structured_explain_header(canonical)
    if header:
        print(header + text)
    else:
        print(text)


def parse_args(argv: Optional[List[str]] = None):
    description = (
        "pi_sstv.py - HamWing SSTV HAB payload controller\n\n"
        "Pipeline: capture (rpicam-still) -> encode (SlowFrame) -> transmit (aplay + GPIO)\n\n"
        "Primary workflows:\n"
        "  Mission run    : --config PATH   (or mission flags)\n"
        "  Single test    : --test MODE [--no-tx]\n"
        "  GPIO key test  : --ptt-test [SECONDS]\n"
        "  ALSA check     : --alsa-volume-check\n\n"
        "Guided help:\n"
        "  --help-quick      Operator-first startup guide\n"
        "  --help-flight     Preflight checklist before enabling TX\n"
        "  --help-examples   Scenario-based command cookbook\n"
        "  --help-topics     List all --explain topics with summaries\n"
        "  --explain TOPIC   Deep reference for one subsystem"
    )
    epilog = (
        "Common entry points:\n"
        "  python3 pi_sstv.py --generate-config\n"
        "  python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg\n"
        "  python3 pi_sstv.py --test r36 --no-tx\n"
        "  python3 pi_sstv.py --list-modes\n"
        "  python3 pi_sstv.py --list-schedules\n\n"
        "For guided navigation:\n"
        "  python3 pi_sstv.py --help-quick\n"
        "  python3 pi_sstv.py --help-flight\n"
        "  python3 pi_sstv.py --help-examples\n"
        "  python3 pi_sstv.py --help-topics\n"
    )

    parser = argparse.ArgumentParser(
        prog="pi_sstv.py",
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog,
    )

    # --- Pipeline mode flags ---
    mode_group = parser.add_argument_group(
        "OPERATOR WORKFLOWS",
        "Choose one info, test, or diagnostic workflow; omit all for a normal mission run.",
    )
    mode_group.add_argument(
        "--explain",
        metavar="TOPIC",
        default=None,
        help=(
            "Print structured operator guidance plus detailed reference for a pipeline topic and exit. "
            "Topics: capture, encode, overlay, mmsstv, modes, schedule, tx, gpio, logging. "
            "Aliases are accepted (e.g. camera, radio, lib, wiring). "
            "Example: python3 pi_sstv.py --explain mmsstv"
        ),
    )
    mode_group.add_argument(
        "--help-topics",
        action="store_true",
        help="List all --explain topics with one-line summaries, then exit.",
    )
    mode_group.add_argument(
        "--help-quick",
        action="store_true",
        help="Print a short operator startup guide (config -> bench test -> mission), then exit.",
    )
    mode_group.add_argument(
        "--help-flight",
        action="store_true",
        help="Print a full preflight checklist for camera/SlowFrame/ALSA/GPIO/mission settings, then exit.",
    )
    mode_group.add_argument(
        "--help-examples",
        action="store_true",
        help="Print scenario-based command examples for mission, test, GPIO, ALSA, and docs, then exit.",
    )
    mode_group.add_argument(
        "--help-all",
        action="store_true",
        help="Show full argparse option reference (long form) and exit.",
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
    mode_group.add_argument(
        "--alsa-volume-check",
        action="store_true",
        help=(
            "Run ALSA mixer validation only (no capture/encode/TX). "
            "Verifies amixer control listing, set, and readback with PASS/FAIL output."
        ),
    )

    # --- Mission settings ---
    mission = parser.add_argument_group(
        "MISSION",
        "Control the main capture-and-transmit mission loop.",
    )
    mission.add_argument(
        "--radio",
        metavar="BAND",
        choices=["vhf", "uhf", "both"],
        default=None,
        help=(
            "Radio band to use for transmission. "
            "vhf = DRA818V only (GPIO27/pin13, default), "
            "uhf = DRA818U only (GPIO17/pin11), "
            "both = key both PTT lines simultaneously for dual-band TX. "
            f"Default: {ACTIVE_RADIO_BAND}."
        ),
    )
    mission.add_argument(
        "--tx-power",
        metavar="LEVEL",
        choices=["low", "high"],
        default=None,
        help=(
            "TX power level policy for the DRA818 H/L line. "
            "low = H/L LOW (~0.5 W), high = H/L HIGH (~1.0 W). "
            f"Default: {TX_POWER_LEVEL}."
        ),
    )
    mission.add_argument(
        "--pd-idle",
        metavar="MODE",
        choices=["release", "sleep"],
        default=None,
        help=(
            "PD behavior after TX/PTT. "
            "release = INPUT (shared back to Feather M0), "
            "sleep = OUTPUT LOW (Pi-enforced idle power-save). "
            f"Default: {PD_IDLE_MODE}."
        ),
    )
    mission.add_argument(
        "--schedule",
        metavar="PRESET",
        default=None,
        help=(
            f"Transmit schedule preset. Built-in choices: {', '.join(TRANSMIT_SCHEDULE_PROFILES)}. "
            "Config-loaded [schedule_profile <name>] sections may add more. "
            f"Default: {TRANSMIT_SCHEDULE_PROFILE}."
        ),
    )
    mission.add_argument(
        "--total",
        metavar="N",
        type=int,
        default=None,
        help=f"Total number of captures to take before the mission ends. Default: {PIC_TOTAL}.",
    )
    mission.add_argument(
        "--interval",
        metavar="SECS",
        type=float,
        default=None,
        help=f"Seconds to wait between captures. Default: {PIC_INTERVAL}.",
    )
    mission.add_argument(
        "--callsign",
        metavar="CALL",
        default=STATION_CALLSIGN or None,
        help=(
            "Station callsign to overlay on transmitted images (e.g. W1AW-11). "
            "Required for encode/test/mission workflows."
        ),
    )
    mission.add_argument(
        "--overlay-text",
        metavar="TEXT",
        default=None,
        help=(
            "Replace the default MODE DATE TIME overlay body text with TEXT. "
            "Callsign remains mandatory and is always prefixed."
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
        "RADIO PROTECTION",
        "Tune duty-cycle and thermal protection parameters.",
    )
    radio.add_argument(
        "--cooldown-method",
        metavar="METHOD",
        type=str,
        default=None,
        help=(
            "Cooldown logic selector: fixed, adaptive_dutycycle, adaptive_avg_dutycycle, estimated. "
            f"Default: {TX_COOLDOWN_METHOD}."
        ),
    )
    radio.add_argument(
        "--fixed-cooldown-seconds",
        metavar="SECS",
        type=float,
        default=None,
        help=(
            "Static cooldown in seconds for --cooldown-method fixed. "
            f"Default: {FIXED_TX_COOLDOWN_SECONDS}s."
        ),
    )
    radio.add_argument(
        "--cooldown-scale",
        metavar="FACTOR",
        type=float,
        default=None,
        help=(
            "Multiply calculated cooldown duration from the selected method by this factor. "
            "1.0 = tuned defaults, 0.75 = more aggressive, 1.5 = more conservative. "
            f"Default: {COOLDOWN_SCALE_FACTOR}."
        ),
    )
    radio.add_argument(
        "--duty-cycle",
        metavar="FRACTION",
        type=float,
        default=None,
        help=(
            "Target transmit duty-cycle fraction (0.0–1.0) used by adaptive cooldown models. "
            f"Default: {MAX_TRANSMIT_DUTY_CYCLE} ({int(MAX_TRANSMIT_DUTY_CYCLE * 100)}%%)."
        ),
    )
    radio.add_argument(
        "--min-captures",
        metavar="N",
        type=int,
        default=None,
        help=(
            "Minimum number of capture cycles that must elapse between transmissions. "
            f"Default: {MIN_CAPTURES_BETWEEN_TRANSMISSIONS}."
        ),
    )
    radio.add_argument(
        "--estimated-flight-minutes",
        metavar="MIN",
        type=float,
        default=None,
        help=(
            "Estimated mission duration in minutes for thermal altitude correction. "
            f"Default: {ESTIMATED_FLIGHT_DURATION_MINUTES}."
        ),
    )
    radio.add_argument(
        "--estimated-freefall-minutes",
        metavar="MIN",
        type=float,
        default=None,
        help=(
            "Final descent/freefall minutes used by the estimated cooldown model. "
            f"Default: {ESTIMATED_FREEFALL_MINUTES}."
        ),
    )

    # --- Encoding ---
    encode = parser.add_argument_group(
        "ENCODING",
        "SlowFrame audio encoding parameters.",
    )
    encode.add_argument(
        "--format",
        metavar="FMT",
        choices=["wav", "aiff", "ogg"],
        default=None,
        help=f"Audio container format passed to SlowFrame. Default: {SLOWFRAME_AUDIO_FORMAT}.",
    )
    encode.add_argument(
        "--sample-rate",
        metavar="HZ",
        type=int,
        default=None,
        help=f"Audio sample rate in Hz. Default: {SLOWFRAME_SAMPLE_RATE}.",
    )
    encode.add_argument(
        "--aspect",
        metavar="MODE",
        choices=["center", "pad", "stretch"],
        default=None,
        help=f"Image aspect-ratio handling. Default: {SLOWFRAME_ASPECT_MODE}.",
    )

    # --- Paths ---
    paths = parser.add_argument_group(
        "PATHS",
        "Override default file and binary paths.",
    )
    paths.add_argument(
        "--output-dir",
        metavar="PATH",
        default=None,
        help=f"Directory where captured images, WAV files, and CSV logs are written. Default: {TIMESTAMPED_DIR}.",
    )
    paths.add_argument(
        "--slowframe",
        metavar="PATH",
        default=None,
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
            "(bw24, m1, m2, r36, r72, s1, s2, sdx). Equivalent to setting "
            f"SLOWFRAME_NO_MMSSTV=1 in the environment."
        ),
    )
    mmsstv.add_argument(
        "--mmsstv-lib",
        metavar="PATH",
        default=None,
        help=(
            "Explicit path to the MMSSTV shared library (prefer libsstv_encoder.so symlink). "
            f"Overrides the {MMSSTV_LIBRARY_ENV_VAR} environment variable."
        ),
    )

    # --- Logging ---
    log_group = parser.add_argument_group(
        "LOGGING",
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

    # --- ALSA / volume guardrails ---
    alsa_group = parser.add_argument_group(
        "ALSA / VOLUME",
        "Override ALSA playback and mixer guardrail settings from CLI.",
    )
    alsa_group.add_argument(
        "--alsa-playback-device",
        metavar="DEVICE",
        default=None,
        help=(
            "Force ALSA playback device (equivalent to PI_SSTV_ALSA_DEVICE), "
            "e.g. plughw:Headphones,0"
        ),
    )
    alsa_group.add_argument(
        "--alsa-mixer-device",
        metavar="DEVICE",
        default=None,
        help=(
            "Mixer target device/card for amixer commands (equivalent to PI_SSTV_ALSA_MIXER_DEVICE), "
            "e.g. Headphones"
        ),
    )
    alsa_group.add_argument(
        "--alsa-mixer-control",
        metavar="CONTROL",
        default=None,
        help=(
            "Mixer control name (equivalent to PI_SSTV_ALSA_MIXER_CONTROL), "
            "e.g. PCM or Headphone"
        ),
    )
    alsa_group.add_argument(
        "--alsa-target-volume",
        metavar="PERCENT",
        type=int,
        default=None,
        help=(
            "Target mixer volume percent for guardrails (equivalent to PI_SSTV_ALSA_TARGET_VOLUME)."
        ),
    )
    alsa_group.add_argument(
        "--alsa-max-safe-volume",
        metavar="PERCENT",
        type=int,
        default=None,
        help=(
            "Safe-volume warning threshold percent (equivalent to PI_SSTV_ALSA_MAX_SAFE_VOLUME)."
        ),
    )
    alsa_group.add_argument(
        "--no-alsa-volume-guardrails",
        action="store_true",
        help="Disable ALSA mixer guardrails (equivalent to PI_SSTV_ALSA_ENFORCE_VOLUME=0).",
    )

    # --- GPS ---
    gps_group = parser.add_argument_group(
        "GPS",
        "GPS module settings for gridsquare and altitude overlay.  Requires pyserial.",
    )
    gps_group.add_argument(
        "--gps",
        action="store_true",
        help=(
            "Enable GPS polling.  Reads one NMEA GGA fix per capture cycle from the "
            "serial device and adds gridsquare + altitude to the image overlay. "
            "Requires pyserial (pip3 install pyserial) and a UART-connected GPS module. "
            "Recommended: u-blox NEO-M8N or NEO-6M on GPIO15/RX (physical pin 10)."
        ),
    )
    gps_group.add_argument(
        "--gps-device",
        metavar="PATH",
        default=GPS_DEVICE,
        help=f"Serial device path for the GPS module. Default: {GPS_DEVICE}.",
    )
    gps_group.add_argument(
        "--gps-baud",
        metavar="BAUD",
        type=int,
        default=GPS_BAUD,
        help=f"GPS serial baud rate. Default: {GPS_BAUD}.",
    )
    gps_group.add_argument(
        "--gps-units",
        metavar="UNITS",
        choices=["m", "ft"],
        default=GPS_ALTITUDE_UNITS,
        help=f"Altitude units for GPS overlay. Choices: m, ft. Default: {GPS_ALTITUDE_UNITS}.",
    )

    # --- Configuration file ---
    cfg_group = parser.add_argument_group(
        "CONFIGURATION FILE",
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
        const=GENERATE_CONFIG_USE_CONFIG_PATH,
        default=None,
        help=(
            "Write a fully-documented default configuration file to PATH and exit. "
            f"If PATH is omitted, writes to --config PATH when provided, otherwise {DEFAULT_CONFIG_PATH}. "
            "Edit the file then run:  python3 pi_sstv.py --config PATH"
        ),
    )

    args = parser.parse_args(argv)

    info_only_requested = any(
        [
            args.list_modes,
            args.list_schedules,
            args.explain is not None,
            args.generate_config is not None,
            args.help_topics,
            args.help_quick,
            args.help_flight,
            args.help_examples,
            args.help_all,
        ]
    )
    if info_only_requested:
        return args

    if args.test:
        args.test = canonicalize_mode_name(args.test)

    if args.test and args.test not in MODE_PROFILES:
        try:
            refresh_mode_profiles_from_slowframe()
            args.test = canonicalize_mode_name(args.test)
        except Exception:
            # Keep argparse resilient when SlowFrame is unavailable during option parsing.
            pass

    if args.test and args.test not in MODE_PROFILES:
        parser.error(
            f"--test: unknown mode '{args.test}'. "
            f"Valid modes: {', '.join(sorted(MODE_PROFILES))}. "
            "Run --list-modes for details."
        )

    if args.test and args.ptt_test is not None:
        parser.error("--test and --ptt-test cannot be used together")

    if args.alsa_volume_check and (args.test or args.ptt_test is not None):
        parser.error("--alsa-volume-check cannot be combined with --test or --ptt-test")

    if args.ptt_test is not None and args.ptt_test <= 0:
        parser.error("--ptt-test duration must be > 0 seconds")

    if args.log_file and args.quiet_log_file:
        parser.error("--log-file and --quiet-log-file cannot be used together")

    if args.cooldown_method is not None:
        normalized_method = _normalize_cooldown_method(args.cooldown_method)
        if normalized_method not in TX_COOLDOWN_METHOD_CHOICES:
            parser.error(
                "--cooldown-method invalid. "
                f"Valid: {', '.join(TX_COOLDOWN_METHOD_CHOICES)}"
            )
        args.cooldown_method = normalized_method

    if args.duty_cycle is not None and not (0 < args.duty_cycle <= 1.0):
        parser.error("--duty-cycle must be > 0 and <= 1.0")

    if args.fixed_cooldown_seconds is not None and args.fixed_cooldown_seconds < 0:
        parser.error("--fixed-cooldown-seconds must be >= 0")

    if args.estimated_flight_minutes is not None and args.estimated_flight_minutes <= 0:
        parser.error("--estimated-flight-minutes must be > 0")

    if args.estimated_freefall_minutes is not None and args.estimated_freefall_minutes < 0:
        parser.error("--estimated-freefall-minutes must be >= 0")

    return args


def ensure_runtime_paths():
    os.makedirs(TIMESTAMPED_DIR, exist_ok=True)

    if not os.path.exists(DATA_CSV):
        with open(DATA_CSV, 'a', newline='') as file_handle:
            writer = csv.writer(file_handle)
            writer.writerow(CSV_HEADERS)


def write_csv(capture_number, string_time):
    row = [capture_number, string_time]
    with open(DATA_CSV, 'a', newline='') as file_handle:
        writer = csv.writer(file_handle)
        writer.writerow(row)


def list_modes():
    native = set(get_native_modes())
    mmsstv = set()
    try:
        discovered_native, discovered_mmsstv, added_count = refresh_mode_profiles_from_slowframe()
        if discovered_native:
            native.update(discovered_native)
        mmsstv = discovered_mmsstv
        if added_count:
            print(f"Discovered {added_count} additional mode profile(s) from SlowFrame.")
    except Exception:
        # Keep list output available even when SlowFrame is unavailable.
        pass

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
    if mmsstv:
        print(f"MMSSTV modes discovered: {', '.join(sorted(mmsstv))}")


def list_schedules():
    WIDE  = "═" * 66
    THIN  = "─" * 66
    print("Schedule presets\n")
    for preset_name, modes in TRANSMIT_SCHEDULE_PROFILES.items():
        is_active    = preset_name == TRANSMIT_SCHEDULE_PROFILE
        active_tag   = "  ◀ active" if is_active else ""
        description  = TRANSMIT_SCHEDULE_DESCRIPTIONS.get(preset_name, "")
        profiles     = [MODE_PROFILES[m] for m in modes if m in MODE_PROFILES]
        fallback_policy = describe_schedule_fallback_policy(preset_name)

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
        print(f"  {'Unavailable default':22}: {fallback_policy}")
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
        f"color={color}",
    ]
    if background_color:
        overlay_parts.append(f"bg={background_color}")
        # Keep a consistent weak-signal-readable background bar style.
        overlay_parts.append("bgbar=true")
        overlay_parts.append("bgbar-margin=4")
    if background_opacity is not None:
        # Different SlowFrame builds parse different opacity keys.
        # Emit compatibility aliases so background transparency is applied reliably.
        overlay_parts.append(f"opacity={background_opacity}")
        overlay_parts.append(f"bg-opacity={background_opacity}")
        overlay_parts.append(f"bgbar-opacity={background_opacity}")
    overlay_parts.append(f"pos={position}")
    return "|".join(overlay_parts)



def get_native_modes():
    return {name for name, profile in MODE_PROFILES.items() if not profile.requires_mmsstv}


def _log_curated_mode_inventory_audit(
    discovered_native: Set[str],
    discovered_mmsstv: Set[str],
    mmsstv_library_detected: bool,
) -> None:
    curated_native = {name for name, profile in MODE_PROFILES.items() if not profile.requires_mmsstv}
    missing_native = sorted(curated_native - discovered_native)
    if missing_native:
        log(
            "SlowFrame discovery: WARNING — curated native mode profile(s) were not reported by slowframe -L: "
            + ", ".join(missing_native)
        )

    if mmsstv_library_detected and discovered_mmsstv:
        curated_mmsstv = {name for name, profile in MODE_PROFILES.items() if profile.requires_mmsstv}
        missing_mmsstv = sorted(curated_mmsstv - discovered_mmsstv)
        if missing_mmsstv:
            log_debug(
                "SlowFrame discovery: curated MMSSTV mode profile(s) not explicitly listed by slowframe -L: "
                + ", ".join(missing_mmsstv)
            )


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

    discovered_native, discovered_mmsstv, added_count = _augment_mode_profiles_from_slowframe_output(output)
    state.available_modes.update(discovered_native)
    state.available_modes.update(discovered_mmsstv)
    _log_curated_mode_inventory_audit(
        discovered_native,
        discovered_mmsstv,
        state.mmsstv_library_detected,
    )
    if added_count:
        log(f"SlowFrame discovery: added {added_count} auto-profiled mode(s) from -L listing")

    mmsstv_status = "MMSSTV enabled" if state.mmsstv_library_detected else "native-only"
    native_modes = sorted(discovered_native or get_native_modes())

    # If the MMSSTV library was detected, add all known MMSSTV mode profiles to
    # available_modes directly.  The -L listing does not enumerate MMSSTV modes
    # in a reliably parseable format, so we populate from MODE_PROFILES instead.
    if state.mmsstv_library_detected:
        for name, profile in MODE_PROFILES.items():
            if profile.requires_mmsstv:
                state.available_modes.add(name)

    mmsstv_modes = sorted(
        m
        for m in state.available_modes
        if m in discovered_mmsstv or (m not in native_modes and MODE_PROFILES.get(m) and MODE_PROFILES[m].requires_mmsstv)
    )
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


def resolve_mode_name(requested_mode, available_modes, default_fallback_mode: Optional[str] = None):
    requested_mode = _normalize_schedule_mode_name(requested_mode) or requested_mode
    fallback_candidates: List[str] = []

    configured_default = _normalize_schedule_mode_name(default_fallback_mode)
    if configured_default and configured_default != requested_mode:
        fallback_candidates.append(configured_default)

    global_default = _normalize_schedule_mode_name(GLOBAL_SCHEDULE_UNAVAILABLE_MODE_FALLBACK)
    if global_default and global_default != requested_mode and global_default not in fallback_candidates:
        fallback_candidates.append(global_default)

    if requested_mode != "r36" and "r36" not in fallback_candidates:
        fallback_candidates.append("r36")

    current_mode = requested_mode
    visited_modes = set()

    while True:
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

            next_mode = _normalize_schedule_mode_name(current_profile.fallback_mode) if current_profile else None
            if next_mode:
                log_debug(f"Mode resolution: trying fallback: {next_mode}")
            current_mode = next_mode

        if not fallback_candidates:
            break

        current_mode = fallback_candidates.pop(0)
        if current_mode in visited_modes:
            current_mode = None
            continue
        log(f"Mode resolution: fallback chain exhausted for {requested_mode}, trying default {current_mode}")

    fallback = next(iter(sorted(available_modes)))
    log(f"Mode resolution: no configured default available for {requested_mode}; using first available mode: {fallback}")
    return fallback


def get_scheduled_mode_name(runtime_state):
    return TRANSMIT_SCHEDULE[runtime_state.schedule_index % len(TRANSMIT_SCHEDULE)]


def _format_seconds_compact(total_seconds: float) -> str:
    seconds = max(0, int(math.ceil(total_seconds)))
    hours, rem = divmod(seconds, 3600)
    minutes, secs = divmod(rem, 60)
    if hours:
        return f"{hours}h {minutes}m {secs}s"
    if minutes:
        return f"{minutes}m {secs}s"
    return f"{secs}s"


def _normalize_cooldown_method(method_name: str) -> str:
    normalized = (method_name or "").strip().lower()
    if normalized == "adapative_avg_dutycycle":
        return "adaptive_avg_dutycycle"
    return normalized


def _duty_cooldown_ratio() -> float:
    duty = max(0.01, min(1.0, MAX_TRANSMIT_DUTY_CYCLE))
    if duty >= 1.0:
        return 0.0
    return (1.0 - duty) / duty


def _schedule_total_tx_seconds() -> float:
    total = 0.0
    for mode_name in TRANSMIT_SCHEDULE:
        profile = MODE_PROFILES.get(mode_name)
        if profile:
            total += profile.duration_seconds
    return total


def _estimated_air_density_factor(capture_number: int) -> float:
    mission_total_seconds = max(1.0, ESTIMATED_FLIGHT_DURATION_MINUTES * 60.0)
    freefall_seconds = max(0.0, ESTIMATED_FREEFALL_MINUTES * 60.0)
    ascent_seconds = max(1.0, mission_total_seconds - freefall_seconds)
    elapsed_seconds = min(mission_total_seconds, max(0.0, capture_number * max(PIC_INTERVAL, 0.1)))

    min_factor = max(0.05, min(1.0, ESTIMATED_MIN_AIR_DENSITY_FACTOR))
    if elapsed_seconds <= ascent_seconds:
        ascent_fraction = elapsed_seconds / ascent_seconds
        return 1.0 - ((1.0 - min_factor) * ascent_fraction)

    if freefall_seconds <= 0:
        return min_factor

    descent_fraction = (elapsed_seconds - ascent_seconds) / freefall_seconds
    descent_fraction = max(0.0, min(1.0, descent_fraction))
    return min_factor + ((1.0 - min_factor) * descent_fraction)


def _compute_required_cooldown_seconds(
    capture_number: int,
    requested_mode: str,
    mode_profile: ModeProfile,
    runtime_state: RuntimeState,
) -> Tuple[float, str, str, float, float]:
    method = _normalize_cooldown_method(TX_COOLDOWN_METHOD)
    ratio = _duty_cooldown_ratio()

    last_mode_name = runtime_state.last_transmit_mode_name or mode_profile.name
    last_duration = runtime_state.last_transmit_duration_seconds
    if last_duration <= 0 and runtime_state.last_transmit_mode_name:
        last_profile = MODE_PROFILES.get(runtime_state.last_transmit_mode_name)
        if last_profile:
            last_duration = float(last_profile.duration_seconds)

    if method == "fixed":
        fixed_seconds = max(0.0, FIXED_TX_COOLDOWN_SECONDS)
        cooldown_required = fixed_seconds
        detail = f"fixed={fixed_seconds:.0f}s"
        effective_duty = (mode_profile.duration_seconds / (mode_profile.duration_seconds + fixed_seconds)) if (mode_profile.duration_seconds + fixed_seconds) > 0 else 1.0
        return cooldown_required * COOLDOWN_SCALE_FACTOR, detail, "fixed", effective_duty, 1.0

    if method == "adaptive_dutycycle":
        base_duration = max(0.0, last_duration)
        cooldown_required = base_duration * ratio
        detail = (
            f"last_tx={base_duration:.0f}s duty={MAX_TRANSMIT_DUTY_CYCLE:.2f}"
            + (f" anchor={last_mode_name}" if base_duration > 0 else " (first TX)")
        )
        return cooldown_required * COOLDOWN_SCALE_FACTOR, detail, "last-tx", MAX_TRANSMIT_DUTY_CYCLE, 1.0

    if method == "adaptive_avg_dutycycle":
        schedule_total = _schedule_total_tx_seconds()
        if schedule_total <= 0:
            schedule_total = float(mode_profile.duration_seconds)
        schedule_cool_total = schedule_total * ratio
        mode_share = max(0.0, mode_profile.duration_seconds / schedule_total)
        cooldown_required = schedule_cool_total * mode_share
        detail = (
            f"schedule_total_tx={schedule_total:.0f}s share={mode_share:.3f} "
            f"duty={MAX_TRANSMIT_DUTY_CYCLE:.2f}"
        )
        return cooldown_required * COOLDOWN_SCALE_FACTOR, detail, "schedule-share", MAX_TRANSMIT_DUTY_CYCLE, 1.0

    # estimated thermal model: duty-ratio cooldown corrected by altitude-dependent convection.
    tx_duration = max(0.0, last_duration)
    base_cooldown = tx_duration * ratio
    density_factor = _estimated_air_density_factor(capture_number)
    h_air = ESTIMATED_AIR_HEAT_TRANSFER_COEFFICIENT * density_factor
    h_total = ESTIMATED_PCB_HEAT_TRANSFER_COEFFICIENT + h_air
    h_total_sea_level = ESTIMATED_PCB_HEAT_TRANSFER_COEFFICIENT + ESTIMATED_AIR_HEAT_TRANSFER_COEFFICIENT
    convection_penalty = h_total_sea_level / max(0.01, h_total)

    heat_power = ESTIMATED_TX_HEAT_POWER_HIGH_W if TX_POWER_LEVEL == "high" else ESTIMATED_TX_HEAT_POWER_LOW_W
    heat_power_ref = ESTIMATED_TX_HEAT_POWER_LOW_W if ESTIMATED_TX_HEAT_POWER_LOW_W > 0 else 1.0
    power_penalty = heat_power / heat_power_ref

    area_penalty = max(0.5, min(2.0, 0.0030 / max(1e-5, ESTIMATED_EFFECTIVE_THERMAL_AREA_M2)))
    estimated_multiplier = convection_penalty * power_penalty * area_penalty * ESTIMATED_COOLDOWN_SAFETY_FACTOR
    cooldown_required = base_cooldown * estimated_multiplier
    detail = (
        f"duty_base={base_cooldown:.0f}s density={density_factor:.2f} "
        f"h_total={h_total:.2f}W/m2K power={heat_power:.2f}W mult={estimated_multiplier:.2f}"
    )
    return cooldown_required * COOLDOWN_SCALE_FACTOR, detail, "estimated-thermal", MAX_TRANSMIT_DUTY_CYCLE, estimated_multiplier


def evaluate_schedule_gate(
    capture_number: int,
    requested_mode: str,
    mode_profile: ModeProfile,
    runtime_state: RuntimeState,
    now_monotonic: float,
) -> dict:
    slot_index = runtime_state.schedule_index % len(TRANSMIT_SCHEDULE)
    slot_label = f"{slot_index + 1}/{len(TRANSMIT_SCHEDULE)}"

    captures_since_last_transmit = capture_number - runtime_state.last_transmit_capture_number
    if capture_number == 1:
        captures_remaining = 0
    else:
        captures_remaining = max(0, MIN_CAPTURES_BETWEEN_TRANSMISSIONS - captures_since_last_transmit)
    capture_wait_seconds = captures_remaining * max(PIC_INTERVAL, 0)

    effective_cooldown, cooldown_detail, cooldown_basis, target_duty, method_multiplier = _compute_required_cooldown_seconds(
        capture_number,
        requested_mode,
        mode_profile,
        runtime_state,
    )
    cooldown_anchor_name = runtime_state.last_transmit_mode_name or mode_profile.name
    elapsed_since_last_transmit = 0.0
    cooldown_remaining = 0.0
    if runtime_state.last_transmit_end_monotonic:
        elapsed_since_last_transmit = now_monotonic - runtime_state.last_transmit_end_monotonic
        cooldown_remaining = max(0.0, effective_cooldown - elapsed_since_last_transmit)

    projected_interval = mode_profile.duration_seconds + effective_cooldown
    effective_duty_cycle = (mode_profile.duration_seconds / projected_interval) if projected_interval > 0 else 1.0

    blockers: List[str] = []
    if captures_remaining > 0:
        blockers.append(f"min-captures gate ({captures_remaining} capture(s) remaining)")
    if cooldown_remaining > 0:
        blockers.append(
            f"cooldown ({cooldown_basis}) after {cooldown_anchor_name} ({_format_seconds_compact(cooldown_remaining)} remaining)"
        )

    wait_seconds = max(capture_wait_seconds, cooldown_remaining)
    can_transmit = len(blockers) == 0

    eta_text = "now"
    if not can_transmit and wait_seconds > 0:
        eta_utc = datetime.now(timezone.utc) + timedelta(seconds=wait_seconds)
        eta_text = eta_utc.strftime("%Y-%m-%d %H:%M:%SZ")

    return {
        "capture_number": capture_number,
        "slot_index": slot_index,
        "slot_label": slot_label,
        "requested_mode": requested_mode,
        "resolved_mode": mode_profile.name,
        "captures_remaining": captures_remaining,
        "capture_wait_seconds": capture_wait_seconds,
        "cooldown_anchor_name": cooldown_anchor_name,
        "cooldown_method": _normalize_cooldown_method(TX_COOLDOWN_METHOD),
        "cooldown_detail": cooldown_detail,
        "cooldown_basis": cooldown_basis,
        "cooldown_elapsed_seconds": elapsed_since_last_transmit,
        "cooldown_required_seconds": effective_cooldown,
        "cooldown_remaining_seconds": cooldown_remaining,
        "target_duty_cycle": target_duty,
        "effective_duty_cycle": effective_duty_cycle,
        "method_multiplier": method_multiplier,
        "wait_seconds": wait_seconds,
        "eta_text": eta_text,
        "blockers": blockers,
        "can_transmit": can_transmit,
    }


def log_schedule_gate_report(gate: dict):
    status = "TRANSMIT NOW" if gate["can_transmit"] else "HOLD"
    blockers = "none" if gate["can_transmit"] else "; ".join(gate["blockers"])
    next_wait = "0s" if gate["can_transmit"] else _format_seconds_compact(gate["wait_seconds"])

    log_stage_header(
        f"Schedule Gate  Capture #{gate['capture_number']}",
        [
            ("slot", gate["slot_label"]),
            ("schedule", gate["requested_mode"]),
            ("resolved", gate["resolved_mode"]),
            ("decision", status),
            ("next_tx", gate["eta_text"]),
            ("wait", next_wait),
            (
                "captures",
                f"remaining={gate['captures_remaining']}  interval={PIC_INTERVAL}s",
            ),
            (
                "cooldown",
                f"anchor={gate['cooldown_anchor_name']}  elapsed={gate['cooldown_elapsed_seconds']:.0f}s  "
                f"required={gate['cooldown_required_seconds']:.0f}s  remaining={gate['cooldown_remaining_seconds']:.0f}s",
            ),
            (
                "method",
                f"{gate['cooldown_method']} ({gate['cooldown_basis']})  scale={COOLDOWN_SCALE_FACTOR:.2f}  detail={gate['cooldown_detail']}",
            ),
            (
                "duty",
                f"target={gate['target_duty_cycle'] * 100:.1f}%  projected={gate['effective_duty_cycle'] * 100:.1f}%  model-mult={gate['method_multiplier']:.2f}",
            ),
            ("blockers", blockers),
        ],
    )
    if gate["can_transmit"]:
        log_stage_footer("TX ELIGIBLE", [("image_action", "capture will be encoded and transmitted")])
    else:
        log_stage_footer("TX DEFERRED", [("image_action", "capture saved only; no transmit this cycle")])


def select_mode_profile(runtime_state):
    requested_mode = get_scheduled_mode_name(runtime_state)
    resolved_mode = resolve_mode_name(
        requested_mode,
        runtime_state.available_modes,
        default_fallback_mode=get_effective_schedule_fallback_mode(TRANSMIT_SCHEDULE_PROFILE),
    )
    return requested_mode, MODE_PROFILES[resolved_mode]


def _scaled_overlay_size(mode_name: str, base_size: int, minimum_size: int = 8) -> int:
    """Scale overlay font by mode geometry and TX duration for on-air readability."""
    profile = MODE_PROFILES.get(mode_name)
    image_width = profile.image_width if profile else 320
    duration = profile.duration_seconds if profile else 60

    # Do not shrink fonts below configured base on low-resolution modes.
    width_scale = max(1.0, image_width / 320.0)

    # Short, weak-signal-friendly modes benefit from a modest readability boost.
    mode_scale = 1.0
    if duration <= 15:
        mode_scale = 1.20
    elif duration <= 30:
        mode_scale = 1.10

    scaled = round(base_size * width_scale * mode_scale)
    return max(minimum_size, scaled)


def _use_compact_overlay_layout(mode_name: str) -> bool:
    """Low-res fast modes are more reliable with one merged overlay text block."""
    profile = MODE_PROFILES.get(mode_name)
    if not profile:
        return False
    return profile.image_width <= 200 or profile.duration_seconds <= 20


def _compact_mode_label(mode_name: str) -> str:
    raw = (get_protocol_token_for_mode(mode_name) or mode_name).upper()
    robot_match = re.match(r"^ROBOT(\d+)BW$", raw)
    if robot_match:
        return f"B/W{robot_match.group(1)}"
    return raw


def _compact_timestamp_label(timestamp_message: Optional[str]) -> str:
    if not timestamp_message:
        return ""
    text = timestamp_message.strip()
    parsed = re.search(r"(\d{4})-(\d{2})-(\d{2})\s+(\d{2}):(\d{2})(?::\d{2})?Z", text)
    if parsed:
        token = f"{parsed.group(1)}-{parsed.group(2)}-{parsed.group(3)} {parsed.group(4)}:{parsed.group(5)}Z"
        if "TEST" in text.upper():
            token += " T"
        return token
    parsed_time_only = re.search(r"(\d{2}):(\d{2})(?::\d{2})?Z", text)
    if parsed_time_only:
        token = f"{parsed_time_only.group(1)}{parsed_time_only.group(2)}Z"
        if "TEST" in text.upper():
            token += " T"
        return token
    return text


def _estimate_overlay_char_budget(mode_name: str, font_size: int) -> int:
    profile = MODE_PROFILES.get(mode_name)
    width = profile.image_width if profile else 320
    # Approximate characters that can fit on one raster line for this mode/font.
    return max(10, int(width / max(5.5, font_size * 0.62)))


def _build_compact_overlay_text(
    mode_name: str,
    timestamp_message: Optional[str],
    gps_text: Optional[str],
    char_budget: int,
) -> str:
    callsign = STATION_CALLSIGN.strip()
    overlay_body = build_overlay_text(mode_name, timestamp_message, gps_text=gps_text)

    # For default body, keep compact mode label but preserve full date+time shape.
    if not OVERLAY_TEXT_OVERRIDE.strip():
        mode_label = _compact_mode_label(mode_name)
        time_label = _compact_timestamp_label(timestamp_message)
        overlay_body = " ".join(p for p in (mode_label, time_label) if p)

    text = " ".join(p for p in (callsign, overlay_body) if p)

    if len(text) <= char_budget:
        return text

    # If still too long, shorten timestamp token but preserve date+time content.
    shorter_body = overlay_body.replace(":", "")
    parts = [p for p in (callsign, shorter_body) if p]
    text = " ".join(parts)
    if len(text) <= char_budget:
        return text

    # Last resort: callsign only.
    return callsign


def build_slowframe_command(input_path, output_path, timestamp_message, mode_name, gps_text: Optional[str] = None):
    protocol_token = get_protocol_token_for_mode(mode_name)
    command = [
        SLOWFRAME_BIN,
        "-i", input_path,
        "-o", output_path,
        "-p", protocol_token,
        "-f", SLOWFRAME_AUDIO_FORMAT,
        "-r", str(SLOWFRAME_SAMPLE_RATE),
        "-a", SLOWFRAME_ASPECT_MODE,
    ]

    if SLOWFRAME_VERBOSE:
        command.append("-v")

    if STATION_CALLSIGN:
        compact_layout = _use_compact_overlay_layout(mode_name)
        if compact_layout:
            # Compact low-res modes need smaller one-line text to keep timestamp visible.
            overlay_size = max(8, _scaled_overlay_size(mode_name, SLOWFRAME_TIMESTAMP_OVERLAY_SIZE, minimum_size=9) - 2)
            char_budget = _estimate_overlay_char_budget(mode_name, overlay_size)
            overlay_text = _build_compact_overlay_text(
                mode_name,
                timestamp_message,
                gps_text,
                char_budget,
            )
        else:
            overlay_size = _scaled_overlay_size(mode_name, SLOWFRAME_TIMESTAMP_OVERLAY_SIZE, minimum_size=10)
            overlay_text = f"{STATION_CALLSIGN}  {build_overlay_text(mode_name, timestamp_message, gps_text=gps_text)}"

        command.extend([
            "-T",
            build_text_overlay(
                text=overlay_text,
                size=overlay_size,
                position=SLOWFRAME_TIMESTAMP_OVERLAY_POSITION,
                color=SLOWFRAME_TIMESTAMP_OVERLAY_COLOR,
                background_color=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR,
                background_opacity=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY,
            ),
        ])

    return command


def setup_gpio():
    log_debug(f"GPIO setup: mode=BCM, VHF_PTT={DRA818_PTT_PIN}, UHF_PTT={DRA818_UHF_PTT_PIN}, PD={DRA818_POWER_DOWN_PIN}, HL={DRA818_POWER_LEVEL_PIN}")
    GPIO.cleanup()                                                  # clear any stale state from a previous run
    GPIO.setmode(GPIO_PIN_MODE)
    # Keep both PTT lines released when idle so Feather diagnostics can safely drive them.
    # PUD_UP preserves inactive/high bias without hard-driving the lines.
    GPIO.setup(DRA818_PTT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)      # VHF PTT — released
    GPIO.setup(DRA818_UHF_PTT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # UHF PTT — released
    GPIO.setup(DRA818_POWER_LEVEL_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DRA818_POWER_DOWN_PIN, GPIO.IN)
    apply_tx_power_level()
    apply_pd_idle_policy()
    log_debug(
        "GPIO setup complete: "
        f"VHF_PTT=INPUT_PULLUP, UHF_PTT=INPUT_PULLUP, HL={TX_POWER_LEVEL.upper()}, PD_IDLE={PD_IDLE_MODE.upper()}"
    )


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


def claim_ptt_line(pin: int = DRA818_PTT_PIN):
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
    log_debug(f"PTT pin {pin} claimed by Pi: OUTPUT HIGH (idle)")


def release_ptt_line(pin: int = DRA818_PTT_PIN):
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    log_debug(f"PTT pin {pin} released by Pi: INPUT_PULLUP")


def capture_image(output_path, stage_label: Optional[str] = None):
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
    stage_title = f"{stage_label}  Image Capture" if stage_label else "Image Capture"
    log_stage_header(
        stage_title,
        [
            *describe_camera_capture(),
            ("output", output_path),
            ("metering", RPICAM_METERING),
            ("exposure", RPICAM_EXPOSURE),
            ("awb", RPICAM_AWB),
            ("quality", RPICAM_QUALITY),
            ("focus", "fixed-infinity"),
        ],
    )
    log_debug(f"rpicam-still command: {' '.join(cmd)}")
    try:
        run(cmd, check=True)
        size = os.path.getsize(output_path)
        log_stage_footer("PASS", [("image", output_path), ("size", f"{size:,} bytes")])
        return output_path
    except Exception as error:
        log_stage_footer(
            "FALLBACK",
            [
                ("reason", error),
                ("fallback", TEST_IMAGE),
            ],
        )
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


def list_alsa_playback_devices() -> List[str]:
    """Return ALSA playback device names from `aplay -L` (first token per entry)."""
    try:
        result = run(["aplay", "-L"], capture_output=True, text=True, check=True, timeout=15)
    except Exception as exc:
        log(f"ALSA: unable to list playback devices ({exc})")
        return []

    devices: List[str] = []
    for line in result.stdout.splitlines():
        entry = line.strip()
        if not entry:
            continue
        if line.startswith(" ") or line.startswith("\t"):
            continue
        devices.append(entry)
    return devices


def resolve_alsa_playback_candidates() -> List[str]:
    """Return ordered playback candidates, preferring operator override if provided."""
    seen = set()
    ordered: List[str] = []

    if ALSA_AUDIO_DEVICE:
        ordered.append(ALSA_AUDIO_DEVICE)
        seen.add(ALSA_AUDIO_DEVICE)

    available = list_alsa_playback_devices()
    for candidate in ALSA_DEVICE_CANDIDATES:
        if candidate in available and candidate not in seen:
            ordered.append(candidate)
            seen.add(candidate)

    if "default" in available and "default" not in seen:
        ordered.append("default")
        seen.add("default")

    # Last resort: try operator override or plain default even if -L listing failed.
    if not ordered:
        if ALSA_AUDIO_DEVICE:
            ordered.append(ALSA_AUDIO_DEVICE)
        else:
            ordered.append("default")

    return ordered


def _extract_card_name_from_device(device: str) -> Optional[str]:
    """Extract ALSA CARD name from device selectors like plughw:CARD=Headphones,DEV=0."""
    card_match = re.search(r"CARD=([^,]+)", device)
    if card_match:
        return card_match.group(1)
    prefix_match = re.match(r"(?:plughw|hw):([^,]+),", device)
    if prefix_match:
        return prefix_match.group(1)
    if device in ("default", "sysdefault"):
        return "default"
    return None


def _amixer_selectors_for_device(mixer_device: str) -> List[List[str]]:
    """Build candidate selector arguments for amixer for a given mixer target."""
    # Try plain amixer first when device is unspecified or set to a default alias.
    if not mixer_device or mixer_device in ("default", "sysdefault"):
        selectors: List[List[str]] = [[], ["-D", mixer_device or "default"]]
    else:
        selectors = [["-D", mixer_device], ["-c", mixer_device], []]
    return selectors


def _run_amixer_with_fallback(mixer_device: str, tail_args: List[str], timeout: int = 10):
    """Run amixer trying selector fallbacks (-D then -c) and return (result, cmd_text)."""
    last_exc: Optional[Exception] = None
    for selector in _amixer_selectors_for_device(mixer_device):
        cmd = ["amixer", *selector, *tail_args]
        try:
            result = run(
                cmd,
                capture_output=True,
                text=True,
                check=True,
                timeout=timeout,
            )
            return result, " ".join(cmd)
        except Exception as exc:
            last_exc = exc

    raise RuntimeError(
        f"amixer failed for {mixer_device} with args {' '.join(tail_args)} ({last_exc})"
    )


def _read_alsa_volume_percent(mixer_device: str, control_name: str) -> Optional[int]:
    try:
        result, _ = _run_amixer_with_fallback(mixer_device, ["sget", control_name], timeout=10)
    except Exception:
        return None

    matches = re.findall(r"\[(\d+)%\]", result.stdout)
    if not matches:
        return None
    return max(int(v) for v in matches)


def _list_alsa_mixer_controls(mixer_device: str) -> List[str]:
    """Return mixer control names from `amixer -D <device> scontrols`."""
    try:
        result, _ = _run_amixer_with_fallback(mixer_device, ["scontrols"], timeout=10)
    except Exception:
        return []

    controls: List[str] = []
    # Typical line format: Simple mixer control 'Headphone',0
    for line in result.stdout.splitlines():
        match = re.search(r"Simple mixer control '([^']+)'", line)
        if match:
            controls.append(match.group(1))
    return controls


def _find_playback_volume_numid(mixer_device: str, control_name: str) -> Optional[int]:
    """Resolve a mixer numid for '<control> Playback Volume' from `amixer controls`."""
    try:
        result, _ = _run_amixer_with_fallback(mixer_device, ["controls"], timeout=10)
    except Exception:
        return None

    expected_names = [
        f"{control_name} Playback Volume",
        f"{control_name} Playback",
        control_name,
    ]
    pattern = re.compile(r"numid=(\d+),.*name='([^']+)'")
    for line in result.stdout.splitlines():
        match = pattern.search(line)
        if not match:
            continue
        numid = int(match.group(1))
        name = match.group(2)
        if name in expected_names:
            return numid
    return None


def _read_alsa_cget_value(mixer_device: str, numid: int) -> Optional[int]:
    """Read raw value from `amixer cget numid=<n>` for additional verification."""
    try:
        result, _ = _run_amixer_with_fallback(mixer_device, ["cget", f"numid={numid}"], timeout=10)
    except Exception:
        return None

    # Typical line: ': values=-2792'
    match = re.search(r":\s*values=([-]?\d+)", result.stdout)
    if not match:
        return None
    return int(match.group(1))


def _resolve_alsa_mixer_control(mixer_device: str) -> Optional[str]:
    global _ALSA_RESOLVED_MIXER_CONTROL
    global _ALSA_MIXER_DISABLED

    if _ALSA_MIXER_DISABLED:
        return None
    if _ALSA_RESOLVED_MIXER_CONTROL:
        return _ALSA_RESOLVED_MIXER_CONTROL

    controls = _list_alsa_mixer_controls(mixer_device)
    if not controls:
        _ALSA_MIXER_DISABLED = True
        log(f"ALSA: no mixer controls visible for device '{mixer_device}' (volume guardrails disabled)")
        return None

    for candidate in ALSA_MIXER_CONTROL_CANDIDATES:
        if candidate in controls:
            _ALSA_RESOLVED_MIXER_CONTROL = candidate
            if candidate != ALSA_MIXER_CONTROL:
                log(f"ALSA: mixer control '{ALSA_MIXER_CONTROL}' not found; using '{candidate}'")
            return _ALSA_RESOLVED_MIXER_CONTROL

    _ALSA_MIXER_DISABLED = True
    log(
        f"ALSA: no usable playback control found on {mixer_device}; "
        f"available controls: {', '.join(controls)} (volume guardrails disabled)"
    )
    return None


def ensure_alsa_volume_guardrails(selected_device: str):
    """Apply/check ALSA mixer volume to reduce overdrive risk into the radio."""
    global _ALSA_MIXER_DISABLED
    global _ALSA_RESOLVED_MIXER_CONTROL

    if not ALSA_ENFORCE_VOLUME:
        log("ALSA: volume guardrails disabled by PI_SSTV_ALSA_ENFORCE_VOLUME")
        return

    if _ALSA_MIXER_DISABLED:
        # A previous TX failed open; retry guardrails on the next TX attempt.
        _ALSA_MIXER_DISABLED = False
        _ALSA_RESOLVED_MIXER_CONTROL = None
        log("ALSA: retrying mixer guardrails after previous failure")

    mixer_device = ALSA_MIXER_DEVICE
    if not mixer_device:
        mixer_device = _extract_card_name_from_device(selected_device) or "default"

    control_name = _resolve_alsa_mixer_control(mixer_device)
    if control_name is None:
        return

    try:
        _, cmd_used = _run_amixer_with_fallback(
            mixer_device,
            ["sset", control_name, f"{ALSA_TARGET_VOLUME_PERCENT}%"],
            timeout=10,
        )
        log(f"ALSA: set {mixer_device}/{control_name} to {ALSA_TARGET_VOLUME_PERCENT}% via `{cmd_used}`")
    except Exception as exc:
        # Fail-open for this TX only; retry on the next transmit attempt.
        _ALSA_MIXER_DISABLED = True
        _ALSA_RESOLVED_MIXER_CONTROL = None
        log(
            f"ALSA: mixer write failed on {mixer_device}/{control_name} ({exc}); "
            "skipping guardrails for this TX and retrying next TX"
        )
        return

    current = _read_alsa_volume_percent(mixer_device, control_name)
    if current is None:
        _ALSA_MIXER_DISABLED = True
        _ALSA_RESOLVED_MIXER_CONTROL = None
        log(
            f"ALSA: mixer read failed on {mixer_device}/{control_name}; "
            "skipping guardrails for this TX and retrying next TX"
        )
        return

    log(f"ALSA: mixer {mixer_device}/{control_name} = {current}%")
    if current > ALSA_MAX_SAFE_VOLUME_PERCENT:
        log(
            f"ALSA WARNING: mixer level {current}% exceeds safe threshold "
            f"{ALSA_MAX_SAFE_VOLUME_PERCENT}% (risk of clipped/overdriven TX audio)"
        )


def run_alsa_volume_check() -> int:
    """CLI verification mode: validate amixer set/read flow and print explicit pass/fail."""
    log_section("ALSA Volume Check")
    log(
        f"Config: mixer_device={ALSA_MIXER_DEVICE}  mixer_control={ALSA_MIXER_CONTROL}  "
        f"target={ALSA_TARGET_VOLUME_PERCENT}%  max_safe={ALSA_MAX_SAFE_VOLUME_PERCENT}%"
    )

    candidates = resolve_alsa_playback_candidates()
    log(f"Playback candidates: {', '.join(candidates)}")

    mixer_targets: List[str] = []
    if ALSA_MIXER_DEVICE:
        mixer_targets.append(ALSA_MIXER_DEVICE)
    for device in candidates:
        card = _extract_card_name_from_device(device)
        if card and card not in mixer_targets:
            mixer_targets.append(card)
    for fallback in ("default", "sysdefault", ""):
        if fallback not in mixer_targets:
            mixer_targets.append(fallback)

    mixer_device = ""
    controls: List[str] = []
    for target in mixer_targets:
        found_controls = _list_alsa_mixer_controls(target)
        if found_controls:
            mixer_device = target
            controls = found_controls
            break

    mixer_label = mixer_device or "<default>"
    if not controls:
        log(
            "ALSA CHECK FAIL: no mixer controls discovered. "
            f"Tried targets: {', '.join(t or '<default>' for t in mixer_targets)}"
        )
        return 1

    log(f"Mixer target selected: {mixer_label}")
    log(f"Mixer controls on {mixer_label}: {', '.join(controls)}")

    global _ALSA_RESOLVED_MIXER_CONTROL
    global _ALSA_MIXER_DISABLED
    _ALSA_RESOLVED_MIXER_CONTROL = None
    _ALSA_MIXER_DISABLED = False
    control_name = _resolve_alsa_mixer_control(mixer_device)
    if control_name is None:
        log(f"ALSA CHECK FAIL: no usable mixer control found for '{mixer_label}'")
        return 1

    try:
        _, cmd_used = _run_amixer_with_fallback(
            mixer_device,
            ["sset", control_name, f"{ALSA_TARGET_VOLUME_PERCENT}%"],
            timeout=10,
        )
        log(f"ALSA CHECK: set command OK via `{cmd_used}`")
    except Exception as exc:
        log(f"ALSA CHECK FAIL: set command failed for {mixer_label}/{control_name} ({exc})")
        return 1

    current = _read_alsa_volume_percent(mixer_device, control_name)
    if current is None:
        log(f"ALSA CHECK FAIL: readback failed for {mixer_label}/{control_name}")
        return 1

    log(f"ALSA CHECK: readback {mixer_label}/{control_name} = {current}%")

    numid = _find_playback_volume_numid(mixer_device, control_name)
    if numid is None:
        log(f"ALSA CHECK NOTE: could not resolve numid for {control_name} via amixer controls")
    else:
        raw_value = _read_alsa_cget_value(mixer_device, numid)
        if raw_value is None:
            log(f"ALSA CHECK NOTE: cget read failed for numid={numid}")
        else:
            log(f"ALSA CHECK: cget numid={numid} raw value={raw_value}")

    if current > ALSA_MAX_SAFE_VOLUME_PERCENT:
        log(
            f"ALSA CHECK WARNING: readback {current}% exceeds safe threshold "
            f"{ALSA_MAX_SAFE_VOLUME_PERCENT}%"
        )

    log("ALSA CHECK PASS: mixer set/read path is operational")
    return 0


def generate_sstv_audio(image_path, timestamp_message, mode_name, wav_path=None, gps_text: Optional[str] = None, stage_label: Optional[str] = None):
    output = wav_path or SSTV_WAV
    profile = MODE_PROFILES.get(mode_name)
    expected_duration = profile.duration_seconds if profile else "?"
    overlay_desc = describe_overlay(mode_name, timestamp_message, gps_text=gps_text)
    stage_title = f"{stage_label}  SSTV Encode" if stage_label else "SSTV Encode"
    log_stage_header(
        stage_title,
        [
            ("input", image_path),
            ("output", output),
            ("mode", f"{mode_name.upper()}  (~{expected_duration}s TX, {describe_mode_geometry(profile)})"),
            ("audio", f"{SLOWFRAME_AUDIO_FORMAT}  {SLOWFRAME_SAMPLE_RATE}Hz  aspect={SLOWFRAME_ASPECT_MODE}"),
            ("overlay", overlay_desc),
        ],
    )

    cmd = build_slowframe_command(image_path, output, timestamp_message, mode_name, gps_text=gps_text)
    log_debug(f"SlowFrame command: {' '.join(cmd)}")
    try:
        run(cmd, check=True)
    except CalledProcessError as error:
        profile = MODE_PROFILES.get(mode_name)
        protocol_error = error.returncode == 112
        should_retry_protocol = bool(profile and profile.requires_mmsstv and protocol_error)
        if not should_retry_protocol:
            raise

        original_token = cmd[cmd.index("-p") + 1] if "-p" in cmd else mode_name
        retry_tokens = [t for t in _candidate_protocol_tokens(mode_name) if t.lower() != original_token.lower()]
        last_error: Optional[CalledProcessError] = error

        for retry_token in retry_tokens:
            retry_cmd = list(cmd)
            if "-p" in retry_cmd:
                retry_cmd[retry_cmd.index("-p") + 1] = retry_token
            log(f"SlowFrame protocol retry: '{original_token}' -> '{retry_token}'")
            log_debug(f"SlowFrame retry command: {' '.join(retry_cmd)}")
            try:
                run(retry_cmd, check=True)
                break
            except CalledProcessError as retry_error:
                last_error = retry_error
        else:
            raise last_error

    size = os.path.getsize(output)
    log_stage_footer("PASS", [("wav", output), ("size", f"{size:,} bytes")])
    time.sleep(SSTV_CONVERSION_SETTLE_SECONDS)


def _read_wav_duration_seconds(path: str) -> Optional[float]:
    try:
        with wave.open(path, "rb") as wav_file:
            frame_rate = wav_file.getframerate()
            frame_count = wav_file.getnframes()
            if frame_rate <= 0:
                return None
            return frame_count / float(frame_rate)
    except Exception:
        return None


def resolve_aplay_timeout_seconds(audio_path: str, expected_duration_seconds: Optional[float] = None) -> int:
    duration_candidates: List[float] = []

    wav_duration = _read_wav_duration_seconds(audio_path)
    if wav_duration is not None and wav_duration > 0:
        duration_candidates.append(wav_duration)

    if expected_duration_seconds is not None and expected_duration_seconds > 0:
        duration_candidates.append(float(expected_duration_seconds))

    if not duration_candidates:
        return APLAY_TIMEOUT_SECONDS

    dynamic_timeout = int(max(duration_candidates) + APLAY_TIMEOUT_MARGIN_SECONDS)
    return max(APLAY_TIMEOUT_SECONDS, dynamic_timeout)


def transmit_sstv_audio(wav_path=None, expected_duration_seconds: Optional[float] = None, stage_label: Optional[str] = None):
    audio_path = wav_path or SSTV_WAV
    ptt_pins = get_active_ptt_pins()
    aplay_timeout = resolve_aplay_timeout_seconds(audio_path, expected_duration_seconds)
    stage_title = f"{stage_label}  Radio TX" if stage_label else "Radio TX"
    expected_display = f"~{expected_duration_seconds:.0f}s" if expected_duration_seconds is not None else "unknown"
    log_stage_header(
        stage_title,
        [
            ("audio", audio_path),
            ("radio", ACTIVE_RADIO_BAND.upper()),
            *describe_radio_control_states(),
            ("ptt_pins", ptt_pins),
            ("playback", ALSA_AUDIO_DEVICE or "auto-select"),
            ("mixer", describe_alsa_guardrails()),
            ("timeout", f"{aplay_timeout}s  (base={APLAY_TIMEOUT_SECONDS}s  margin={APLAY_TIMEOUT_MARGIN_SECONDS}s)"),
            ("timing", f"wake={RADIO_WAKE_DELAY_SECONDS}s  key={PTT_KEY_DELAY_SECONDS}s  post={POST_PLAYBACK_DELAY_SECONDS}s"),
            ("expected", expected_display),
        ],
    )
    log("TX: PD -> OUTPUT HIGH (Pi claims line, radio wake)")
    transmit_started_at = time.monotonic()
    selected_device: Optional[str] = None
    failure_reason: Optional[str] = None
    elapsed = 0.0
    # Re-assert configured power policy before every TX key event.
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
        candidates = resolve_alsa_playback_candidates()
        if not candidates:
            raise RuntimeError("no ALSA playback candidates available")

        playback_ok = False
        last_error: Optional[Exception] = None
        for device in candidates:
            try:
                ensure_alsa_volume_guardrails(device)
                if device in ("default", "sysdefault"):
                    log(f"TX: attempting ALSA playback via {device}")
                    run(["aplay", "-q", audio_path], check=True, timeout=aplay_timeout)
                else:
                    log(f"TX: attempting ALSA playback via device '{device}'")
                    run(["aplay", "-q", "-D", device, audio_path], check=True, timeout=aplay_timeout)
                log(f"TX: ALSA playback successful via {device}")
                selected_device = device
                playback_ok = True
                break
            except Exception as exc:
                last_error = exc
                log(f"TX: ALSA playback failed on {device} ({exc})")

        if not playback_ok:
            raise RuntimeError(f"all ALSA playback attempts failed ({last_error})")
    except FileNotFoundError:
        failure_reason = "audio file or aplay not found"
        log("TX: audio file or aplay not found")
        raise
    except Exception as exc:
        failure_reason = str(exc)
        log(f"TX: ALSA playback error: {exc}")
        raise
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
        footer_details = [
            ("device", selected_device or "none"),
            ("elapsed", f"{elapsed:.1f}s"),
        ]
        if failure_reason:
            footer_details.append(("reason", failure_reason))
            log_stage_footer("FAIL", footer_details)
        else:
            log_stage_footer("PASS", footer_details)

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


def process_capture(index):
    current_time = datetime.now()
    picture_time, capture_path = get_capture_path(current_time)
    capture_label = f"Capture #{index + 1}"
    captured_image_path = capture_image(capture_path, stage_label=capture_label)
    timestamp_message = format_overlay_timestamp(current_time)

    write_csv(index + 1, picture_time)

    gps_text = build_gps_overlay_text() if GPS_ENABLED else None
    if gps_text:
        log(f"GPS: overlay text = '{gps_text}'")
    else:
        gps_state = "no-fix" if GPS_ENABLED else "disabled"
        log(f"{capture_label}: timestamp={current_time.strftime('%Y-%m-%d %H:%M:%S')}  gps={gps_state}")

    return captured_image_path, timestamp_message, gps_text


def run_test_pipeline(mode_name: str, args, runtime_state: RuntimeState):
    """Execute a single-shot pipeline test.

    Validates each stage — camera capture, SSTV encode, and optionally radio TX —
    using a consistent file prefix so every output artifact can be identified and
    inspected.  Exits with code 0 on full success, 1 on any stage failure.
    """
    test_id = datetime.now(timezone.utc).strftime("TEST-%Y%m%d-%H%M%S")
    output_dir = TIMESTAMPED_DIR or args.output_dir or BASE_DIR
    resolved_mode = resolve_mode_name(
        mode_name,
        runtime_state.available_modes,
        default_fallback_mode=get_effective_schedule_fallback_mode(),
    )
    wav_path = os.path.join(output_dir, f"{test_id}-{resolved_mode}.wav")

    # Resolve the image source before printing the header so the logged path is accurate.
    if args.test_image is not None:
        capture_path = args.test_image
        capture_source = "supplied"
    else:
        capture_path = os.path.join(output_dir, f"{test_id}-capture.jpg")
        capture_source = "camera"

    log_stage_header(
        "Test Pipeline",
        [
            ("run-id", test_id),
            ("mode", resolved_mode + (f"  (fallback from {mode_name})" if resolved_mode != mode_name else "")),
            ("capture", f"{capture_path}  [{capture_source}]"),
            ("wav", wav_path),
            ("tx", "disabled (--no-tx)" if args.no_tx else "enabled"),
        ],
    )

    profile = MODE_PROFILES.get(resolved_mode)
    if not profile:
        log(f"ERROR: resolved mode '{resolved_mode}' not found in MODE_PROFILES")
        sys.exit(1)

    # --- Stage 1: Image capture ---
    if args.test_image is not None:
        log_stage_header("Stage 1/3  Image Capture", [("source", "supplied image"), ("path", capture_path)])
        # User explicitly supplied an image — skip the camera entirely.
        if not os.path.isfile(capture_path):
            log_stage_footer("FAIL", [("reason", f"--test-image path not found: {capture_path}")])
            sys.exit(1)
        size = os.path.getsize(capture_path)
        log_stage_footer("PASS", [("image", capture_path), ("size", f"{size:,} bytes")])
    else:
        # Try the camera; fall back to the default test image on failure.
        captured = capture_image(capture_path, stage_label="Stage 1/3")
        if captured != capture_path:
            if captured and os.path.isfile(captured):
                capture_path = captured
                size = os.path.getsize(capture_path)
                log(f"Stage 1/3 note: camera unavailable; using test image: {capture_path}  ({size:,} bytes)")
            else:
                log_stage_footer("FAIL", [("reason", f"camera unavailable and no test image found: {TEST_IMAGE}")])
                sys.exit(1)

    # --- Stage 2: SSTV encode ---
    timestamp_message = format_overlay_timestamp(datetime.now(timezone.utc), is_test=True)
    gps_text = build_gps_overlay_text() if GPS_ENABLED else None
    if gps_text:
        log(f"  GPS: '{gps_text}'")
    try:
        generate_sstv_audio(
            capture_path,
            timestamp_message,
            resolved_mode,
            wav_path=wav_path,
            gps_text=gps_text,
            stage_label="Stage 2/3",
        )
        size = os.path.getsize(wav_path)
        duration_est = profile.duration_seconds
        log(f"Stage 2/3 summary: {wav_path}  ({size:,} bytes, ~{duration_est}s expected TX)")
    except Exception as error:
        log_stage_footer("FAIL", [("reason", f"Encoding failed: {error}")])
        sys.exit(1)

    # --- Stage 3: Radio TX ---
    if args.no_tx:
        log_stage_header(
            "Stage 3/3  Radio TX",
            [
                ("status", "skipped (--no-tx)"),
                ("radio", ACTIVE_RADIO_BAND.upper()),
                ("radio_sel", describe_radio_selection()),
                ("ptt_pins", get_active_ptt_pins()),
            ],
        )
        log_stage_footer("SKIPPED", [("reason", "operator requested encode-only test")])
        log_stage_footer("PASS", [("pipeline", "encode-only")])
        return

    try:
        duration = transmit_sstv_audio(
            wav_path=wav_path,
            expected_duration_seconds=profile.duration_seconds,
            stage_label="Stage 3/3",
        )
        log(f"Stage 3/3 summary: transmitted {duration:.1f}s")
    except Exception as error:
        log_stage_footer("FAIL", [("reason", f"Transmission failed: {error}")])
        sys.exit(1)

    log_stage_footer("PASS", [("pipeline", "full pipeline")])


def _describe_run_mode(args) -> str:
    if args.alsa_volume_check:
        return "alsa-volume-check"
    if args.ptt_test is not None:
        return "ptt-test"
    if args.test:
        return "test-encode-only" if args.no_tx else "test-full-pipeline"
    return "mission-encode-only" if args.no_tx else "mission"


def print_runtime_startup_summary(args):
    log_stage_header(
        "Runtime Startup",
        [
            ("run_mode", _describe_run_mode(args)),
            ("config", args.config or "none (defaults + CLI/env)"),
            ("schedule", f"{TRANSMIT_SCHEDULE_PROFILE}  fallback={describe_schedule_fallback_policy(TRANSMIT_SCHEDULE_PROFILE)}"),
            ("output_dir", TIMESTAMPED_DIR),
            ("slowframe", SLOWFRAME_BIN),
            ("test_image", args.test_image or TEST_IMAGE),
            ("camera", f"{CAMERA_NAME} ({CAMERA_MODEL})"),
            ("capture", f"quality={RPICAM_QUALITY}  metering={RPICAM_METERING}  exposure={RPICAM_EXPOSURE}  awb={RPICAM_AWB}"),
            ("encode", f"format={SLOWFRAME_AUDIO_FORMAT}  rate={SLOWFRAME_SAMPLE_RATE}Hz  aspect={SLOWFRAME_ASPECT_MODE}"),
            ("overlay", f"timestamp={SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY}  callsign={SLOWFRAME_ENABLE_CALLSIGN_OVERLAY}  id='{STATION_CALLSIGN or '-'}'"),
            *describe_radio_control_states(),
            ("ptt_pins", get_active_ptt_pins()),
            ("gps", f"enabled={GPS_ENABLED}  device={GPS_DEVICE}  baud={GPS_BAUD}  units={GPS_ALTITUDE_UNITS}"),
            ("cooldown", f"method={TX_COOLDOWN_METHOD}  fixed={FIXED_TX_COOLDOWN_SECONDS:.0f}s  duty={MAX_TRANSMIT_DUTY_CYCLE * 100:.1f}%  scale={COOLDOWN_SCALE_FACTOR:.2f}"),
            ("alsa", f"playback={ALSA_AUDIO_DEVICE or 'auto-select'}  mixer={describe_alsa_guardrails()}"),
            ("aplay", f"base_timeout={APLAY_TIMEOUT_SECONDS}s  margin={APLAY_TIMEOUT_MARGIN_SECONDS}s"),
        ],
    )
    log_stage_footer("READY")


def print_mission_summary(runtime_state):
    mmsstv_status = "enabled" if runtime_state.mmsstv_library_detected else "disabled (native modes only)"

    log_stage_header(
        "Mission Summary",
        [
            ("schedule", TRANSMIT_SCHEDULE_PROFILE),
            ("fallback", describe_schedule_fallback_policy(TRANSMIT_SCHEDULE_PROFILE)),
            ("mmsstv", mmsstv_status),
            ("capture", f"interval={PIC_INTERVAL}s  total={PIC_TOTAL}  min_between_tx={MIN_CAPTURES_BETWEEN_TRANSMISSIONS}"),
            ("duty_target", f"{MAX_TRANSMIT_DUTY_CYCLE * 100:.1f}%"),
            ("cooldown", f"method={TX_COOLDOWN_METHOD}  fixed={FIXED_TX_COOLDOWN_SECONDS:.0f}s  scale={COOLDOWN_SCALE_FACTOR:.2f}"),
            ("estimated", f"flight={ESTIMATED_FLIGHT_DURATION_MINUTES:.0f}m  freefall={ESTIMATED_FREEFALL_MINUTES:.0f}m  h_pcb={ESTIMATED_PCB_HEAT_TRANSFER_COEFFICIENT:.2f}  h_air0={ESTIMATED_AIR_HEAT_TRANSFER_COEFFICIENT:.1f}"),
            *describe_radio_control_states(),
            ("ptt_pins", get_active_ptt_pins()),
            ("audio", f"{SLOWFRAME_AUDIO_FORMAT}  {SLOWFRAME_SAMPLE_RATE}Hz  aspect={SLOWFRAME_ASPECT_MODE}"),
            ("alsa", f"playback={ALSA_AUDIO_DEVICE or 'auto-select'}  mixer={describe_alsa_guardrails()}"),
            ("tx_timing", f"wake={RADIO_WAKE_DELAY_SECONDS}s  key={PTT_KEY_DELAY_SECONDS}s  post={POST_PLAYBACK_DELAY_SECONDS}s"),
        ],
    )
    log("Scheduled modes:")

    seen = set()
    for mode_name in TRANSMIT_SCHEDULE:
        if mode_name in seen:
            continue
        seen.add(mode_name)
        resolved = resolve_mode_name(
            mode_name,
            runtime_state.available_modes,
            default_fallback_mode=get_effective_schedule_fallback_mode(TRANSMIT_SCHEDULE_PROFILE),
        )
        profile = MODE_PROFILES[resolved]
        ratio = _duty_cooldown_ratio()
        if TX_COOLDOWN_METHOD == "fixed":
            effective_cooldown = int(FIXED_TX_COOLDOWN_SECONDS * COOLDOWN_SCALE_FACTOR)
        else:
            effective_cooldown = int(profile.duration_seconds * ratio * COOLDOWN_SCALE_FACTOR)
        fallback_note = f" [fallback from {mode_name}]" if resolved != mode_name else ""
        log(f"  {resolved:<12} {profile.duration_seconds:>4}s TX  {effective_cooldown:>5}s cooldown{fallback_note}")

    log_stage_footer("READY")


def main():
    cli_argv = sys.argv[1:]

    # Keep top-level help concise by default; route to full argparse help on demand.
    if any(arg in ("-h", "--help") for arg in cli_argv) and "--help-all" not in cli_argv:
        print_help_cli_overview()
        sys.exit(0)

    if "--help-all" in cli_argv:
        cli_argv = [arg for arg in cli_argv if arg != "--help-all"]
        if not any(arg in ("-h", "--help") for arg in cli_argv):
            cli_argv.append("--help")

    # Show usage hint when called with no arguments at all.
    if not cli_argv:
        print(
            "pi_sstv.py - HamWing SSTV HAB payload controller\n"
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
            "    python3 pi_sstv.py --help-all\n"
            "\n"
            "  Use guided operator help:\n"
            "    python3 pi_sstv.py --help-quick\n"
            "    python3 pi_sstv.py --help-flight\n"
            "    python3 pi_sstv.py --help-examples\n"
            "    python3 pi_sstv.py --help-topics\n"
        )
        sys.exit(0)

    args = parse_args(cli_argv)

    # Info-only modes — no GPIO, no paths, no subprocess needed
    if args.help_quick:
        print_help_quick()
        return
    if args.help_flight:
        print_help_flight()
        return
    if args.help_examples:
        print_help_examples()
        return
    if args.help_all:
        # If this path is hit directly, show the concise overview and exit.
        # Full argparse help is handled by the pre-parse routing above.
        print_help_cli_overview()
        return
    if args.help_topics:
        print_help_topics()
        return
    if args.list_modes:
        list_modes()
        return
    if args.list_schedules:
        config_path = args.config
        if config_path is None and os.path.isfile(DEFAULT_CONFIG_PATH):
            config_path = DEFAULT_CONFIG_PATH
        if config_path:
            load_config(config_path)
        list_schedules()
        return
    if args.explain:
        print_explain(args.explain)
        return
    if args.generate_config is not None:
        output_path = args.generate_config
        if output_path == GENERATE_CONFIG_USE_CONFIG_PATH:
            output_path = args.config or DEFAULT_CONFIG_PATH
        generate_default_config(output_path)
        return

    selected_log_file = args.quiet_log_file or args.log_file
    quiet_stdout = args.quiet_log_file is not None
    configure_logging(debug=args.debug, log_file=selected_log_file, quiet_stdout=quiet_stdout)

    # Apply config file settings (before CLI overrides so CLI always wins).
    # If --config is omitted, automatically use the default config path when present.
    config_path = args.config
    if config_path is None and os.path.isfile(DEFAULT_CONFIG_PATH):
        config_path = DEFAULT_CONFIG_PATH
        args.config = config_path
        log(f"Config: auto-loading default config at {config_path}")
    if config_path:
        load_config(config_path)

    # Apply CLI overrides to module-level configuration
    global TRANSMIT_SCHEDULE, TRANSMIT_SCHEDULE_PROFILE
    global PIC_TOTAL, PIC_INTERVAL
    global STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global OVERLAY_TEXT_OVERRIDE
    global COOLDOWN_SCALE_FACTOR, MAX_TRANSMIT_DUTY_CYCLE, MIN_CAPTURES_BETWEEN_TRANSMISSIONS
    global TX_COOLDOWN_METHOD, FIXED_TX_COOLDOWN_SECONDS
    global ESTIMATED_FLIGHT_DURATION_MINUTES, ESTIMATED_FREEFALL_MINUTES
    global SLOWFRAME_AUDIO_FORMAT, SLOWFRAME_SAMPLE_RATE, SLOWFRAME_ASPECT_MODE
    global TIMESTAMPED_DIR, SLOWFRAME_BIN, TEST_IMAGE, SSTV_WAV
    global ACTIVE_RADIO_BAND, TX_POWER_LEVEL, PD_IDLE_MODE
    global GPS_ENABLED, GPS_DEVICE, GPS_BAUD, GPS_ALTITUDE_UNITS

    # Only apply CLI overrides if explicitly provided (not None) — config file values take priority
    if args.radio is not None:
        ACTIVE_RADIO_BAND = args.radio
    if args.tx_power is not None:
        TX_POWER_LEVEL = args.tx_power
    if args.pd_idle is not None:
        PD_IDLE_MODE = args.pd_idle
    if args.schedule is not None:
        schedule_name = _normalize_schedule_profile_name(args.schedule)
        if schedule_name not in TRANSMIT_SCHEDULE_PROFILES:
            print(
                f"ERROR: unknown schedule preset '{args.schedule}'. Valid: {', '.join(sorted(TRANSMIT_SCHEDULE_PROFILES))}",
                file=sys.stderr,
            )
            sys.exit(2)
        TRANSMIT_SCHEDULE_PROFILE = schedule_name
        TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES[schedule_name]
    else:
        TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES.get(TRANSMIT_SCHEDULE_PROFILE, TRANSMIT_SCHEDULE_PROFILES["hab_cruise"])
    if args.total is not None:
        PIC_TOTAL = args.total
    if args.interval is not None:
        PIC_INTERVAL = args.interval
    if args.callsign:
        STATION_CALLSIGN = args.callsign
        SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = True
    if args.overlay_text is not None:
        OVERLAY_TEXT_OVERRIDE = args.overlay_text.strip()
    if args.cooldown_method is not None:
        TX_COOLDOWN_METHOD = _normalize_cooldown_method(args.cooldown_method)
    if args.fixed_cooldown_seconds is not None:
        FIXED_TX_COOLDOWN_SECONDS = max(0.0, args.fixed_cooldown_seconds)
    if args.cooldown_scale is not None:
        COOLDOWN_SCALE_FACTOR = args.cooldown_scale
    if args.duty_cycle is not None:
        MAX_TRANSMIT_DUTY_CYCLE = args.duty_cycle
    if args.min_captures is not None:
        MIN_CAPTURES_BETWEEN_TRANSMISSIONS = args.min_captures
    if args.estimated_flight_minutes is not None:
        ESTIMATED_FLIGHT_DURATION_MINUTES = max(1.0, args.estimated_flight_minutes)
    if args.estimated_freefall_minutes is not None:
        ESTIMATED_FREEFALL_MINUTES = max(0.0, args.estimated_freefall_minutes)
    if args.format is not None:
        SLOWFRAME_AUDIO_FORMAT = args.format
    if args.sample_rate is not None:
        SLOWFRAME_SAMPLE_RATE = args.sample_rate
    if args.aspect is not None:
        SLOWFRAME_ASPECT_MODE = args.aspect
    if args.output_dir is not None:
        TIMESTAMPED_DIR = args.output_dir
    if args.slowframe is not None:
        SLOWFRAME_BIN = args.slowframe
    if args.test_image is not None:
        TEST_IMAGE = args.test_image
    SSTV_WAV = os.path.join(TIMESTAMPED_DIR, "HAB-SSTV.wav")

    if args.no_mmsstv:
        os.environ[MMSSTV_DISABLE_ENV_VAR] = "1"
    if args.mmsstv_lib:
        os.environ[MMSSTV_LIBRARY_ENV_VAR] = args.mmsstv_lib

    # ALSA / volume overrides
    if args.alsa_playback_device:
        ALSA_AUDIO_DEVICE = args.alsa_playback_device.strip()
    if args.alsa_mixer_device:
        ALSA_MIXER_DEVICE = args.alsa_mixer_device.strip()
    if args.alsa_mixer_control:
        ALSA_MIXER_CONTROL = args.alsa_mixer_control.strip()
    if args.alsa_target_volume is not None:
        ALSA_TARGET_VOLUME_PERCENT = max(0, min(100, int(args.alsa_target_volume)))
    if args.alsa_max_safe_volume is not None:
        ALSA_MAX_SAFE_VOLUME_PERCENT = max(0, min(100, int(args.alsa_max_safe_volume)))
    if args.no_alsa_volume_guardrails:
        ALSA_ENFORCE_VOLUME = False

    # GPS overrides
    if args.gps:
        GPS_ENABLED = True
    GPS_DEVICE = args.gps_device
    GPS_BAUD = args.gps_baud
    GPS_ALTITUDE_UNITS = args.gps_units

    # Callsign is mandatory for encode/test/mission workflows.
    requires_overlay_callsign = not args.alsa_volume_check and args.ptt_test is None
    if requires_overlay_callsign and not STATION_CALLSIGN.strip():
        print(
            "ERROR: callsign is required for transmission/encoding overlays. "
            "Set --callsign CALL or [mission] callsign in the config file.",
            file=sys.stderr,
        )
        sys.exit(2)
    if requires_overlay_callsign:
        SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = True

    print_runtime_startup_summary(args)

    if args.alsa_volume_check:
        exit_code = run_alsa_volume_check()
        sys.exit(exit_code)

    ensure_runtime_paths()

    # Determine whether GPIO is needed.
    # Any TX path or explicit PTT test requires GPIO; pure encode/list/info does not.
    needs_gpio = (args.ptt_test is not None) or (not args.no_tx)
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
            image_path, timestamp_message, gps_text = process_capture(index)

            time.sleep(PIC_INTERVAL)

            requested_mode, mode_profile = select_mode_profile(runtime_state)
            now_monotonic = time.monotonic()
            gate = evaluate_schedule_gate(
                capture_number,
                requested_mode,
                mode_profile,
                runtime_state,
                now_monotonic,
            )
            log_schedule_gate_report(gate)

            if not gate["can_transmit"]:
                log(f"[#{capture_number}] Schedule decision: NO-TX ({'; '.join(gate['blockers'])})")
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
                generate_sstv_audio(image_path, timestamp_message, mode_profile.name, gps_text=gps_text)
            except Exception as error:
                log(f"Slowframe conversion failed: {error}")
                continue

            try:
                actual_transmit_duration = transmit_sstv_audio(
                    expected_duration_seconds=mode_profile.duration_seconds,
                )
            except Exception as error:
                log(f"Playback failed: {error}")
                continue

            runtime_state.last_transmit_capture_number = capture_number
            runtime_state.last_transmit_end_monotonic = time.monotonic()
            runtime_state.last_transmit_mode_name = mode_profile.name
            runtime_state.last_transmit_duration_seconds = float(mode_profile.duration_seconds)
            runtime_state.schedule_index += 1
            log(
                f"[#{capture_number}] TX done: {mode_profile.name}, {actual_transmit_duration:.1f}s, "
                f"cooldown_method={TX_COOLDOWN_METHOD}, next gate re-evaluates on next capture"
            )
    finally:
        if needs_gpio:
            GPIO.cleanup()


if __name__ == '__main__':
    main()
