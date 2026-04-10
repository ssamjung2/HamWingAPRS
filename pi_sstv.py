#!/usr/bin/env python3

import argparse
import logging
import sys
from dataclasses import dataclass, field
import time
from subprocess import run
from datetime import datetime
import csv
import os
import re
from typing import List, Optional, Set, Tuple
import RPi.GPIO as GPIO

logger = logging.getLogger("pi_sstv")

# Paths and constants
BASE_DIR = "/home/pi-user"
TIMESTAMPED_DIR = os.path.join(BASE_DIR, "Desktop/HAB")
PI_SSTV_BIN = os.path.join(BASE_DIR, "pi-sstv", "pi-sstv")

# Alternate/updated converter: Slowframe
SLOWFRAME_BIN = "/home/pi-user/Desktop/Slowframe/bin/slowframe"
TEST_IMAGE = os.path.join(BASE_DIR, "pi-sstv", "test.jpg")
SSTV_WAV = os.path.join(TIMESTAMPED_DIR, "HAB-SSTV.wav")
DATA_CSV = os.path.join(BASE_DIR, "data.csv")
RPICAM_BIN = "/usr/bin/rpicam-still"
SLOWFRAME_LIST_TIMEOUT_SECONDS = 15
MMSSTV_LIBRARY_ENV_VAR = "SLOWFRAME_MMSSTV_LIB"
MMSSTV_DISABLE_ENV_VAR = "SLOWFRAME_NO_MMSSTV"

# SlowFrame output settings
SLOWFRAME_AUDIO_FORMAT = "wav"
SLOWFRAME_SAMPLE_RATE = 22050
SLOWFRAME_ASPECT_MODE = "center"
SLOWFRAME_VERBOSE = False

# SlowFrame text overlay settings
SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY = True
SLOWFRAME_TIMESTAMP_OVERLAY_SIZE = 18
SLOWFRAME_TIMESTAMP_OVERLAY_POSITION = "LT"
SLOWFRAME_TIMESTAMP_OVERLAY_COLOR = "255,255,255"
SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND = True
SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR = "0,0,0"
SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY = 0.7

# Optional station ID overlay / CW ID
STATION_CALLSIGN = ""
SLOWFRAME_ENABLE_CALLSIGN_OVERLAY = False
SLOWFRAME_CALLSIGN_OVERLAY_SIZE = 22
SLOWFRAME_CALLSIGN_OVERLAY_POSITION = "LB"
SLOWFRAME_CALLSIGN_OVERLAY_COLOR = "255,255,255"
SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND = True

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
        requires_mmsstv=True,
        fallback_mode="bw24",
        description="Ultra-fast MMSSTV monochrome status frame for tight duty-cycle budgets.",
    ),
    "robot12bw": ModeProfile(
        name="robot12bw",
        duration_seconds=12,
        cooldown_seconds=90,
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
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Higher-quality MMSSTV mode with a larger cooldown budget.",
    ),
    "pd160": ModeProfile(
        name="pd160",
        duration_seconds=160,
        cooldown_seconds=660,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="Slower MMSSTV quality mode for longer detail passes.",
    ),
    "pd180": ModeProfile(
        name="pd180",
        duration_seconds=180,
        cooldown_seconds=720,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="High-detail MMSSTV mode, best for occasional mission snapshots.",
    ),
    "fax480": ModeProfile(
        name="fax480",
        duration_seconds=180,
        cooldown_seconds=720,
        requires_mmsstv=True,
        fallback_mode="m1",
        description="High-detail MMSSTV mode best reserved for test windows.",
    ),
}

TRANSMIT_SCHEDULE_PROFILE = "hab_balanced"
TRANSMIT_SCHEDULE_PROFILES = {
    "hab_fast": (
        "robot12bw",
        "r36",
        "m2",
        "robot12bw",
        "r36",
        "pd50",
    ),
    "hab_balanced": (
        "robot12bw",
        "r36",
        "m2",
        "pd90",
        "s2",
        "r72",
        "m1",
    ),
    "hab_quality": (
        "r36",
        "pd90",
        "robot12bw",
        "pd120",
        "robot12bw",
        "r36"
    ),
}
TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES.get(
    TRANSMIT_SCHEDULE_PROFILE,
    TRANSMIT_SCHEDULE_PROFILES["hab_balanced"],
)


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


def parse_args():
    description = (
        "HamWing SSTV HAB payload controller.\n\n"
        "Captures images with rpicam-still, encodes to SSTV audio using SlowFrame,\n"
        "and transmits over a DRA818 module on the HamWing carrier board.\n"
        "Supports a configurable schedule of SSTV modes with duty-cycle protection\n"
        "and optional MMSSTV library modes (pd50, pd90, pd120, pd160, pd180, fax480)."
    )
    epilog = (
        "EXAMPLES\n"
        "  Normal HAB mission with default balanced schedule:\n"
        "    python3 pi_sstv.py\n\n"
        "  Mission with fast schedule and station callsign overlay:\n"
        "    python3 pi_sstv.py --schedule hab_fast --callsign W1AW-11\n\n"
        "  Mission with quality schedule, 300 captures, 15s interval:\n"
        "    python3 pi_sstv.py --schedule hab_quality --total 300 --interval 15\n\n"
        "  Test the full pipeline for r36 (capture + encode + transmit):\n"
        "    python3 pi_sstv.py --test r36\n\n"
        "  Test pd90 encode only — no radio TX, safe on a bench:\n"
        "    python3 pi_sstv.py --test pd90 --no-tx\n\n"
        "  Test a specific mode using an existing image instead of the camera:\n"
        "    python3 pi_sstv.py --test m1 --test-image /home/pi-user/photo.jpg --no-tx\n\n"
        "  Brief PTT keying test (keys transmitter for 1 second):\n"
        "    python3 pi_sstv.py --ptt-test\n\n"
        "  Brief PTT keying test with custom key duration:\n"
        "    python3 pi_sstv.py --ptt-test 0.75\n\n"
        "  List all known SSTV mode profiles:\n"
        "    python3 pi_sstv.py --list-modes\n\n"
        "  List all schedule presets:\n"
        "    python3 pi_sstv.py --list-schedules\n\n"
        "  Full debug logging to file and stdout:\n"
        "    python3 pi_sstv.py --debug --log-file /home/pi-user/mission.log\n\n"
        "  Quiet mode logging to file only (no stdout):\n"
        "    python3 pi_sstv.py --quiet-log-file /home/pi-user/mission.log\n\n"
        "  Conservative thermal profile with shorter mission:\n"
        "    python3 pi_sstv.py --total 100 --cooldown-scale 1.5\n\n"
        "  Disable MMSSTV library (native modes only):\n"
        "    python3 pi_sstv.py --no-mmsstv\n"
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
        default=TEST_IMAGE,
        help=f"Fallback image used when the camera is unavailable. Default: {TEST_IMAGE}.",
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

    args = parser.parse_args()

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
    print("Schedule presets:\n")
    for preset_name, modes in TRANSMIT_SCHEDULE_PROFILES.items():
        active_marker = "  (active)" if preset_name == TRANSMIT_SCHEDULE_PROFILE else ""
        sequence = " → ".join(modes)
        print(f"  {preset_name:<18}{active_marker}")
        print(f"    {sequence}\n")


def wait_for_file(path, timeout=5):
    """Wait until file exists and has non-zero size or timeout (seconds)."""
    start = time.time()
    while time.time() - start < timeout:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            return True
        time.sleep(0.1)
    return False


def build_text_overlay(text, size, position, color, background, background_color=None, background_opacity=None):
    overlay_parts = [
        f"text:{text}",
        f"size:{size}",
        f"pos:{position}",
        f"color:{color}",
    ]

    if background:
        overlay_parts.append("bg:yes")
        if background_color:
            overlay_parts.append(f"bgcolor:{background_color}")
        if background_opacity is not None:
            overlay_parts.append(f"bgopacity:{background_opacity}")

    return "|".join(overlay_parts)


def get_native_modes():
    return {name for name, profile in MODE_PROFILES.items() if not profile.requires_mmsstv}


def discover_slowframe_capabilities():
    state = RuntimeState(available_modes=set(get_native_modes()))

    if os.environ.get(MMSSTV_DISABLE_ENV_VAR):
        log("MMSSTV support disabled by environment; using native modes only.")
        return state

    command = [SLOWFRAME_BIN, "-L"]
    if SLOWFRAME_VERBOSE:
        command.insert(1, "-v")

    log_debug(f"SlowFrame capability probe command: {' '.join(command)}")

    try:
        result = run(
            command,
            capture_output=True,
            text=True,
            check=True,
            timeout=SLOWFRAME_LIST_TIMEOUT_SECONDS,
        )
    except Exception as error:
        log(f"SlowFrame capability probe failed; using native modes only: {error}")
        return state

    output = "\n".join(filter(None, [result.stdout, result.stderr]))
    log_debug(f"SlowFrame -L raw output:\n{output}")

    for line in output.splitlines():
        stripped_line = line.strip()

        if stripped_line.lower().startswith("mmsstv library detected:"):
            state.mmsstv_library_detected = True
            state.mmsstv_library_path = stripped_line.split(":", 1)[1].strip()

        mode_match = re.match(r"^([a-z0-9_]+)\s+-", stripped_line.lower())
        if mode_match:
            state.available_modes.add(mode_match.group(1))

    mmsstv_status = "MMSSTV enabled" if state.mmsstv_library_detected else "native-only"
    log(f"SlowFrame discovery: {len(state.available_modes)} modes available ({mmsstv_status})")

    if state.mmsstv_library_path:
        log(f"MMSSTV library: {state.mmsstv_library_path}")

    return state


def resolve_mode_name(requested_mode, available_modes):
    current_mode = requested_mode
    visited_modes = set()

    while current_mode and current_mode not in visited_modes:
        visited_modes.add(current_mode)
        if current_mode in available_modes:
            if current_mode != requested_mode:
                log_debug(f"Mode resolution: {requested_mode} → {current_mode} (via fallback chain)")
            return current_mode

        current_profile = MODE_PROFILES.get(current_mode)
        next_mode = current_profile.fallback_mode if current_profile else None
        log_debug(f"Mode resolution: {current_mode} not available, trying fallback: {next_mode}")
        current_mode = next_mode

    if "r36" in available_modes:
        log_debug(f"Mode resolution: fallback chain exhausted for {requested_mode}, using r36")
        return "r36"

    fallback = next(iter(sorted(available_modes)))
    log_debug(f"Mode resolution: r36 unavailable, using first sorted mode: {fallback}")
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

    if SLOWFRAME_ENABLE_TIMESTAMP_OVERLAY:
        command.extend([
            "-T",
            build_text_overlay(
                text=timestamp_message,
                size=SLOWFRAME_TIMESTAMP_OVERLAY_SIZE,
                position=SLOWFRAME_TIMESTAMP_OVERLAY_POSITION,
                color=SLOWFRAME_TIMESTAMP_OVERLAY_COLOR,
                background=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND,
                background_color=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_COLOR,
                background_opacity=SLOWFRAME_TIMESTAMP_OVERLAY_BACKGROUND_OPACITY,
            ),
        ])

    if SLOWFRAME_ENABLE_CALLSIGN_OVERLAY and STATION_CALLSIGN:
        command.extend([
            "-T",
            build_text_overlay(
                text=STATION_CALLSIGN,
                size=SLOWFRAME_CALLSIGN_OVERLAY_SIZE,
                position=SLOWFRAME_CALLSIGN_OVERLAY_POSITION,
                color=SLOWFRAME_CALLSIGN_OVERLAY_COLOR,
                background=SLOWFRAME_CALLSIGN_OVERLAY_BACKGROUND,
            ),
        ])

    return command


def setup_gpio():
    log_debug(f"GPIO setup: mode=BCM, PTT={DRA818_PTT_PIN}, PD={DRA818_POWER_DOWN_PIN}, HL={DRA818_POWER_LEVEL_PIN}")
    GPIO.setmode(GPIO_PIN_MODE)
    GPIO.cleanup()
    GPIO.setup(DRA818_PTT_PIN, GPIO.OUT, initial=GPIO.HIGH)       # PTT idle = HIGH (unkeyed)
    GPIO.setup(DRA818_POWER_LEVEL_PIN, GPIO.OUT, initial=GPIO.LOW) # HL LOW = low power
    GPIO.setup(DRA818_POWER_DOWN_PIN, GPIO.OUT, initial=GPIO.LOW)  # PD LOW = radio off
    log_debug("GPIO setup complete: PTT=HIGH(idle), HL=LOW(low-power), PD=LOW(off)")


def capture_image(output_path):
    cmd = [
        RPICAM_BIN,
        "--nopreview",
        "-o", output_path,
        "--quality", "93",
        "--metering", "average",
        "--exposure", "normal",
        "--awb", "auto",
        "--lens-position", "0",
    ]
    log_debug(f"rpicam-still command: {' '.join(cmd)}")
    try:
        run(cmd, check=True)
        size = os.path.getsize(output_path)
        log(f"Image captured: {output_path} ({size} bytes)")
        return output_path
    except Exception as error:
        log(f"Image capture failed, using test image: {error}")
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
    cmd = build_slowframe_command(image_path, output, timestamp_message, mode_name)
    log_debug(f"SlowFrame command: {' '.join(cmd)}")
    run(cmd, check=True)
    size = os.path.getsize(output)
    log_debug(f"SSTV WAV produced: {output} ({size} bytes)")
    time.sleep(SSTV_CONVERSION_SETTLE_SECONDS)


def transmit_sstv_audio(wav_path=None):
    audio_path = wav_path or SSTV_WAV
    log_debug(f"TX start: PD→HIGH (radio wake), file={audio_path}")
    transmit_started_at = time.monotonic()
    GPIO.output(DRA818_POWER_DOWN_PIN, GPIO.HIGH)
    time.sleep(RADIO_WAKE_DELAY_SECONDS)
    log_debug("TX: PTT→LOW (keyed)")
    GPIO.output(DRA818_PTT_PIN, GPIO.LOW)
    time.sleep(PTT_KEY_DELAY_SECONDS)

    try:
        run(["aplay", audio_path], check=True)
    finally:
        time.sleep(POST_PLAYBACK_DELAY_SECONDS)
        log_debug("TX: PTT→HIGH (unkeyed), PD→LOW (radio off)")
        GPIO.output(DRA818_PTT_PIN, GPIO.HIGH)
        GPIO.output(DRA818_POWER_DOWN_PIN, GPIO.LOW)

    return time.monotonic() - transmit_started_at


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
    captured_image_path = capture_image(capture_path)
    timestamp_message = current_time.strftime("%Y.%m.%d - %H:%M:%S")

    write_csv(index, picture_time)
    log(f"Capture #{index + 1} recorded: {picture_time}")

    return captured_image_path, timestamp_message


def run_test_pipeline(mode_name: str, args, runtime_state: RuntimeState):
    """Execute a single-shot pipeline test.

    Validates each stage — camera capture, SSTV encode, and optionally radio TX —
    using a consistent file prefix so every output artifact can be identified and
    inspected.  Exits with code 0 on full success, 1 on any stage failure.
    """
    test_id = datetime.utcnow().strftime("TEST-%Y%m%d-%H%M%S")
    output_dir = args.output_dir
    capture_path = os.path.join(output_dir, f"{test_id}-capture.jpg")
    resolved_mode = resolve_mode_name(mode_name, runtime_state.available_modes)
    wav_path = os.path.join(output_dir, f"{test_id}-{resolved_mode}.wav")

    separator = "=" * 56
    log(separator)
    log(f"Test pipeline  run-id : {test_id}")
    log(f"               mode   : {resolved_mode}" + (f"  (fallback from {mode_name})" if resolved_mode != mode_name else ""))
    log(f"               capture: {capture_path}")
    log(f"               wav    : {wav_path}")
    log(f"               no-tx  : {args.no_tx}")
    log(separator)

    profile = MODE_PROFILES.get(resolved_mode)
    if not profile:
        log(f"ERROR: resolved mode '{resolved_mode}' not found in MODE_PROFILES")
        sys.exit(1)

    # --- Stage 1: Image capture ---
    log("Stage 1/3  Image capture")
    captured = capture_image(capture_path)
    if captured == TEST_IMAGE:
        log(f"  NOTE  camera unavailable; test image used: {TEST_IMAGE}")
        capture_path = TEST_IMAGE
    else:
        size = os.path.getsize(capture_path)
        log(f"  PASS  {capture_path}  ({size:,} bytes)")

    # --- Stage 2: SSTV encode ---
    log("Stage 2/3  SSTV encode")
    timestamp_message = datetime.utcnow().strftime("%Y.%m.%d - %H:%M:%S UTC [TEST]")
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
    separator = "-" * 52

    log(separator)
    log(f"Mission start — schedule: {TRANSMIT_SCHEDULE_PROFILE}")
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

    log(separator)


def main():
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

    # Apply CLI overrides to module-level configuration
    global TRANSMIT_SCHEDULE, TRANSMIT_SCHEDULE_PROFILE
    global PIC_TOTAL, PIC_INTERVAL
    global STATION_CALLSIGN, SLOWFRAME_ENABLE_CALLSIGN_OVERLAY
    global COOLDOWN_SCALE_FACTOR, MAX_TRANSMIT_DUTY_CYCLE, MIN_CAPTURES_BETWEEN_TRANSMISSIONS
    global SLOWFRAME_AUDIO_FORMAT, SLOWFRAME_SAMPLE_RATE, SLOWFRAME_ASPECT_MODE
    global TIMESTAMPED_DIR, SLOWFRAME_BIN, TEST_IMAGE, SSTV_WAV

    TRANSMIT_SCHEDULE_PROFILE = args.schedule
    TRANSMIT_SCHEDULE = TRANSMIT_SCHEDULE_PROFILES.get(args.schedule, TRANSMIT_SCHEDULE_PROFILES["hab_balanced"])
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
