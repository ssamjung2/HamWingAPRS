# pi_sstv.py Configuration + CLI Reference

This page is a full reference for the current pi_sstv.py runtime behavior.
It documents all config-file keys and all CLI options, with valid values, constraints, defaults, and guidance.

## 1) Configuration Model

### 1.1 Source precedence

When the script runs, values are applied in this order:

1. Built-in defaults in pi_sstv.py
2. Config file values (if loaded)
3. CLI flags (explicit overrides)

CLI always wins over config if both are provided.

### 1.2 Config file loading behavior

- Explicit load:
  - python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg
- Auto-load fallback:
  - If --config is omitted and /home/pi-user/pi_sstv.cfg exists, it is auto-loaded.

### 1.3 Generate template

- Generate a fully-commented template:
  - python3 pi_sstv.py --generate-config
- Generate to custom path:
  - python3 pi_sstv.py --generate-config /path/to/pi_sstv.cfg

### 1.4 Config parsing notes

- Unknown keys are ignored (forward compatibility).
- Invalid typed values (bad int/float/bool/enum) cause immediate startup error.
- Some documented [logging] keys are intentionally not loaded from config; logging control is CLI-driven.

## 2) Schedule and TX Gating Overview

A transmit attempt for each capture is controlled by:

1. Schedule slot selection from [mission] schedule.
2. Mode availability/fallback resolution.
3. Gate checks:
   - min captures gate
   - selected cooldown method gate
4. Encode/TX success.

Schedule index advances only after a successful transmit.

## 3) Cooldown Methods (Current Runtime)

Set with:
- [radio] cooldown_method
- CLI: --cooldown-method

Valid methods:
- fixed
- adaptive_dutycycle
- adaptive_avg_dutycycle
- estimated

### 3.1 fixed

- Cooldown is static:
  - cooldown = fixed_tx_cooldown_seconds
- Best when deterministic cadence is preferred.

### 3.2 adaptive_dutycycle

- Uses last TX duration and duty target:
  - cooldown = last_tx_duration * ((1 - duty) / duty)
- Good general-purpose dynamic pacing.

### 3.3 adaptive_avg_dutycycle

- Uses total TX duration of the active schedule block.
- Allocates cooldown proportional to each mode's share of total schedule TX time.
- Good when you want fairness across mixed short/long modes.

### 3.4 estimated

- Starts with adaptive duty cooldown.
- Multiplies by a simplified thermal factor using:
  - TX power level
  - PCB/air heat transfer coefficients
  - mission timeline estimate (flight/freefall)
  - safety factor
- Best when thermal behavior needs conservative modeling.

### 3.5 Global cooldown multiplier

- Applied to all methods:
  - [radio] cooldown_scale_factor
  - CLI: --cooldown-scale

## 4) Config File Reference (All Keys)

## [paths]

| Key | Type | Default | Guidance |
|---|---|---|---|
| output_dir | string path | /home/pi-user/Desktop/HAB | Output folder for captures/WAV artifacts. |
| slowframe | string path | /home/pi-user/Desktop/Slowframe/bin/slowframe | SlowFrame binary path override. |
| test_image | string path | /home/pi-user/pi-sstv/test.jpg | Fallback/bench image source. |
| data_csv | string path | /home/pi-user/data.csv | Capture index log path. |

## [mission]

| Key | Type | Valid values | Default | Guidance |
|---|---|---|---|---|
| schedule | enum | hab_climb, hab_rapid, hab_cruise, hab_float | hab_cruise | Choose mode rotation profile for mission phase. |
| total | int | > 0 | 500 | Number of capture cycles before mission exit. |
| interval | float seconds | > 0 recommended | 10 | Capture interval; affects schedule gate timing. |
| callsign | string | non-empty required for encode/test/mission workflows | empty by default | Mandatory for overlay-bearing workflows. |
| min_captures_between_transmissions | int | >= 0 | 12 | Hard minimum capture cycles between TX attempts. |
| no_tx | bool | true/false | false | Appears in generated template comments, but is not currently loaded from config; use CLI --no-tx. |

Schedule preset value definitions:

- hab_climb: robot8bw -> robot12bw -> bw24 -> m4 -> robot12bw -> r36
- hab_rapid: robot12bw -> m4 -> r36 -> robot12bw -> m4 -> pd50
- hab_cruise: robot12bw -> r36 -> m2 -> pd90 -> s2 -> r72 -> m1 -> pd120
- hab_float: r36 -> pd90 -> robot12bw -> pd120 -> robot12bw -> pd180 -> pd240 -> pd290

## [radio]

| Key | Type | Valid values / range | Default | Guidance |
|---|---|---|---|---|
| band | enum | vhf, uhf, both | vhf | Select PTT line set for TX. |
| tx_power_level | enum | low, high | low | Select DRA818 H/L behavior. |
| pd_idle_mode | enum | release, sleep | release | release hands PD back to Feather; sleep forces LOW. |
| cooldown_method | enum | fixed, adaptive_dutycycle, adaptive_avg_dutycycle, estimated | adaptive_dutycycle | Core TX pacing method. |
| fixed_tx_cooldown_seconds | float seconds | >= 0 | 30.0 | Used only by fixed method. |
| max_transmit_duty_cycle | float fraction | > 0 and <= 1.0 | 0.35 | Duty target for adaptive methods. |
| cooldown_scale_factor | float | > 0 recommended | 1.0 | Global cooldown aggressiveness multiplier. |
| estimated_flight_duration_minutes | float minutes | > 0 | 120.0 | Mission length estimate for estimated method. |
| estimated_freefall_minutes | float minutes | >= 0 | 30.0 | End-of-flight descent window for estimated method. |
| estimated_pcb_heat_transfer_coefficient | float | > 0 | 0.4 | PCB heat transfer coefficient, W/(m^2 K). |
| estimated_air_heat_transfer_coefficient | float | > 0 | 10.0 | Sea-level still-air coefficient, W/(m^2 K). |
| estimated_min_air_density_factor | float fraction | 0.05..1.0 | 0.22 | Convective reduction factor near peak altitude. |
| estimated_effective_thermal_area_m2 | float area | > 0 | 0.0030 | Effective thermal area estimate. |
| estimated_tx_heat_power_low_w | float watts | > 0 | 0.60 | Heat estimate at low TX power. |
| estimated_tx_heat_power_high_w | float watts | > 0 | 1.10 | Heat estimate at high TX power. |
| estimated_cooldown_safety_factor | float | > 0 | 1.10 | Conservative multiplier for estimated method. |
| radio_wake_delay_seconds | float seconds | >= 0 recommended | 1.0 | Delay after PD high before keying PTT. |
| ptt_key_delay_seconds | float seconds | >= 0 recommended | 0.1 | Delay after keying PTT before audio. |
| post_playback_delay_seconds | float seconds | >= 0 recommended | 0.5 | Delay after playback before unkeying. |

## [capture]

| Key | Type | Valid values | Default | Guidance |
|---|---|---|---|---|
| quality | int | 1..100 | 93 | JPEG quality for rpicam-still capture. |
| metering | string | camera-supported modes | matrix | matrix is recommended for HAB contrast scenes. |
| exposure | string | camera-supported modes | sport | sport reduces motion blur in gondola movement. |
| awb | string | camera-supported modes | auto | auto adapts through flight light changes. |
| capture_file_timeout | int seconds | >= 0 | 8 | Wait before fallback to test image. |

## [encode]

| Key | Type | Valid values | Default | Guidance |
|---|---|---|---|---|
| format | enum | wav, aiff, ogg | wav | wav required for direct aplay TX pipeline. |
| sample_rate | int Hz | positive int | 22050 | 22050 is standard SSTV sample rate. |
| aspect | enum | center, pad, stretch | center | center avoids distortion. |
| verbose | bool | true/false | false | Enables verbose SlowFrame diagnostics. |
| sstv_conversion_settle_seconds | float seconds | >= 0 | 0.5 | Buffer before playback after encode. |

## [overlay]

| Key | Type | Valid values | Default | Guidance |
|---|---|---|---|---|
| enable_timestamp | bool | true/false | true | Controls timestamp rendering enable. |
| timestamp_size | int | positive int | 11 | Base overlay font size for timestamp. |
| timestamp_position | string | top-left/top-right/bottom-left/bottom-right/top/bottom/left/right/center | top-left | Position hint for merged overlay text path. |
| timestamp_color | string | CSS color or #RRGGBB | white | Foreground color. |
| timestamp_background_color | string | CSS color or #RRGGBB | black | Background bar color. |
| timestamp_background_opacity | int | 0..100 | 50 | Background opacity aliases applied for compatibility. |
| enable_callsign | bool | true/false | false unless callsign present | Callsign overlay enable path. |
| callsign_size | int | positive int | 14 | Base callsign size. |
| callsign_position | string | same as timestamp positions | top-right | Callsign position setting. |
| callsign_color | string | CSS color or #RRGGBB | white | Callsign text color. |
| callsign_background_color | string | CSS color or #RRGGBB | black | Callsign background bar color. |
| callsign_background_opacity | int | 0..100 | 50 | Callsign background opacity. |
| custom_text | string | any text | empty | Replaces default mode/date/time body; callsign still prepended when required. |

## [mmsstv]

| Key | Type | Valid values | Default | Guidance |
|---|---|---|---|---|
| lib_path | string path | valid .so path | empty | Explicitly point to libsstv_encoder.so when needed. |
| disable | bool | true/false | false | Force native-only mode behavior. |

## [alsa]

| Key | Type | Valid values | Default | Guidance |
|---|---|---|---|---|
| playback_device | string | ALSA device spec | env/default-driven | Pin output device (example: plughw:Headphones,0). |
| mixer_device | string | ALSA card/device selector | env/default-driven | Mixer target for amixer controls. |
| mixer_control | string | control name | env/default-driven | Preferred control (PCM, Headphone, Master, etc). |
| target_volume_percent | int | 0..100 | 70 | Guardrail setpoint before TX playback. |
| max_safe_volume_percent | int | 0..100 | 85 | Warning threshold for overdrive risk. |
| enforce_volume | bool | true/false | true | Enable/disable mixer guardrails. |
| aplay_timeout_seconds | int | >= 10 | 360 | Base playback timeout. |
| aplay_timeout_margin_seconds | int | >= 10 | 45 | Margin added to expected duration for timeout. |

## [gps]

| Key | Type | Valid values | Default | Guidance |
|---|---|---|---|---|
| enable | bool | true/false | false | Enables GPS polling and overlay content. |
| device | string path | serial device | /dev/serial0 | UART device used for NMEA reads. |
| baud | int | positive int | 9600 | Typical u-blox default. |
| units | enum | m, ft | m | Overlay altitude unit. |

## [logging]

Keys are documented in generated config but not loaded by load_config.
Runtime logging behavior is controlled by CLI:
- --debug
- --log-file
- --quiet-log-file

## 5) CLI Reference (All Flags)

## Information and guided help

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --explain | TOPIC | none | Print deep reference for one topic and exit. |
| --help-topics | none | false | List explain topics and aliases. |
| --help-quick | none | false | Print short operator startup flow. |
| --help-flight | none | false | Print preflight checklist. |
| --help-examples | none | false | Print command cookbook. |
| --help-all | none | false | Show full argparse reference. |
| --list-modes | none | false | List mode profiles and discovered capabilities. |
| --list-schedules | none | false | List schedule presets and metrics. |
| --generate-config | optional PATH | none | Emit commented config template and exit. |

## Test and diagnostics

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --test | MODE | none | Single-shot capture->encode->optional TX path. |
| --ptt-test | optional SECONDS | none / const 1.0 | GPIO-only PTT key test and exit. |
| --alsa-volume-check | none | false | Mixer guardrail validation run and exit. |

## Mission controls

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --radio | BAND | config/default | vhf, uhf, both. |
| --tx-power | LEVEL | config/default | low or high. |
| --pd-idle | MODE | config/default | release or sleep. |
| --schedule | PRESET | config/default | hab_climb, hab_rapid, hab_cruise, hab_float. |
| --total | N | config/default | Total capture count. |
| --interval | SECS | config/default | Capture interval seconds. |
| --callsign | CALL | config/default | Required for encode/test/mission workflows. |
| --overlay-text | TEXT | config/default | Override body text of merged overlay. |
| --no-tx | none | false | Skip transmit stage (capture+encode only). |

## Radio protection and cooldown model

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --cooldown-method | METHOD | config/default | fixed, adaptive_dutycycle, adaptive_avg_dutycycle, estimated. |
| --fixed-cooldown-seconds | SECS | config/default | Static cooldown for fixed method. |
| --cooldown-scale | FACTOR | config/default | Global multiplier for computed cooldown. |
| --duty-cycle | FRACTION | config/default | Adaptive method target duty; must be >0 and <=1.0. |
| --min-captures | N | config/default | Min capture cycles between TX attempts. |
| --estimated-flight-minutes | MIN | config/default | Estimated mission length for estimated method. |
| --estimated-freefall-minutes | MIN | config/default | Estimated final descent window for estimated method. |

## Encoding controls

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --format | FMT | config/default | wav, aiff, ogg |
| --sample-rate | HZ | config/default | Audio sample rate |
| --aspect | MODE | config/default | center, pad, stretch |

## Path overrides

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --output-dir | PATH | config/default | Artifact output directory |
| --slowframe | PATH | config/default | SlowFrame binary path |
| --test-image | PATH | none | Source image for test mode |

## MMSSTV controls

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --no-mmsstv | none | false | Force native-only mode availability |
| --mmsstv-lib | PATH | none | Explicit libsstv_encoder.so path |

## Logging controls

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --debug | none | false | Enable debug-level logging |
| --log-file | PATH | none | Log to stdout + file |
| --quiet-log-file | PATH | none | Log to file only |

## ALSA controls

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --alsa-playback-device | DEVICE | none | Force playback target |
| --alsa-mixer-device | DEVICE | none | Mixer selector |
| --alsa-mixer-control | CONTROL | none | Mixer control name |
| --alsa-target-volume | PERCENT | none | Guardrail setpoint |
| --alsa-max-safe-volume | PERCENT | none | Safety threshold |
| --no-alsa-volume-guardrails | none | false | Disable mixer guardrail enforcement |

## GPS controls

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --gps | none | false | Enable GPS polling and overlay |
| --gps-device | PATH | /dev/serial0 | UART device path |
| --gps-baud | BAUD | 9600 | GPS baud rate |
| --gps-units | UNITS | m | m or ft |

## Config file controls

| Flag | Arguments | Default | Guidance |
|---|---|---|---|
| --config | PATH | none | Load config before applying CLI overrides |
| --generate-config | optional PATH | none | Write default template and exit |

## 6) Validation and Guardrails

The script enforces these checks:

- --test and --ptt-test cannot be combined
- --alsa-volume-check cannot be combined with --test or --ptt-test
- --ptt-test must be > 0
- --log-file and --quiet-log-file cannot be combined
- --cooldown-method must be a valid method
- --duty-cycle must be > 0 and <= 1.0
- --fixed-cooldown-seconds must be >= 0
- --estimated-flight-minutes must be > 0
- --estimated-freefall-minutes must be >= 0
- [radio] band, tx_power_level, pd_idle_mode, cooldown_method are validated enums
- [gps] units must be m or ft
- [encode] format and aspect are validated enums

## 7) Deprecated Key Warnings

Deprecated config keys are detected and warned during load.

Current deprecated key:
- [radio] rolling_duty_cycle_window_seconds

Behavior:
- warning emitted to stderr
- key ignored

## 8) Practical Use Guidance

- Use fixed for strict deterministic cadence testing.
- Use adaptive_dutycycle for simpler duty-target pacing.
- Use adaptive_avg_dutycycle when your schedule mixes very short and very long modes and you want proportional fairness.
- Use estimated when thermal conservatism matters and you have a realistic flight timeline estimate.
- Keep cooldown_scale_factor as your top-level aggressiveness dial for field tuning.
- Keep min_captures_between_transmissions non-zero to avoid back-to-back TX attempts when capture loop timing is very short.
- Start with --no-tx bench tests whenever changing cooldown model parameters.

## 9) Example Config Snippet (radio section)

```ini
[radio]
band = vhf
tx_power_level = low
pd_idle_mode = release
cooldown_method = estimated
fixed_tx_cooldown_seconds = 30
max_transmit_duty_cycle = 0.35
cooldown_scale_factor = 1.00
estimated_flight_duration_minutes = 120
estimated_freefall_minutes = 30
estimated_pcb_heat_transfer_coefficient = 0.4
estimated_air_heat_transfer_coefficient = 10.0
estimated_min_air_density_factor = 0.22
estimated_effective_thermal_area_m2 = 0.0030
estimated_tx_heat_power_low_w = 0.60
estimated_tx_heat_power_high_w = 1.10
estimated_cooldown_safety_factor = 1.10
radio_wake_delay_seconds = 1.0
ptt_key_delay_seconds = 0.1
post_playback_delay_seconds = 0.5
```

## 10) Example Commands

- Full mission with config:
  - python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg

- Force fixed cooldown for a quick test:
  - python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg --cooldown-method fixed --fixed-cooldown-seconds 30

- Adaptive average duty with conservative scaling:
  - python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg --cooldown-method adaptive_avg_dutycycle --duty-cycle 0.30 --cooldown-scale 1.25

- Estimated thermal model with longer mission profile:
  - python3 pi_sstv.py --config /home/pi-user/pi_sstv.cfg --cooldown-method estimated --estimated-flight-minutes 150 --estimated-freefall-minutes 35
