# Feather_M0_Testing Module Summary

## Purpose

Feather_M0_Testing is a hardware bring-up and diagnostics sketch for the HamWing dual-radio board (DRA818 VHF + DRA818 UHF) on Feather M0.

It focuses on:
- Safe GPIO initialization for PD/PTT/H-L control lines
- Radio AT handshake and default programming (frequency, squelch, volume, filters)
- PTT validation and polarity diagnostics
- Optional RX squelch scan testing
- UHF UART mapping diagnosis and auto-retry behavior
- Optional GPIO handoff to Raspberry Pi control after startup configuration

## What the sketch configures by default

- Active module selection: BOTH radios (VHF + UHF)
- VHF defaults: 144.3900 MHz TX/RX, squelch 4, volume 6, 12.5 kHz bandwidth
- UHF defaults: 440.8000 MHz TX/RX, squelch 4, volume 6, 12.5 kHz bandwidth
- PTT polarity: active-low at MCU pin (LOW keys TX)
- RF power: high power enabled in test mode
- Development logging: enabled over USB Serial (115200)

## Pin mapping used in this sketch

- Shared control:
  - D12: PD (power-down sleep control), shared by both radios
  - D18: H/L power level
- VHF path:
  - D0/D1: Serial1 RX/TX to VHF module
  - D16: VHF PTT
  - D14: VHF SQL (squelch detect)
- UHF path:
  - D11/D10: Serial2 preferred mapping RX/TX (from/to UHF module)
  - D17: UHF PTT
  - D15: UHF SQL
- Other:
  - LED_BUILTIN: status/heartbeat/test indication
  - A7: battery voltage sense

## Runtime behavior

## setup()

1. Starts Serial (115200), Serial1 (VHF), and Serial2 (UHF preferred mapping).
2. Initializes GPIO for PD/PTT/H-L/SQL and forces safe non-TX state.
3. Prints a detailed configuration banner.
4. Runs released-line diagnostics for PTT nets.
5. Applies radio defaults:
   - VHF on Serial1
   - UHF on preferred Serial2 mapping, then fallback mapping if needed
6. Optional: releases PD/PTT/H-L lines for Raspberry Pi ownership if handoff mode is enabled.

## loop()

1. Keeps PD high (radio awake), enforces no unintended TX state.
2. Prints battery and status snapshots.
3. Re-runs released-line diagnostics (PTT + UHF UART).
4. Retries UHF configuration if previous attempts failed.
5. Runs one selected test path:
   - PTT polarity probe (default in this sketch)
   - RX scan test
   - Raw GPIO PTT sweep
   - Normal timed PTT test
6. Prints post-test status and performs heartbeat LED fades during inter-loop delay.

## Test modes and key compile-time toggles

These are set near the top of the sketch:

- ACTIVE_MODULE
  - MODULE_VHF, MODULE_UHF, MODULE_BOTH
- NO_TX_MODE
  - 1 disables all PTT keying for safe bench work
- PTT_TEST_ENABLED
  - Enables standard timed VHF then UHF key test
- PTT_POLARITY_PROBE
  - Forces explicit LOW/HIGH phase testing to diagnose real key polarity
- PTT_GPIO_SWEEP_MODE
  - Bypasses macros and sweeps raw V/U PTT pin levels in phases
- SCAN_TEST_MODE
  - Runs RX scan around default frequencies using SQL transitions
- TRY_UHF_BOTH_UART_MAPPINGS
  - Enables preferred + fallback UHF UART mapping attempts
- HANDOFF_GPIO_TO_PI_AFTER_CONFIG
  - Releases PD/PTT/H-L lines to Pi after startup config
- HANDOFF_REQUIRE_CONFIG_OK
  - Requires successful startup config before handoff
- DEVMODE
  - Enables verbose [debug] logs

Only one loop test path runs at a time due to preprocessor ordering:
- PTT_POLARITY_PROBE (highest priority)
- else SCAN_TEST_MODE
- else PTT_GPIO_SWEEP_MODE
- else runPttTest()

## UHF-specific diagnostics implemented

- Preferred/fallback UART mapping support with runtime selection
- Raw AT+DMOCONNECT probing during retries
- PD cycle recovery path before reattempting defaults
- Clear serial messages for silent UART vs malformed response scenarios
- Released-line D10/D11 GPIO checks to detect contention/load from external devices

## Safety behavior

- Sketch starts in a non-transmitting state.
- setPttIdle() is called frequently before and after tests.
- ensurePdReadyForPtt() blocks keying if PD is not awake.
- NO_TX_MODE can hard-disable all TX actions for bench diagnostics.

## Typical usage flow

1. Open Serial Monitor at 115200.
2. Flash sketch and observe startup config banner and radio handshake results.
3. Confirm both radios configure successfully.
4. Use default polarity probe output to confirm true keying behavior.
5. If UHF fails, review mapping retry logs and released-line diagnostics.
6. Enable NO_TX_MODE for safe dry diagnostics without RF transmission.

## Notes for integration with Raspberry Pi

This sketch can be used as a one-time radio initializer before Pi ownership.
If HANDOFF_GPIO_TO_PI_AFTER_CONFIG is enabled, Feather releases control lines and loop actions are disabled so Pi can fully own PD/PTT/H-L nets.
