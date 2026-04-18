# HamWing_RadioConfig — Feather M0 Radio Configurator

## Overview

A clean, production-ready sketch for configuring VHF and/or UHF DRA818 radio modules on the HamWing board via Feather M0. The Feather initializes both radios at startup, then releases PD/PTT/H/L control lines to the Raspberry Pi for exclusive operation.

## Key Features

- **Single Configuration Pass:** Radios configured once at startup, then hands off to Pi
- **Modular Module Selection:** Configure VHF only, UHF only, or both
- **Flexible Radio Settings:** Easy-to-edit frequency, volume, squelch, filters, CTCSS
- **Smart LED Feedback:**
  - Slow fade in/out: Configuration successful, idle
  - Fast blink (100ms): Configuration failed, check logs
  - Solid ON: Pi is actively keying transmitter
- **Pi Line Monitoring:** Feather reads-only monitors PD/PTT/H/L to detect Pi TX activity
- **Debug Mode:** Optional comprehensive status output showing pin states, config progress, timestamps
- **Safe GPIO Release:** After config, PD/PTT/H/L pins are set to INPUT so Pi is sole owner

## Build Configuration

Edit these flags at the top of the sketch:

```cpp
#define ACTIVE_MODULE MODULE_BOTH    // MODULE_VHF, MODULE_UHF, or MODULE_BOTH
#define DEBUG_MODE 1                 // 1 = verbose output, 0 = minimal
```

## Radio Configuration

Edit `VHF_CONFIG` and `UHF_CONFIG` structures to set your frequencies and preferences:

```cpp
RadioConfig VHF_CONFIG = {
  144.3900,   // TX frequency (MHz)
  144.3900,   // RX frequency (MHz)
  4,          // Squelch (0-8)
  6,          // Volume (1-8)
  0,          // CTCSS TX (0 = none, 1-38 = tone)
  0,          // CTCSS RX (0 = none, 1-38 = tone)
  DRA818_12K5,// Bandwidth (DRA818_12K5 or DRA818_25K)
  true,       // Pre-filter enable
  true,       // High-pass filter enable
  true        // Low-pass filter enable
};
```

## Pin Assignments (Feather M0 + HamWing)

All pins are predefined in the sketch:

| Pin | Function | Owner | Notes |
|-----|----------|-------|-------|
| D0  | VHF RX   | Feather | Serial1 SERCOM0 |
| D1  | VHF TX   | Feather | Serial1 SERCOM0 |
| D10 | UHF TX   | Feather | Serial2 SERCOM1 PAD2 |
| D11 | UHF RX   | Feather | Serial2 SERCOM1 PAD0 |
| D12 | PD       | **Pi**  | Shared, released after config |
| D14 | VHF SQL  | Feather | Analog input, read-only |
| D15 | UHF SQL  | Feather | Analog input, read-only |
| D16 | VHF PTT  | **Pi**  | Shared, released after config |
| D17 | UHF PTT  | **Pi**  | Shared, released after config |
| D18 | H/L      | **Pi**  | Shared, released after config |
| LED | Status   | Feather | Slow fade (success) or fast blink (fail) |

**Bold** = Pi owns and controls these lines. Feather reads them for monitoring only.

## Serial Output

### On Success
```
HamWing_RadioConfig starting...
===== HamWing Radio Configuration Banner =====
Module selection: BOTH (VHF + UHF)
...
[UART] VHF Serial1 opened
[UART] UHF Serial2 opened
[VHF] Starting configuration...
[VHF] Handshake OK
[VHF] Frequency/squelch OK
[VHF] Volume OK
[VHF] Filters OK
VHF: Configuration COMPLETE
[UHF] Starting configuration...
[UHF] Handshake OK
[UHF] Frequency/squelch OK
[UHF] Volume OK
[UHF] Filters OK
UHF: Configuration COMPLETE

All configured radios are READY. Handing off control to Raspberry Pi.

========== STATUS BANNER ==========
Config state VHF/UHF: OK / OK
PD=1 (radios ACTIVE), H/L=0 (0.5W low power), VPTT=1 (idle), UPTT=1 (idle)
VBAT: 4.20 V
==================================
```

### On Failure
```
VHF: Handshake FAILED

Configuration FAILED. Check wiring and module power.
```

## Raspberry Pi Integration

After the Feather successfully configures the radios, the Pi gains exclusive control:

### Pi GPIO Ownership After Feather Startup

```python
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

# These are now safe for the Pi to own exclusively:
GPIO.setup(4, GPIO.OUT, initial=GPIO.HIGH)      # PD (power-down)
GPIO.setup(27, GPIO.OUT, initial=GPIO.HIGH)     # PTT (VHF + UHF)
GPIO.setup(22, GPIO.OUT, initial=GPIO.LOW)      # H/L (power level)
```

Recommended Pi boot sequence:
1. Disable UART getty services: `sudo systemctl disable serial-getty@ttyAMA0.service`
2. Set `enable_uart=0` in `/boot/config.txt` to prevent Pi UART console contention
3. Let Feather start and configure radios (takes ~5 seconds)
4. Pi application then uses GPIO 4/27/22 for PTT/PD/HL

## LED Feedback States

| State | Meaning | Blink Pattern |
|-------|---------|---------------|
| Slow fade in/out | Config success, idle | ~1s up, ~1s down |
| Fast blink | Config failed | 100ms ON, 100ms OFF |
| Solid ON | Pi is transmitting | Constant HIGH |
| OFF | Should not occur; indicates error or power loss | |

## Debug Mode

Set `DEBUG_MODE 1` to enable comprehensive logging:

- Configuration progress (each AT command step)
- Pin state reads (PD, H/L, PTT, SQL)
- Battery voltage (VBAT)
- Pi TX detection and release
- Periodic status banner (every 10 seconds)

## Troubleshooting

### VHF Configuration Fails
- Check D0/D1 wiring to VHF module
- Verify VHF module power (PD pin should read HIGH)
- Ensure Serial1 is not being used elsewhere

### UHF Configuration Fails
- Remove Raspberry Pi hat if connected (UHF UART contention on D10/D11)
- Check D10/D11 wiring to UHF module via JP1/JP2
- Verify JP1 and JP2 are soldered on the HamWing board
- If still failing with hat removed, UHF module may be unpowered or damaged

### LED Not Responding
- Set `DEBUG_MODE 1` and check serial output
- Verify `D13` or `LED_BUILTIN` is not already in use by another peripheral
- Test LED manually: upload a simple blink sketch to confirm hardware

### Pi PTT Not Working After Config
- Ensure Pi UART services are disabled: `sudo systemctl disable serial-getty@ttyAMA0.service`
- Confirm Feather startup serial shows "Handing off control to Raspberry Pi"
- Check Pi GPIO ownership in your application; make sure GPIO 27 is being driven LOW to key PTT

## Extending the Sketch

To add features (e.g., SQL baseline calibration, frequency scanning):

1. Add a new function in the "RADIO CONFIGURATION" section
2. Call it from `setup()` after successful radio config
3. Add debug output via `debugPrint()` for visibility
4. If you need loop-time monitoring, add it to `loop()` within the gConfigurationComplete block

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│ Feather M0 + HamWing                                        │
│                                                              │
│  Serial1 ──→ VHF Module (D0/D1)                            │
│  Serial2 ──→ UHF Module (D10/D11)                          │
│                                                              │
│  D12 (PD)   ═══ Released to Pi after config                │
│  D16 (VPTT) ═══ Released to Pi after config                │
│  D17 (UPTT) ═══ Released to Pi after config                │
│  D18 (H/L)  ═══ Released to Pi after config                │
│                                                              │
│  LED (D13) ← Slow fade (success) or fast blink (failure)    │
└─────────────────────────────────────────────────────────────┘
           ↓ (HamWing connector/hat header)
┌─────────────────────────────────────────────────────────────┐
│ Raspberry Pi Zero 2W                                        │
│                                                              │
│  GPIO 4  ← PD (power-down control)                          │
│  GPIO 27 ← PTT (key transmitter)                            │
│  GPIO 22 ← H/L (power level 1W/0.5W)                        │
│  GPIO 12/13 → LPF → DRA818 mic input (audio from Pi app)    │
└─────────────────────────────────────────────────────────────┘
```
