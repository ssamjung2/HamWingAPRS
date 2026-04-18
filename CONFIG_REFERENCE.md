# HamWing_RadioConfig — Configuration Reference

## Quick Start Customization

Edit these sections in `HamWing_RadioConfig.ino` to customize your deployment.

---

## 1. Frequency Configuration

Locate the `RadioConfig` definitions around line 96-121:

```cpp
RadioConfig VHF_CONFIG = {
  144.3900,   // ← Change these frequencies (MHz)
  144.3900,
  4,          // squelch
  6,          // volume
  0,          // ctcss_tx
  0,          // ctcss_rx
  DRA818_12K5,
  true, true, true
};

RadioConfig UHF_CONFIG = {
  440.8000,   // ← Change these frequencies (MHz)
  440.8000,
  4, 6, 0, 0,
  DRA818_12K5,
  true, true, true
};
```

### Common Amateur Radio Frequencies

| Band | TX Freq | RX Freq | BW | Notes |
|------|---------|---------|-----|-------|
| **2m (VHF)** | | | |
| Simplex | 145.5100 | 145.5100 | 12.5k | Common freq |
| Repeater output | 145.0000+ | 145.6000+ | 12.5k | Area dependent |
| **70cm (UHF)** | | | |
| Simplex | 446.0000 | 446.0000 | 12.5k | Common UHF simplex |
| Repeater output | 442.0000+ | 447.0000+ | 12.5k | Area dependent |

**Note:** Always check local regulations (FCC Part 97 in USA) before using any frequency.

---

## 2. Audio & Filtering Configuration

### Squelch (SQL)

Controls when the radio is muted. Typical values:

- `0` – Always open (never muted)
- `2` – Very sensitive (picks up weak signals)
- `4` – Medium (balanced, recommended for most)
- `6` – Sensitive (requires stronger signals)
- `8` – High squelch (only loud/close signals get through)

Change in config:
```cpp
squelch: 4,  // Change to 2, 4, 6, or 8
```

### Volume

Speaker output level:
- `1` – Quietest
- `4` – Moderate (recommended)
- `6` – Loud
- `8` – Maximum

Change in config:
```cpp
volume: 6,  // Change to 1-8
```

### Bandwidth (DRA818 Modulation)

Determines channel spacing and audio quality:

```cpp
bandwidth: DRA818_12K5,  // Narrow band, better selectivity
// OR
bandwidth: DRA818_25K,   // Wide band, better audio quality
```

- `DRA818_12K5` – 12.5 kHz (required for most amateur repeaters, default)
- `DRA818_25K` – 25 kHz (wide bandwidth, less interference, better quality)

### Filters

Pre/High/Low filters improve audio quality and reduce interference:

```cpp
filter_pre: true,   // Pre-emphasis filter (typical)
filter_high: true,  // High-pass filter (removes low rumble)
filter_low: true,   // Low-pass filter (removes high noise)
```

Recommended settings:
- All `true` for clean audio with filters
- All `false` for raw unfiltered audio (testing only)
- Mix as needed for your specific hardware/location

---

## 3. CTCSS (PL Tone) Configuration

CTCSS (Continuous Tone-Coded Squelch System) requires a specific audio tone to unlock a repeater.

### CTCSS Tone Table (DRA818 supported)

| Value | Frequency | Value | Frequency |
|-------|-----------|-------|-----------|
| 0 | None | 20 | 156.7 Hz |
| 1 | 67.0 Hz | 21 | 159.8 Hz |
| 2 | 71.9 Hz | 22 | 162.2 Hz |
| 3 | 74.4 Hz | 23 | 165.5 Hz |
| 4 | 77.0 Hz | 24 | 167.9 Hz |
| 5 | 79.7 Hz | 25 | 171.3 Hz |
| 6 | 82.5 Hz | 26 | 173.8 Hz |
| 7 | 85.4 Hz | 27 | 177.3 Hz |
| 8 | 88.5 Hz | 28 | 179.9 Hz |
| 9 | 91.5 Hz | 29 | 183.5 Hz |
| 10 | 94.8 Hz | 30 | 186.2 Hz |
| 11 | 97.4 Hz | 31 | 189.9 Hz |
| 12 | 100.0 Hz | 32 | 192.8 Hz |
| 13 | 103.5 Hz | 33 | 196.6 Hz |
| 14 | 107.2 Hz | 34 | 199.5 Hz |
| 15 | 110.9 Hz | 35 | 203.5 Hz |
| 16 | 114.8 Hz | 36 | 206.5 Hz |
| 17 | 118.8 Hz | 37 | 210.7 Hz |
| 18 | 123.0 Hz | 38 | 214.1 Hz |
| 19 | 127.3 Hz | | |

### Example: W5XYZ Repeater (requires 100.0 Hz CTCSS)

```cpp
RadioConfig VHF_CONFIG = {
  145.3900,   // TX
  145.0100,   // RX (offset -3.8 MHz typical)
  4,
  6,
  12,         // ← CTCSS TX: 100.0 Hz (value 12)
  12,         // ← CTCSS RX: 100.0 Hz (value 12)
  DRA818_12K5,
  true, true, true
};
```

---

## 4. Build Flags

At the top of the sketch (around line 19-25):

```cpp
// Uncomment one:
#define ACTIVE_MODULE MODULE_VHF        // VHF only
// #define ACTIVE_MODULE MODULE_UHF        // UHF only
// #define ACTIVE_MODULE MODULE_BOTH       // Both (default)

#define DEBUG_MODE 1   // 1=verbose, 0=quiet
```

### Module Selection Examples

**Testing only VHF:**
```cpp
#define ACTIVE_MODULE MODULE_VHF
```
Feather will skip UHF init, config only VHF, and hand off PD/PTT/HL to Pi. Useful for isolated testing.

**Testing only UHF:**
```cpp
#define ACTIVE_MODULE MODULE_UHF
```
Skip VHF, test UHF in isolation (useful if VHF has hardware issues).

**Production (both):**
```cpp
#define ACTIVE_MODULE MODULE_BOTH
```
Configure and test both radios at startup.

### Debug Mode

**Development (verbose):**
```cpp
#define DEBUG_MODE 1
```
Outputs every step: handshakes, AT commands, pin reads, periodic status. ~10-20 lines per 10 seconds.

**Production (quiet):**
```cpp
#define DEBUG_MODE 0
```
Only critical messages: config success/failure, major errors. ~1-2 lines per 10 seconds.

---

## 5. Complete Configuration Example

### Example 1: 2m/70cm APRS System (Simplex)

```cpp
// VHF 2m APRS (144.39 MHz primary)
RadioConfig VHF_CONFIG = {
  144.3900, 144.3900,
  2,        // low squelch (weak APRS packets)
  4,        // moderate volume
  0,        // no CTCSS
  0,
  DRA818_12K5,
  true, true, true
};

// UHF 70cm cross-band (446.0 MHz simplex)
RadioConfig UHF_CONFIG = {
  446.0000, 446.0000,
  2,        // low squelch
  4,
  0, 0,
  DRA818_12K5,
  true, true, true
};

#define ACTIVE_MODULE MODULE_BOTH
#define DEBUG_MODE 1    // Enable for APRS troubleshooting
```

### Example 2: Repeater Linked System (with CTCSS)

```cpp
// VHF: W5XYZ Repeater (145.01 RX, 145.39 TX, PL 100.0)
RadioConfig VHF_CONFIG = {
  145.3900,  // TX (higher freq)
  145.0100,  // RX (lower freq, typical -3.8 MHz offset)
  4,         // squelch
  6,         // louder volume for repeater
  12,        // CTCSS TX: 100.0 Hz (value 12)
  12,        // CTCSS RX: 100.0 Hz (value 12)
  DRA818_12K5,
  true, true, true
};

// UHF: K5LLL Repeater (442.1 RX, 447.1 TX, PL 103.5)
RadioConfig UHF_CONFIG = {
  447.1000,  // TX (higher freq)
  442.1000,  // RX (lower freq, typical -5 MHz offset)
  4,
  6,
  13,        // CTCSS TX: 103.5 Hz (value 13)
  13,        // CTCSS RX: 103.5 Hz (value 13)
  DRA818_25K,  // Wider BW for repeater audio quality
  true, true, true
};

#define ACTIVE_MODULE MODULE_BOTH
#define DEBUG_MODE 0    // Less verbose for production
```

### Example 3: Digital Voice (DMR/Fusion) Test

```cpp
// Set frequencies to digital voice repeaters
// (DRA818 itself doesn't handle digital modes, but can provide RF link)

RadioConfig VHF_CONFIG = {
  145.3900, 145.0100,
  4, 6,
  0, 0,  // No PL tone, digital modes use other signaling
  DRA818_12K5,
  true, true, true
};

#define ACTIVE_MODULE MODULE_VHF
#define DEBUG_MODE 1    // Verbose for debugging digital link
```

---

## 6. Pin Customization (Advanced)

If your HamWing board has different pin assignments, edit the pin definitions section (lines ~45-75):

```cpp
#define PIN_PD       12    // HamWing power-down (shared)
#define PIN_HL       18    // HamWing power level (shared)
#define PIN_LED      LED_BUILTIN  // Change to your LED pin

#define PIN_V_DRA_RX 0     // VHF UART RX
#define PIN_V_DRA_TX 1     // VHF UART TX
#define PIN_V_PTT    16    // VHF PTT (shared)
#define PIN_V_SQL    14    // VHF squelch analog

#define PIN_U_DRA_RX 11    // UHF UART RX
#define PIN_U_DRA_TX 10    // UHF UART TX
#define PIN_U_PTT    17    // UHF PTT (shared)
#define PIN_U_SQL    15    // UHF squelch analog
```

**Warning:** Changing these pins requires understanding SERCOM mux and may break your build. Only modify if your board is wired differently.

---

## 7. Testing Checklist

After customizing, verify with this sequence:

```
[ ] 1. Edit VHF_CONFIG and UHF_CONFIG frequencies to match your radios
[ ] 2. Set ACTIVE_MODULE to MODULE_VHF, MODULE_UHF, or MODULE_BOTH as needed
[ ] 3. Set DEBUG_MODE to 1 for testing, 0 for quiet operation
[ ] 4. Compile and upload to Feather M0
[ ] 5. Open Serial Monitor (115200 baud), watch startup sequence
[ ] 6. Should see: "Config state VHF/UHF: OK / OK"
[ ] 7. Should see: "Handing off control to Raspberry Pi"
[ ] 8. LED should fade in and out (success) or fast blink (failure)
[ ] 9. Remove hat (if present) and test again—both radios should still config OK
[ ] 10. If UHF failed with hat installed, follow Pi UART disable guide (PI_SETUP_GUIDE.md)
```

---

## Troubleshooting Configuration

| Symptom | Check |
|---------|-------|
| VHF config fails | Is D0/D1 wired to VHF module TX/RX? Is VHF module powered? |
| UHF config fails with hat present | Follow PI_SETUP_GUIDE.md Phase 1 (disable Pi UART) |
| UHF config fails with hat removed | Is D10/D11 wired to UHF module TX/RX? Are JP1/JP2 soldered on HamWing? |
| Radio keys but sounds bad | Raise volume (6-8), check audio path, verify filters settings |
| Radio doesn't key from Pi | Did you run `hamwing_gpio_init.py`? Verify GPIO27/4/22 in use |
| LED not responding to config | Check `PIN_LED` definition, is LED_BUILTIN available on your Feather? |

---

## Advanced: Custom AT Commands

If you need finer control (not in the DRA818 library), the sketch can be extended:

```cpp
// Add to a new function in the RADIO CONFIGURATION section:
bool sendCustomAtCommand(Stream &serial, const char *cmd, const char *expectedResp) {
  debugPrint("AT", cmd);
  serial.println(cmd);
  
  unsigned long timeout = millis() + 500;
  String response = "";
  while (millis() < timeout) {
    if (serial.available()) {
      response += (char)serial.read();
    }
  }
  
  if (response.indexOf(expectedResp) >= 0) {
    debugPrint("RESP", response.c_str());
    return true;
  }
  return false;
}
```

Then call from `setup()` after radio config for custom AT sequences.

---

## Next Steps

1. Choose your frequencies from the tables above
2. Update VHF_CONFIG and UHF_CONFIG in the sketch
3. Set ACTIVE_MODULE and DEBUG_MODE appropriately
4. Compile, upload, and test
5. Follow PI_SETUP_GUIDE.md to integrate with Raspberry Pi

Good luck with your deployment! 73 de HamWing_RadioConfig
