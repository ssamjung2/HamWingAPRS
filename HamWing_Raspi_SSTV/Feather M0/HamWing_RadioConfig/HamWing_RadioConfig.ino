/*
 * HamWing_RadioConfig.ino — Feather M0 + HamWing board radio configurator
 *
 * Purpose:
 *   - Configures VHF and/or UHF DRA818 radio modules at startup via AT commands
 *   - Releases control of PD, PTT, and H/L lines to Raspberry Pi after successful config
 *   - Monitors Pi's control of shared lines and provides LED feedback
 *   - Supports flexible debugging and status output
 *
 * Architecture:
 *   - Feather M0 configures radios once at startup, then hands off control to Pi
 *   - Pi (via hat) owns PD, PTT, H/L lines exclusively during operation
 *   - Feather monitors these shared lines for feedback/debugging purposes only
 *
 * HamWing Wiring (Feather M0 + HamWing board):
 *   - VHF UART: Serial1 (D0 RX, D1 TX)
 *   - UHF UART: Serial2/SERCOM1 (D11 RX from module, D10 TX to module)
 *   - Shared PD: D12 (HIGH=active, LOW=sleep)
 *   - Shared H/L: D18 (HIGH=1W, LOW=0.5W)
 *   - VHF PTT: D16 (active-LOW)
 *   - UHF PTT: D17 (active-LOW)
 *   - VHF SQL: D14 (analog squelch input)
 *   - UHF SQL: D15 (analog squelch input)
 *   - Pi audio PWM: GPIO12, GPIO13 (via hat) → LPF → DRA818 mic input
 *
 * LED Feedback:
 *   - Slow fade in/out: Configuration succeeded
 *   - Fast blink (100ms on/off): Configuration failed
 *   - Solid ON: Pi is keying transmitter (PTT LOW detected)
 *   - OFF: Idle, Pi not transmitting
 */

#include <SPI.h>
#include <Wire.h>
#include <DRA818.h>
#include "wiring_private.h"  // pinPeripheral() for SERCOM
#include <WiFi101.h>          // Needed to call WiFi.end() to power down ATWINC1500
#include <ArduinoLowPower.h>  // SAMD21 IDLE sleep — keeps SERCOMs/timers/USB alive
#include <Adafruit_SleepyDog.h>  // Hardware watchdog for SAMD21

// ATWINC1500 control pins (Feather M0 WiFi)
#define WINC_CS   8
#define WINC_IRQ  7
#define WINC_RST  4
#define WINC_EN   2

//////////////////////////////////////////////////////////////////////////////
// BUILD CONFIGURATION — Customize these flags for your deployment
//////////////////////////////////////////////////////////////////////////////

// Module selection: MODULE_VHF, MODULE_UHF, or MODULE_BOTH
#define MODULE_VHF  1
#define MODULE_UHF  2
#define MODULE_BOTH 3
#define ACTIVE_MODULE MODULE_UHF

// Keep false by default; fallback mapping can be enabled for diagnostics.
#define TRY_UHF_BOTH_UART_MAPPINGS 0

// Debug/verbose output (0 = minimal, 1 = comprehensive)
#define DEBUG_MODE 1

//////////////////////////////////////////////////////////////////////////////
// (LED styles are controlled directly in the LED state machine below)
//////////////////////////////////////////////////////////////////////////////
// PIN DEFINITIONS — Feather M0 + HamWing board
//////////////////////////////////////////////////////////////////////////////

// Power and control
#define PIN_PD       12  // Shared power-down (HIGH=active, LOW=sleep) — Pi owns this
#define PIN_HL       18  // Shared power level (HIGH=1W, LOW=0.5W) — Pi owns this
#define STATUS_LED_PIN LED_BUILTIN

// VHF Radio Module (Serial1, hardware UART)
#define PIN_V_DRA_RX 0   // D0, SERCOM0 PAD3
#define PIN_V_DRA_TX 1   // D1, SERCOM0 PAD2
#define PIN_V_PTT    16  // Active-LOW PTT — Pi owns this
#define PIN_V_SQL    14  // Analog squelch input

// UHF Radio Module (Serial2, SERCOM1)
#define PIN_U_DRA_RX 11  // D11, SERCOM1 PAD0, receives from module TX
#define PIN_U_DRA_TX 10  // D10, SERCOM1 PAD2, transmits to module RX
#define PIN_U_PTT    17  // Active-LOW PTT — Pi owns this
#define PIN_U_SQL    15  // Analog squelch input

// Battery monitoring (optional)
#define PIN_VBAT     A7

// DRA818 command timing/retry settings
#define DRA_STEP_RETRIES 3
#define DRA_AT_TIMEOUT_MS 700

// LED timing settings (ms)
#define LED_HEARTBEAT_FADE_UP_MS 1500
#define LED_HEARTBEAT_FADE_DOWN_MS 1500
#define LED_FAILURE_BLINK_HALF_PERIOD_MS 100
#define LED_CONFIG_BLINK_HALF_PERIOD_MS 200
#define LED_TX_HEARTBEAT_FADE_UP_MS 250
#define LED_TX_HEARTBEAT_FADE_DOWN_MS 250
#define LED_WARNING_BLINK_HALF_PERIOD_MS 600

// LED state IDs used in early-declared functions before the enum definition.
// These must stay in sync with the anonymous enum near the LED state machine.
#define LED_STATE_IDLE 0
#define LED_STATE_CONFIG_ACTIVE 1
#define LED_STATE_SUCCESS_FADE 2
#define LED_STATE_FAILURE_BLINK 3
#define LED_STATE_TX_HEARTBEAT 4
#define LED_STATE_WARNING_BLINK 5
#define LED_STATE_LOW_VOLTAGE 6
// TX detection behavior
#define TX_REQUIRE_PD_HIGH_FOR_CONFIRMED_TX 1

// Status table behavior
#define STATUS_HEADER_REPEAT_ROWS 25
#define STATUS_ROW_INTERVAL_MS 500
// 1 = compact periodic rows by default, 0 = full periodic rows by default
#define STATUS_PERIODIC_COMPACT 1
#define SERIAL_BACKPRESSURE_GUARD 1
#define SERIAL_MIN_FREE_FOR_DEBUG 24
#define SERIAL_MIN_FREE_FOR_STATUS 8
#define PD_LOW_WARN_INTERVAL_MS 7000

// Low voltage cutoff — uses VBAT on A7 via Feather onboard 1/2 divider (single-cell LiPo)
// When VBAT drops below LOW_VOLTAGE_CUTOFF_V the radios are put to sleep (PD LOW) and
// PTT diagnostic pulses are blocked.  Normal operation resumes above LOW_VOLTAGE_RECOVER_V.
#define LOW_VOLTAGE_CUTOFF_ENABLE  1
#define LOW_VOLTAGE_CUTOFF_V       3.30f   // Sleep radios below this (V)
#define LOW_VOLTAGE_RECOVER_V      3.40f   // Resume above this — hysteresis prevents chatter
#define LOW_VOLTAGE_CHECK_MS       5000    // Interval between voltage checks (ms)
#define LOW_VOLTAGE_WARN_MS        15000   // Interval between repeated serial warnings (ms)

// Startup/config UX timing
#define PRE_CONFIG_DELAY_MS 500

// Shared-line handoff behavior
#define HANDOFF_USE_PULLUPS 1

// PTT line handoff bias:
// 0 = release as INPUT (no pull-up, least loading)
// 1 = release as INPUT_PULLUP
#define HANDOFF_PTT_USE_PULLUPS 0

// PD post-config behavior:
// 0 = release PD to Pi (INPUT/INPUT_PULLUP per HANDOFF_USE_PULLUPS)
// 1 = Feather keeps PD actively HIGH after config (prevents unintended sleep)
#define HOLD_PD_HIGH_AFTER_CONFIG 0

// Interactive diagnostics over USB serial (115200):
//   h = help, p = print status, v = pulse VHF PTT, u = pulse UHF PTT, b = pulse both
#define ENABLE_SERIAL_DIAG_COMMANDS 1
#define DIAG_LOCAL_PTT_PULSE_MS 1200
#define NO_PTT_EDGE_WARN_INTERVAL_MS 5000
#define DIAG_HELP_LOG_PAUSE_MS 6000
#define DIAG_MAX_BYTES_PER_LOOP 32

// Stuck PTT watchdog — detect if PTT is held LOW longer than expected
// When TX_DETECT_CONFIRMED exceeds threshold, force-unkey and emit alert
#define STUCK_PTT_WATCHDOG_ENABLE 1
#define STUCK_PTT_MAX_TX_MS (7 * 60 * 1000)  // 7 minutes in milliseconds

// Battery voltage: set ADC resolution for better accuracy
#define VBAT_ADC_RESOLUTION 12  // 12-bit ADC for SAMD21 (default is 10-bit)

// Hardware watchdog for SAMD21 — auto-resets if sketch hangs
#define HARDWARE_WATCHDOG_ENABLE 1
#define HARDWARE_WATCHDOG_TIMEOUT_MS 10000  // 10 second timeout

//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION STRUCTURE
//////////////////////////////////////////////////////////////////////////////

struct RadioConfig {
  float freq_tx;      // TX frequency (MHz)
  float freq_rx;      // RX frequency (MHz)
  uint8_t squelch;    // 0-8
  uint8_t volume;     // 1-8
  uint8_t ctcss_tx;   // 0-38, 0=none
  uint8_t ctcss_rx;   // 0-38, 0=none
  uint8_t bandwidth;  // DRA818_12K5 or DRA818_25K
  bool filter_pre;
  bool filter_high;
  bool filter_low;
};

// Default configurations — edit these to match your frequencies and preferences
// For SSTV/data, keep DRA818 audio shaping filters OFF for the flattest AF path.
RadioConfig VHF_CONFIG = {
  144.3900,   // TX freq
  144.3900,   // RX freq
  4,          // squelch
  6,          // volume
  0,          // CTCSS TX
  0,          // CTCSS RX
  DRA818_12K5,
  false,
  false,
  false
};

RadioConfig UHF_CONFIG = {
  440.8000,   // TX freq
  440.8000,   // RX freq
  4,          // squelch
  6,          // volume
  0,          // CTCSS TX
  0,          // CTCSS RX
  DRA818_12K5,
  false,
  false,
  false
};

//////////////////////////////////////////////////////////////////////////////
// GLOBAL STATE
//////////////////////////////////////////////////////////////////////////////

bool gVhfConfigured = false;
bool gUhfConfigured = false;
bool gConfigurationComplete = false;

// Feather owns shared lines only during startup config; Pi owns them afterward.
bool gSharedLinesReleasedToPi = false;
unsigned long gLastPdLowWarnMs = 0;
unsigned long gLastNoPttEdgeWarnMs = 0;
uint32_t gVhfPttEdgeCount = 0;
uint32_t gUhfPttEdgeCount = 0;
int gLastVhfPttSense = -1;
int gLastUhfPttSense = -1;
bool gHlDiagOverrideActive = false;
bool gHlDiagLevelHigh = false;
bool gPdLockOn = (HOLD_PD_HIGH_AFTER_CONFIG != 0);
unsigned long gLastPdLockWarnMs = 0;
unsigned long gStatusPauseUntilMs = 0;

// Low voltage cutoff state
bool gLowVoltageCutoffActive = false;
unsigned long gLastLowVoltageWarnMs = 0;
unsigned long gLastVbatCheckMs = 0;
unsigned long gLastPeriodicStatusMs = 0;
bool gPeriodicStatusCompact = (STATUS_PERIODIC_COMPACT != 0);

char gVhfPttCtrlOwner[16] = "BOOT";
char gUhfPttCtrlOwner[16] = "BOOT";
char gVhfPttCtrlAction[16] = "INIT";
char gUhfPttCtrlAction[16] = "INIT";
unsigned long gVhfPttCtrlMs = 0;
unsigned long gUhfPttCtrlMs = 0;

// Stuck PTT watchdog state
unsigned long gTxConfirmedSinceMs = 0;
bool gStuckPttAlertEmitted = false;

// SQL (squelch) signal sensing
int gLastVhfSqlSense = -1;
int gLastUhfSqlSense = -1;
uint32_t gVhfSqlOpenCount = 0;
uint32_t gUhfSqlOpenCount = 0;

extern uint16_t gStatusRowsPrinted;
void printStatusBanner(bool forceHeader = true, bool ignoreBackpressure = false);
void printStatusCompactRow();
void printStatusSnapshotFramed();

// DRA818 instances
DRA818 draV((Stream *)&Serial1, DRA818_VHF);

#if TRY_UHF_BOTH_UART_MAPPINGS
Uart Serial2Preferred(&sercom1, PIN_U_DRA_RX, PIN_U_DRA_TX, SERCOM_RX_PAD_0, UART_TX_PAD_2);
Uart Serial2Fallback(&sercom1, PIN_U_DRA_TX, PIN_U_DRA_RX, SERCOM_RX_PAD_2, UART_TX_PAD_0);
Uart *activeUhfSerial = &Serial2Preferred;
bool activeUhfPreferredMapping = true;
#else
Uart Serial2(&sercom1, PIN_U_DRA_RX, PIN_U_DRA_TX, SERCOM_RX_PAD_0, UART_TX_PAD_2);
Uart *activeUhfSerial = &Serial2;
#endif

DRA818 draU((Stream *)activeUhfSerial, DRA818_UHF);

void SERCOM1_Handler() {
  activeUhfSerial->IrqHandler();
}

void setPttControlStateByPin(uint8_t pin, const char *owner, const char *action, bool emitLog) {
  char *ownerDst = NULL;
  char *actionDst = NULL;
  unsigned long *msDst = NULL;
  const char *lineName = "PTT";

  if (pin == PIN_V_PTT) {
    ownerDst = gVhfPttCtrlOwner;
    actionDst = gVhfPttCtrlAction;
    msDst = &gVhfPttCtrlMs;
    lineName = "VHF_PTT";
  } else if (pin == PIN_U_PTT) {
    ownerDst = gUhfPttCtrlOwner;
    actionDst = gUhfPttCtrlAction;
    msDst = &gUhfPttCtrlMs;
    lineName = "UHF_PTT";
  } else {
    return;
  }

  strncpy(ownerDst, owner, 15);
  ownerDst[15] = '\0';
  strncpy(actionDst, action, 15);
  actionDst[15] = '\0';
  *msDst = millis();

  if (emitLog) {
    char msg[96];
    snprintf(msg, sizeof(msg), "%s owner=%s action=%s", lineName, ownerDst, actionDst);
    debugPrint("PTTCTL", msg);
  }
}

void applyPdOwnershipState(bool emitLog) {
  if (gPdLockOn) {
    pinMode(PIN_PD, OUTPUT);
    digitalWrite(PIN_PD, HIGH);
    if (emitLog) {
      debugPrint("PD", "PD lock enabled: Feather drives PD HIGH");
    }
  } else {
#if HANDOFF_USE_PULLUPS
    pinMode(PIN_PD, INPUT_PULLUP);
#else
    pinMode(PIN_PD, INPUT);
#endif
    if (emitLog) {
      debugPrint("PD", "PD lock disabled: PD released to Pi");
    }
  }
}

void setPdLock(bool enabled) {
  gPdLockOn = enabled;
  applyPdOwnershipState(true);
}

void enforcePdLockIfEnabled() {
#if LOW_VOLTAGE_CUTOFF_ENABLE
  if (gLowVoltageCutoffActive) {
    return;  // LVCO owns PD (forced LOW); do not reassert HIGH
  }
#endif
  if (!gPdLockOn) {
    return;
  }

  // Reassert PD HIGH every pass so any external low drive gets overwritten quickly.
  pinMode(PIN_PD, OUTPUT);
  digitalWrite(PIN_PD, HIGH);

  int pdSense = digitalRead(PIN_PD);
  if (pdSense == LOW && (millis() - gLastPdLockWarnMs) > 1000) {
    gLastPdLockWarnMs = millis();
    debugPrint("PD", "PD lock active but sensed LOW; likely external contention/short");
  }
}

#if LOW_VOLTAGE_CUTOFF_ENABLE
// Check VBAT and enter/exit low voltage cutoff.
// When active: PD is driven LOW to sleep the radios and reduce current draw.
// LED is forced to LED_LOW_VOLTAGE.  Normal operation resumes once voltage
// rises above LOW_VOLTAGE_RECOVER_V (hysteresis prevents oscillation).
void checkLowVoltageCutoff() {
  unsigned long now = millis();
  if ((now - gLastVbatCheckMs) < (unsigned long)LOW_VOLTAGE_CHECK_MS) {
    return;
  }
  gLastVbatCheckMs = now;

  float vbat = readBatteryVoltage();
  char msg[96];

  if (!gLowVoltageCutoffActive && vbat < LOW_VOLTAGE_CUTOFF_V) {
    gLowVoltageCutoffActive = true;
    gLastLowVoltageWarnMs = now;
    // Force PD LOW: sleep radios regardless of PD lock setting.
    pinMode(PIN_PD, OUTPUT);
    digitalWrite(PIN_PD, LOW);
    setLedState(LED_STATE_LOW_VOLTAGE);
    snprintf(msg, sizeof(msg),
      "LOW VOLTAGE CUTOFF: %.2fV < %.2fV — radios sleeping",
      vbat, (float)LOW_VOLTAGE_CUTOFF_V);
    debugPrint("LVCO", msg);
  } else if (gLowVoltageCutoffActive && vbat >= LOW_VOLTAGE_RECOVER_V) {
    gLowVoltageCutoffActive = false;
    // Restore PD to normal ownership (lock or Pi-controlled).
    applyPdOwnershipState(true);
    setLedState(gConfigurationComplete ? LED_STATE_SUCCESS_FADE : LED_STATE_FAILURE_BLINK);
    snprintf(msg, sizeof(msg),
      "Voltage recovered: %.2fV >= %.2fV — resuming normal operation",
      vbat, (float)LOW_VOLTAGE_RECOVER_V);
    debugPrint("LVCO", msg);
  } else if (gLowVoltageCutoffActive &&
             (now - gLastLowVoltageWarnMs) > (unsigned long)LOW_VOLTAGE_WARN_MS) {
    gLastLowVoltageWarnMs = now;
    snprintf(msg, sizeof(msg),
      "Still in low voltage cutoff: %.2fV (recover at %.2fV)",
      vbat, (float)LOW_VOLTAGE_RECOVER_V);
    debugPrint("LVCO", msg);
  }
}
#endif  // LOW_VOLTAGE_CUTOFF_ENABLE

void releasePttPinToPi(uint8_t pin) {
#if HANDOFF_PTT_USE_PULLUPS
  pinMode(pin, INPUT_PULLUP);
#else
  pinMode(pin, INPUT);
#endif
  setPttControlStateByPin(pin, "PI", "RELEASED", true);
}

void releasePttPinsToPi() {
  releasePttPinToPi(PIN_V_PTT);
  releasePttPinToPi(PIN_U_PTT);
}

void driveSharedLinesForStartupConfig() {
  // Keep both radios awake and unkeyed during AT configuration.
  pinMode(PIN_PD, OUTPUT);
  digitalWrite(PIN_PD, HIGH);      // Radios active

  pinMode(PIN_HL, OUTPUT);
  digitalWrite(PIN_HL, LOW);       // Default low power during setup

  pinMode(PIN_V_PTT, OUTPUT);
  digitalWrite(PIN_V_PTT, HIGH);   // Active-LOW PTT: HIGH = idle
  setPttControlStateByPin(PIN_V_PTT, "FEATHER_CFG", "CLAIM_IDLE", false);

  pinMode(PIN_U_PTT, OUTPUT);
  digitalWrite(PIN_U_PTT, HIGH);   // Active-LOW PTT: HIGH = idle
  setPttControlStateByPin(PIN_U_PTT, "FEATHER_CFG", "CLAIM_IDLE", false);
}

void releaseSharedLinesToPi() {
  if (gSharedLinesReleasedToPi) {
    return;
  }

  // PD ownership is runtime-controlled by gPdLockOn.
  applyPdOwnershipState(false);

  pinMode(PIN_HL, INPUT);
  gHlDiagOverrideActive = false;
  releasePttPinsToPi();

  gSharedLinesReleasedToPi = true;
  if (gPdLockOn) {
    debugPrint("HANDOFF", "Released HL/PTT to Pi; Feather holds PD HIGH (lock ON)");
  } else {
    debugPrint("HANDOFF", "Released PD/HL/PTT lines to Raspberry Pi (PD lock OFF)");
  }
}

void setHlDiagLevel(bool high) {
  pinMode(PIN_HL, OUTPUT);
  digitalWrite(PIN_HL, high ? HIGH : LOW);
  gHlDiagOverrideActive = true;
  gHlDiagLevelHigh = high;

  char msg[96];
  snprintf(msg, sizeof(msg), "Feather override H/L -> %s (Pi temporarily bypassed)", high ? "HIGH" : "LOW");
  debugPrint("HL", msg);
}

void releaseHlDiagToPi() {
  pinMode(PIN_HL, INPUT);
  gHlDiagOverrideActive = false;
  debugPrint("HL", "Released H/L line to Pi (INPUT)");
}

void printCurrentConfigSnapshot() {
  Serial.println();
  Serial.println("================ CFG SNAPSHOT BEGIN ================");
  Serial.println("-------------------- CFG CURRENT -------------------");
  printConfigPlanTableHeader();
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigPlanRow("VHF", VHF_CONFIG);
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigPlanRow("UHF", UHF_CONFIG);
#endif

  Serial.println("----------------------------------------------------");
  printConfigAppliedTableHeader();
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigAppliedRow("VHF", VHF_CONFIG, gVhfConfigured);
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigAppliedRow("UHF", UHF_CONFIG, gUhfConfigured);
#endif

  Serial.println();
  Serial.print("CFG_COMPLETE=");
  Serial.print(gConfigurationComplete ? "YES" : "NO");
  Serial.print("  HANDOFF_TO_PI=");
  Serial.print(gSharedLinesReleasedToPi ? "YES" : "NO");
  Serial.print("  PD_LOCK=");
  Serial.print(gPdLockOn ? "ON" : "OFF");
  Serial.print("  HL_OVERRIDE=");
  if (gHlDiagOverrideActive) {
    Serial.print("FEATHER_");
    Serial.print(gHlDiagLevelHigh ? "HIGH" : "LOW");
  } else {
    Serial.print("PI");
  }
  Serial.println();

  Serial.println("------------------- LIVE STATUS --------------------");
  printStatusBanner();
  Serial.println("================= CFG SNAPSHOT END =================");
  Serial.println();
}

void pauseStatusLogs(unsigned long ms) {
  gStatusPauseUntilMs = millis() + ms;
}

void printDiagHelpDetailed() {
  pauseStatusLogs(DIAG_HELP_LOG_PAUSE_MS);

  Serial.println();
  Serial.println("============================================================");
  Serial.println(" HAMWING FIELD DIAGNOSTIC HELP");
  Serial.println("============================================================");
  Serial.print(" Status rows paused for ");
  Serial.print(DIAG_HELP_LOG_PAUSE_MS / 1000);
  Serial.println("s so this guide stays readable.");
  Serial.println();

  Serial.println("[Quick Readouts]");
  Serial.println("  h       Show this help screen");
  Serial.println("  s       Print one status row now");
  Serial.println("  c       Print current config snapshot + status");
  Serial.println("  z <m>   Periodic status mode: t=toggle, c=compact, f=full");
  Serial.println();

  Serial.println("[Power / Ownership Control]");
  Serial.println("  l <m>   H/L control with mode m=toggle/high/low/release");
  Serial.println("          Modes: t=toggle, h=HIGH, l=LOW, r=release to Pi");
  Serial.println("          Examples: lt  lh  ll  lr");
  Serial.println("  d <m>   PD lock control with mode m=toggle/on/off");
  Serial.println("          Modes: t=toggle, o=ON (lock high), n=OFF (release)");
  Serial.println("          Examples: dt  do  dn");
  Serial.println();

  Serial.println("[Radio Runtime Tuning]");
  Serial.println("  mN      Set volume on active modules, N=1..8");
  Serial.println("          Example: m6");
  Serial.println("  qN      Set squelch on active modules, N=0..8");
  Serial.println("          Example: q4");
  Serial.println("  fXYZ    Set filters: X=pre, Y=high, Z=low (0/1 each)");
  Serial.println("          Example: f111 (all on), f010 (high-pass only)");
  Serial.println("  g M D F Set module M(v/u) direction D(tx/rx) to freq F MHz");
  Serial.println("          Example: g v tx 144.3900   or   g u rx 440.8000");
  Serial.println();

  Serial.println("[PTT Diagnostic Pulses]");
  Serial.println("  v       Local VHF PTT pulse");
  Serial.println("  u       Local UHF PTT pulse");
  Serial.println("  b       Pulse VHF then UHF");
  Serial.println();

  Serial.println("[Field Workflow]");
  Serial.println("  1) Run c to capture baseline state.");
  Serial.println("  2) Use do to force PD lock ON during troubleshooting.");
  Serial.println("  3) Use v/u and watch PTTCTL + status columns for ownership transitions.");
  Serial.println("  4) Adjust mN/qN/fXYZ/g and confirm [RUNTIME] lines report OK.");
  Serial.println("  5) Use lr and dn to hand shared lines back to Pi when done.");
  Serial.println();

  Serial.println("============================================================");
  Serial.println(" END HELP - periodic status rows will auto-resume shortly");
  Serial.println("============================================================");
}

bool parseUint8Token(const char *token, uint8_t minVal, uint8_t maxVal, uint8_t &out) {
  if (!token || !token[0]) {
    return false;
  }

  uint16_t value = 0;
  for (size_t i = 0; token[i] != '\0'; ++i) {
    if (token[i] < '0' || token[i] > '9') {
      return false;
    }
    value = (uint16_t)(value * 10 + (token[i] - '0'));
    if (value > 255) {
      return false;
    }
  }

  if (value < minVal || value > maxVal) {
    return false;
  }

  out = (uint8_t)value;
  return true;
}

bool parseFilterToken(const char *token, bool &pre, bool &high, bool &low) {
  if (!token || token[0] == '\0' || token[1] == '\0' || token[2] == '\0' || token[3] != '\0') {
    return false;
  }
  if ((token[0] != '0' && token[0] != '1') ||
      (token[1] != '0' && token[1] != '1') ||
      (token[2] != '0' && token[2] != '1')) {
    return false;
  }

  pre = (token[0] == '1');
  high = (token[1] == '1');
  low = (token[2] == '1');
  return true;
}

bool parseFreqToken(const char *token, float &outMhz) {
  if (!token || !token[0]) {
    return false;
  }

  bool seenDot = false;
  bool seenDigit = false;
  for (size_t i = 0; token[i] != '\0'; ++i) {
    char c = token[i];
    if (c >= '0' && c <= '9') {
      seenDigit = true;
      continue;
    }
    if (c == '.' && !seenDot) {
      seenDot = true;
      continue;
    }
    return false;
  }

  if (!seenDigit) {
    return false;
  }

  float mhz = (float)atof(token);
  if (mhz < 100.0f || mhz > 1000.0f) {
    return false;
  }

  outMhz = mhz;
  return true;
}

bool readDiagArgToken(char *out, size_t outSize) {
  if (!out || outSize < 2) {
    return false;
  }

  // Skip leading spaces/tabs after command char.
  while (Serial.available() > 0) {
    char c = (char)Serial.peek();
    if (c == ' ' || c == '\t') {
      Serial.read();
    } else {
      break;
    }
  }

  size_t idx = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.peek();
    if (c == '\n' || c == '\r' || c == ' ' || c == '\t') {
      break;
    }
    c = (char)Serial.read();
    if (idx < (outSize - 1)) {
      out[idx++] = c;
    }
  }
  out[idx] = '\0';
  return idx > 0;
}

bool applyVolumeWithRetries(DRA818 &radio, const char *label, uint8_t volume) {
  bool ok = false;
  uint8_t libTry = 0;
  uint8_t rawTry = 0;
  const char *method = "LIB";

  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    libTry++;
    ok = (radio.volume(volume) == 1);
  }
  if (!ok) {
    method = "RAW";
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      rawTry++;
      ok = setVolumeFallbackRaw(radio, label, volume);
    }
  }

  char msg[120];
  snprintf(msg, sizeof(msg), "%s runtime VOLUME=%u %s (lib=%u raw=%u method=%s)", label, volume, ok ? "OK" : "FAIL", libTry, rawTry, method);
  debugPrint("RUNTIME", msg);
  return ok;
}

bool applySquelchWithRetries(DRA818 &radio, const char *label, RadioConfig &cfg, uint8_t squelch) {
  RadioConfig candidate = cfg;
  candidate.squelch = squelch;

  bool ok = false;
  uint8_t libTry = 0;
  uint8_t rawTry = 0;
  const char *method = "LIB";

  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    libTry++;
    ok = (radio.group(candidate.bandwidth, candidate.freq_tx, candidate.freq_rx, candidate.ctcss_tx, candidate.squelch, candidate.ctcss_rx) == 1);
  }
  if (!ok) {
    method = "RAW";
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      rawTry++;
      ok = setGroupFallbackRaw(radio, label, candidate);
    }
  }

  if (ok) {
    cfg.squelch = squelch;
  }

  char msg[120];
  snprintf(msg, sizeof(msg), "%s runtime SQUELCH=%u %s (lib=%u raw=%u method=%s)", label, squelch, ok ? "OK" : "FAIL", libTry, rawTry, method);
  debugPrint("RUNTIME", msg);
  return ok;
}

bool applyFiltersWithRetries(DRA818 &radio, const char *label, RadioConfig &cfg, bool pre, bool high, bool low) {
  bool ok = false;
  uint8_t libTry = 0;
  uint8_t rawTry = 0;
  const char *method = "LIB";

  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    libTry++;
    ok = (radio.filters(pre, high, low) == 1);
  }
  if (!ok) {
    method = "RAW";
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      rawTry++;
      ok = setFilterFallbackRaw(radio, label, pre, high, low);
    }
  }

  if (ok) {
    cfg.filter_pre = pre;
    cfg.filter_high = high;
    cfg.filter_low = low;
  }

  char msg[140];
  snprintf(msg, sizeof(msg), "%s runtime FILTER=%u%u%u %s (lib=%u raw=%u method=%s)", label, pre ? 1 : 0, high ? 1 : 0, low ? 1 : 0, ok ? "OK" : "FAIL", libTry, rawTry, method);
  debugPrint("RUNTIME", msg);
  return ok;
}

bool applyVolumeAll(uint8_t volume) {
  bool allOk = true;
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  allOk = applyVolumeWithRetries(draV, "VHF", volume) && allOk;
  if (allOk) {
    VHF_CONFIG.volume = volume;
  }
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  bool uhfOk = applyVolumeWithRetries(draU, "UHF", volume);
  allOk = uhfOk && allOk;
  if (uhfOk) {
    UHF_CONFIG.volume = volume;
  }
#endif
  return allOk;
}

bool applySquelchAll(uint8_t squelch) {
  bool allOk = true;
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  allOk = applySquelchWithRetries(draV, "VHF", VHF_CONFIG, squelch) && allOk;
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  allOk = applySquelchWithRetries(draU, "UHF", UHF_CONFIG, squelch) && allOk;
#endif
  return allOk;
}

bool applyFiltersAll(bool pre, bool high, bool low) {
  bool allOk = true;
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  allOk = applyFiltersWithRetries(draV, "VHF", VHF_CONFIG, pre, high, low) && allOk;
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  allOk = applyFiltersWithRetries(draU, "UHF", UHF_CONFIG, pre, high, low) && allOk;
#endif
  return allOk;
}

bool applyFrequencyWithRetries(DRA818 &radio, const char *label, RadioConfig &cfg, bool setTx, float mhz) {
  RadioConfig candidate = cfg;
  if (setTx) {
    candidate.freq_tx = mhz;
  } else {
    candidate.freq_rx = mhz;
  }

  bool ok = false;
  uint8_t libTry = 0;
  uint8_t rawTry = 0;
  const char *method = "LIB";

  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    libTry++;
    ok = (radio.group(candidate.bandwidth, candidate.freq_tx, candidate.freq_rx, candidate.ctcss_tx, candidate.squelch, candidate.ctcss_rx) == 1);
  }
  if (!ok) {
    method = "RAW";
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      rawTry++;
      ok = setGroupFallbackRaw(radio, label, candidate);
    }
  }

  if (ok) {
    cfg = candidate;
  }

  char msg[160];
  snprintf(msg, sizeof(msg), "%s runtime %s_FREQ=%.4f %s (lib=%u raw=%u method=%s)", label, setTx ? "TX" : "RX", mhz, ok ? "OK" : "FAIL", libTry, rawTry, method);
  debugPrint("RUNTIME", msg);
  return ok;
}

bool applyFrequencyCommand(char moduleSel, bool setTx, float mhz) {
  if (moduleSel == 'v' || moduleSel == 'V') {
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
    return applyFrequencyWithRetries(draV, "VHF", VHF_CONFIG, setTx, mhz);
#else
    Serial.println("[DIAG] VHF is not active in this build");
    return false;
#endif
  }

  if (moduleSel == 'u' || moduleSel == 'U') {
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
    return applyFrequencyWithRetries(draU, "UHF", UHF_CONFIG, setTx, mhz);
#else
    Serial.println("[DIAG] UHF is not active in this build");
    return false;
#endif
  }

  Serial.println("[DIAG] Frequency module selector must be v or u");
  return false;
}

bool applyHlModeCommand(char modeChar) {
  switch (modeChar) {
    case 't':
    case 'T':
      setHlDiagLevel(!gHlDiagLevelHigh);
      return true;
    case 'h':
    case 'H':
    case '1':
      setHlDiagLevel(true);
      return true;
    case 'l':
    case 'L':
    case '0':
      setHlDiagLevel(false);
      return true;
    case 'r':
    case 'R':
      releaseHlDiagToPi();
      return true;
    default:
      return false;
  }
}

bool applyPdModeCommand(char modeChar) {
  switch (modeChar) {
    case 't':
    case 'T':
      setPdLock(!gPdLockOn);
      return true;
    case 'o':
    case 'O':
    case '1':
      setPdLock(true);
      return true;
    case 'n':
    case 'N':
    case '0':
      setPdLock(false);
      return true;
    default:
      return false;
  }
}

bool applyStatusModeCommand(char modeChar) {
  bool oldCompact = gPeriodicStatusCompact;

  switch (modeChar) {
    case 't':
    case 'T':
      gPeriodicStatusCompact = !gPeriodicStatusCompact;
      break;
    case 'c':
    case 'C':
      gPeriodicStatusCompact = true;
      break;
    case 'f':
    case 'F':
      gPeriodicStatusCompact = false;
      break;
    default:
      return false;
  }

  // If switching into full periodic mode, force the next full row to include headers.
  if (oldCompact && !gPeriodicStatusCompact) {
    gStatusRowsPrinted = 0;
  }

  return true;
}

void runLocalPttPulse(const char *label, uint8_t pin, unsigned long pulseMs) {
#if LOW_VOLTAGE_CUTOFF_ENABLE
  if (gLowVoltageCutoffActive) {
    Serial.print("[DIAG] ");
    Serial.print(label);
    Serial.println(" PTT pulse BLOCKED — low voltage cutoff active");
    return;
  }
#endif
  Serial.print("[DIAG] Local ");
  Serial.print(label);
  Serial.print(" PTT pulse start, ");
  Serial.print(pulseMs);
  Serial.print(" ms, pin D");
  Serial.println(pin);

  // Read pin before driving (should be HIGH/IDLE if released to INPUT)
  int preDrive = digitalRead(pin);
  Serial.print("[DIAG] Pre-drive pin D");
  Serial.print(pin);
  Serial.print(": ");
  Serial.println(preDrive == LOW ? "LOW (already keyed?)" : "HIGH (idle, expected)");

  // Active-low keying: assert idle HIGH first, then drive LOW to key.
  pinMode(pin, OUTPUT);
  setPttControlStateByPin(pin, "FEATHER_DIAG", "CLAIM_IDLE", true);
  digitalWrite(pin, HIGH);

  auto serviceBackground = []() {
    // Keep LED animation and watchdog logic alive during long local pulses.
    updateLed();
    enforcePdLockIfEnabled();
#if LOW_VOLTAGE_CUTOFF_ENABLE
    checkLowVoltageCutoff();
#endif

    if (DEBUG_MODE && millis() >= gStatusPauseUntilMs &&
        (millis() - gLastPeriodicStatusMs > STATUS_ROW_INTERVAL_MS)) {
      gLastPeriodicStatusMs = millis();
      if (gPeriodicStatusCompact) {
        printStatusCompactRow();
      } else {
        printStatusBanner();
      }
    }

    // Use a short delay here (not LowPower.idle) so millis() always advances
    // while this routine waits; otherwise the pulse timer can stall and stick PTT low.
    delay(1);
  };

  auto waitWithService = [&](unsigned long waitMs) {
    unsigned long start = millis();
    while ((millis() - start) < waitMs) {
      serviceBackground();
    }
  };

  waitWithService(20);
  digitalWrite(pin, LOW);
  setPttControlStateByPin(pin, "FEATHER_DIAG", "KEY_LOW", true);

  // SAMD21: digitalRead on OUTPUT reads actual pad voltage (PORT->IN), not
  // just the output register.  If external logic overrides our LOW, this
  // will read HIGH and alert us.
  int midDrive = digitalRead(pin);
  Serial.print("[DIAG] Mid-drive pin D");
  Serial.print(pin);
  Serial.print(": ");
  Serial.println(midDrive == LOW ? "LOW (confirmed keyed)" : "HIGH (FAIL - external override or wrong pin!)");

  // Print a live status row while pin is held LOW so table shows VHF_PTT=TX
  waitWithService(50);
  Serial.println("[DIAG] Status while keyed:");
  printStatusBanner();

  if (pulseMs > 50) {
    waitWithService(pulseMs - 50);
  }
  digitalWrite(pin, HIGH);
  setPttControlStateByPin(pin, "FEATHER_DIAG", "UNKEY_HIGH", true);
  waitWithService(20);

  // Return ownership to Pi/hat release state.
  releasePttPinToPi(pin);

  Serial.print("[DIAG] Local ");
  Serial.print(label);
  Serial.println(" PTT pulse complete");
}

void handleSerialDiagCommands() {
#if ENABLE_SERIAL_DIAG_COMMANDS
  uint8_t processedBytes = 0;
  while (Serial.available() > 0 && processedBytes < DIAG_MAX_BYTES_PER_LOOP) {
    char cmd = (char)Serial.read();
    processedBytes++;
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') {
      continue;
    }

    switch (cmd) {
      case 'h':
      case 'H':
        printDiagHelpDetailed();
        break;
      case 's':
      case 'S':
        printStatusSnapshotFramed();
        break;
      case 'c':
      case 'C':
        printCurrentConfigSnapshot();
        break;
      case 'z':
      case 'Z': {
        char token[12];
        if (!readDiagArgToken(token, sizeof(token)) || !token[0] || !applyStatusModeCommand(token[0])) {
          Serial.println("[DIAG] Usage: z <mode> where mode=t|c|f (toggle/compact/full)");
          break;
        }
        Serial.print("[DIAG] Periodic status mode -> ");
        Serial.println(gPeriodicStatusCompact ? "COMPACT" : "FULL");
        break;
      }
      case 'l':
      case 'L': {
        char token[12];
        if (!readDiagArgToken(token, sizeof(token)) || !token[0] || !applyHlModeCommand(token[0])) {
          Serial.println("[DIAG] Usage: l <mode> where mode=t|h|l|r (or 1/0 for high/low)");
          break;
        }
        printStatusBanner();
        break;
      }
      case 'd':
      case 'D': {
        char token[12];
        if (!readDiagArgToken(token, sizeof(token)) || !token[0] || !applyPdModeCommand(token[0])) {
          Serial.println("[DIAG] Usage: d <mode> where mode=t|o|n (or 1/0 for on/off)");
          break;
        }
        printStatusBanner();
        break;
      }
      case 'm':
      case 'M': {
        char token[12];
        uint8_t value = 0;
        if (!readDiagArgToken(token, sizeof(token)) || !parseUint8Token(token, 1, 8, value)) {
          Serial.println("[DIAG] Usage: mN or m N (N=1..8)");
          break;
        }
        bool ok = applyVolumeAll(value);
        Serial.print("[DIAG] Runtime volume set to ");
        Serial.print(value);
        Serial.print(ok ? " OK" : " FAILED");
        Serial.println(" (see [RUNTIME] lines)");
        printStatusBanner();
        break;
      }
      case 'q':
      case 'Q': {
        char token[12];
        uint8_t value = 0;
        if (!readDiagArgToken(token, sizeof(token)) || !parseUint8Token(token, 0, 8, value)) {
          Serial.println("[DIAG] Usage: qN or q N (N=0..8)");
          break;
        }
        bool ok = applySquelchAll(value);
        Serial.print("[DIAG] Runtime squelch set to ");
        Serial.print(value);
        Serial.print(ok ? " OK" : " FAILED");
        Serial.println(" (see [RUNTIME] lines)");
        printStatusBanner();
        break;
      }
      case 'f':
      case 'F': {
        char token[12];
        bool pre = false;
        bool high = false;
        bool low = false;
        if (!readDiagArgToken(token, sizeof(token)) || !parseFilterToken(token, pre, high, low)) {
          Serial.println("[DIAG] Usage: fXYZ or f XYZ (X=pre,Y=high,Z=low; each 0/1), e.g. f111");
          break;
        }
        bool ok = applyFiltersAll(pre, high, low);
        Serial.print("[DIAG] Runtime filter set to ");
        Serial.print(pre ? 1 : 0);
        Serial.print(high ? 1 : 0);
        Serial.print(low ? 1 : 0);
        Serial.print(ok ? " OK" : " FAILED");
        Serial.println(" (see [RUNTIME] lines)");
        printStatusBanner();
        break;
      }
      case 'g':
      case 'G': {
        char modTok[12];
        char dirTok[12];
        char freqTok[20];
        float mhz = 0.0f;

        if (!readDiagArgToken(modTok, sizeof(modTok)) ||
            !readDiagArgToken(dirTok, sizeof(dirTok)) ||
            !readDiagArgToken(freqTok, sizeof(freqTok))) {
          Serial.println("[DIAG] Usage: g <v|u> <tx|rx> <freq_mhz>  e.g. g v tx 144.3900");
          break;
        }

        bool setTx = false;
        if (dirTok[0] == 't' || dirTok[0] == 'T') {
          setTx = true;
        } else if (dirTok[0] == 'r' || dirTok[0] == 'R') {
          setTx = false;
        } else {
          Serial.println("[DIAG] Direction must be tx or rx");
          break;
        }

        if (!parseFreqToken(freqTok, mhz)) {
          Serial.println("[DIAG] Frequency format invalid. Example: 144.3900");
          break;
        }

        bool ok = applyFrequencyCommand(modTok[0], setTx, mhz);
        Serial.print("[DIAG] Runtime frequency set ");
        Serial.print(ok ? "OK" : "FAILED");
        Serial.println(" (see [RUNTIME] lines)");
        printStatusBanner();
        break;
      }
      case 'v':
      case 'V':
        runLocalPttPulse("VHF", PIN_V_PTT, DIAG_LOCAL_PTT_PULSE_MS);
        break;
      case 'u':
      case 'U':
        runLocalPttPulse("UHF", PIN_U_PTT, DIAG_LOCAL_PTT_PULSE_MS);
        break;
      case 'b':
      case 'B':
        runLocalPttPulse("VHF", PIN_V_PTT, DIAG_LOCAL_PTT_PULSE_MS);
        runLocalPttPulse("UHF", PIN_U_PTT, DIAG_LOCAL_PTT_PULSE_MS);
        break;
      default:
        Serial.print("[DIAG] Unknown command: ");
        Serial.println(cmd);
        break;
    }
  }
#endif
}

//////////////////////////////////////////////////////////////////////////////
// LED FEEDBACK STATE MACHINE
//////////////////////////////////////////////////////////////////////////////

enum {
  LED_IDLE,           // Off
  LED_CONFIG_ACTIVE,  // Distinct blink while startup configuration is running
  LED_SUCCESS_FADE,   // Slow fade in/out
  LED_FAILURE_BLINK,  // Fast blink
  LED_TX_HEARTBEAT,   // Faster heartbeat while TX is confirmed
  LED_WARNING_BLINK,  // Slow warning blink for blocked TX requests
  LED_LOW_VOLTAGE     // Short pulse / long gap — low battery cutoff active
};

uint8_t currentLedState = LED_IDLE;
unsigned long ledTimestamp = 0;
uint16_t gStatusRowsPrinted = 0;

enum {
  TX_DETECT_IDLE,
  TX_DETECT_CANDIDATE,
  TX_DETECT_CONFIRMED,
  TX_DETECT_BLOCKED_PD_LOW
};

uint8_t gTxDetectState = TX_DETECT_IDLE;

void setLedState(uint8_t newState) {
  if (currentLedState != newState) {
    currentLedState = newState;
    ledTimestamp = millis();
  }
}

void updateLed() {
  unsigned long now = millis();
  unsigned long elapsed = now - ledTimestamp;

  switch (currentLedState) {
    case LED_IDLE:
      analogWrite(STATUS_LED_PIN, 0);
      break;

    case LED_CONFIG_ACTIVE:
      // Medium blink while startup config is in progress.
      if ((elapsed / LED_CONFIG_BLINK_HALF_PERIOD_MS) % 2 == 0) {
        analogWrite(STATUS_LED_PIN, 200);
      } else {
        analogWrite(STATUS_LED_PIN, 10);
      }
      break;

    case LED_SUCCESS_FADE: {
      // Slow fade: configurable cycle (default 4s = 2s up, 2s down)
      const unsigned long totalCycleMs = LED_HEARTBEAT_FADE_UP_MS + LED_HEARTBEAT_FADE_DOWN_MS;
      unsigned long cycleMs = elapsed % totalCycleMs;
      uint8_t brightness = (cycleMs < LED_HEARTBEAT_FADE_UP_MS)
        ? map(cycleMs, 0, LED_HEARTBEAT_FADE_UP_MS, 0, 255)
        : map(cycleMs, LED_HEARTBEAT_FADE_UP_MS, totalCycleMs, 255, 0);
      analogWrite(STATUS_LED_PIN, brightness);
      break;
    }

    case LED_FAILURE_BLINK: {
      // Fast blink: 100ms on, 100ms off
      if ((elapsed / LED_FAILURE_BLINK_HALF_PERIOD_MS) % 2 == 0) {
        analogWrite(STATUS_LED_PIN, 255);
      } else {
        analogWrite(STATUS_LED_PIN, 0);
      }
      break;
    }

    case LED_TX_HEARTBEAT: {
      // Fast heartbeat during confirmed TX
      const unsigned long totalCycleMs = LED_TX_HEARTBEAT_FADE_UP_MS + LED_TX_HEARTBEAT_FADE_DOWN_MS;
      unsigned long cycleMs = elapsed % totalCycleMs;
      uint8_t brightness = (cycleMs < LED_TX_HEARTBEAT_FADE_UP_MS)
        ? map(cycleMs, 0, LED_TX_HEARTBEAT_FADE_UP_MS, 40, 255)
        : map(cycleMs, LED_TX_HEARTBEAT_FADE_UP_MS, totalCycleMs, 255, 40);
      analogWrite(STATUS_LED_PIN, brightness);
      break;
    }

    case LED_WARNING_BLINK:
      // Slow warning blink: indicates PTT request while PD is low (TX blocked)
      if ((elapsed / LED_WARNING_BLINK_HALF_PERIOD_MS) % 2 == 0) {
        analogWrite(STATUS_LED_PIN, 180);
      } else {
        analogWrite(STATUS_LED_PIN, 0);
      }
      break;

    case LED_LOW_VOLTAGE:
      // Brief pulse then long off — distinct low-battery indicator
      // Pattern: 200 ms ON, 1800 ms OFF (2 s total cycle)
      if ((elapsed % 2000UL) < 200UL) {
        analogWrite(STATUS_LED_PIN, 255);
      } else {
        analogWrite(STATUS_LED_PIN, 0);
      }
      break;
  }
}

const char *txDetectStateName(uint8_t s) {
  switch (s) {
    case TX_DETECT_IDLE: return "IDLE";
    case TX_DETECT_CANDIDATE: return "CANDIDATE";
    case TX_DETECT_CONFIRMED: return "CONFIRMED";
    case TX_DETECT_BLOCKED_PD_LOW: return "BLOCKED_PD_LOW";
    default: return "UNKNOWN";
  }
}

const char *ledStateName(uint8_t s) {
  switch (s) {
    case LED_IDLE: return "IDLE";
    case LED_CONFIG_ACTIVE: return "CFG_BLINK";
    case LED_SUCCESS_FADE: return "OK_FADE";
    case LED_FAILURE_BLINK: return "FAIL_BLINK";
    case LED_TX_HEARTBEAT: return "TX_HEART";
    case LED_WARNING_BLINK: return "WARN_BLINK";
    case LED_LOW_VOLTAGE:   return "LV_CUTOFF";
    default: return "UNKNOWN";
  }
}

bool updateTxDetectState(bool pttAsserted, bool pdHigh) {
  uint8_t prev = gTxDetectState;

#if TX_REQUIRE_PD_HIGH_FOR_CONFIRMED_TX
  bool txConfidence = pttAsserted && pdHigh;
#else
  bool txConfidence = pttAsserted;
#endif

  switch (gTxDetectState) {
    case TX_DETECT_IDLE:
      if (pttAsserted) {
        gTxDetectState = pdHigh ? TX_DETECT_CANDIDATE : TX_DETECT_BLOCKED_PD_LOW;
      }
      break;

    case TX_DETECT_CANDIDATE:
      if (!pttAsserted) {
        gTxDetectState = TX_DETECT_IDLE;
      } else if (txConfidence) {
        gTxDetectState = TX_DETECT_CONFIRMED;
      } else {
        gTxDetectState = TX_DETECT_BLOCKED_PD_LOW;
      }
      break;

    case TX_DETECT_BLOCKED_PD_LOW:
      if (!pttAsserted) {
        gTxDetectState = TX_DETECT_IDLE;
      } else if (pdHigh) {
        gTxDetectState = TX_DETECT_CANDIDATE;
      }
      break;

    case TX_DETECT_CONFIRMED:
      if (!txConfidence) {
        gTxDetectState = pttAsserted ? TX_DETECT_BLOCKED_PD_LOW : TX_DETECT_IDLE;
      }
      break;
  }

  if (DEBUG_MODE && prev != gTxDetectState) {
    char msg[64];
    snprintf(msg, sizeof(msg), "%s -> %s", txDetectStateName(prev), txDetectStateName(gTxDetectState));
    debugPrint("TXSTATE", msg);
  }

  return (gTxDetectState == TX_DETECT_CONFIRMED);
}

uint8_t ledStateForTxDetectState() {
  switch (gTxDetectState) {
    case TX_DETECT_CONFIRMED:
      return LED_TX_HEARTBEAT;
    case TX_DETECT_BLOCKED_PD_LOW:
      return LED_WARNING_BLINK;
    case TX_DETECT_IDLE:
    case TX_DETECT_CANDIDATE:
    default:
      return LED_SUCCESS_FADE;
  }
}

//////////////////////////////////////////////////////////////////////////////
// DEBUG OUTPUT
//////////////////////////////////////////////////////////////////////////////

void debugPrint(const char *label, const char *msg) {
#if DEBUG_MODE
  if (SERIAL_BACKPRESSURE_GUARD && Serial.availableForWrite() < SERIAL_MIN_FREE_FOR_DEBUG) {
    return;
  }
  Serial.print("[");
  Serial.print(label);
  Serial.print("] ");
  Serial.println(msg);
#endif
}

void printConfigBanner() {
  Serial.println();
  Serial.println("===== HamWing Radio Configuration Banner =====");
  Serial.print("Module selection: ");
#if ACTIVE_MODULE == MODULE_VHF
  Serial.println("VHF only");
#elif ACTIVE_MODULE == MODULE_UHF
  Serial.println("UHF only");
#else
  Serial.println("BOTH (VHF + UHF)");
#endif
  Serial.print("Debug mode: ");
  Serial.println(DEBUG_MODE ? "ON" : "OFF");
  
  Serial.println("\nVHF Configuration:");
  Serial.print("  TX: ");
  Serial.print(VHF_CONFIG.freq_tx, 4);
  Serial.print(" MHz, RX: ");
  Serial.print(VHF_CONFIG.freq_rx, 4);
  Serial.println(" MHz");
  Serial.print("  Squelch: ");
  Serial.print(VHF_CONFIG.squelch);
  Serial.print(", Volume: ");
  Serial.println(VHF_CONFIG.volume);

  Serial.println("\nUHF Configuration:");
  Serial.print("  TX: ");
  Serial.print(UHF_CONFIG.freq_tx, 4);
  Serial.print(" MHz, RX: ");
  Serial.print(UHF_CONFIG.freq_rx, 4);
  Serial.println(" MHz");
  Serial.print("  Squelch: ");
  Serial.print(UHF_CONFIG.squelch);
  Serial.print(", Volume: ");
  Serial.println(UHF_CONFIG.volume);

  Serial.println("\nPin Definitions:");
  Serial.print("  PD (Pi controlled): D");
  Serial.print(PIN_PD);
  Serial.print(", H/L (Pi controlled): D");
  Serial.print(PIN_HL);
  Serial.print(", LED: D");
  Serial.println(STATUS_LED_PIN);
  Serial.print("  VHF PTT (Pi controlled): D");
  Serial.print(PIN_V_PTT);
  Serial.print(", UHF PTT (Pi controlled): D");
  Serial.println(PIN_U_PTT);

  Serial.println("============================================");
  Serial.println();
}

// Read battery voltage using Feather M0 onboard 1/2 VBAT divider on A7.
float readBatteryVoltage() {
  return analogRead(PIN_VBAT) * 2.0f * 3.3f / 1024.0f;
}

void printStatusBanner(bool forceHeader, bool ignoreBackpressure) {
  if (!ignoreBackpressure && SERIAL_BACKPRESSURE_GUARD && Serial.availableForWrite() < SERIAL_MIN_FREE_FOR_STATUS) {
    return;
  }

  int pd = digitalRead(PIN_PD);
  int hl = digitalRead(PIN_HL);
  int vptt = digitalRead(PIN_V_PTT);
  int uptt = digitalRead(PIN_U_PTT);
  int vhfSql = digitalRead(PIN_V_SQL);
  int uhfSql = digitalRead(PIN_U_SQL);
  bool pttAsserted = (vptt == LOW) || (uptt == LOW);
  bool txConfidence = pttAsserted && (pd == HIGH);
  float vbat = readBatteryVoltage();

  const char *pdTxt = (pd == HIGH) ? "ACTIVE" : "SLEEP";
  const char *pdLockTxt = gPdLockOn ? "LOCK_ON" : "PI_CTRL";
  const char *hlTxt = (hl == HIGH) ? "HIGH" : "LOW";
  const char *vhfPttTxt = (vptt == LOW) ? "TX" : "IDLE";
  const char *uhfPttTxt = (uptt == LOW) ? "TX" : "IDLE";
  const char *vhfSqlTxt = (vhfSql == HIGH) ? "OPEN" : "CLOSED";
  const char *uhfSqlTxt = (uhfSql == HIGH) ? "OPEN" : "CLOSED";
  const char *pttReqTxt = pttAsserted ? "YES" : "NO";
  const char *txConfTxt = txConfidence ? "YES" : "NO";
  unsigned long now = millis();
  unsigned long vhfAgeMs = (gVhfPttCtrlMs == 0) ? 0 : (now - gVhfPttCtrlMs);
  unsigned long uhfAgeMs = (gUhfPttCtrlMs == 0) ? 0 : (now - gUhfPttCtrlMs);

  if (forceHeader || ((gStatusRowsPrinted % STATUS_HEADER_REPEAT_ROWS) == 0)) {
    Serial.println("LEGEND: CFG=VHF/UHF (O=OK,F=FAIL). VHF_PTT/UHF_PTT show TX when pin is LOW (active-low). TX_CONF means PTT requested while PD is ACTIVE.");
    Serial.println("LEGEND: SQL shows squelch status (OPEN=signal present). V_OWNER/V_ACT/V_AGE and U_OWNER/U_ACT/U_AGE show last control owner/action and ms since that action.");
    Serial.println();
    Serial.println("ms      CFG   PD      PD_LOCK H/L   VHF_PTT VHF_SQL UHF_PTT UHF_SQL PTT_REQ TX_CONF TX_STATE         LED         VBAT  V_OWNER      V_ACT        V_AGE  U_OWNER      U_ACT        U_AGE");
    Serial.println("------  ----  ------  ------- ----  ------- ------- ------- ------- ------- ------- --------------   ----------  ----  -----------  -----------  -----  -----------  -----------  -----");
  }

  char cfg[8];
  snprintf(cfg, sizeof(cfg), "%c/%c", gVhfConfigured ? 'O' : 'F', gUhfConfigured ? 'O' : 'F');

  char row[380];
  snprintf(
    row,
    sizeof(row),
    "%6lu  %-4s  %-6s  %-7s %-4s  %-7s %-7s %-7s %-7s %-7s %-7s %-14s   %-10s  %4.2f  %-11s  %-11s  %5lu  %-11s  %-11s  %5lu",
    now,
    cfg,
    pdTxt,
    pdLockTxt,
    hlTxt,
    vhfPttTxt,
    vhfSqlTxt,
    uhfPttTxt,
    uhfSqlTxt,
    pttReqTxt,
    txConfTxt,
    txDetectStateName(gTxDetectState),
    ledStateName(currentLedState),
    vbat,
    gVhfPttCtrlOwner,
    gVhfPttCtrlAction,
    vhfAgeMs,
    gUhfPttCtrlOwner,
    gUhfPttCtrlAction,
    uhfAgeMs);
  Serial.println(row);

  gStatusRowsPrinted++;
}

void printStatusSnapshotFramed() {
  Serial.println();
  Serial.println("================ STATUS SNAPSHOT BEGIN ================");
  printStatusBanner(true, true);
  Serial.println("================= STATUS SNAPSHOT END =================");
  Serial.println();
}

void printStatusCompactRow() {
  // Compact periodic row to reduce USB serial blocking during long TX windows.
  // Full table remains available through printStatusBanner() (manual snapshots).
  if (SERIAL_BACKPRESSURE_GUARD && Serial.availableForWrite() < 96) {
    return;
  }

  int pd = digitalRead(PIN_PD);
  int vptt = digitalRead(PIN_V_PTT);
  int uptt = digitalRead(PIN_U_PTT);
  int vhfSql = digitalRead(PIN_V_SQL);
  int uhfSql = digitalRead(PIN_U_SQL);
  bool pttAsserted = (vptt == LOW) || (uptt == LOW);
  bool txConfidence = pttAsserted && (pd == HIGH);
  float vbat = readBatteryVoltage();

  if ((gStatusRowsPrinted % STATUS_HEADER_REPEAT_ROWS) == 0) {
    Serial.println();
    Serial.println("ms      PD     VHF  VSQL UHF  USQL PTT TXCF TX_STATE       LED       VBAT");
    Serial.println("------  -----  ---  ---- ---  ---- --- ---- ------------  --------  ----");
  }

  char row[128];
  snprintf(
    row,
    sizeof(row),
    "%6lu  %-5s  %-3s  %-4s %-3s  %-4s %-3s %-4s %-12s  %-8s  %4.2f",
    millis(),
    (pd == HIGH) ? "ACT" : "SLP",
    (vptt == LOW) ? "TX" : "IDL",
    (vhfSql == HIGH) ? "OPEN" : "CLOS",
    (uptt == LOW) ? "TX" : "IDL",
    (uhfSql == HIGH) ? "OPEN" : "CLOS",
    pttAsserted ? "YES" : "NO",
    txConfidence ? "YES" : "NO",
    txDetectStateName(gTxDetectState),
    ledStateName(currentLedState),
    vbat);
  Serial.println(row);

  gStatusRowsPrinted++;
}

//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

void printStartupHeaderBlock() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println(" HAMWING RADIO CONFIG SESSION");
  Serial.print(" Boot ms: ");
  Serial.println(millis());
  Serial.print(" Target modules: ");
#if ACTIVE_MODULE == MODULE_VHF
  Serial.println("VHF");
#elif ACTIVE_MODULE == MODULE_UHF
  Serial.println("UHF");
#else
  Serial.println("VHF+UHF");
#endif
  Serial.println("============================================================");
}

void printConfigPlanTableHeader() {
  Serial.println();
  Serial.println("CFG PLAN");
  Serial.println("MOD   TX_MHZ    RX_MHZ    SQL VOL BW   CTCSS_TX CTCSS_RX FILTERS");
  Serial.println("----  --------  --------  --- --- ---- -------- -------- -------");
}

void printConfigPlanRow(const char *module, const RadioConfig &cfg) {
  const char *bw = (cfg.bandwidth == DRA818_25K) ? "25K" : "12K5";
  char filters[8];
  snprintf(filters, sizeof(filters), "%d%d%d", cfg.filter_pre ? 1 : 0, cfg.filter_high ? 1 : 0, cfg.filter_low ? 1 : 0);

  char row[160];
  snprintf(
    row,
    sizeof(row),
    "%-4s  %8.4f  %8.4f  %3u %3u %-4s %8u %8u %-7s",
    module,
    cfg.freq_tx,
    cfg.freq_rx,
    cfg.squelch,
    cfg.volume,
    bw,
    cfg.ctcss_tx,
    cfg.ctcss_rx,
    filters);
  Serial.println(row);
}

void printConfigAppliedTableHeader() {
  Serial.println();
  Serial.println("CFG APPLIED");
  Serial.println("MOD   RESULT TX_MHZ    RX_MHZ    SQL VOL BW   CTCSS_TX CTCSS_RX FILTERS");
  Serial.println("----  ------ --------  --------  --- --- ---- -------- -------- -------");
}

void printConfigAppliedRow(const char *module, const RadioConfig &cfg, bool ok) {
  const char *bw = (cfg.bandwidth == DRA818_25K) ? "25K" : "12K5";
  char filters[8];
  snprintf(filters, sizeof(filters), "%d%d%d", cfg.filter_pre ? 1 : 0, cfg.filter_high ? 1 : 0, cfg.filter_low ? 1 : 0);

  char row[180];
  snprintf(
    row,
    sizeof(row),
    "%-4s  %-6s %8.4f  %8.4f  %3u %3u %-4s %8u %8u %-7s",
    module,
    ok ? "OK" : "FAIL",
    cfg.freq_tx,
    cfg.freq_rx,
    cfg.squelch,
    cfg.volume,
    bw,
    cfg.ctcss_tx,
    cfg.ctcss_rx,
    filters);
  Serial.println(row);
}

void printConfigSummaryTableHeader() {
  Serial.println();
  Serial.println("----------------------------------------------");
  Serial.println("CFG SUMMARY");
  Serial.println("MOD   RESULT  NOTE");
  Serial.println("----  ------  ----------------------------------------------");
}

void printConfigSummaryRow(const char *module, bool ok) {
  char row[120];
  snprintf(
    row,
    sizeof(row),
    "%-4s  %-6s  %-46s",
    module,
    ok ? "OK" : "FAIL",
    ok ? "All config steps succeeded" : "One or more config steps failed");
  Serial.println(row);
}

void printConfigStepTableHeader(const char *module) {
  Serial.println();
  Serial.print("CFG STEPS - ");
  Serial.println(module);
  Serial.println("ms      MOD  STEP       LIB_ATT RAW_ATT LAST_METHOD STEP_RESULT");
  Serial.println("------  ---  ---------  ------- ------- ----------- -----------");
}

void printConfigStepRow(
  const char *module,
  const char *step,
  uint8_t libTry,
  uint8_t rawTry,
  const char *method,
  bool ok) {
  char row[128];
  snprintf(
    row,
    sizeof(row),
    "%6lu  %-3s  %-9s  %7u %7u %-11s %-11s",
    millis(),
    module,
    step,
    libTry,
    rawTry,
    method,
    ok ? "OK" : "FAIL");
  Serial.println(row);
}

void drainSerialInput(Stream &port) {
  while (port.available()) {
    port.read();
  }
}

void formatFreq(char *out, size_t outSize, float mhz) {
  // Keep float formatting deterministic without relying on printf float support.
  long scaled = (long)(mhz * 10000.0f + 0.5f);
  long whole = scaled / 10000;
  long frac = scaled % 10000;
  if (frac < 0) frac = -frac;
  snprintf(out, outSize, "%ld.%04ld", whole, frac);
}

bool sendATExpect(Stream &port, const char *label, const char *cmd, const char *expected, unsigned long timeoutMs) {
  drainSerialInput(port);

  port.print(cmd);
  port.print("\r\n");

  unsigned long start = millis();
  String response = "";
  while ((millis() - start) < timeoutMs) {
    while (port.available()) {
      char c = (char)port.read();
      response += c;
      if (response.indexOf(expected) >= 0) {
        if (DEBUG_MODE) {
          Serial.print("[");
          Serial.print(label);
          Serial.print("] AT OK: ");
          Serial.print(cmd);
          Serial.print(" => ");
          Serial.println(response);
        }
        return true;
      }
    }
  }

  if (DEBUG_MODE) {
    Serial.print("[");
    Serial.print(label);
    Serial.print("] AT FAIL: ");
    Serial.print(cmd);
    Serial.print(" => ");
    Serial.println(response.length() ? response : "<no response>");
  }

  return false;
}

bool setGroupFallbackRaw(DRA818 &radio, const char *label, const RadioConfig &config) {
  char txStr[16];
  char rxStr[16];
  char groupCmd[96];

  formatFreq(txStr, sizeof(txStr), config.freq_tx);
  formatFreq(rxStr, sizeof(rxStr), config.freq_rx);

  snprintf(
    groupCmd,
    sizeof(groupCmd),
    "AT+DMOSETGROUP=%u,%s,%s,%04u,%u,%04u",
    config.bandwidth,
    txStr,
    rxStr,
    config.ctcss_tx,
    config.squelch,
    config.ctcss_rx);

  return sendATExpect(*radio.serial, label, groupCmd, "DMOSETGROUP:0", DRA_AT_TIMEOUT_MS);
}

bool setVolumeFallbackRaw(DRA818 &radio, const char *label, uint8_t volume) {
  char volCmd[32];
  snprintf(volCmd, sizeof(volCmd), "AT+DMOSETVOLUME=%u", volume);
  return sendATExpect(*radio.serial, label, volCmd, "DMOSETVOLUME:0", DRA_AT_TIMEOUT_MS);
}

bool setFilterFallbackRaw(DRA818 &radio, const char *label, bool pre, bool high, bool low) {
  char filterCmd[40];
  snprintf(filterCmd, sizeof(filterCmd), "AT+SETFILTER=%u,%u,%u", pre ? 1 : 0, high ? 1 : 0, low ? 1 : 0);
  return sendATExpect(*radio.serial, label, filterCmd, "DMOSETFILTER:0", DRA_AT_TIMEOUT_MS);
}

bool configureRadio(DRA818 &radio, const char *label, RadioConfig &config) {
  printConfigStepTableHeader(label);
  
  // Handshake
  bool ok = false;
  uint8_t libTry = 0;
  uint8_t rawTry = 0;
  const char *method = "LIB";
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    libTry++;
    ok = (radio.handshake() == 1);
  }
  if (!ok) {
    method = "RAW";
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      rawTry++;
      ok = sendATExpect(*radio.serial, label, "AT+DMOCONNECT", "DMOCONNECT:0", DRA_AT_TIMEOUT_MS);
    }
  }
  printConfigStepRow(label, "HANDSHAKE", libTry, rawTry, method, ok);
  if (!ok) {
    Serial.print(label);
    Serial.println(": Handshake FAILED");
    return false;
  }

  // Set group (frequency + squelch)
  ok = false;
  libTry = 0;
  rawTry = 0;
  method = "LIB";
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    libTry++;
    ok = (radio.group(config.bandwidth, config.freq_tx, config.freq_rx, config.ctcss_tx, config.squelch, config.ctcss_rx) == 1);
  }
  if (!ok) {
    method = "RAW";
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      rawTry++;
      ok = setGroupFallbackRaw(radio, label, config);
    }
  }
  printConfigStepRow(label, "GROUP", libTry, rawTry, method, ok);
  if (!ok) {
    Serial.print(label);
    Serial.println(": Group config FAILED");
    return false;
  }

  // Set volume
  ok = false;
  libTry = 0;
  rawTry = 0;
  method = "LIB";
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    libTry++;
    ok = (radio.volume(config.volume) == 1);
  }
  if (!ok) {
    method = "RAW";
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      rawTry++;
      ok = setVolumeFallbackRaw(radio, label, config.volume);
    }
  }
  printConfigStepRow(label, "VOLUME", libTry, rawTry, method, ok);
  if (!ok) {
    Serial.print(label);
    Serial.println(": Volume config FAILED");
    return false;
  }

  // Set filters
  ok = false;
  libTry = 0;
  rawTry = 0;
  method = "LIB";
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    libTry++;
    ok = (radio.filters(config.filter_pre, config.filter_high, config.filter_low) == 1);
  }
  if (!ok) {
    method = "RAW";
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      rawTry++;
      ok = setFilterFallbackRaw(radio, label, config.filter_pre, config.filter_high, config.filter_low);
    }
  }
  printConfigStepRow(label, "FILTER", libTry, rawTry, method, ok);
  if (!ok) {
    Serial.print(label);
    Serial.println(": Filter config FAILED");
    return false;
  }

  printConfigStepRow(label, "COMPLETE", 0, 0, "-", true);
  return true;
}

void ensureSerial2Mux() {
  pinPeripheral(PIN_U_DRA_RX, PIO_SERCOM);
  pinPeripheral(PIN_U_DRA_TX, PIO_SERCOM);
}

void activateUhfUart(bool usePreferred) {
#if TRY_UHF_BOTH_UART_MAPPINGS
  activeUhfPreferredMapping = usePreferred;
  activeUhfSerial = usePreferred ? &Serial2Preferred : &Serial2Fallback;
#endif
  activeUhfSerial->begin(9600);
  ensureSerial2Mux();
}

#if STUCK_PTT_WATCHDOG_ENABLE
void checkStuckPttWatchdog() {
  // Monitor if TX_DETECT_CONFIRMED has been held for too long (stuck PTT).
  // If it exceeds STUCK_PTT_MAX_TX_MS, force-unkey the PTT line and emit alert.
  
  if (gTxDetectState != TX_DETECT_CONFIRMED) {
    // PTT is not in confirmed TX state, reset tracking
    gTxConfirmedSinceMs = 0;
    gStuckPttAlertEmitted = false;
    return;
  }

  unsigned long now = millis();
  
  // First time entering TX_DETECT_CONFIRMED, record the timestamp
  if (gTxConfirmedSinceMs == 0) {
    gTxConfirmedSinceMs = now;
    return;
  }

  unsigned long txDurationMs = now - gTxConfirmedSinceMs;
  
  if (txDurationMs > (unsigned long)STUCK_PTT_MAX_TX_MS) {
    if (!gStuckPttAlertEmitted) {
      // First time threshold is exceeded: force-unkey and emit alert
      gStuckPttAlertEmitted = true;
      
      char msg[160];
      snprintf(msg, sizeof(msg),
        "STUCK PTT DETECTED: TX held for %.1f minutes. Force-unkeyng PTT.",
        txDurationMs / 60000.0f);
      debugPrint("STUCK_PTT", msg);
      
      // Temporarily reclaim PTT ownership to force-unkey
      int vptt = digitalRead(PIN_V_PTT);
      int uptt = digitalRead(PIN_U_PTT);
      
      if (vptt == LOW) {
        pinMode(PIN_V_PTT, OUTPUT);
        digitalWrite(PIN_V_PTT, HIGH);
        delay(10);
        pinMode(PIN_V_PTT, INPUT);  // Release back to Pi
        debugPrint("STUCK_PTT", "VHF PTT forced HIGH");
      }
      
      if (uptt == LOW) {
        pinMode(PIN_U_PTT, OUTPUT);
        digitalWrite(PIN_U_PTT, HIGH);
        delay(10);
        pinMode(PIN_U_PTT, INPUT);  // Release back to Pi
        debugPrint("STUCK_PTT", "UHF PTT forced HIGH");
      }
    } else {
      // Alert already emitted, emit periodic warning
      static unsigned long gLastStuckPttWarnMs = 0;
      if ((now - gLastStuckPttWarnMs) > 30000) {
        gLastStuckPttWarnMs = now;
        char msg[96];
        snprintf(msg, sizeof(msg),
          "Still in STUCK_PTT state: TX held for %.1f minutes",
          txDurationMs / 60000.0f);
        debugPrint("STUCK_PTT", msg);
      }
    }
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
// SETUP
//////////////////////////////////////////////////////////////////////////////

void setup() {
  // Initialize hardware watchdog if enabled
#if HARDWARE_WATCHDOG_ENABLE
  int watchdogID = Watchdog.enable(HARDWARE_WATCHDOG_TIMEOUT_MS);
  if (watchdogID < 0) {
    Serial.println("WARNING: Watchdog failed to initialize");
  } else {
    Serial.print("Watchdog initialized: ");
    Serial.print(HARDWARE_WATCHDOG_TIMEOUT_MS);
    Serial.println(" ms timeout");
  }
#endif

  // Set ADC resolution for better battery voltage accuracy
  analogReadResolution(VBAT_ADC_RESOLUTION);

  // Shut down the ATWINC1500 WiFi chip immediately — not used in this firmware.
  // Saves ~50-80 mA.  Must be done before anything else touches SPI.
  WiFi.setPins(WINC_CS, WINC_IRQ, WINC_RST, WINC_EN);
  WiFi.end();

  Serial.begin(115200);
  delay(1000);  // Let serial stabilize

  Serial.println("----------------------------------------------");
  Serial.println("HamWing_RadioConfig starting...");
  Serial.println("----------------------------------------------");
  delay(500);

  // Feather temporarily owns shared lines so radios are definitely awake for config.
  driveSharedLinesForStartupConfig();

  // Non-shared inputs
  pinMode(PIN_V_SQL, INPUT);
  pinMode(PIN_U_SQL, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  printStartupHeaderBlock();
  printConfigBanner();

  printConfigPlanTableHeader();
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigPlanRow("VHF", VHF_CONFIG);
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigPlanRow("UHF", UHF_CONFIG);
#endif

  // Open serial interfaces
  Serial1.begin(9600);
  debugPrint("UART", "VHF Serial1 opened");

  activateUhfUart(true);
  NVIC_EnableIRQ(SERCOM1_IRQn);
  debugPrint("UART", "UHF Serial2 opened");

  // Distinct LED state while module configuration is running.
  setLedState(LED_CONFIG_ACTIVE);

  Serial.print("Starting module configuration in ");
  Serial.print(PRE_CONFIG_DELAY_MS);
  Serial.println(" ms...");
  delay(PRE_CONFIG_DELAY_MS);

  // Configure radios
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  gVhfConfigured = configureRadio(draV, "VHF", VHF_CONFIG);
  if (!gVhfConfigured) {
    Serial.println("VHF configuration FAILED");
  }
#endif

#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  gUhfConfigured = configureRadio(draU, "UHF", UHF_CONFIG);
  if (!gUhfConfigured) {
    Serial.println("UHF configuration FAILED");
  }
#endif

  // Print applied configuration summary as one contiguous table.
  printConfigAppliedTableHeader();
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigAppliedRow("VHF", VHF_CONFIG, gVhfConfigured);
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigAppliedRow("UHF", UHF_CONFIG, gUhfConfigured);
#endif

  printConfigSummaryTableHeader();
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigSummaryRow("VHF", gVhfConfigured);
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  printConfigSummaryRow("UHF", gUhfConfigured);
#endif

  // Determine overall success
  bool allConfigured = true;
#if ACTIVE_MODULE == MODULE_VHF
  allConfigured = gVhfConfigured;
#elif ACTIVE_MODULE == MODULE_UHF
  allConfigured = gUhfConfigured;
#else
  allConfigured = gVhfConfigured && gUhfConfigured;
#endif

  if (allConfigured) {
    Serial.println("\nAll configured radios are READY. Handing off control to Raspberry Pi.");
    setLedState(LED_SUCCESS_FADE);
    gConfigurationComplete = true;
  } else {
    Serial.println("\nConfiguration FAILED. Check wiring and module power.");
    setLedState(LED_FAILURE_BLINK);
  }

  // Always release lines after startup attempt so Pi can take control.
  releaseSharedLinesToPi();

  printStatusBanner();
  Serial.println("[DIAG] Serial commands ready: h,s,c,z<mode>,l<mode>,d<mode>,mN,qN,fXYZ,g M D F,v,u,b");
  delay(2000);
}

//////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
//////////////////////////////////////////////////////////////////////////////

void loop() {
  // Reset hardware watchdog to prevent auto-reset
#if HARDWARE_WATCHDOG_ENABLE
  Watchdog.reset();
#endif

  // Update LED feedback
  updateLed();

  // Optional PD lock watchdog keeps radios awake despite external PD changes.
  enforcePdLockIfEnabled();

#if LOW_VOLTAGE_CUTOFF_ENABLE
  // Check battery voltage and enter/exit cutoff state as needed.
  checkLowVoltageCutoff();
#endif

  // Optional interactive diagnostics over USB serial.
  handleSerialDiagCommands();

  // Monitor Pi control lines and adjust LED accordingly
  if (gConfigurationComplete) {
    int pd = digitalRead(PIN_PD);
    int vptt = digitalRead(PIN_V_PTT);
    int uptt = digitalRead(PIN_U_PTT);
    
    // Read SQL (squelch open) signals
    int vhfSql = digitalRead(PIN_V_SQL);
    int uhfSql = digitalRead(PIN_U_SQL);

    if (gLastVhfPttSense < 0) {
      gLastVhfPttSense = vptt;
    }
    if (gLastUhfPttSense < 0) {
      gLastUhfPttSense = uptt;
    }
    if (gLastVhfSqlSense < 0) {
      gLastVhfSqlSense = vhfSql;
    }
    if (gLastUhfSqlSense < 0) {
      gLastUhfSqlSense = uhfSql;
    }

    if (vptt != gLastVhfPttSense) {
      gVhfPttEdgeCount++;
      gLastVhfPttSense = vptt;
      setPttControlStateByPin(PIN_V_PTT, "PI", vptt == LOW ? "USE_LOW" : "RELEASE_HIGH", true);
      char msg[80];
      snprintf(msg, sizeof(msg), "VHF_PTT edge #%lu now %s", (unsigned long)gVhfPttEdgeCount, vptt == LOW ? "LOW" : "HIGH");
      debugPrint("PTT", msg);
    }
    if (uptt != gLastUhfPttSense) {
      gUhfPttEdgeCount++;
      gLastUhfPttSense = uptt;
      setPttControlStateByPin(PIN_U_PTT, "PI", uptt == LOW ? "USE_LOW" : "RELEASE_HIGH", true);
      char msg[80];
      snprintf(msg, sizeof(msg), "UHF_PTT edge #%lu now %s", (unsigned long)gUhfPttEdgeCount, uptt == LOW ? "LOW" : "HIGH");
      debugPrint("PTT", msg);
    }
    
    // SQL edge detection
    if (vhfSql != gLastVhfSqlSense) {
      gLastVhfSqlSense = vhfSql;
      if (vhfSql == HIGH) {
        gVhfSqlOpenCount++;
        debugPrint("SQL", "VHF squelch OPEN");
      } else {
        debugPrint("SQL", "VHF squelch CLOSED");
      }
    }
    if (uhfSql != gLastUhfSqlSense) {
      gLastUhfSqlSense = uhfSql;
      if (uhfSql == HIGH) {
        gUhfSqlOpenCount++;
        debugPrint("SQL", "UHF squelch OPEN");
      } else {
        debugPrint("SQL", "UHF squelch CLOSED");
      }
    }

    bool pttAsserted = (vptt == LOW) || (uptt == LOW);
    uint8_t prevTxState = gTxDetectState;
    updateTxDetectState(pttAsserted, pd == HIGH);

#if STUCK_PTT_WATCHDOG_ENABLE
    // Check for stuck/held PTT condition
    checkStuckPttWatchdog();
#endif

#if LOW_VOLTAGE_CUTOFF_ENABLE
    uint8_t desiredLedState = gLowVoltageCutoffActive ? LED_LOW_VOLTAGE : ledStateForTxDetectState();
#else
    uint8_t desiredLedState = ledStateForTxDetectState();
#endif
    if (currentLedState != desiredLedState) {
      setLedState(desiredLedState);
    }

    if (prevTxState != gTxDetectState) {
      const char *pdTxt = (pd == HIGH) ? "HIGH(ACTIVE)" : "LOW(SLEEP)";
      const char *vpttTxt = (vptt == LOW) ? "LOW(TX_REQ)" : "HIGH(IDLE)";
      const char *upttTxt = (uptt == LOW) ? "LOW(TX_REQ)" : "HIGH(IDLE)";
      char trn[220];
      snprintf(
        trn,
        sizeof(trn),
        "ms=%lu TX %s->%s PD=%s VHF_PTT=%s UHF_PTT=%s LED=%s",
        millis(),
        txDetectStateName(prevTxState),
        txDetectStateName(gTxDetectState),
        pdTxt,
        vpttTxt,
        upttTxt,
        ledStateName(currentLedState));
      debugPrint("TRN", trn);
    }

        // If config succeeded but PD remains low, radios are asleep; emit periodic warning.
        if (pd == LOW && (millis() - gLastPdLowWarnMs) > PD_LOW_WARN_INTERVAL_MS) {
      gLastPdLowWarnMs = millis();
    #if HOLD_PD_HIGH_AFTER_CONFIG
      debugPrint("PD", "PD read LOW while HOLD_PD_HIGH_AFTER_CONFIG=1; check external load/short on PD line");
    #else
      debugPrint("PD", "PD is LOW after handoff; Pi/hat is keeping radios asleep");
    #endif
        }

            if ((gVhfPttEdgeCount + gUhfPttEdgeCount) == 0 && (millis() - gLastNoPttEdgeWarnMs) > NO_PTT_EDGE_WARN_INTERVAL_MS) {
          gLastNoPttEdgeWarnMs = millis();
          debugPrint("PTT", "No PTT edges seen on D16/D17 yet. Pi may not be driving expected PTT net.");
            }
  }

  // Periodic status output (configured interval)
  if (millis() >= gStatusPauseUntilMs && (millis() - gLastPeriodicStatusMs > STATUS_ROW_INTERVAL_MS)) {
    gLastPeriodicStatusMs = millis();
    if (DEBUG_MODE) {
      if (gPeriodicStatusCompact) {
        printStatusCompactRow();
      } else {
        printStatusBanner(false);
      }
    }
  }

  LowPower.idle(100);  // SAMD21 IDLE sleep — CPU halted, SERCOMs/timers/USB still run
}
