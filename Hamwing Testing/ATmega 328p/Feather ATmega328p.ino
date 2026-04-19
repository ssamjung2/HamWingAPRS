#include <DRA818.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////
// Pin definitions based on provided mappings for Hamwing board.
// Define the DRA818 radio modules connections below.
#define RfPwrSleepPin 12  // RF power sleep control
#define RfPwrHLPin 18     // RF power high/low control
// VHF Radio Module
#define PIN_V_DRA_RX 0  // VHF DRA RX on hardware UART RX (D0)
#define PIN_V_DRA_TX 1  // VHF DRA TX on hardware UART TX (D1)
#define V_SQLPin 14     // VHF squelch
#define V_PTTPin 16     // VHF Push-To-Talk (PTT)
// UHF Radio Module
#define PIN_U_DRA_RX 11  // UHF from module TXD via JP1/F4 (SoftwareSerial RX)
#define PIN_U_DRA_TX 10  // UHF to module RXD via JP2/F3 (SoftwareSerial TX)
#define U_SQLPin 15      // UHF squelch
#define U_PTTPin 17      // UHF Push-To-Talk (PTT)

#define LEDPin LED_BUILTIN  // On-board LED
#define VBatPin A7          // Battery voltage monitoring
//////////////////////////////////////////////////////////////////////////////

#define ADC_REFERENCE DEFAULT
#define OPEN_SQUELCH false

// Shared power-down (PD) line — controls both radio modules together.
#define PowerSavingOn digitalWrite(RfPwrSleepPin, LOW)    // PD LOW -> both radios powered down
#define PowerSavingOff digitalWrite(RfPwrSleepPin, HIGH)  // PD HIGH -> both radios active

// Per-module PTT control.
// DRA818 PTT pin is active-low at the module, but many HamWing variants use
// an NPN inverting stage so the Feather pin that keys TX is active-HIGH.
#define PTT_ACTIVE_LOW 1  // Set to 0 only if your board has an inverting PTT stage.
#if PTT_ACTIVE_LOW
#define RfON_VHF digitalWrite(V_PTTPin, LOW)    // Key VHF transmitter
#define RfOFF_VHF digitalWrite(V_PTTPin, HIGH)  // Unkey VHF transmitter
#define RfON_UHF digitalWrite(U_PTTPin, LOW)    // Key UHF transmitter
#define RfOFF_UHF digitalWrite(U_PTTPin, HIGH)  // Unkey UHF transmitter
#else
#define RfON_VHF digitalWrite(V_PTTPin, HIGH)  // Key VHF transmitter (inverted stage)
#define RfOFF_VHF digitalWrite(V_PTTPin, LOW)  // Unkey VHF transmitte (inverted stage)
#define RfON_UHF digitalWrite(U_PTTPin, HIGH)  // Key UHF transmitter (inverted stage)
#define RfOFF_UHF digitalWrite(U_PTTPin, LOW)  // Unkey UHF transmitter (inverted stage)
#endif

#define LedON digitalWrite(LEDPin, HIGH)
#define LedOff digitalWrite(LEDPin, LOW)

// DRA818 pin 7 (H/L): LOW = 0.5W, floated = 1W.
#define RfPwrHigh pinMode(RfPwrHLPin, INPUT)
#define RfPwrLow \
  pinMode(RfPwrHLPin, OUTPUT); \
  digitalWrite(RfPwrHLPin, LOW)

// KiCad-verified HamWing wiring:
// - DRA818 PD is a single shared line through the PD jumper to F5 (Feather D12).
//   Both radio modules share RfPwrSleepPin (D12). There is no separate VHF or UHF PD pin.
// - UHF UART is routed via JP1/JP2 to F4/F3 (Feather D11/D10, Serial2).
#define ENABLE_UHF_ALT_PORT 0
#define NO_TX_MODE 0        // Set to 1 to inhibit all PTT for bench testing without antennas.
#define PTT_TEST_ENABLED 1  // Set to 1 to run a 3-second PTT key test for each module in loop().
#define PTT_TEST_DURATION_MS 3000
#define PTT_POLARITY_PROBE 1      // 1 = run LOW-vs-HIGH VHF key probe each loop for hardware diagnosis.
#define PTT_GPIO_SWEEP_MODE 0     // 1 = bypass PTT macros and sweep raw V/U PTT pin levels for hardware diagnosis.
#define PTT_GPIO_SWEEP_STEP_MS 4000
#define SCAN_TEST_MODE 0           // 1 = run RX frequency scan test instead of PTT tests.
#define HANDOFF_GPIO_TO_PI_AFTER_CONFIG 0  // 1 = Feather configures radios, then releases PD/PTT/HL for Pi ownership.
#define HANDOFF_REQUIRE_CONFIG_OK 1        // 1 = only hand off if startup radio config succeeded.
#define SCAN_RANGE_MHZ 0.2500f     // Sweep from default frequency +/- this range.
#define SCAN_STEP_MHZ 0.010f      // 12.5 kHz step.
#define SCAN_DWELL_MS 180          // Dwell time at each scan frequency.
#define SCAN_PASS_PAUSE_MS 500
#define PD_WAKE_SETTLE_MS 800      // Delay after forcing PD high before keying PTT.
#define RF_TEST_USE_HIGH_POWER 1  // 1 = high power for easier nearby RF verification
#define AT_TIMEOUT_MS 2500
#define AT_CMD_GAP_MS 120
#define DEVMODE 1  // Set to 1 to enable verbose debug logging over Serial.
#define TRY_UHF_BOTH_UART_MAPPINGS 1

//////////////////////////////////////////////////////////////////////////////

// ---------------------- User-configurable defaults ----------------------
// Edit these values to set startup radio defaults.
struct RadioDefaults {
  float freq_tx;
  float freq_rx;
  uint8_t squelch;    // 0-8
  uint8_t volume;     // 1-8
  uint8_t ctcss_tx;   // 0-38
  uint8_t ctcss_rx;   // 0-38
  uint8_t bandwidth;  // DRA818_12K5 or DRA818_25K
  bool filter_pre;
  bool filter_high;
  bool filter_low;
};

RadioDefaults VHF_DEFAULTS = {
  144.3900,  // TX frequency (MHz)
  144.3900,  // RX frequency (MHz)
  4,         // squelch
  6,         // volume
  0,         // CTCSS TX
  0,         // CTCSS RX
  DRA818_12K5,
  true,
  true,
  true
};

RadioDefaults UHF_DEFAULTS = {
  440.8000,  // TX frequency (MHz)
  440.8000,  // RX frequency (MHz)
  4,
  6,
  0,
  0,
  DRA818_12K5,
  true,
  true,
  true
};

#define MODULE_VHF 0
#define MODULE_UHF 1
#define MODULE_BOTH 2
#define ACTIVE_MODULE MODULE_BOTH

//////////////////////////////////////////////////////////////////////////////
// ATmega328P UART layout:
// - VHF uses the hardware UART on D0/D1 (Serial).
// - UHF uses SoftwareSerial so both modules can still be managed from one MCU.
// Note: the single hardware UART means USB console logging shares wires with VHF AT traffic.
// Keep this sketch focused on radio diagnostics/config where that tradeoff is acceptable.
SoftwareSerial Serial2Preferred(PIN_U_DRA_RX, PIN_U_DRA_TX);
#if TRY_UHF_BOTH_UART_MAPPINGS
// Legacy/opposite mapping retained as a diagnostic fallback.
SoftwareSerial Serial2Fallback(PIN_U_DRA_TX, PIN_U_DRA_RX);
#endif
SoftwareSerial *activeUhfSerial = &Serial2Preferred;
bool activeUhfPreferredMapping = true;
//////////////////////////////////////////////////////////////////////////////

// Comment the above, and uncomment the following to use Hardware Serial.
//DRA818 dra(&Serial, PTT);
DRA818 draV((Stream *)&Serial, DRA818_VHF);
DRA818 draUPreferred((Stream *)&Serial2Preferred, DRA818_UHF);
#if TRY_UHF_BOTH_UART_MAPPINGS
DRA818 draUFallback((Stream *)&Serial2Fallback, DRA818_UHF);
#endif

#define BAND_NONE 0
#define BAND_VHF 1
#define BAND_UHF 2

bool gVhfConfigured = false;
bool gUhfConfigured = false;
int gVhfSqlBaseline = HIGH;
int gUhfSqlBaseline = HIGH;
bool gPiHandoffComplete = false;

bool isDebugEnabled() {
#if DEVMODE
  return true;
#else
  return false;
#endif
}

void logDebugPrefix(const __FlashStringHelper *scope) {
  if (!isDebugEnabled()) {
    return;
  }
  Serial.print(F("[debug] "));
  if (scope != NULL) {
    Serial.print(scope);
    Serial.print(F(": "));
  }
}

void logDebugMessage(const __FlashStringHelper *scope, const __FlashStringHelper *message) {
  if (!isDebugEnabled()) {
    return;
  }
  logDebugPrefix(scope);
  Serial.println(message);
}

void logDebugText(const __FlashStringHelper *scope, const __FlashStringHelper *label, const char *value) {
  if (!isDebugEnabled()) {
    return;
  }
  logDebugPrefix(scope);
  Serial.print(label);
  Serial.println(value);
}

void logDebugBool(const __FlashStringHelper *scope, const __FlashStringHelper *label, bool value, const __FlashStringHelper *trueText, const __FlashStringHelper *falseText) {
  if (!isDebugEnabled()) {
    return;
  }
  logDebugPrefix(scope);
  Serial.print(label);
  Serial.println(value ? trueText : falseText);
}

void logStepResult(const __FlashStringHelper *scope, const __FlashStringHelper *step, bool ok) {
  Serial.print(scope);
  Serial.print(F(": "));
  Serial.print(step);
  Serial.println(ok ? F(" OK") : F(" FAILED"));
}

void logPinStateSnapshot(const __FlashStringHelper *scope, const __FlashStringHelper *context) {
  if (!isDebugEnabled()) {
    return;
  }
  logDebugPrefix(scope);
  Serial.print(context);
  Serial.print(F(" | PD="));
  Serial.print(digitalRead(RfPwrSleepPin));
  Serial.print(F(" H/L="));
  Serial.print(digitalRead(RfPwrHLPin));
  Serial.print(F(" VPTT="));
  Serial.print(digitalRead(V_PTTPin));
  Serial.print(F(" UPTT="));
  Serial.println(digitalRead(U_PTTPin));
}

int sampleReleasedPttLine(uint8_t pin) {
  // Temporarily release the PTT line and read the physical wire level.
  // If this reads LOW while released, an external device is pulling the
  // line down (common cause: Pi GPIO default pulldown or wiring short).
  pinMode(pin, INPUT);
  delay(2);
  int level = digitalRead(pin);

  // Restore to a deterministic, safe idle drive state.
  pinMode(pin, OUTPUT);
#if PTT_ACTIVE_LOW
  digitalWrite(pin, HIGH);
#else
  digitalWrite(pin, LOW);
#endif
  return level;
}

void diagnoseReleasedPttLines() {
  setPttIdle();
  int vhfReleased = sampleReleasedPttLine(V_PTTPin);
  int uhfReleased = sampleReleasedPttLine(U_PTTPin);

  Serial.print(F("PTT released-line sense V/U: "));
  Serial.print(vhfReleased);
  Serial.print(F("/"));
  Serial.println(uhfReleased);

  if (vhfReleased == LOW) {
    Serial.println(F("WARNING: VHF PTT line reads LOW when released. External pull-down/load detected on D16/VHF PTT net."));
  }
  if (uhfReleased == LOW) {
    Serial.println(F("WARNING: UHF PTT line reads LOW when released. External pull-down/load detected on D17/UHF PTT net."));
  }
}

void diagnoseReleasedUhfUartLines() {
  // Temporarily read the raw D10/D11 line levels as plain GPIO inputs.
  // UART idle level should be HIGH. A LOW here indicates an external pull-down,
  // active driver, or short on the UHF UART net.
  pinMode(PIN_U_DRA_RX, INPUT);
  pinMode(PIN_U_DRA_TX, INPUT);
  delay(2);
  int d11Level = digitalRead(PIN_U_DRA_RX);  // D11: Feather RX from UHF TXD
  int d10Level = digitalRead(PIN_U_DRA_TX);  // D10: Feather TX to UHF RXD

  Serial.print(F("UHF UART released-line sense D11/D10: "));
  Serial.print(d11Level);
  Serial.print(F("/"));
  Serial.println(d10Level);

  if (d11Level == LOW || d10Level == LOW) {
    Serial.println(F("WARNING: UHF UART line LOW while released. External loading likely (hat/UART overlap, pull-down, or short)."));
  }

  // Restore UART pin mux after diagnostic sampling.
  ensureSerial2Mux();
}

void handoffControlLinesToPi() {
  // Place radios in non-transmit state and then release shared control lines.
  // INPUT/INPUT_PULLUP leaves Pi as sole active owner of PD/PTT/HL nets.
  setPttIdle();
  PowerSavingOff;

  pinMode(V_PTTPin, INPUT_PULLUP);    // Keep inactive-high bias while released.
  pinMode(U_PTTPin, INPUT_PULLUP);    // Keep inactive-high bias while released.
  pinMode(RfPwrSleepPin, INPUT_PULLUP);  // Keep awake-high bias while released.
  pinMode(RfPwrHLPin, INPUT);         // Release H/L net; Pi controls TX power level.

  Serial.println(F("HANDOFF: Released PD/PTT/HL lines to Raspberry Pi control."));
  Serial.println(F("HANDOFF: Feather loop TX/scan actions are disabled while handoff is active."));
}

bool ensurePdReadyForPtt(const __FlashStringHelper *scope) {
  int pdBefore = digitalRead(RfPwrSleepPin);
  logDebugPrefix(scope);
  Serial.print(F("PD pre-check: "));
  Serial.println(pdBefore == HIGH ? F("HIGH (awake)") : F("LOW (sleep)"));

  if (pdBefore == LOW) {
    logDebugPrefix(scope);
    Serial.println(F("Forcing PD HIGH to wake radio modules before PTT."));
    PowerSavingOff;
    delay(PD_WAKE_SETTLE_MS);
  }

  int pdAfter = digitalRead(RfPwrSleepPin);
  logDebugPrefix(scope);
  Serial.print(F("PD ready state: "));
  Serial.println(pdAfter == HIGH ? F("HIGH (PTT allowed)") : F("LOW (PTT blocked)"));

  if (pdAfter != HIGH) {
    Serial.println(F("PTT blocked: radio modules still in power-save (PD LOW)."));
    return false;
  }

  return true;
}

void printConfigBanner() {
  char vhfTx[16];
  char vhfRx[16];
  char uhfTx[16];
  char uhfRx[16];
  formatFreq(vhfTx, sizeof(vhfTx), VHF_DEFAULTS.freq_tx);
  formatFreq(vhfRx, sizeof(vhfRx), VHF_DEFAULTS.freq_rx);
  formatFreq(uhfTx, sizeof(uhfTx), UHF_DEFAULTS.freq_tx);
  formatFreq(uhfRx, sizeof(uhfRx), UHF_DEFAULTS.freq_rx);

  Serial.println(F(""));
  Serial.println(F("================ HAMWING CONFIG BANNER ================"));
  Serial.println(F("Build: Feather_M0_Testing"));
#if ACTIVE_MODULE == MODULE_BOTH
  Serial.println(F("Active module selection: BOTH"));
#elif ACTIVE_MODULE == MODULE_UHF
  Serial.println(F("Active module selection: UHF only"));
#else
  Serial.println(F("Active module selection: VHF only"));
#endif
  Serial.print(F("PTT polarity at MCU pin: "));
  Serial.println(PTT_ACTIVE_LOW ? F("ACTIVE-LOW") : F("ACTIVE-HIGH (inverted stage)"));
  Serial.print(F("NO_TX_MODE: "));
  Serial.println(NO_TX_MODE ? F("ON") : F("OFF"));
  Serial.print(F("PTT_TEST_ENABLED: "));
  Serial.println(PTT_TEST_ENABLED ? F("ON") : F("OFF"));
  Serial.print(F("PTT_POLARITY_PROBE: "));
  Serial.println(PTT_POLARITY_PROBE ? F("ON") : F("OFF"));
  Serial.print(F("PTT_GPIO_SWEEP_MODE: "));
  Serial.println(PTT_GPIO_SWEEP_MODE ? F("ON") : F("OFF"));
  Serial.print(F("HANDOFF_GPIO_TO_PI_AFTER_CONFIG: "));
  Serial.println(HANDOFF_GPIO_TO_PI_AFTER_CONFIG ? F("ON") : F("OFF"));
  Serial.print(F("HANDOFF_REQUIRE_CONFIG_OK: "));
  Serial.println(HANDOFF_REQUIRE_CONFIG_OK ? F("ON") : F("OFF"));
  Serial.print(F("PTT test duration (ms): "));
  Serial.println(PTT_TEST_DURATION_MS);
  Serial.print(F("SCAN_TEST_MODE: "));
  Serial.println(SCAN_TEST_MODE ? F("ON") : F("OFF"));
  Serial.print(F("SCAN range/step/dwell: +/-"));
  Serial.print(SCAN_RANGE_MHZ, 4);
  Serial.print(F(" MHz, "));
  Serial.print(SCAN_STEP_MHZ, 4);
  Serial.print(F(" MHz, "));
  Serial.print(SCAN_DWELL_MS);
  Serial.println(F(" ms"));
  Serial.print(F("PD wake settle (ms): "));
  Serial.println(PD_WAKE_SETTLE_MS);

  Serial.print(F("VHF TX/RX MHz: "));
  Serial.print(vhfTx);
  Serial.print(F(" / "));
  Serial.println(vhfRx);
  Serial.print(F("UHF TX/RX MHz: "));
  Serial.print(uhfTx);
  Serial.print(F(" / "));
  Serial.println(uhfRx);

  Serial.print(F("Pins: PD="));
  Serial.print(RfPwrSleepPin);
  Serial.print(F(" HL="));
  Serial.print(RfPwrHLPin);
  Serial.print(F(" VPTT="));
  Serial.print(V_PTTPin);
  Serial.print(F(" UPTT="));
  Serial.print(U_PTTPin);
  Serial.print(F(" VSQL="));
  Serial.print(V_SQLPin);
  Serial.print(F(" USQL="));
  Serial.println(U_SQLPin);
  Serial.println(F("========================================================"));
}

void printStatusBanner(float measuredvbat, const __FlashStringHelper *phase) {
  Serial.println(F(""));
  Serial.println(F("---------------- STATUS BANNER ----------------"));
  Serial.print(F("Phase: "));
  Serial.println(phase);
  Serial.print(F("Uptime ms: "));
  Serial.println(millis());
  Serial.print(F("VBAT: "));
  Serial.println(measuredvbat);
  Serial.print(F("Config state VHF/UHF: "));
  Serial.print(gVhfConfigured ? F("OK") : F("FAIL"));
  Serial.print(F("/"));
  Serial.println(gUhfConfigured ? F("OK") : F("FAIL"));
  Serial.print(F("PD/HL/VPTT/UPTT/VSQL/USQL: "));
  Serial.print(digitalRead(RfPwrSleepPin));
  Serial.print(F("/"));
  Serial.print(digitalRead(RfPwrHLPin));
  Serial.print(F("/"));
  Serial.print(digitalRead(V_PTTPin));
  Serial.print(F("/"));
  Serial.print(digitalRead(U_PTTPin));
  Serial.print(F("/"));
  Serial.print(digitalRead(V_SQLPin));
  Serial.print(F("/"));
  Serial.println(digitalRead(U_SQLPin));
  Serial.print(F("SQL baseline V/U: "));
  Serial.print(gVhfSqlBaseline);
  Serial.print(F("/"));
  Serial.println(gUhfSqlBaseline);
  Serial.println(F("-----------------------------------------------"));
}

void calibrateSqlBaselines() {
  long vSum = 0;
  long uSum = 0;
  const int samples = 25;

  for (int i = 0; i < samples; i++) {
    vSum += digitalRead(V_SQLPin);
    uSum += digitalRead(U_SQLPin);
    delay(4);
  }

  gVhfSqlBaseline = (vSum >= (samples / 2)) ? HIGH : LOW;
  gUhfSqlBaseline = (uSum >= (samples / 2)) ? HIGH : LOW;

  Serial.print(F("[SCAN] SQL baseline calibrated VHF/UHF: "));
  Serial.print(gVhfSqlBaseline);
  Serial.print(F("/"));
  Serial.println(gUhfSqlBaseline);
}

bool tuneRadioForScan(const __FlashStringHelper *label, DRA818 &radio, const RadioDefaults &baseCfg, float freqMHz) {
  RadioDefaults scanCfg = baseCfg;
  scanCfg.freq_tx = freqMHz;
  scanCfg.freq_rx = freqMHz;

  bool ok = (radio.group(scanCfg.bandwidth, scanCfg.freq_tx, scanCfg.freq_rx, scanCfg.ctcss_tx, scanCfg.squelch, scanCfg.ctcss_rx) == 1);
  if (!ok) {
    char groupCmd[80];
    makeGroupCmd(groupCmd, sizeof(groupCmd), scanCfg);
    ok = sendATExpect(radio.serial, label, groupCmd, "DMOSETGROUP:0", AT_TIMEOUT_MS);
  }

  return ok;
}

int scanBandRange(const __FlashStringHelper *label, DRA818 &radio, const RadioDefaults &baseCfg, int sqlPin, int sqlBaseline) {
  float startFreq = baseCfg.freq_rx - SCAN_RANGE_MHZ;
  float endFreq = baseCfg.freq_rx + SCAN_RANGE_MHZ;
  int hits = 0;

  Serial.print(F("[SCAN] "));
  Serial.print(label);
  Serial.print(F(" range: "));
  Serial.print(startFreq, 4);
  Serial.print(F(" to "));
  Serial.print(endFreq, 4);
  Serial.print(F(" MHz, step "));
  Serial.print(SCAN_STEP_MHZ, 4);
  Serial.println(F(" MHz"));

  for (float f = startFreq; f <= (endFreq + 0.0001f); f += SCAN_STEP_MHZ) {
    if (!tuneRadioForScan(label, radio, baseCfg, f)) {
      Serial.print(F("[SCAN] tune failed @ "));
      Serial.print(f, 4);
      Serial.print(F(" MHz on "));
      Serial.println(label);
      continue;
    }

    delay(SCAN_DWELL_MS);
    int sqlRaw = digitalRead(sqlPin);
    bool hit = (sqlRaw != sqlBaseline);

    if (hit) {
      hits++;
      Serial.print(F("[SCAN HIT] "));
      Serial.print(label);
      Serial.print(F(" @ "));
      Serial.print(f, 4);
      Serial.print(F(" MHz | SQL raw="));
      Serial.print(sqlRaw);
      Serial.print(F(" baseline="));
      Serial.println(sqlBaseline);
    }
  }

  Serial.print(F("[SCAN] "));
  Serial.print(label);
  Serial.print(F(" pass complete, hits="));
  Serial.println(hits);
  return hits;
}

void runScanTest() {
  if (!ensurePdReadyForPtt(F("SCAN"))) {
    Serial.println(F("[SCAN] aborted: PD not ready."));
    return;
  }

  setPttIdle();
  calibrateSqlBaselines();

  Serial.println(F(""));
  Serial.println(F("================ RX SCAN TEST START ================"));
  int vhfHits = 0;
  int uhfHits = 0;

  if (gVhfConfigured) {
    vhfHits = scanBandRange(F("VHF"), draV, VHF_DEFAULTS, V_SQLPin, gVhfSqlBaseline);
  } else {
    Serial.println(F("[SCAN] skipping VHF: module not configured."));
  }

  if (gUhfConfigured) {
#if TRY_UHF_BOTH_UART_MAPPINGS
    DRA818 &uhfRadio = activeUhfPreferredMapping ? draUPreferred : draUFallback;
    uhfHits = scanBandRange(F("UHF"), uhfRadio, UHF_DEFAULTS, U_SQLPin, gUhfSqlBaseline);
#else
    uhfHits = scanBandRange(F("UHF"), draUPreferred, UHF_DEFAULTS, U_SQLPin, gUhfSqlBaseline);
#endif
  } else {
    Serial.println(F("[SCAN] skipping UHF: module not configured."));
  }

  Serial.print(F("[SCAN SUMMARY] hits VHF/UHF = "));
  Serial.print(vhfHits);
  Serial.print(F("/"));
  Serial.println(uhfHits);
  Serial.println(F("================= RX SCAN TEST END ================="));
  delay(SCAN_PASS_PAUSE_MS);
}

Stream *getActiveUhfPort() {
#if TRY_UHF_BOTH_UART_MAPPINGS
  return activeUhfPreferredMapping ? draUPreferred.serial : draUFallback.serial;
#else
  return draUPreferred.serial;
#endif
}

// Heartbeat LED pattern: slow PWM fade-in then fade-out on the built-in LED.
// LED_BUILTIN (pin 13) supports analogWrite() on the Feather M0 via SAMD21 PWM.
// Call this after both radios are confirmed configured.
void heartbeatLED() {
  const int steps = 64;  // Number of brightness steps
  const int stepMs = 8;  // Milliseconds per step (total ~512 ms for one fade cycle)
  // Fade in
  for (int i = 0; i <= steps; i++) {
    analogWrite(LEDPin, (i * 255) / steps);
    delay(stepMs);
  }
  // Fade out
  for (int i = steps; i >= 0; i--) {
    analogWrite(LEDPin, (i * 255) / steps);
    delay(stepMs);
  }
  analogWrite(LEDPin, 0);  // Ensure fully off
}

// PTT test: key each radio module for PTT_TEST_DURATION_MS then unkey.
// Gated by PTT_TEST_ENABLED and NO_TX_MODE.
void runPttTest() {
#if PTT_TEST_ENABLED && !NO_TX_MODE
  if (!ensurePdReadyForPtt(F("PTT"))) {
    setPttIdle();
    return;
  }

  logDebugMessage(F("PTT"), F("Starting PTT test cycle."));
  setPttIdle();
  logPinStateSnapshot(F("PTT"), F("Before VHF key"));
  Serial.println(F("PTT test: Keying VHF transmitter..."));
  logDebugMessage(F("PTT"), F("VHF PTT asserted (TX ON)."));
  LedON;
  RfON_VHF;
  logPinStateSnapshot(F("PTT"), F("During VHF key"));
  delay(PTT_TEST_DURATION_MS);
  RfOFF_VHF;
  LedOff;
  Serial.println(F("PTT test: VHF unkeyed."));
  logDebugMessage(F("PTT"), F("VHF PTT released (TX OFF)."));
  logPinStateSnapshot(F("PTT"), F("After VHF unkey"));

  delay(500);

  logPinStateSnapshot(F("PTT"), F("Before UHF key"));
  Serial.println(F("PTT test: Keying UHF transmitter..."));
  logDebugMessage(F("PTT"), F("UHF PTT asserted (TX ON)."));
  LedON;
  RfON_UHF;
  logPinStateSnapshot(F("PTT"), F("During UHF key"));
  delay(PTT_TEST_DURATION_MS);
  RfOFF_UHF;
  LedOff;
  Serial.println(F("PTT test: UHF unkeyed."));
  logDebugMessage(F("PTT"), F("UHF PTT released (TX OFF)."));
  logPinStateSnapshot(F("PTT"), F("After UHF unkey"));
  setPttIdle();
  logDebugMessage(F("PTT"), F("Completed PTT test cycle."));
#else
  Serial.println(F("PTT test skipped (NO_TX_MODE or PTT_TEST_ENABLED=0)."));
  logDebugMessage(F("PTT"), F("PTT test skipped due to configuration."));
#endif
}

void runVhfPttPolarityProbe() {
#if !NO_TX_MODE
  // DIAGNOSTIC: Verify PD pin is HIGH (radios active) before probe starts.
  Serial.println(F("\n=== PD PIN STATE CHECK ==="));
  int pdPinReadBefore = digitalRead(RfPwrSleepPin);
  Serial.print(F("PD pin before probe: "));
  Serial.println(pdPinReadBefore);
  if (pdPinReadBefore == LOW) {
    Serial.println(F("WARNING: PD pin is LOW (radios in sleep mode). Forcing PD HIGH to wake radios..."));
    PowerSavingOff;
    delay(500);
    int pdPinReadAfter = digitalRead(RfPwrSleepPin);
    Serial.print(F("PD pin after wake attempt: "));
    Serial.println(pdPinReadAfter);
  } else {
    Serial.println(F("OK: PD pin is HIGH (radios active and ready)."));
  }
  Serial.println(F("=== END PD CHECK ===\n"));

  setPttIdle();
  RfOFF_UHF;

  Serial.println(F("PTT probe (VHF): forcing MCU VPTT LOW for 3 seconds..."));
  logDebugMessage(F("PTT/Probe"), F("Phase A: VPTT=LOW"));
  LedON;
  digitalWrite(V_PTTPin, LOW);
  logPinStateSnapshot(F("PTT/Probe"), F("Phase A pin state"));
  delay(3000);

  digitalWrite(V_PTTPin, HIGH);
  LedOff;
  logPinStateSnapshot(F("PTT/Probe"), F("After Phase A"));
  delay(1000);

  Serial.println(F("PTT probe (VHF): forcing MCU VPTT HIGH for 3 seconds..."));
  logDebugMessage(F("PTT/Probe"), F("Phase B: VPTT=HIGH"));
  LedON;
  digitalWrite(V_PTTPin, HIGH);
  logPinStateSnapshot(F("PTT/Probe"), F("Phase B pin state"));
  delay(3000);

  setPttIdle();
  LedOff;
  delay(1000);

  Serial.println(F("PTT probe: forcing BOTH PTT outputs LOW for 3 seconds..."));
  logDebugMessage(F("PTT/Probe"), F("Phase C: VPTT=LOW and UPTT=LOW"));
  LedON;
  digitalWrite(V_PTTPin, LOW);
  digitalWrite(U_PTTPin, LOW);
  logPinStateSnapshot(F("PTT/Probe"), F("Phase C pin state"));
  delay(3000);

  setPttIdle();
  LedOff;
  delay(1000);

  // PHASE D: Keep VPTT LOW and send AT+DMOCONNECT to detect if PTT input affects module responsiveness.
  Serial.println(F("\nPTT probe (Phase D): Holding VPTT LOW while sending AT command..."));
  logDebugMessage(F("PTT/Probe"), F("Phase D: VPTT=LOW + AT command during TX key"));
  LedON;
  digitalWrite(V_PTTPin, LOW);
  logPinStateSnapshot(F("PTT/Probe"), F("Phase D pin state (PTT LOW)"));

  // Send AT+DMOCONNECT while PTT is held LOW.
  Serial.println(F("Sending AT+DMOCONNECT while VPTT=LOW..."));
  draV.serial->println(F("AT+DMOCONNECT"));
  unsigned long atWaitStart = millis();
  bool atResponsedWhilePttLow = false;
  while (millis() - atWaitStart < 1000) {
    if (draV.serial->available()) {
      String response = draV.serial->readStringUntil('\n');
      Serial.print(F("  [while VPTT=LOW] AT response: "));
      Serial.println(response);
      atResponsedWhilePttLow = true;
    }
  }
  if (!atResponsedWhilePttLow) {
    Serial.println(F("  WARNING: No AT response while VPTT=LOW. PTT may be blocking module communication."));
  }

  delay(500);
  setPttIdle();
  LedOff;
  logPinStateSnapshot(F("PTT/Probe"), F("Probe complete; returned to idle"));
#else
  Serial.println(F("PTT probe skipped (NO_TX_MODE=1)."));
#endif
}

void driveRawPttPins(bool vhfLevel, bool uhfLevel, const __FlashStringHelper *label) {
#if !NO_TX_MODE
  digitalWrite(V_PTTPin, vhfLevel ? HIGH : LOW);
  digitalWrite(U_PTTPin, uhfLevel ? HIGH : LOW);

  Serial.print(F("Raw PTT phase: "));
  Serial.print(label);
  Serial.print(F(" | VPTT="));
  Serial.print(vhfLevel ? F("HIGH") : F("LOW"));
  Serial.print(F(" UPTT="));
  Serial.print(uhfLevel ? F("HIGH") : F("LOW"));
  Serial.print(F(" | PD="));
  Serial.println(digitalRead(RfPwrSleepPin));

  logPinStateSnapshot(F("PTT/GPIO"), F("Raw phase snapshot"));
  delay(PTT_GPIO_SWEEP_STEP_MS);
#else
  (void)vhfLevel;
  (void)uhfLevel;
  (void)label;
  Serial.println(F("Raw PTT sweep skipped (NO_TX_MODE=1)."));
#endif
}

void runRawPttSweep() {
#if !NO_TX_MODE
  Serial.println(F("\n=== RAW PTT GPIO SWEEP START ==="));
  Serial.println(F("Use HT/spectrum monitor to note which phase produces carrier on VHF/UHF."));
  if (!ensurePdReadyForPtt(F("PTT/GPIO"))) {
    setPttIdle();
    Serial.println(F("=== RAW PTT GPIO SWEEP ABORTED (PD not ready) ===\n"));
    return;
  }

  driveRawPttPins(false, false, F("Phase 1 (both LOW)"));
  driveRawPttPins(true, false, F("Phase 2 (VHF HIGH, UHF LOW)"));
  driveRawPttPins(false, true, F("Phase 3 (VHF LOW, UHF HIGH)"));
  driveRawPttPins(true, true, F("Phase 4 (both HIGH)"));

  setPttIdle();
  Serial.println(F("=== RAW PTT GPIO SWEEP END ===\n"));
#else
  Serial.println(F("Raw PTT sweep skipped (NO_TX_MODE=1)."));
#endif
}

void enforceNoTx() {
#if NO_TX_MODE
  // Keep both radios unkeyed regardless of configured PTT polarity.
  RfOFF_VHF;
  RfOFF_UHF;
#endif
}

void setPttIdle() {
  // DRA818 PTT is active-low: HIGH keeps transmitter off.
  RfOFF_VHF;
  RfOFF_UHF;
}

void ensureSerial2Mux() {
  // No pin mux operation is required on ATmega328P SoftwareSerial.
}

void activateUhfUart(bool usePreferredMapping) {
#if TRY_UHF_BOTH_UART_MAPPINGS
  activeUhfPreferredMapping = usePreferredMapping;
  activeUhfSerial = usePreferredMapping ? &Serial2Preferred : &Serial2Fallback;
#else
  (void)usePreferredMapping;
  activeUhfPreferredMapping = true;
  activeUhfSerial = &Serial2Preferred;
#endif
  activeUhfSerial->begin(9600);
  activeUhfSerial->listen();
  ensureSerial2Mux();
  logDebugMessage(F("UHF UART"), usePreferredMapping ? F("Using preferred KiCad-derived RX/TX mapping.") : F("Using fallback RX/TX mapping for diagnosis."));
}

void printUhfMappingSelection() {
  logDebugMessage(
    F("UHF UART"),
    activeUhfPreferredMapping ? F("Selected preferred mapping: Feather D11 receives from UHF, Feather D10 transmits to UHF.")
                              : F("Selected fallback mapping: Feather D10 receives from UHF, Feather D11 transmits to UHF."));
}

void printEscapedResp(const char *resp) {
  if (resp == NULL || resp[0] == '\0') {
    Serial.print(F("<empty>"));
    return;
  }

  for (size_t i = 0; resp[i] != '\0'; ++i) {
    char c = resp[i];
    if (c == '\r') {
      Serial.print(F("\\r"));
    } else if (c == '\n') {
      Serial.print(F("\\n"));
    } else if (c == '\t') {
      Serial.print(F("\\t"));
    } else {
      Serial.print(c);
    }
  }
}

bool sendATExpect(Stream *port, const __FlashStringHelper *label, const char *cmd, const char *expect, unsigned long timeoutMs) {
  while (port->available()) {
    port->read();
  }

  port->print(cmd);
  port->print("\r\n");

  char resp[128];
  size_t used = 0;
  resp[0] = '\0';

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (port->available()) {
      char c = (char)port->read();
      if (used < sizeof(resp) - 1) {
        resp[used++] = c;
        resp[used] = '\0';
      } else {
        memmove(resp, resp + 1, sizeof(resp) - 2);
        resp[sizeof(resp) - 2] = c;
        resp[sizeof(resp) - 1] = '\0';
      }

      if (strstr(resp, expect) != NULL) {
        logDebugText(label, F("AT command acknowledged: "), cmd);
        logDebugPrefix(label);
        Serial.print(F("Raw radio response: "));
        printEscapedResp(resp);
        Serial.println();
        delay(AT_CMD_GAP_MS);
        return true;
      }
    }
  }

  logDebugText(label, F("AT command timed out: "), cmd);
  logDebugPrefix(label);
  Serial.print(F("Last bytes received from the radio: "));
  printEscapedResp(resp);
  Serial.println();
  delay(AT_CMD_GAP_MS);
  return false;
}

void formatFreq(char *out, size_t outSize, float freq) {
  long scaled = (long)(freq * 10000.0f + (freq >= 0 ? 0.5f : -0.5f));
  long whole = scaled / 10000;
  unsigned long frac = (unsigned long)labs(scaled % 10000);
  snprintf(out, outSize, "%ld.%04lu", whole, frac);
}

bool isFreqValidForType(float freq, uint8_t radioType) {
  if (radioType == DRA818_UHF) {
    return (freq >= DRA818_UHF_MIN && freq <= DRA818_UHF_MAX);
  }
  return (freq >= DRA818_VHF_MIN && freq <= DRA818_VHF_MAX);
}

void logGroupConfig(const __FlashStringHelper *label, const RadioDefaults &cfg, uint8_t radioType) {
  char txStr[16];
  char rxStr[16];
  char groupCmd[80];
  formatFreq(txStr, sizeof(txStr), cfg.freq_tx);
  formatFreq(rxStr, sizeof(rxStr), cfg.freq_rx);
  makeGroupCmd(groupCmd, sizeof(groupCmd), cfg);

  logDebugMessage(label, radioType == DRA818_UHF ? F("Preparing UHF group configuration.") : F("Preparing VHF group configuration."));
  logDebugPrefix(label);
  Serial.print(F("TX frequency check: "));
  Serial.print(txStr);
  Serial.print(F(" MHz -> "));
  Serial.println(isFreqValidForType(cfg.freq_tx, radioType) ? F("inside valid range") : F("outside valid range"));

  logDebugPrefix(label);
  Serial.print(F("RX frequency check: "));
  Serial.print(rxStr);
  Serial.print(F(" MHz -> "));
  Serial.println(isFreqValidForType(cfg.freq_rx, radioType) ? F("inside valid range") : F("outside valid range"));

  logDebugText(label, F("Exact group command: "), groupCmd);
}

void makeGroupCmd(char *out, size_t outSize, const RadioDefaults &cfg) {
  char txStr[16];
  char rxStr[16];

  // Avoid %f formatting on embedded toolchains where float printf may be disabled.
  formatFreq(txStr, sizeof(txStr), cfg.freq_tx);
  formatFreq(rxStr, sizeof(rxStr), cfg.freq_rx);

  snprintf(
    out,
    outSize,
    "AT+DMOSETGROUP=%u,%s,%s,%04u,%u,%04u",
    cfg.bandwidth,
    txStr,
    rxStr,
    cfg.ctcss_tx,
    cfg.squelch,
    cfg.ctcss_rx);
}

bool applyDefaults(const __FlashStringHelper *label, DRA818 &radio, const RadioDefaults &cfg) {
  Serial.print(F("Programming radio on "));
  Serial.println(label);

  uint8_t radioType = (cfg.freq_tx >= DRA818_UHF_MIN || cfg.freq_rx >= DRA818_UHF_MIN) ? DRA818_UHF : DRA818_VHF;
  logGroupConfig(label, cfg, radioType);

  Stream *port = radio.serial;

  bool ok = (radio.handshake() == 1);
  if (!ok) {
    ok = sendATExpect(port, label, "AT+DMOCONNECT", "DMOCONNECT:0", AT_TIMEOUT_MS);
  }
  logStepResult(label, F("Radio handshake"), ok);
  if (!ok) return false;

  ok = (radio.group(cfg.bandwidth, cfg.freq_tx, cfg.freq_rx, cfg.ctcss_tx, cfg.squelch, cfg.ctcss_rx) == 1);
  if (!ok) {
    char groupCmd[80];
    makeGroupCmd(groupCmd, sizeof(groupCmd), cfg);
    ok = sendATExpect(port, label, groupCmd, "DMOSETGROUP:0", AT_TIMEOUT_MS);
  }
  logStepResult(label, F("Frequency and squelch configuration"), ok);
  if (!ok) return false;

  ok = (radio.volume(cfg.volume) == 1);
  if (!ok) {
    char volCmd[32];
    snprintf(volCmd, sizeof(volCmd), "AT+DMOSETVOLUME=%u", cfg.volume);
    ok = sendATExpect(port, label, volCmd, "DMOSETVOLUME:0", AT_TIMEOUT_MS);
  }
  if (ok) {
    logStepResult(label, F("Audio volume configuration"), true);
  } else {
    Serial.print(label);
    Serial.println(F(": Audio volume command was not acknowledged; continuing."));
  }

  ok = (radio.filters(cfg.filter_pre, cfg.filter_high, cfg.filter_low) == 1);
  if (!ok) {
    char filterCmd[40];
    // Command is AT+SETFILTER, response token is DMOSETFILTER.
    snprintf(filterCmd, sizeof(filterCmd), "AT+SETFILTER=%u,%u,%u", cfg.filter_pre ? 1 : 0, cfg.filter_high ? 1 : 0, cfg.filter_low ? 1 : 0);
    ok = sendATExpect(port, label, filterCmd, "DMOSETFILTER:0", AT_TIMEOUT_MS);
  }
  if (ok) {
    logStepResult(label, F("Audio filter configuration"), true);
  } else {
    Serial.print(label);
    Serial.println(F(": Audio filter command was not acknowledged; continuing."));
  }
  return true;
}

int probeBand(const __FlashStringHelper *label, DRA818 &radio) {
  logDebugMessage(label, F("Probing the radio to determine which band accepts the current command set."));

  // Build both band-typed views on the same UART so range clamping does not
  // invalidate probing when the physical module band differs from object type.
  DRA818 probeVHF(radio.serial, DRA818_VHF);
  DRA818 probeUHF(radio.serial, DRA818_UHF);

  Stream *port = radio.serial;
  bool connected = (probeVHF.handshake() == 1);
  if (!connected) {
    connected = sendATExpect(port, label, "AT+DMOCONNECT", "DMOCONNECT:0", AT_TIMEOUT_MS);
  }
  if (!connected) {
    logDebugMessage(label, F("Band probe failed because the radio never acknowledged the handshake."));
    return BAND_NONE;
  }

  char vhfCmd[80];
  logGroupConfig(label, VHF_DEFAULTS, DRA818_VHF);
  makeGroupCmd(vhfCmd, sizeof(vhfCmd), VHF_DEFAULTS);
  if ((probeVHF.group(VHF_DEFAULTS.bandwidth, VHF_DEFAULTS.freq_tx, VHF_DEFAULTS.freq_rx, VHF_DEFAULTS.ctcss_tx, VHF_DEFAULTS.squelch, VHF_DEFAULTS.ctcss_rx) == 1) || sendATExpect(port, label, vhfCmd, "DMOSETGROUP:0", AT_TIMEOUT_MS)) {
    logDebugMessage(label, F("The radio accepted a VHF group command."));
    return BAND_VHF;
  }

  char uhfCmd[80];
  logGroupConfig(label, UHF_DEFAULTS, DRA818_UHF);
  makeGroupCmd(uhfCmd, sizeof(uhfCmd), UHF_DEFAULTS);
  if ((probeUHF.group(UHF_DEFAULTS.bandwidth, UHF_DEFAULTS.freq_tx, UHF_DEFAULTS.freq_rx, UHF_DEFAULTS.ctcss_tx, UHF_DEFAULTS.squelch, UHF_DEFAULTS.ctcss_rx) == 1) || sendATExpect(port, label, uhfCmd, "DMOSETGROUP:0", AT_TIMEOUT_MS)) {
    logDebugMessage(label, F("The radio accepted a UHF group command."));
    return BAND_UHF;
  }

  logDebugMessage(label, F("The radio rejected both VHF and UHF group commands during probing."));
  return BAND_NONE;
}

bool applyDefaultsAuto(const __FlashStringHelper *label, DRA818 &radio) {
  DRA818 radioVHF(radio.serial, DRA818_VHF);
  DRA818 radioUHF(radio.serial, DRA818_UHF);

  int band = probeBand(label, radio);
  if (band == BAND_VHF) {
    return applyDefaults(F("AUTO->VHF"), radioVHF, VHF_DEFAULTS);
  }
  if (band == BAND_UHF) {
    return applyDefaults(F("AUTO->UHF"), radioUHF, UHF_DEFAULTS);
  }
  return false;
}

void recoverUhfPortAfterFailure() {
  Serial.println(F("UHF recover: cycling PD and reinitializing UART path."));
  setPttIdle();
  PowerSavingOn;
  delay(150);
  PowerSavingOff;
  delay(PD_WAKE_SETTLE_MS);
  activeUhfSerial->begin(9600);
  ensureSerial2Mux();
}

bool configureUhfOnCurrentMapping(const __FlashStringHelper *label, DRA818 &radio) {
  if (applyDefaults(label, radio, UHF_DEFAULTS)) {
    return true;
  }

  recoverUhfPortAfterFailure();
  Serial.println(F("UHF retry: attempting automatic band probe/config on current mapping..."));
  return applyDefaultsAuto(label, radio);
}

bool applyActiveDefaults() {
#if ACTIVE_MODULE == MODULE_UHF
  activateUhfUart(true);
  gVhfConfigured = false;
  gUhfConfigured = false;

  if (configureUhfOnCurrentMapping(F("SERIAL2/UHF PORT [PREFERRED MAP]"), draUPreferred)) {
    gUhfConfigured = true;
    printUhfMappingSelection();
    return true;
  }
#if TRY_UHF_BOTH_UART_MAPPINGS
  Serial.println(F("Retrying UHF with fallback D10/D11 UART mapping..."));
  activateUhfUart(false);
  bool ok = configureUhfOnCurrentMapping(F("SERIAL2/UHF PORT [FALLBACK MAP]"), draUFallback);
  gUhfConfigured = ok;
  if (ok) {
    printUhfMappingSelection();
  }
  return ok;
#else
  return false;
#endif
#elif ACTIVE_MODULE == MODULE_BOTH
  gVhfConfigured = false;
  gUhfConfigured = false;
  bool vhfOk = applyDefaults(F("SERIAL/VHF PORT"), draV, VHF_DEFAULTS);
  gVhfConfigured = vhfOk;
  delay(100);

  activateUhfUart(true);
  bool uhfOk = configureUhfOnCurrentMapping(F("SERIAL2/UHF PORT [PREFERRED MAP]"), draUPreferred);
  gUhfConfigured = uhfOk;
  if (uhfOk) {
    printUhfMappingSelection();
  }
#if TRY_UHF_BOTH_UART_MAPPINGS
  if (!uhfOk) {
    Serial.println(F("Retrying UHF with fallback D10/D11 UART mapping..."));
    activateUhfUart(false);
    uhfOk = configureUhfOnCurrentMapping(F("SERIAL2/UHF PORT [FALLBACK MAP]"), draUFallback);
    gUhfConfigured = uhfOk;
    if (uhfOk) {
      printUhfMappingSelection();
    }
  }
#endif
  if (!uhfOk) {
    Serial.println(F("UHF wiring check: verify D11 RX from module TX and D10 TX to module RX; then check UHF PTT/PD paths."));
  }
  return (vhfOk && uhfOk);
#else
  gUhfConfigured = false;
  gVhfConfigured = applyDefaults(F("SERIAL/VHF PORT"), draV, VHF_DEFAULTS);
  return gVhfConfigured;
#endif
}

////////////////////////////
// Initial Setup
////////////////////////////
void setup() {

  Serial.begin(115200);
  Serial.println(F("HamWing dual-radio diagnostic sketch starting."));

  logDebugMessage(F("Startup"), F("Opening the VHF and UHF UART interfaces."));
  activateUhfUart(true);
  // VHF uses hardware UART (Serial on D0/D1) and UHF uses SoftwareSerial on D11/D10.

  logDebugMessage(F("Startup"), F("ATmega328P mapping: VHF on D0/D1 hardware UART; UHF on D11/D10 SoftwareSerial. Shared PD control is on Feather D12."));
#if TRY_UHF_BOTH_UART_MAPPINGS
  logDebugMessage(F("Startup"), F("Fallback UHF RX/TX mapping is enabled for troubleshooting only."));
#endif

  logDebugMessage(F("Startup"), F("Configuring GPIO pins for radio control, squelch, and push-to-talk."));
  pinMode(RfPwrSleepPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(V_PTTPin, OUTPUT);
  pinMode(U_PTTPin, OUTPUT);
  pinMode(V_SQLPin, INPUT);
  pinMode(U_SQLPin, INPUT);
  setPttIdle();
  printConfigBanner();
  logDebugMessage(F("Startup"), PTT_ACTIVE_LOW ? F("PTT polarity configured: active-LOW (LOW keys TX).") : F("PTT polarity configured: active-HIGH (HIGH keys TX)."));
  logDebugMessage(F("Startup"), PTT_GPIO_SWEEP_MODE ? F("Raw PTT GPIO sweep mode is ENABLED.") : F("Raw PTT GPIO sweep mode is disabled."));
  logPinStateSnapshot(F("Startup"), F("Post GPIO init"));
  diagnoseReleasedPttLines();

  logDebugMessage(F("Startup"), F("Placing radios into a safe startup state before configuration."));
  PowerSavingOn;
#if RF_TEST_USE_HIGH_POWER
  RfPwrHigh;
#else
  RfPwrLow;
#endif
  enforceNoTx();
  Serial.println(F("Startup safety state applied: both radios held non-transmitting."));

  PowerSavingOff;
  delay(1200);
  enforceNoTx();

  bool activeOk = applyActiveDefaults();
#if ACTIVE_MODULE == MODULE_BOTH
  Serial.print(F("Initial dual-radio configuration: "));
#else
  Serial.print(F("Initial radio configuration: "));
#endif
  Serial.println(activeOk ? F("OK") : F("FAILED"));
  if (activeOk) { heartbeatLED(); }

#if HANDOFF_GPIO_TO_PI_AFTER_CONFIG
#if HANDOFF_REQUIRE_CONFIG_OK
  if (activeOk) {
    handoffControlLinesToPi();
    gPiHandoffComplete = true;
  } else {
    Serial.println(F("HANDOFF: Skipped because startup configuration failed."));
  }
#else
  handoffControlLinesToPi();
  gPiHandoffComplete = true;
#endif
#endif

  logDebugMessage(F("Startup"), F("Setup complete; pausing 5 seconds before loop begins."));
  delay(5000);
}

// the loop function runs over and over again forever
void loop() {
#if HANDOFF_GPIO_TO_PI_AFTER_CONFIG
  if (gPiHandoffComplete) {
    logDebugMessage(F("Loop"), F("Pi handoff mode active; skipping Feather TX/scan control in loop."));
    delay(1000);
    return;
  }
#endif

  // CRITICAL: Ensure PD pin stays HIGH (radios active) throughout execution.
  PowerSavingOff;  // D12 -> HIGH to keep radios awake.

  enforceNoTx();
  logDebugMessage(F("Loop"), F("Loop iteration started."));

  float measuredvbat = analogRead(VBatPin);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  Serial.print(F("Battery voltage: "));
  Serial.println(measuredvbat);
  printStatusBanner(measuredvbat, F("Pre-Test"));
  diagnoseReleasedPttLines();
  diagnoseReleasedUhfUartLines();

  // --- UHF configuration retry (runs every loop while UHF is not yet configured) ---
  if (!gUhfConfigured) {
    Serial.println(F("[UHF-RETRY] UHF not yet configured. Re-attempting UHF UART init..."));

    // Log the raw pin/UART state before touching anything.
    Serial.print(F("[UHF-RETRY] PD pin before retry: "));
    Serial.println(digitalRead(RfPwrSleepPin));
    Serial.print(F("[UHF-RETRY] Active UHF serial RX bytes available before init: "));
    Serial.println(activeUhfSerial->available());

    // Force PD high and allow full settle time before any AT traffic.
    PowerSavingOff;
    delay(PD_WAKE_SETTLE_MS);
    Serial.print(F("[UHF-RETRY] PD pin after settle: "));
    Serial.println(digitalRead(RfPwrSleepPin));

    // Preferred-map handshake attempt.
    activateUhfUart(true);
    Serial.println(F("[UHF-RETRY] UART re-initialized (preferred map)."));
    Serial.print(F("[UHF-RETRY] RX bytes after preferred init: "));
    Serial.println(activeUhfSerial->available());
    Serial.println(F("[UHF-RETRY] Sending raw AT+DMOCONNECT handshake (preferred map)..."));
    activeUhfSerial->print(F("AT+DMOCONNECT\r\n"));
    delay(AT_TIMEOUT_MS);
    String preferredResp = "";
    while (activeUhfSerial->available()) {
      preferredResp += (char)activeUhfSerial->read();
    }
    Serial.print(F("[UHF-RETRY] Preferred raw handshake response: ["));
    Serial.print(preferredResp);
    Serial.println(F("]"));

    bool preferredOk = (preferredResp.indexOf("+DMOCONNECT:0") >= 0);
    bool fallbackOk = false;
    String fallbackResp = "";

#if TRY_UHF_BOTH_UART_MAPPINGS
    if (!preferredOk) {
      Serial.println(F("[UHF-RETRY] Preferred map did not handshake. Trying fallback map..."));
      activateUhfUart(false);
      Serial.println(F("[UHF-RETRY] UART re-initialized (fallback map)."));
      Serial.print(F("[UHF-RETRY] RX bytes after fallback init: "));
      Serial.println(activeUhfSerial->available());
      Serial.println(F("[UHF-RETRY] Sending raw AT+DMOCONNECT handshake (fallback map)..."));
      activeUhfSerial->print(F("AT+DMOCONNECT\r\n"));
      delay(AT_TIMEOUT_MS);
      while (activeUhfSerial->available()) {
        fallbackResp += (char)activeUhfSerial->read();
      }
      Serial.print(F("[UHF-RETRY] Fallback raw handshake response: ["));
      Serial.print(fallbackResp);
      Serial.println(F("]"));
      fallbackOk = (fallbackResp.indexOf("+DMOCONNECT:0") >= 0);
    }
#endif

    if (preferredOk || fallbackOk) {
      Serial.println(F("[UHF-RETRY] Handshake OK. Applying full UHF defaults..."));
      DRA818 &activeUhfRadio = activeUhfPreferredMapping ? draUPreferred : draUFallback;
      if (configureUhfOnCurrentMapping(F("LOOP-RETRY/UHF"), activeUhfRadio)) {
        gUhfConfigured = true;
        Serial.println(F("[UHF-RETRY] UHF configuration succeeded on retry."));
        printUhfMappingSelection();
      } else {
        Serial.println(F("[UHF-RETRY] Full defaults still failed after handshake OK. Check frequency/band config."));
      }
    } else {
      bool bothSilent = (preferredResp.length() == 0) && (fallbackResp.length() == 0);
      if (bothSilent) {
        Serial.println(F("[UHF-RETRY] No response on BOTH UART mappings. Likely external loading/contention on D10/D11, module power issue, or broken JP1/JP2 path."));
      } else {
        Serial.println(F("[UHF-RETRY] Handshake failed with non-empty response; inspect response bytes above."));
      }
    }
  }

#if PTT_POLARITY_PROBE
  runVhfPttPolarityProbe();
#elif SCAN_TEST_MODE
  runScanTest();
#elif PTT_GPIO_SWEEP_MODE
  runRawPttSweep();
#else
  runPttTest();
#endif

  printStatusBanner(measuredvbat, F("Post-Test"));

  Serial.println(F("------------------------"));
  // Fill the 5-second inter-loop pause with repeating heartbeat fades.
  logDebugMessage(F("Loop"), F("Entering 5-second heartbeat wait window."));
  unsigned long pauseStart = millis();
  while (millis() - pauseStart < 5000) {
    heartbeatLED();
  }
  logDebugMessage(F("Loop"), F("5-second heartbeat wait window complete."));
}