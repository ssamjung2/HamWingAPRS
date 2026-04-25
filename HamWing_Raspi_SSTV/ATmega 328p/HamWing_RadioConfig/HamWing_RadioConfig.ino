/*
 * HamWing_RadioConfig.ino - ATmega328P parity port of Feather M0 behavior
 *
 * Goal:
 *   - Configure VHF/UHF DRA818 modules at startup (selectable)
 *   - Hand shared control lines (PD/H/L/PTT) to Raspberry Pi after startup
 *   - Monitor Pi ownership of shared lines and expose diagnostics over USB serial
 *
 * ATmega328P hardware differences from M0:
 *   - UHF uses SoftwareSerial on D11 (RX from module), D10 (TX to module)
 *   - VHF shares hardware Serial (D0/D1) with USB; heavy console traffic can interfere
 *   - No SERCOM/LowPower/WiFi APIs on AVR
 */

#include <SoftwareSerial.h>
#include <DRA818.h>
#include <string.h>
#include <stdio.h>
#include <avr/wdt.h>

//////////////////////////////////////////////////////////////////////////////
// BUILD CONFIGURATION
//////////////////////////////////////////////////////////////////////////////

#define MODULE_VHF 1
#define MODULE_UHF 2
#define MODULE_BOTH 3
#define ACTIVE_MODULE MODULE_BOTH

#define DEBUG_MODE 1

// UART mapping (KiCad-verified)
#define PIN_UART_VHF_RX 0
#define PIN_UART_VHF_TX 1
#define PIN_UART_UHF_RX 11  // D11/PB3 (MOSI): MCU RX from module TXD
#define PIN_UART_UHF_TX 10  // D10/PB2 (SS): MCU TX to module RXD

// Shared/control lines (KiCad-verified)
#define PIN_PD 12          // shared PD, HIGH=active
#define PIN_HL 18          // shared H/L, HIGH=1W, LOW=0.5W
#define PIN_SQL_VHF A0     // D14
#define PIN_SQL_UHF A1     // D15
#define PIN_PTT_VHF 16     // D16/A2, active LOW request
#define PIN_PTT_UHF 17     // D17/A3, active LOW request
#define PIN_VBAT A7        // Feather VBAT monitor divider node
#define STATUS_LED_PIN LED_BUILTIN

// Radio defaults
#define SERIAL_BAUD 9600
#define DRA_BAUD 9600
#define DRA_STEP_RETRIES 3
#define DRA_AT_TIMEOUT_MS 700
#define PRE_CONFIG_DELAY_MS 2000

// Runtime behavior
#define STATUS_ROW_INTERVAL_MS 1000
#define STATUS_HEADER_REPEAT_ROWS 25
#define STATUS_PERIODIC_COMPACT 1
#define DIAG_MAX_BYTES_PER_LOOP 32
#define DIAG_LOCAL_PTT_PULSE_MS 1200
#define RELEASED_PTT_ASSERT_HOLD_MS 120

#define HANDOFF_USE_PULLUPS 1
#define HANDOFF_PTT_USE_PULLUPS 1
#define HOLD_PD_HIGH_AFTER_CONFIG 0

#define LOW_VOLTAGE_CUTOFF_ENABLE 1
#define LOW_VOLTAGE_CUTOFF_V 3.30f
#define LOW_VOLTAGE_RECOVER_V 3.40f
#define LOW_VOLTAGE_CHECK_MS 5000
#define LOW_VOLTAGE_WARN_MS 15000
#define BATTERY_PRESENT_MIN_V 2.50f
#define BATTERY_PRESENT_MAX_V 4.35f

#define STUCK_PTT_WATCHDOG_ENABLE 1
#define STUCK_PTT_MAX_TX_MS (7UL * 60UL * 1000UL)

#define HARDWARE_WATCHDOG_ENABLE 1
#define HARDWARE_WATCHDOG_TIMEOUT WDTO_8S

//////////////////////////////////////////////////////////////////////////////
// DATA STRUCTURES
//////////////////////////////////////////////////////////////////////////////

struct RadioConfig {
  float freq_tx;
  float freq_rx;
  uint8_t squelch;
  uint8_t volume;
  uint8_t ctcss_tx;
  uint8_t ctcss_rx;
  uint8_t bandwidth;
  bool filter_pre;
  bool filter_high;
  bool filter_low;
};

RadioConfig VHF_CONFIG = {
  144.5500,
  144.5500,
  4,
  6,
  0,
  0,
  DRA818_12K5,
  false,
  false,
  false
};

RadioConfig UHF_CONFIG = {
  440.8000,
  440.8000,
  4,
  6,
  0,
  0,
  DRA818_12K5,
  false,
  false,
  false
};

//////////////////////////////////////////////////////////////////////////////
// GLOBAL STATE
//////////////////////////////////////////////////////////////////////////////

SoftwareSerial gUhfSerialPreferred(PIN_UART_UHF_RX, PIN_UART_UHF_TX);
SoftwareSerial gUhfSerialFallback(PIN_UART_UHF_TX, PIN_UART_UHF_RX);

DRA818 gVhfRadio((Stream *)&Serial, DRA818_VHF);
DRA818 gUhfRadioPreferred((Stream *)&gUhfSerialPreferred, DRA818_UHF);
DRA818 gUhfRadioFallback((Stream *)&gUhfSerialFallback, DRA818_UHF);

SoftwareSerial *gActiveUhfSerial = &gUhfSerialPreferred;
DRA818 *gActiveUhfRadio = &gUhfRadioPreferred;
bool gUhfUsingFallbackMapping = false;

bool gVhfConfigured = false;
bool gUhfConfigured = false;
bool gConfigurationComplete = false;

uint8_t gVhfFailStep = 0;
uint8_t gUhfFailStep = 0;

bool gSharedLinesReleasedToPi = false;
bool gPdLockOn = (HOLD_PD_HIGH_AFTER_CONFIG != 0);
bool gHlDiagOverrideActive = false;
bool gHlDiagLevelHigh = false;
bool gConsoleReady = false;
bool gConsoleMuted = false;

bool gLowVoltageCutoffActive = false;
unsigned long gLastLowVoltageWarnMs = 0;
unsigned long gLastVbatCheckMs = 0;

unsigned long gLastStatusMs = 0;
unsigned long gLastSqlLogMs = 0;
uint16_t gStatusRowsPrinted = 0;
bool gPeriodicStatusCompact = (STATUS_PERIODIC_COMPACT != 0);

int gLastVhfPttSense = -1;
int gLastUhfPttSense = -1;
int gLastVhfSqlSense = -1;
int gLastUhfSqlSense = -1;
uint32_t gVhfPttEdgeCount = 0;
uint32_t gUhfPttEdgeCount = 0;
uint32_t gVhfSqlOpenCount = 0;
uint32_t gUhfSqlOpenCount = 0;

unsigned long gTxConfirmedSinceMs = 0;
bool gStuckPttAlertEmitted = false;
unsigned long gStatusPauseUntilMs = 0;
bool gLocalPttPulseActive = false;
uint8_t gLocalPttPulsePin = 0xFF;

unsigned long gLastPdLockWarnMs = 0;
unsigned long gLastReleasedPttWarnMs = 0;
unsigned long gVhfReleasedLowSinceMs = 0;
unsigned long gUhfReleasedLowSinceMs = 0;
uint8_t gResetCauseFlags = 0;

enum {
  LED_IDLE,
  LED_CONFIG_ACTIVE,
  LED_SUCCESS_FADE,
  LED_FAILURE_BLINK,
  LED_TX_HEARTBEAT,
  LED_WARNING_BLINK,
  LED_LOW_VOLTAGE
};

enum {
  TX_DETECT_IDLE,
  TX_DETECT_CANDIDATE,
  TX_DETECT_CONFIRMED,
  TX_DETECT_BLOCKED_PD_LOW
};

uint8_t gCurrentLedState = LED_IDLE;
uint8_t gTxDetectState = TX_DETECT_IDLE;
unsigned long gLedTimestamp = 0;

//////////////////////////////////////////////////////////////////////////////
// HELPERS
//////////////////////////////////////////////////////////////////////////////

bool canLogToConsole() {
  return gConsoleReady && !gConsoleMuted;
}

void debugPrint(const __FlashStringHelper *label, const __FlashStringHelper *msg) {
#if DEBUG_MODE
  if (!canLogToConsole()) return;
  Serial.print('[');
  Serial.print(label);
  Serial.print(F("] "));
  Serial.println(msg);
#else
  (void)label;
  (void)msg;
#endif
}

void debugPrintC(const __FlashStringHelper *label, const char *msg) {
#if DEBUG_MODE
  if (!canLogToConsole()) return;
  Serial.print('[');
  Serial.print(label);
  Serial.print(F("] "));
  Serial.println(msg);
#else
  (void)label;
  (void)msg;
#endif
}

void pauseStatusLogs(unsigned long ms) {
  gStatusPauseUntilMs = millis() + ms;
}

void formatFreq(char *out, size_t outSize, float mhz) {
  long scaled = (long)(mhz * 10000.0f + (mhz >= 0 ? 0.5f : -0.5f));
  long whole = scaled / 10000;
  unsigned long frac = (unsigned long)labs(scaled % 10000);
  snprintf(out, outSize, "%ld.%04lu", whole, frac);
}

void drainSerialInput(Stream &port) {
  while (port.available()) {
    port.read();
  }
}

void printWindowEscaped(const char *buf, size_t len) {
  if (len == 0) { Serial.print(F("<none>")); return; }
  for (size_t i = 0; i < len; i++) {
    uint8_t b = (uint8_t)buf[i];
    if (b >= 0x20 && b < 0x7F) Serial.print((char)b);
    else if (b == '\r') Serial.print(F("\\r"));
    else if (b == '\n') Serial.print(F("\\n"));
    else { Serial.print(F("\\x")); if (b < 0x10) Serial.print('0'); Serial.print(b, HEX); }
  }
}

void selectUhfMapping(bool useFallback) {
  gUhfUsingFallbackMapping = useFallback;
  gActiveUhfSerial = useFallback ? &gUhfSerialFallback : &gUhfSerialPreferred;
  gActiveUhfRadio = useFallback ? &gUhfRadioFallback : &gUhfRadioPreferred;
  gActiveUhfSerial->listen();
}

bool isUhfPort(Stream &port) {
  return (&port == (Stream *)&gUhfSerialPreferred) || (&port == (Stream *)&gUhfSerialFallback);
}

const __FlashStringHelper *currentUhfMappingName() {
  return gUhfUsingFallbackMapping ? F("FALLBACK D10-RX/D11-TX") : F("PREFERRED D11-RX/D10-TX");
}

void printLevelWord(int level) {
  Serial.print(level == LOW ? F("LOW") : F("HIGH"));
}

void printUhfUartIdleLevels() {
  pinMode(PIN_UART_UHF_RX, INPUT);
  pinMode(PIN_UART_UHF_TX, INPUT);
  delay(2);

  int rxLevel = digitalRead(PIN_UART_UHF_RX);
  int txLevel = digitalRead(PIN_UART_UHF_TX);

  Serial.print(F("[UHF-LINES] D11/D10 idle="));
  printLevelWord(rxLevel);
  Serial.print('/');
  printLevelWord(txLevel);
  Serial.println();

  if (rxLevel == LOW || txLevel == LOW) {
    Serial.println(F("[UHF-LINES] WARNING: D11 or D10 is LOW at idle; external loading, swapped wiring, or module drive is likely."));
  }

  gActiveUhfSerial->listen();
}

bool rawHandshakeProbe(Stream &port, const __FlashStringHelper *label, unsigned long timeoutMs) {
  if (isUhfPort(port)) {
    gActiveUhfSerial->listen();
  }

  if (&port == (Stream *)&Serial) {
    Serial.flush();
    delay(80);
  }

  drainSerialInput(port);
  port.print(F("AT+DMOCONNECT\r\n"));

  char window[96];
  size_t used = 0;
  window[0] = '\0';

  unsigned long start = millis();
  while ((millis() - start) < timeoutMs) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    while (port.available()) {
      char c = (char)port.read();
      if (used < sizeof(window) - 1) {
        window[used++] = c;
        window[used] = '\0';
      } else {
        memmove(window, window + 1, sizeof(window) - 2);
        window[sizeof(window) - 2] = c;
        window[sizeof(window) - 1] = '\0';
      }
    }
  }

  if (canLogToConsole()) {
    Serial.print('[');
    Serial.print(label);
    Serial.print(F("] RAW DMOCONNECT => \""));
    printWindowEscaped(window, used);
    Serial.println('"');
  }

  return strstr_P(window, PSTR("DMOCONNECT:0")) != NULL;
}

bool sendATExpect(Stream &port, const char *cmd, const __FlashStringHelper *expected, unsigned long timeoutMs) {
  const __FlashStringHelper *lbl = isUhfPort(port) ? F("UHF") : F("VHF");
  drainSerialInput(port);
  port.print(cmd);
  port.print(F("\r\n"));

  char window[64];
  size_t used = 0;
  window[0] = '\0';

  unsigned long start = millis();
  while ((millis() - start) < timeoutMs) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    while (port.available()) {
      char c = (char)port.read();
      if (used < sizeof(window) - 1) {
        window[used++] = c;
        window[used] = '\0';
      } else {
        memmove(window, window + 1, sizeof(window) - 2);
        window[sizeof(window) - 2] = c;
        window[sizeof(window) - 1] = '\0';
      }

      if (strstr_P(window, (PGM_P)expected) != NULL) {
#if DEBUG_MODE
  if (canLogToConsole()) {
    Serial.print('['); Serial.print(lbl); Serial.print(F("] AT OK: "));
    Serial.print(cmd); Serial.print(F(" => \""));
    printWindowEscaped(window, used);
    Serial.println('"');
  }
#endif
        return true;
      }
    }
  }

#if DEBUG_MODE
  if (canLogToConsole()) {
    Serial.print('['); Serial.print(lbl); Serial.print(F("] AT FAIL: "));
    Serial.print(cmd); Serial.print(F(" => \""));
    printWindowEscaped(window, used);
    Serial.println('"');
  }
#endif
  return false;
}

bool setGroupFallbackRaw(DRA818 &radio, const RadioConfig &config) {
  char groupCmd[96];
  buildDirectGroupCmd(config, groupCmd, sizeof(groupCmd));

  return sendATExpect(*radio.serial, groupCmd, F("DMOSETGROUP:0"), DRA_AT_TIMEOUT_MS);
}

bool setVolumeFallbackRaw(DRA818 &radio, uint8_t volume) {
  char cmd[32];
  buildDirectVolumeCmd(volume, cmd, sizeof(cmd));
  return sendATExpect(*radio.serial, cmd, F("DMOSETVOLUME:0"), DRA_AT_TIMEOUT_MS);
}

bool setFilterFallbackRaw(DRA818 &radio, bool pre, bool high, bool low) {
  char cmd[40];
  buildDirectFilterCmd(pre, high, low, cmd, sizeof(cmd));
  return sendATExpect(*radio.serial, cmd, F("DMOSETFILTER:0"), DRA_AT_TIMEOUT_MS);
}

bool configureRadio(DRA818 &radio, RadioConfig &cfg, const __FlashStringHelper *label, uint8_t &failStep) {
  bool ok = false;
  failStep = 0;

  char msg[48];
  snprintf_P(msg, sizeof(msg), PSTR("%S handshake"), label);
  debugPrintC(F("CFG"), msg);

  // Flush any pending debug-print TX and drain module RX before starting AT exchange.
  // For VHF (Serial shared with USB), debug prints above went to the module as garbage;
  // the module may have emitted an error response that would corrupt handshake() reads.
  Serial.flush();
  delay(50);
  drainSerialInput(*radio.serial);

  if (isUhfPort(*radio.serial)) {
    printUhfUartIdleLevels();
  }

  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    int r = radio.handshake();
    debugPrint(F("CFG"), r == 1 ? F("handshake lib: OK") : F("handshake lib: FAIL"));
    ok = (r == 1);
  }
  if (!ok) {
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      ok = sendATExpect(*radio.serial, "AT+DMOCONNECT", F("DMOCONNECT:0"), DRA_AT_TIMEOUT_MS);
    }
  }
  if (!ok) {
    rawHandshakeProbe(*radio.serial, label, DRA_AT_TIMEOUT_MS);
    failStep = 1;
    return false;
  }

  ok = false;
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    int r = radio.group(cfg.bandwidth, cfg.freq_tx, cfg.freq_rx, cfg.ctcss_tx, cfg.squelch, cfg.ctcss_rx);
    debugPrint(F("CFG"), r == 1 ? F("group lib: OK") : F("group lib: FAIL"));
    ok = (r == 1);
  }
  if (!ok) {
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      ok = setGroupFallbackRaw(radio, cfg);
    }
  }
  if (!ok) {
    failStep = 2;
    return false;
  }

  ok = false;
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    int r = radio.volume(cfg.volume);
    debugPrint(F("CFG"), r == 1 ? F("volume lib: OK") : F("volume lib: FAIL"));
    ok = (r == 1);
  }
  if (!ok) {
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      ok = setVolumeFallbackRaw(radio, cfg.volume);
    }
  }
  if (!ok) {
    failStep = 3;
    return false;
  }

  ok = false;
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    int r = radio.filters(cfg.filter_pre, cfg.filter_high, cfg.filter_low);
    debugPrint(F("CFG"), r == 1 ? F("filters lib: OK") : F("filters lib: FAIL"));
    ok = (r == 1);
  }
  if (!ok) {
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      ok = setFilterFallbackRaw(radio, cfg.filter_pre, cfg.filter_high, cfg.filter_low);
    }
  }
  if (!ok) {
    failStep = 4;
    return false;
  }

  return true;
}

bool configureUhfRadio(RadioConfig &cfg, uint8_t &failStep) {
  debugPrint(F("UHF"), F("Trying preferred mapping: D11 RX, D10 TX"));
  selectUhfMapping(false);
  if (configureRadio(*gActiveUhfRadio, cfg, F("UHF"), failStep)) {
    debugPrint(F("UHF"), F("Preferred UHF mapping works"));
    return true;
  }

  debugPrint(F("UHF"), F("Trying fallback mapping: D10 RX, D11 TX"));
  selectUhfMapping(true);
  if (configureRadio(*gActiveUhfRadio, cfg, F("UHF"), failStep)) {
    debugPrint(F("UHF"), F("Fallback UHF mapping works"));
    return true;
  }

  return false;
}

void applyPdOwnershipState(bool emitLog) {
  if (gPdLockOn) {
    pinMode(PIN_PD, OUTPUT);
    digitalWrite(PIN_PD, HIGH);
    if (emitLog) debugPrint(F("PD"), F("PD lock ON: driving HIGH"));
  } else {
#if HANDOFF_USE_PULLUPS
    pinMode(PIN_PD, INPUT_PULLUP);
#else
    pinMode(PIN_PD, INPUT);
#endif
    if (emitLog) debugPrint(F("PD"), F("PD lock OFF: released to Pi"));
  }
}

void setPdLock(bool enabled) {
  gPdLockOn = enabled;
  applyPdOwnershipState(true);
}

void enforcePdLockIfEnabled() {
#if LOW_VOLTAGE_CUTOFF_ENABLE
  if (gLowVoltageCutoffActive) return;
#endif
  if (!gPdLockOn) return;

  pinMode(PIN_PD, OUTPUT);
  digitalWrite(PIN_PD, HIGH);

  int pdSense = digitalRead(PIN_PD);
  if (pdSense == LOW && (millis() - gLastPdLockWarnMs) > 1000UL) {
    gLastPdLockWarnMs = millis();
    debugPrint(F("PD"), F("PD lock active but sensed LOW (external contention?)"));
  }
}

void setPttPinsIdleOutput() {
  pinMode(PIN_PTT_VHF, OUTPUT);
  pinMode(PIN_PTT_UHF, OUTPUT);
  digitalWrite(PIN_PTT_VHF, HIGH);  // active-low idle
  digitalWrite(PIN_PTT_UHF, HIGH);  // active-low idle
}

void releasePttPinToPi(uint8_t pin) {
#if HANDOFF_PTT_USE_PULLUPS
  pinMode(pin, INPUT_PULLUP);
#else
  pinMode(pin, INPUT);
#endif
}

void releasePttPinsToPi() {
  releasePttPinToPi(PIN_PTT_VHF);
  releasePttPinToPi(PIN_PTT_UHF);
}

void printReleasedPttLineDiagnosis() {
  delay(2);
  int vhfReleased = digitalRead(PIN_PTT_VHF);
  int uhfReleased = digitalRead(PIN_PTT_UHF);

  Serial.print(F("[HANDOFF] PTT released-line sense V/U="));
  Serial.print(vhfReleased == LOW ? F("LOW") : F("HIGH"));
  Serial.print('/');
  Serial.println(uhfReleased == LOW ? F("LOW") : F("HIGH"));

  if (vhfReleased == LOW) {
    Serial.println(F("[HANDOFF] WARNING: VHF PTT still LOW when released; external pull-down or Pi drive is asserting TX."));
  }
  if (uhfReleased == LOW) {
    Serial.println(F("[HANDOFF] WARNING: UHF PTT still LOW when released; external pull-down or Pi drive is asserting TX."));
  }
}

void driveSharedLinesForStartupConfig() {
  pinMode(PIN_PD, OUTPUT);
  digitalWrite(PIN_PD, HIGH);

  pinMode(PIN_HL, OUTPUT);
  digitalWrite(PIN_HL, LOW);

  setPttPinsIdleOutput();
  gHlDiagOverrideActive = false;
}

void releaseHlDiagToPi() {
  pinMode(PIN_HL, INPUT);
  gHlDiagOverrideActive = false;
  debugPrint(F("HL"), F("Released H/L to Pi"));
}

void setHlDiagLevel(bool high) {
  pinMode(PIN_HL, OUTPUT);
  digitalWrite(PIN_HL, high ? HIGH : LOW);
  gHlDiagOverrideActive = true;
  gHlDiagLevelHigh = high;
  debugPrint(high ? F("HL") : F("HL"), high ? F("H/L override -> HIGH") : F("H/L override -> LOW"));
}

void releaseSharedLinesToPi() {
  if (gSharedLinesReleasedToPi) return;

  applyPdOwnershipState(false);
  releaseHlDiagToPi();
  releasePttPinsToPi();
  printReleasedPttLineDiagnosis();
  gSharedLinesReleasedToPi = true;

  if (gPdLockOn) {
    debugPrint(F("HANDOFF"), F("Released HL/PTT; PD lock remains ON"));
  } else {
    debugPrint(F("HANDOFF"), F("Released PD/HL/PTT to Pi"));
  }
}

long readVccMillivolts() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) {}
  uint16_t result = ADC;
  if (result == 0) return 5000;
  return 1125300L / result;
}

float readBatteryVoltage() {
  const uint8_t samples = 16;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    acc += (uint16_t)analogRead(PIN_VBAT);
  }
  float adc = (float)acc / (float)samples;
  float vcc = (float)readVccMillivolts() / 1000.0f;
  return (adc / 1023.0f) * vcc * 2.0f;
}

bool isBatteryVoltagePlausible(float vbat) {
  return (vbat >= BATTERY_PRESENT_MIN_V) && (vbat <= BATTERY_PRESENT_MAX_V);
}

void setLedState(uint8_t newState) {
  if (gCurrentLedState != newState) {
    gCurrentLedState = newState;
    gLedTimestamp = millis();
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
    case LED_LOW_VOLTAGE: return "LV_CUTOFF";
    default: return "UNKNOWN";
  }
}

void updateLed() {
  unsigned long now = millis();
  unsigned long elapsed = now - gLedTimestamp;

  switch (gCurrentLedState) {
    case LED_IDLE:
      analogWrite(STATUS_LED_PIN, 0);
      break;
    case LED_CONFIG_ACTIVE:
      analogWrite(STATUS_LED_PIN, ((elapsed / 200UL) % 2UL) ? 20 : 200);
      break;
    case LED_SUCCESS_FADE:
      analogWrite(STATUS_LED_PIN, (elapsed / 8UL) % 255UL);
      break;
    case LED_FAILURE_BLINK:
      digitalWrite(STATUS_LED_PIN, ((elapsed / 100UL) % 2UL) ? HIGH : LOW);
      break;
    case LED_TX_HEARTBEAT:
      analogWrite(STATUS_LED_PIN, 180 + ((elapsed / 4UL) % 75UL));
      break;
    case LED_WARNING_BLINK:
      digitalWrite(STATUS_LED_PIN, ((elapsed / 600UL) % 2UL) ? HIGH : LOW);
      break;
    case LED_LOW_VOLTAGE:
      digitalWrite(STATUS_LED_PIN, ((elapsed % 2000UL) < 200UL) ? HIGH : LOW);
      break;
  }
}

bool updateTxDetectState(bool pttAsserted, bool pdHigh) {
  uint8_t prev = gTxDetectState;
  bool txConfidence = pttAsserted && pdHigh;

  switch (gTxDetectState) {
    case TX_DETECT_IDLE:
      if (pttAsserted) gTxDetectState = pdHigh ? TX_DETECT_CANDIDATE : TX_DETECT_BLOCKED_PD_LOW;
      break;
    case TX_DETECT_CANDIDATE:
      if (!pttAsserted) gTxDetectState = TX_DETECT_IDLE;
      else if (txConfidence) gTxDetectState = TX_DETECT_CONFIRMED;
      else gTxDetectState = TX_DETECT_BLOCKED_PD_LOW;
      break;
    case TX_DETECT_BLOCKED_PD_LOW:
      if (!pttAsserted) gTxDetectState = TX_DETECT_IDLE;
      else if (pdHigh) gTxDetectState = TX_DETECT_CANDIDATE;
      break;
    case TX_DETECT_CONFIRMED:
      if (!txConfidence) gTxDetectState = pttAsserted ? TX_DETECT_BLOCKED_PD_LOW : TX_DETECT_IDLE;
      break;
  }

  return prev != gTxDetectState;
}

uint8_t ledStateForTxDetectState() {
  if (gTxDetectState == TX_DETECT_CONFIRMED) return LED_TX_HEARTBEAT;
  if (gTxDetectState == TX_DETECT_BLOCKED_PD_LOW) return LED_WARNING_BLINK;
  return LED_SUCCESS_FADE;
}

void checkLowVoltageCutoff() {
#if LOW_VOLTAGE_CUTOFF_ENABLE
  unsigned long now = millis();
  if ((now - gLastVbatCheckMs) < LOW_VOLTAGE_CHECK_MS) return;
  gLastVbatCheckMs = now;

  float vbat = readBatteryVoltage();
  if (!isBatteryVoltagePlausible(vbat)) return;

  if (!gLowVoltageCutoffActive && vbat < LOW_VOLTAGE_CUTOFF_V) {
    gLowVoltageCutoffActive = true;
    gLastLowVoltageWarnMs = now;
    pinMode(PIN_PD, OUTPUT);
    digitalWrite(PIN_PD, LOW);
    setLedState(LED_LOW_VOLTAGE);
    debugPrint(F("LVCO"), F("Low voltage cutoff active: PD forced LOW"));
  } else if (gLowVoltageCutoffActive && vbat >= LOW_VOLTAGE_RECOVER_V) {
    gLowVoltageCutoffActive = false;
    applyPdOwnershipState(true);
    setLedState(gConfigurationComplete ? LED_SUCCESS_FADE : LED_FAILURE_BLINK);
    debugPrint(F("LVCO"), F("Voltage recovered: normal operation restored"));
  } else if (gLowVoltageCutoffActive && (now - gLastLowVoltageWarnMs) > LOW_VOLTAGE_WARN_MS) {
    gLastLowVoltageWarnMs = now;
    debugPrint(F("LVCO"), F("Still in low voltage cutoff"));
  }
#endif
}

void checkStuckPttWatchdog(int pd, int vptt, int uptt) {
#if STUCK_PTT_WATCHDOG_ENABLE
  bool txConfirmed = ((vptt == LOW) || (uptt == LOW)) && (pd == HIGH);
  if (!txConfirmed) {
    gTxConfirmedSinceMs = 0;
    gStuckPttAlertEmitted = false;
    return;
  }

  if (gTxConfirmedSinceMs == 0) {
    gTxConfirmedSinceMs = millis();
    return;
  }

  unsigned long heldMs = millis() - gTxConfirmedSinceMs;
  if (heldMs > STUCK_PTT_MAX_TX_MS && !gStuckPttAlertEmitted) {
    gStuckPttAlertEmitted = true;
    debugPrint(F("STUCK_PTT"), F("TX held > 7 min. Forcing temporary unkey."));

    // Temporary reclaim, force HIGH idle, then release.
    pinMode(PIN_PTT_VHF, OUTPUT);
    pinMode(PIN_PTT_UHF, OUTPUT);
    digitalWrite(PIN_PTT_VHF, HIGH);
    digitalWrite(PIN_PTT_UHF, HIGH);
    delay(10);
    releasePttPinsToPi();
  }
#else
  (void)pd;
  (void)vptt;
  (void)uptt;
#endif
}

bool parseUint8Token(const char *token, uint8_t minVal, uint8_t maxVal, uint8_t &out) {
  if (!token || !token[0]) return false;
  uint16_t value = 0;
  for (size_t i = 0; token[i] != '\0'; ++i) {
    if (token[i] < '0' || token[i] > '9') return false;
    value = (uint16_t)(value * 10 + (token[i] - '0'));
    if (value > 255) return false;
  }
  if (value < minVal || value > maxVal) return false;
  out = (uint8_t)value;
  return true;
}

bool parseFreqToken(const char *token, float &outMhz) {
  if (!token || !token[0]) return false;
  bool seenDot = false;
  for (size_t i = 0; token[i] != '\0'; ++i) {
    char c = token[i];
    if (c >= '0' && c <= '9') continue;
    if (c == '.' && !seenDot) {
      seenDot = true;
      continue;
    }
    return false;
  }
  outMhz = atof(token);
  return (outMhz >= 100.0f && outMhz <= 1000.0f);
}

bool parseFilterToken(const char *token, bool &pre, bool &high, bool &low) {
  if (!token || strlen(token) != 3) return false;
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

bool readDiagArgToken(char *out, size_t outSize) {
  if (!out || outSize < 2) return false;

  while (Serial.available() > 0) {
    char c = (char)Serial.peek();
    if (c == ' ' || c == '\t') Serial.read();
    else break;
  }

  size_t idx = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.peek();
    if (c == '\n' || c == '\r' || c == ' ' || c == '\t') break;
    c = (char)Serial.read();
    if (idx < (outSize - 1)) out[idx++] = c;
  }
  out[idx] = '\0';
  return idx > 0;
}

bool applyVolumeFor(DRA818 &radio, RadioConfig &cfg, uint8_t volume, bool useSoftListen) {
  if (useSoftListen) gActiveUhfSerial->listen();

  bool ok = (radio.volume(volume) == 1) || setVolumeFallbackRaw(radio, volume);
  if (ok) cfg.volume = volume;
  return ok;
}

bool applySquelchFor(DRA818 &radio, RadioConfig &cfg, uint8_t squelch, bool useSoftListen) {
  if (useSoftListen) gActiveUhfSerial->listen();

  RadioConfig cand = cfg;
  cand.squelch = squelch;
  bool ok = (radio.group(cand.bandwidth, cand.freq_tx, cand.freq_rx, cand.ctcss_tx, cand.squelch, cand.ctcss_rx) == 1) ||
            setGroupFallbackRaw(radio, cand);
  if (ok) cfg = cand;
  return ok;
}

bool applyFiltersFor(DRA818 &radio, RadioConfig &cfg, bool pre, bool high, bool low, bool useSoftListen) {
  if (useSoftListen) gActiveUhfSerial->listen();

  bool ok = (radio.filters(pre, high, low) == 1) || setFilterFallbackRaw(radio, pre, high, low);
  if (ok) {
    cfg.filter_pre = pre;
    cfg.filter_high = high;
    cfg.filter_low = low;
  }
  return ok;
}

bool applyFrequencyFor(DRA818 &radio, RadioConfig &cfg, bool setTx, float mhz, bool useSoftListen) {
  if (useSoftListen) gActiveUhfSerial->listen();

  RadioConfig cand = cfg;
  if (setTx) cand.freq_tx = mhz;
  else cand.freq_rx = mhz;

  bool ok = (radio.group(cand.bandwidth, cand.freq_tx, cand.freq_rx, cand.ctcss_tx, cand.squelch, cand.ctcss_rx) == 1) ||
            setGroupFallbackRaw(radio, cand);
  if (ok) cfg = cand;
  return ok;
}

bool applyVolumeAll(uint8_t volume) {
  bool allOk = true;

#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  if (gVhfConfigured) allOk = applyVolumeFor(gVhfRadio, VHF_CONFIG, volume, false) && allOk;
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  if (gUhfConfigured) allOk = applyVolumeFor(*gActiveUhfRadio, UHF_CONFIG, volume, true) && allOk;
#endif

  return allOk;
}

bool applySquelchAll(uint8_t squelch) {
  bool allOk = true;

#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  if (gVhfConfigured) allOk = applySquelchFor(gVhfRadio, VHF_CONFIG, squelch, false) && allOk;
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  if (gUhfConfigured) allOk = applySquelchFor(*gActiveUhfRadio, UHF_CONFIG, squelch, true) && allOk;
#endif

  return allOk;
}

bool applyFiltersAll(bool pre, bool high, bool low) {
  bool allOk = true;

#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  if (gVhfConfigured) allOk = applyFiltersFor(gVhfRadio, VHF_CONFIG, pre, high, low, false) && allOk;
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  if (gUhfConfigured) allOk = applyFiltersFor(*gActiveUhfRadio, UHF_CONFIG, pre, high, low, true) && allOk;
#endif

  return allOk;
}

bool applyFrequencyCommand(char moduleSel, bool setTx, float mhz) {
  if (moduleSel == 'v' || moduleSel == 'V') {
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
    if (!gVhfConfigured) return false;
    return applyFrequencyFor(gVhfRadio, VHF_CONFIG, setTx, mhz, false);
#else
    return false;
#endif
  }

  if (moduleSel == 'u' || moduleSel == 'U') {
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
    if (!gUhfConfigured) return false;
    return applyFrequencyFor(*gActiveUhfRadio, UHF_CONFIG, setTx, mhz, true);
#else
    return false;
#endif
  }

  return false;
}

bool directATExchange(Stream &port, const char *cmd, unsigned long timeoutMs, char *out, size_t outSize) {
  if (!out || outSize < 2) return false;
  out[0] = '\0';

  drainSerialInput(port);
  port.print(cmd);
  port.print(F("\r\n"));

  size_t used = 0;
  unsigned long start = millis();
  while ((millis() - start) < timeoutMs) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    while (port.available()) {
      char c = (char)port.read();
      if (used < outSize - 1) {
        out[used++] = c;
        out[used] = '\0';
      } else {
        memmove(out, out + 1, outSize - 2);
        out[outSize - 2] = c;
        out[outSize - 1] = '\0';
      }
    }
  }
  return used > 0;
}

void buildDirectConnectCmd(char *out, size_t outSize) {
  snprintf_P(out, outSize, PSTR("AT+DMOCONNECT"));
}

void buildDirectVolumeCmd(uint8_t volume, char *out, size_t outSize) {
  snprintf_P(out, outSize, PSTR("AT+DMOSETVOLUME=%u"), volume);
}

void buildDirectFilterCmd(bool pre, bool high, bool low, char *out, size_t outSize) {
  snprintf_P(out, outSize, PSTR("AT+SETFILTER=%u,%u,%u"), pre ? 1 : 0, high ? 1 : 0, low ? 1 : 0);
}

void printDirectAtResult(const __FlashStringHelper *scope, const char *cmd, const char *resp) {
  Serial.print(F("["));
  Serial.print(scope);
  Serial.print(F("] TX: "));
  Serial.println(cmd);
  Serial.print(F("["));
  Serial.print(scope);
  Serial.print(F("] RX: \""));
  printWindowEscaped(resp, strlen(resp));
  Serial.println('"');
}

bool directResponseHasToken(const char *resp, const char *primaryToken, const char *altToken) {
  if (!primaryToken) return true;
  if (strstr(resp, primaryToken) != NULL) return true;
  if (altToken && strstr(resp, altToken) != NULL) return true;
  return false;
}

bool sendDirectATVhf(const char *cmd, unsigned long timeoutMs, const char *expectToken, const char *altExpectToken = NULL) {
  // VHF shares USB UART; keep this window tight and avoid extra prints while exchanging.
  Serial.flush();
  delay(30);

  char resp[128];
  bool any = directATExchange(Serial, cmd, timeoutMs, resp, sizeof(resp));
  printDirectAtResult(F("VHF-DIRECT"), cmd, resp);

  if (!any) return false;
  return directResponseHasToken(resp, expectToken, altExpectToken);
}

bool sendDirectATUhfOn(SoftwareSerial &port, const __FlashStringHelper *label, const char *cmd, unsigned long timeoutMs, const char *expectToken, const char *altExpectToken = NULL) {
  port.listen();
  char resp[128];
  bool any = directATExchange(port, cmd, timeoutMs, resp, sizeof(resp));
  printDirectAtResult(label, cmd, resp);
  if (!any) return false;
  return directResponseHasToken(resp, expectToken, altExpectToken);
}

void buildDirectGroupCmd(const RadioConfig &cfg, char *out, size_t outSize) {
  char txStr[16];
  char rxStr[16];
  formatFreq(txStr, sizeof(txStr), cfg.freq_tx);
  formatFreq(rxStr, sizeof(rxStr), cfg.freq_rx);
  snprintf_P(
    out,
    outSize,
    PSTR("AT+DMOSETGROUP=%u,%s,%s,%04u,%u,%04u"),
    cfg.bandwidth,
    txStr,
    rxStr,
    cfg.ctcss_tx,
    cfg.squelch,
    cfg.ctcss_rx);
}

bool runDirectConfigSequenceVhf() {
  char connectCmd[20];
  char groupCmd[96];
  char volumeCmd[32];
  char filterCmd[32];
  buildDirectConnectCmd(connectCmd, sizeof(connectCmd));
  buildDirectGroupCmd(VHF_CONFIG, groupCmd, sizeof(groupCmd));
  buildDirectVolumeCmd(VHF_CONFIG.volume, volumeCmd, sizeof(volumeCmd));
  buildDirectFilterCmd(VHF_CONFIG.filter_pre, VHF_CONFIG.filter_high, VHF_CONFIG.filter_low, filterCmd, sizeof(filterCmd));

  bool ok = true;
  ok = sendDirectATVhf(connectCmd, DRA_AT_TIMEOUT_MS, "DMOCONNECT:0") && ok;
  ok = sendDirectATVhf(groupCmd, DRA_AT_TIMEOUT_MS, "DMOSETGROUP:0", "DMOCONNECT:0") && ok;
  ok = sendDirectATVhf(volumeCmd, DRA_AT_TIMEOUT_MS, "DMOSETVOLUME:0") && ok;
  ok = sendDirectATVhf(filterCmd, DRA_AT_TIMEOUT_MS, "DMOSETFILTER:0") && ok;
  return ok;
}

bool runDirectConfigSequenceUhf() {
  char connectCmd[20];
  char groupCmd[96];
  char volumeCmd[32];
  char filterCmd[32];
  buildDirectConnectCmd(connectCmd, sizeof(connectCmd));
  buildDirectGroupCmd(UHF_CONFIG, groupCmd, sizeof(groupCmd));
  buildDirectVolumeCmd(UHF_CONFIG.volume, volumeCmd, sizeof(volumeCmd));
  buildDirectFilterCmd(UHF_CONFIG.filter_pre, UHF_CONFIG.filter_high, UHF_CONFIG.filter_low, filterCmd, sizeof(filterCmd));

  Serial.println(F("[UHF-DIRECT] Trying preferred mapping (D11 RX, D10 TX)"));
  bool pref = true;
  pref = sendDirectATUhfOn(gUhfSerialPreferred, F("UHF-DIRECT/PREF"), connectCmd, DRA_AT_TIMEOUT_MS, "DMOCONNECT:0") && pref;
  pref = sendDirectATUhfOn(gUhfSerialPreferred, F("UHF-DIRECT/PREF"), groupCmd, DRA_AT_TIMEOUT_MS, "DMOSETGROUP:0", "DMOCONNECT:0") && pref;
  pref = sendDirectATUhfOn(gUhfSerialPreferred, F("UHF-DIRECT/PREF"), volumeCmd, DRA_AT_TIMEOUT_MS, "DMOSETVOLUME:0") && pref;
  pref = sendDirectATUhfOn(gUhfSerialPreferred, F("UHF-DIRECT/PREF"), filterCmd, DRA_AT_TIMEOUT_MS, "DMOSETFILTER:0") && pref;

  Serial.println(F("[UHF-DIRECT] Trying fallback mapping (D10 RX, D11 TX)"));
  bool fb = true;
  fb = sendDirectATUhfOn(gUhfSerialFallback, F("UHF-DIRECT/FALLBACK"), connectCmd, DRA_AT_TIMEOUT_MS, "DMOCONNECT:0") && fb;
  fb = sendDirectATUhfOn(gUhfSerialFallback, F("UHF-DIRECT/FALLBACK"), groupCmd, DRA_AT_TIMEOUT_MS, "DMOSETGROUP:0", "DMOCONNECT:0") && fb;
  fb = sendDirectATUhfOn(gUhfSerialFallback, F("UHF-DIRECT/FALLBACK"), volumeCmd, DRA_AT_TIMEOUT_MS, "DMOSETVOLUME:0") && fb;
  fb = sendDirectATUhfOn(gUhfSerialFallback, F("UHF-DIRECT/FALLBACK"), filterCmd, DRA_AT_TIMEOUT_MS, "DMOSETFILTER:0") && fb;

  if (pref) {
    selectUhfMapping(false);
    Serial.println(F("[UHF-DIRECT] Preferred mapping passed all steps."));
  } else if (fb) {
    selectUhfMapping(true);
    Serial.println(F("[UHF-DIRECT] Fallback mapping passed all steps."));
  } else {
    Serial.println(F("[UHF-DIRECT] Neither mapping passed full direct sequence."));
  }

  return pref || fb;
}

void runDirectSingleCommand(char moduleSel, const char *cmd) {
  if (moduleSel == 'v' || moduleSel == 'V') {
    (void)sendDirectATVhf(cmd, DRA_AT_TIMEOUT_MS, NULL);
    return;
  }

  if (moduleSel == 'u' || moduleSel == 'U') {
    Serial.println(F("[UHF-DIRECT] Single command on preferred mapping"));
    (void)sendDirectATUhfOn(gUhfSerialPreferred, F("UHF-DIRECT/PREF"), cmd, DRA_AT_TIMEOUT_MS, NULL);
    Serial.println(F("[UHF-DIRECT] Single command on fallback mapping"));
    (void)sendDirectATUhfOn(gUhfSerialFallback, F("UHF-DIRECT/FALLBACK"), cmd, DRA_AT_TIMEOUT_MS, NULL);
    return;
  }

  Serial.println(F("[DIAG] Usage: a <v|u> <AT+...>"));
}

void runLocalPttPulse(const __FlashStringHelper *label, uint8_t pin, unsigned long pulseMs) {
#if LOW_VOLTAGE_CUTOFF_ENABLE
  if (gLowVoltageCutoffActive) {
    debugPrint(F("DIAG"), F("PTT pulse blocked: low voltage cutoff active"));
    return;
  }
#endif

  gLocalPttPulseActive = true;
  gLocalPttPulsePin = pin;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(20);
  digitalWrite(pin, LOW);

  Serial.print(F("[DIAG] Local "));
  Serial.print(label);
  Serial.print(F(" PTT pulse "));
  Serial.print(pulseMs);
  Serial.println(F(" ms"));

  unsigned long start = millis();
  while ((millis() - start) < pulseMs) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    updateLed();
    enforcePdLockIfEnabled();
    delay(1);
  }

  digitalWrite(pin, HIGH);
  releasePttPinToPi(pin);
  gLocalPttPulseActive = false;
  gLocalPttPulsePin = 0xFF;
}

void updateReleasedPttLowTimers(int vptt, int uptt) {
  if (!gSharedLinesReleasedToPi) {
    gVhfReleasedLowSinceMs = 0;
    gUhfReleasedLowSinceMs = 0;
    return;
  }

  unsigned long now = millis();
  bool vhfLow = (vptt == LOW) && !(gLocalPttPulseActive && gLocalPttPulsePin == PIN_PTT_VHF);
  bool uhfLow = (uptt == LOW) && !(gLocalPttPulseActive && gLocalPttPulsePin == PIN_PTT_UHF);

  if (vhfLow) {
    if (gVhfReleasedLowSinceMs == 0) gVhfReleasedLowSinceMs = now;
  } else {
    gVhfReleasedLowSinceMs = 0;
  }

  if (uhfLow) {
    if (gUhfReleasedLowSinceMs == 0) gUhfReleasedLowSinceMs = now;
  } else {
    gUhfReleasedLowSinceMs = 0;
  }
}

bool isReleasedPttLowStable(uint8_t pin) {
  unsigned long now = millis();
  if (pin == PIN_PTT_VHF) {
    return (gVhfReleasedLowSinceMs != 0) && ((now - gVhfReleasedLowSinceMs) >= RELEASED_PTT_ASSERT_HOLD_MS);
  }
  if (pin == PIN_PTT_UHF) {
    return (gUhfReleasedLowSinceMs != 0) && ((now - gUhfReleasedLowSinceMs) >= RELEASED_PTT_ASSERT_HOLD_MS);
  }
  return false;
}

void warnIfReleasedPttAsserted(int vptt, int uptt) {
  (void)vptt;
  (void)uptt;
  if (!gSharedLinesReleasedToPi) return;
  if ((millis() - gLastReleasedPttWarnMs) < 1000UL) return;

  bool vhfUnexpectedLow = isReleasedPttLowStable(PIN_PTT_VHF);
  bool uhfUnexpectedLow = isReleasedPttLowStable(PIN_PTT_UHF);
  if (!vhfUnexpectedLow && !uhfUnexpectedLow) return;

  gLastReleasedPttWarnMs = millis();
  if (vhfUnexpectedLow) {
    Serial.println(F("[HANDOFF] WARNING: VHF PTT reads LOW while released; external drive/load is asserting TX."));
  }
  if (uhfUnexpectedLow) {
    Serial.println(F("[HANDOFF] WARNING: UHF PTT reads LOW while released; external drive/load is asserting TX."));
  }
}

void printCfgStateToken() {
#if ACTIVE_MODULE == MODULE_BOTH
  Serial.print(gVhfConfigured ? 'O' : 'F');
  if (!gVhfConfigured && gVhfFailStep > 0) {
    Serial.print('/');
    Serial.print(gVhfFailStep);
  }
  Serial.print(':');
  Serial.print(gUhfConfigured ? 'O' : 'F');
  if (!gUhfConfigured && gUhfFailStep > 0) {
    Serial.print('/');
    Serial.print(gUhfFailStep);
  }
#elif ACTIVE_MODULE == MODULE_VHF
  Serial.print(gVhfConfigured ? F("OK") : F("FAIL"));
  if (!gVhfConfigured && gVhfFailStep > 0) {
    Serial.print('/');
    Serial.print(gVhfFailStep);
  }
#else
  Serial.print(gUhfConfigured ? F("OK") : F("FAIL"));
  if (!gUhfConfigured && gUhfFailStep > 0) {
    Serial.print('/');
    Serial.print(gUhfFailStep);
  }
#endif
}

void printStatusFull(bool forceHeader) {
  int pd = digitalRead(PIN_PD);
  int hl = digitalRead(PIN_HL);
  int vptt = digitalRead(PIN_PTT_VHF);
  int uptt = digitalRead(PIN_PTT_UHF);
  int vsql = digitalRead(PIN_SQL_VHF);
  int usql = digitalRead(PIN_SQL_UHF);
  bool pttReq = (vptt == LOW) || (uptt == LOW);
  bool txConf = pttReq && (pd == HIGH);
  float vbat = readBatteryVoltage();

  if (forceHeader || ((gStatusRowsPrinted % STATUS_HEADER_REPEAT_ROWS) == 0)) {
    Serial.println(F("ms      cfg      PD     HL    VPTT UPTT VSQL USQL PTT TXCF TX_STATE         LED        VBAT"));
    Serial.println(F("------  -------  -----  ----  ---- ---- ---- ---- --- ---- --------------   ---------  ----"));
  }

  Serial.print(millis());
  Serial.print(F("  "));
  printCfgStateToken();
  Serial.print(F("  "));
  Serial.print(pd == HIGH ? F("ACT") : F("SLP"));
  Serial.print(F("   "));
  Serial.print(hl == HIGH ? F("HIGH") : F("LOW "));
  Serial.print(F("  "));
  Serial.print(vptt == LOW ? F("TX  ") : F("IDL "));
  Serial.print(F(" "));
  Serial.print(uptt == LOW ? F("TX  ") : F("IDL "));
  Serial.print(F(" "));
  Serial.print(vsql == HIGH ? F("OPEN") : F("CLOS"));
  Serial.print(F(" "));
  Serial.print(usql == HIGH ? F("OPEN") : F("CLOS"));
  Serial.print(F(" "));
  Serial.print(pttReq ? F("YES") : F("NO "));
  Serial.print(F(" "));
  Serial.print(txConf ? F("YES ") : F("NO  "));
  Serial.print(F(" "));
  Serial.print(txDetectStateName(gTxDetectState));
  Serial.print(F("   "));
  Serial.print(ledStateName(gCurrentLedState));
  Serial.print(F("  "));
  if (isBatteryVoltagePlausible(vbat)) Serial.print(vbat, 2);
  else Serial.print(F("N/A"));
  Serial.println();

  gStatusRowsPrinted++;
}

void printStatusCompactRow() {
  int pd = digitalRead(PIN_PD);
  int vptt = digitalRead(PIN_PTT_VHF);
  int uptt = digitalRead(PIN_PTT_UHF);
  int vsql = digitalRead(PIN_SQL_VHF);
  int usql = digitalRead(PIN_SQL_UHF);
  bool pttReq = (vptt == LOW) || (uptt == LOW);
  bool txConf = pttReq && (pd == HIGH);
  float vbat = readBatteryVoltage();

  if ((gStatusRowsPrinted % STATUS_HEADER_REPEAT_ROWS) == 0) {
    Serial.println(F("ms      PD   VHF  UHF  VSQL USQL PTT TXCF TX_STATE      LED       VBAT"));
    Serial.println(F("------  ---  ---  ---  ---- ---- --- ---- ------------  --------  ----"));
  }

  Serial.print(millis());
  Serial.print(F("  "));
  Serial.print(pd == HIGH ? F("ACT") : F("SLP"));
  Serial.print(F("  "));
  Serial.print(vptt == LOW ? F("TX ") : F("IDL"));
  Serial.print(F("  "));
  Serial.print(uptt == LOW ? F("TX ") : F("IDL"));
  Serial.print(F("  "));
  Serial.print(vsql == HIGH ? F("OPEN") : F("CLOS"));
  Serial.print(F(" "));
  Serial.print(usql == HIGH ? F("OPEN") : F("CLOS"));
  Serial.print(F(" "));
  Serial.print(pttReq ? F("YES") : F("NO "));
  Serial.print(F(" "));
  Serial.print(txConf ? F("YES ") : F("NO  "));
  Serial.print(F(" "));
  Serial.print(txDetectStateName(gTxDetectState));
  Serial.print(F("  "));
  Serial.print(ledStateName(gCurrentLedState));
  Serial.print(F("  "));
  if (isBatteryVoltagePlausible(vbat)) Serial.print(vbat, 2);
  else Serial.print(F("N/A"));
  Serial.println();

  gStatusRowsPrinted++;
}

void printStatusSnapshotFramed() {
  Serial.println();
  Serial.println(F("================ STATUS SNAPSHOT BEGIN ================"));
  printStatusFull(true);
  Serial.println(F("================= STATUS SNAPSHOT END ================="));
  Serial.println();
}

void printCurrentConfigSnapshot() {
  Serial.println();
  Serial.println(F("================ CFG SNAPSHOT BEGIN ================"));
  Serial.println(F("CFG (VHF/UHF):"));
  printCfgStateToken();
  Serial.println();

#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  Serial.print(F("VHF TX/RX: "));
  Serial.print(VHF_CONFIG.freq_tx, 4);
  Serial.print(F(" / "));
  Serial.println(VHF_CONFIG.freq_rx, 4);
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  Serial.print(F("UHF TX/RX: "));
  Serial.print(UHF_CONFIG.freq_tx, 4);
  Serial.print(F(" / "));
  Serial.println(UHF_CONFIG.freq_rx, 4);
#endif

  Serial.print(F("PD lock: "));
  Serial.println(gPdLockOn ? F("ON") : F("OFF"));
  Serial.print(F("HL override: "));
  if (gHlDiagOverrideActive) Serial.println(gHlDiagLevelHigh ? F("HIGH") : F("LOW"));
  else Serial.println(F("RELEASED_TO_PI"));

#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  Serial.print(F("VHF configured: "));
  Serial.print(gVhfConfigured ? F("YES") : F("NO"));
  if (!gVhfConfigured) {
    Serial.print(F("  <-- failStep="));
    Serial.print(gVhfFailStep);
    Serial.print(F("  (1=handshake 2=group 3=volume 4=filters)"));
  }
  Serial.println();
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  Serial.print(F("UHF configured: "));
  Serial.print(gUhfConfigured ? F("YES") : F("NO"));
  if (!gUhfConfigured) {
    Serial.print(F("  <-- failStep="));
    Serial.print(gUhfFailStep);
    Serial.print(F("  (1=handshake 2=group 3=volume 4=filters)"));
  }
  Serial.println();
  Serial.print(F("UHF mapping: "));
  Serial.println(currentUhfMappingName());
#endif

  printStatusFull(true);
  Serial.println(F("================= CFG SNAPSHOT END ================="));
  Serial.println();
}

void printDiagHelpDetailed() {
  pauseStatusLogs(6000);
  Serial.println();
  Serial.println(F("[DIAG] Commands:"));
  Serial.println(F("  h           help"));
  Serial.println(F("  s           one framed status snapshot"));
  Serial.println(F("  c           config snapshot"));
  Serial.println(F("  z t|c|f     periodic status mode toggle/compact/full"));
  Serial.println(F("  d t|o|n     PD lock toggle/on/off"));
  Serial.println(F("  l t|h|l|r   H/L toggle/high/low/release"));
  Serial.println(F("  r           retry startup config"));
  Serial.println(F("  mN          volume 1..8 (all active modules)"));
  Serial.println(F("  qN          squelch 0..8 (all active modules)"));
  Serial.println(F("  fXYZ        filters pre/high/low bits (e.g. f010)"));
  Serial.println(F("  g M D F     set module M(v|u) direction D(tx|rx) frequency F"));
  Serial.println(F("  v|u|b       local VHF/UHF/both PTT pulse"));
  Serial.println(F("  a M CMD     direct AT command to module M(v|u), bypassing DRA818 methods"));
  Serial.println(F("  k M         direct startup sequence check for module M(v|u)"));
  Serial.println(F("  x           raw UHF UART probe (AT+DMOCONNECT hex+ascii)"));
  Serial.println(F("  y           raw VHF UART probe (no typing during 1.5s window!)"));
  Serial.println();
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

  if (oldCompact && !gPeriodicStatusCompact) gStatusRowsPrinted = 0;
  return true;
}

void rawUhfUartProbe() {
  pauseStatusLogs(3500);

  selectUhfMapping(false);
  Serial.println(F("[UHF-PROBE] Preferred mapping: D11 RX, D10 TX"));
  printUhfUartIdleLevels();
  bool preferredOk = rawHandshakeProbe(*gActiveUhfSerial, F("UHF-PROBE/PREF"), 1500UL);

  if (!preferredOk) {
    selectUhfMapping(true);
    Serial.println(F("[UHF-PROBE] Fallback mapping: D10 RX, D11 TX"));
    printUhfUartIdleLevels();
    bool fallbackOk = rawHandshakeProbe(*gActiveUhfSerial, F("UHF-PROBE/FALLBACK"), 1500UL);
    if (!fallbackOk) {
      Serial.println(F("[UHF-PROBE] No response on either mapping -- check JP1/JP2 routing, D10/D11 continuity, or module power."));
    }
  }
}

// VHF is on hardware Serial (D0/D1), which is shared with USB.
// The probe flushes pending TX, drains stale RX, then sends AT+DMOCONNECT and
// collects the raw response.  Do NOT type in serial monitor during the 1.5 s window.
void rawVhfUartProbe() {
  Serial.println(F("[VHF-PROBE] Flush+drain, then AT+DMOCONNECT (no typing for 1.5 s)..."));
  Serial.flush();          // wait for pending debug TX to finish transmitting to module
  delay(80);               // allow module to process/respond to any stale bytes
  drainSerialInput(Serial); // discard module responses to stale input

  // Send probe command -- goes to VHF module RXD and to USB host (that is expected).
  Serial.print(F("AT+DMOCONNECT\r\n"));

  uint8_t rxBuf[64];
  uint8_t byteCount = 0;
  unsigned long deadline = millis() + 1500UL;
  while (millis() < deadline && byteCount < sizeof(rxBuf)) {
#if HARDWARE_WATCHDOG_ENABLE
    wdt_reset();
#endif
    if (Serial.available()) rxBuf[byteCount++] = (uint8_t)Serial.read();
  }

  // Print results after exchange window closes.
  Serial.print(F("[VHF-PROBE] rx hex:"));
  for (uint8_t i = 0; i < byteCount; i++) {
    Serial.print(' '); if (rxBuf[i] < 0x10) Serial.print('0'); Serial.print(rxBuf[i], HEX);
  }
  if (byteCount == 0) Serial.print(F(" (none)"));
  Serial.println();
  Serial.print(F("[VHF-PROBE] rx ascii: \""));
  for (uint8_t i = 0; i < byteCount; i++) {
    uint8_t b = rxBuf[i];
    if (b >= 0x20 && b < 0x7F) Serial.print((char)b);
    else if (b == '\r') Serial.print(F("\\r"));
    else if (b == '\n') Serial.print(F("\\n"));
    else { Serial.print(F("\\x")); if (b < 0x10) Serial.print('0'); Serial.print(b, HEX); }
  }
  Serial.println('"');
  Serial.print(F("[VHF-PROBE] "));
  Serial.print(byteCount);
  if (byteCount == 0) Serial.println(F(" byte(s) -- no response from VHF module"));
  else Serial.println(F(" byte(s) (may include USB echo if monitor active)"));
}

void retryStartupConfiguration() {
  Serial.println(F("[DIAG] Retrying radio config..."));

  gPdLockOn = true;
  applyPdOwnershipState(false);
  delay(PRE_CONFIG_DELAY_MS);

#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  gConsoleMuted = true;
  gVhfConfigured = configureRadio(gVhfRadio, VHF_CONFIG, F("VHF"), gVhfFailStep);
  gConsoleMuted = false;
#endif
#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  gUhfConfigured = configureUhfRadio(UHF_CONFIG, gUhfFailStep);
#endif

#if ACTIVE_MODULE == MODULE_VHF
  gConfigurationComplete = gVhfConfigured;
#elif ACTIVE_MODULE == MODULE_UHF
  gConfigurationComplete = gUhfConfigured;
#else
  gConfigurationComplete = gVhfConfigured && gUhfConfigured;
#endif

  if (gConfigurationComplete) {
    debugPrint(F("CFG"), F("Retry succeeded"));
    setLedState(LED_SUCCESS_FADE);
    releaseSharedLinesToPi();
  } else {
    debugPrint(F("CFG"), F("Retry FAILED"));
    setLedState(LED_FAILURE_BLINK);
  }

  printStatusSnapshotFramed();
}

void handleSerialDiagCommands() {
  uint8_t processed = 0;
  while (Serial.available() > 0 && processed < DIAG_MAX_BYTES_PER_LOOP) {
    char cmd = (char)Serial.read();
    processed++;

    if (cmd == '\n' || cmd == '\r' || cmd == ' ') continue;
    if (((uint8_t)cmd) < 0x20 || ((uint8_t)cmd) > 0x7E) continue;

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
          Serial.println(F("[DIAG] Usage: z <t|c|f>"));
          break;
        }
        Serial.print(F("[DIAG] Periodic mode -> "));
        Serial.println(gPeriodicStatusCompact ? F("COMPACT") : F("FULL"));
        break;
      }
      case 'd':
      case 'D': {
        char token[12];
        if (!readDiagArgToken(token, sizeof(token)) || !token[0] || !applyPdModeCommand(token[0])) {
          Serial.println(F("[DIAG] Usage: d <t|o|n>"));
          break;
        }
        printStatusSnapshotFramed();
        break;
      }
      case 'l':
      case 'L': {
        char token[12];
        if (!readDiagArgToken(token, sizeof(token)) || !token[0] || !applyHlModeCommand(token[0])) {
          Serial.println(F("[DIAG] Usage: l <t|h|l|r>"));
          break;
        }
        printStatusSnapshotFramed();
        break;
      }
      case 'r':
      case 'R':
        retryStartupConfiguration();
        break;
      case 'm':
      case 'M': {
        char token[12];
        uint8_t val = 0;
        if (!readDiagArgToken(token, sizeof(token)) || !parseUint8Token(token, 1, 8, val)) {
          Serial.println(F("[DIAG] Usage: mN or m N (N=1..8)"));
          break;
        }
        bool ok = applyVolumeAll(val);
        Serial.print(F("[DIAG] volume "));
        Serial.println(ok ? F("OK") : F("FAIL"));
        break;
      }
      case 'q':
      case 'Q': {
        char token[12];
        uint8_t val = 0;
        if (!readDiagArgToken(token, sizeof(token)) || !parseUint8Token(token, 0, 8, val)) {
          Serial.println(F("[DIAG] Usage: qN or q N (N=0..8)"));
          break;
        }
        bool ok = applySquelchAll(val);
        Serial.print(F("[DIAG] squelch "));
        Serial.println(ok ? F("OK") : F("FAIL"));
        break;
      }
      case 'f':
      case 'F': {
        char token[12];
        bool pre = false;
        bool high = false;
        bool low = false;
        if (!readDiagArgToken(token, sizeof(token)) || !parseFilterToken(token, pre, high, low)) {
          Serial.println(F("[DIAG] Usage: fXYZ"));
          break;
        }
        bool ok = applyFiltersAll(pre, high, low);
        Serial.print(F("[DIAG] filters "));
        Serial.println(ok ? F("OK") : F("FAIL"));
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
          Serial.println(F("[DIAG] Usage: g <v|u> <tx|rx> <freq_mhz>"));
          break;
        }

        bool setTx = false;
        if (dirTok[0] == 't' || dirTok[0] == 'T') setTx = true;
        else if (dirTok[0] == 'r' || dirTok[0] == 'R') setTx = false;
        else {
          Serial.println(F("[DIAG] Direction must be tx or rx"));
          break;
        }

        if (!parseFreqToken(freqTok, mhz)) {
          Serial.println(F("[DIAG] Invalid frequency"));
          break;
        }

        bool ok = applyFrequencyCommand(modTok[0], setTx, mhz);
        Serial.print(F("[DIAG] frequency "));
        Serial.println(ok ? F("OK") : F("FAIL"));
        break;
      }
      case 'v':
      case 'V':
        runLocalPttPulse(F("VHF"), PIN_PTT_VHF, DIAG_LOCAL_PTT_PULSE_MS);
        break;
      case 'u':
      case 'U':
        runLocalPttPulse(F("UHF"), PIN_PTT_UHF, DIAG_LOCAL_PTT_PULSE_MS);
        break;
      case 'b':
      case 'B':
        runLocalPttPulse(F("VHF"), PIN_PTT_VHF, DIAG_LOCAL_PTT_PULSE_MS);
        runLocalPttPulse(F("UHF"), PIN_PTT_UHF, DIAG_LOCAL_PTT_PULSE_MS);
        break;
      case 'a':
      case 'A': {
        char modTok[12];
        char cmdTok[80];
        if (!readDiagArgToken(modTok, sizeof(modTok)) || !readDiagArgToken(cmdTok, sizeof(cmdTok))) {
          Serial.println(F("[DIAG] Usage: a <v|u> <AT+...>"));
          break;
        }
        runDirectSingleCommand(modTok[0], cmdTok);
        break;
      }
      case 'k':
      case 'K': {
        char modTok[12];
        if (!readDiagArgToken(modTok, sizeof(modTok))) {
          Serial.println(F("[DIAG] Usage: k <v|u>"));
          break;
        }
        if (modTok[0] == 'v' || modTok[0] == 'V') {
          bool ok = runDirectConfigSequenceVhf();
          Serial.println(ok ? F("[VHF-DIRECT] Sequence PASS") : F("[VHF-DIRECT] Sequence FAIL"));
        } else if (modTok[0] == 'u' || modTok[0] == 'U') {
          bool ok = runDirectConfigSequenceUhf();
          Serial.println(ok ? F("[UHF-DIRECT] Sequence PASS") : F("[UHF-DIRECT] Sequence FAIL"));
        } else {
          Serial.println(F("[DIAG] Usage: k <v|u>"));
        }
        break;
      }
      case 'x':
      case 'X':
        rawUhfUartProbe();
        break;
      case 'y':
      case 'Y':
        rawVhfUartProbe();
        break;
      default:
        Serial.print(F("[DIAG] Unknown command: "));
        Serial.println(cmd);
        break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// SETUP / LOOP
//////////////////////////////////////////////////////////////////////////////

void setup() {
#if HARDWARE_WATCHDOG_ENABLE
  gResetCauseFlags = MCUSR;
  MCUSR = 0;
  wdt_reset();
  wdt_disable();
#endif

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(PIN_SQL_VHF, INPUT);
  pinMode(PIN_SQL_UHF, INPUT);
  pinMode(PIN_PTT_VHF, INPUT_PULLUP);
  pinMode(PIN_PTT_UHF, INPUT_PULLUP);

  gUhfSerialPreferred.begin(DRA_BAUD);
  gUhfSerialFallback.begin(DRA_BAUD);
  selectUhfMapping(false);

  driveSharedLinesForStartupConfig();
  setLedState(LED_CONFIG_ACTIVE);
  delay(PRE_CONFIG_DELAY_MS);

#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  // Configure VHF in a quiet UART window at the radio baud.
  // Keep console on the same baud afterward to avoid a mid-boot baud switch.
  Serial.begin(DRA_BAUD);
  delay(50);
  gConsoleMuted = true;
  gVhfConfigured = configureRadio(gVhfRadio, VHF_CONFIG, F("VHF"), gVhfFailStep);
  gConsoleMuted = false;
#endif

#if ACTIVE_MODULE == MODULE_UHF
  Serial.begin(SERIAL_BAUD);
  delay(400);
#else
  delay(400);
#endif
  gConsoleReady = true;

  Serial.println(F("HamWing_RadioConfig (ATmega328P parity port) starting..."));
#if ACTIVE_MODULE == MODULE_VHF
  Serial.println(F("Active modules: VHF"));
#elif ACTIVE_MODULE == MODULE_UHF
  Serial.println(F("Active modules: UHF"));
#else
  Serial.println(F("Active modules: VHF+UHF"));
#endif
  Serial.println(F("NOTE: VHF uses hardware Serial (D0/D1), shared with USB console."));
  Serial.println(F("NOTE: Startup VHF config runs in a quiet window before USB logging starts."));
#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  Serial.println(F("NOTE: Console remains at 9600 baud when VHF is active (shared UART)."));
  Serial.println(F("NOTE: Pre-banner AT traffic is expected while VHF config is running."));
#else
  Serial.println(F("NOTE: Console is at 9600 baud."));
#endif

#if ACTIVE_MODULE == MODULE_VHF || ACTIVE_MODULE == MODULE_BOTH
  Serial.print(F("[BOOT] Quiet-window VHF result: "));
  Serial.print(gVhfConfigured ? F("OK") : F("FAIL"));
  if (!gVhfConfigured && gVhfFailStep > 0) {
    Serial.print(F(" (failStep="));
    Serial.print(gVhfFailStep);
    Serial.print(')');
  }
  Serial.println();
#endif

#if HARDWARE_WATCHDOG_ENABLE
  if (gResetCauseFlags != 0) {
    Serial.print(F("[BOOT] Reset cause flags=0x"));
    Serial.println(gResetCauseFlags, HEX);
  }
#endif

  Serial.print(F("Startup configuration delay was "));
  Serial.print(PRE_CONFIG_DELAY_MS);
  Serial.println(F(" ms."));

#if ACTIVE_MODULE == MODULE_UHF || ACTIVE_MODULE == MODULE_BOTH
  gUhfConfigured = configureUhfRadio(UHF_CONFIG, gUhfFailStep);
#endif

#if ACTIVE_MODULE == MODULE_VHF
  gConfigurationComplete = gVhfConfigured;
#elif ACTIVE_MODULE == MODULE_UHF
  gConfigurationComplete = gUhfConfigured;
#else
  gConfigurationComplete = gVhfConfigured && gUhfConfigured;
#endif

  if (gConfigurationComplete) {
    debugPrint(F("CFG"), F("Startup radio configuration succeeded"));
    setLedState(LED_SUCCESS_FADE);
  } else {
    debugPrint(F("CFG"), F("Startup radio configuration FAILED"));
    gPdLockOn = true;
    setLedState(LED_FAILURE_BLINK);
  }

  releaseSharedLinesToPi();
  printDiagHelpDetailed();
  printStatusSnapshotFramed();

#if HARDWARE_WATCHDOG_ENABLE
  wdt_enable(HARDWARE_WATCHDOG_TIMEOUT);
  wdt_reset();
#endif
}

void loop() {
#if HARDWARE_WATCHDOG_ENABLE
  wdt_reset();
#endif

  updateLed();
  enforcePdLockIfEnabled();
  checkLowVoltageCutoff();
  handleSerialDiagCommands();

  int pd = digitalRead(PIN_PD);
  int vptt = digitalRead(PIN_PTT_VHF);
  int uptt = digitalRead(PIN_PTT_UHF);
  int vsql = digitalRead(PIN_SQL_VHF);
  int usql = digitalRead(PIN_SQL_UHF);

  if (gLastVhfPttSense < 0) gLastVhfPttSense = vptt;
  if (gLastUhfPttSense < 0) gLastUhfPttSense = uptt;
  if (gLastVhfSqlSense < 0) {
    gLastVhfSqlSense = vsql;
    if (vsql == HIGH) gVhfSqlOpenCount++;
  }
  if (gLastUhfSqlSense < 0) {
    gLastUhfSqlSense = usql;
    if (usql == HIGH) gUhfSqlOpenCount++;
  }

  if (vptt != gLastVhfPttSense) {
    gVhfPttEdgeCount++;
    gLastVhfPttSense = vptt;
    debugPrint(vptt == LOW ? F("PTT") : F("PTT"), vptt == LOW ? F("VHF PTT LOW") : F("VHF PTT HIGH"));
  }
  if (uptt != gLastUhfPttSense) {
    gUhfPttEdgeCount++;
    gLastUhfPttSense = uptt;
    debugPrint(uptt == LOW ? F("PTT") : F("PTT"), uptt == LOW ? F("UHF PTT LOW") : F("UHF PTT HIGH"));
  }
  if (vsql != gLastVhfSqlSense) {
    gLastVhfSqlSense = vsql;
    if (vsql == HIGH) gVhfSqlOpenCount++;
  }
  if (usql != gLastUhfSqlSense) {
    gLastUhfSqlSense = usql;
    if (usql == HIGH) gUhfSqlOpenCount++;
  }

  updateReleasedPttLowTimers(vptt, uptt);
  warnIfReleasedPttAsserted(vptt, uptt);

  bool pttAsserted = (vptt == LOW) || (uptt == LOW);
  if (gSharedLinesReleasedToPi) {
    // While released to Pi, only treat sustained low as asserted to suppress brief spikes.
    pttAsserted = isReleasedPttLowStable(PIN_PTT_VHF) || isReleasedPttLowStable(PIN_PTT_UHF);
    // If PD is asleep and lines are released, do not drive TX state machine into BLOCKED_PD_LOW on noise.
    if (pd == LOW) pttAsserted = false;
  }

  bool txStateChanged = updateTxDetectState(pttAsserted, pd == HIGH);
  if (txStateChanged) {
    char trn[128];
    snprintf_P(trn, sizeof(trn), PSTR("TX state -> %s"), txDetectStateName(gTxDetectState));
    debugPrintC(F("TRN"), trn);
  }

#if LOW_VOLTAGE_CUTOFF_ENABLE
  uint8_t desiredLed = gLowVoltageCutoffActive ? LED_LOW_VOLTAGE : ledStateForTxDetectState();
#else
  uint8_t desiredLed = ledStateForTxDetectState();
#endif
  if (!gConfigurationComplete) desiredLed = LED_FAILURE_BLINK;
  setLedState(desiredLed);

  checkStuckPttWatchdog(pd, vptt, uptt);

  if (millis() >= gStatusPauseUntilMs && (millis() - gLastStatusMs) > STATUS_ROW_INTERVAL_MS) {
    gLastStatusMs = millis();
    if (gPeriodicStatusCompact) printStatusCompactRow();
    else printStatusFull(false);
  }

  if ((millis() - gLastSqlLogMs) > 15000UL) {
    gLastSqlLogMs = millis();
    Serial.print(F("[SQL] open_count V/U="));
    Serial.print(gVhfSqlOpenCount);
    Serial.print('/');
    Serial.println(gUhfSqlOpenCount);
  }

  delay(25);
}
