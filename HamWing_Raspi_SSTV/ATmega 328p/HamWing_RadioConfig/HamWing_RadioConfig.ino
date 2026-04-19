/*
 * HamWing_RadioConfig.ino — ATmega328P port
 *
 * Purpose:
 *   - Configure one selected DRA818 path at startup (VHF or UHF)
 *   - Release PD control to Raspberry Pi after config
 *   - Monitor Pi activity (PD/PTT/SQL) and provide diagnostics over Serial
 *   - Include stuck-PTT detection, SQL edge logging, battery monitoring, watchdog
 *
 * Notes for ATmega328P:
 *   - UHF path uses SoftwareSerial (D10 RX, D2 TX)
 *   - VHF path uses hardware Serial (D0/D1), which shares USB logging lines
 *   - A6/A7 are input-only; this sketch detects stuck PTT but cannot force-unkey
 */

#include <SoftwareSerial.h>
#include <DRA818.h>
#include <string.h>
#include <stdio.h>
#include <avr/wdt.h>

//////////////////////////////////////////////////////////////////////////////
// BUILD CONFIGURATION
//////////////////////////////////////////////////////////////////////////////

#define RADIO_PATH_VHF 0
#define RADIO_PATH_UHF 1
#define ACTIVE_RADIO_PATH RADIO_PATH_UHF

#define DEBUG_MODE 1

// UART mapping
#define PIN_UART_VHF_RX 0
#define PIN_UART_VHF_TX 1
#define PIN_UART_UHF_RX 10
#define PIN_UART_UHF_TX 2

// HamWing control/sense lines
#define PIN_PD_VHF 4
#define PIN_PD_UHF 3
#define PIN_SQL_VHF A0
#define PIN_SQL_UHF A1
#define PIN_PTT_VHF A6
#define PIN_PTT_UHF A7
#define PIN_VBAT A3
#define STATUS_LED_PIN LED_BUILTIN

#if ACTIVE_RADIO_PATH == RADIO_PATH_UHF
  #define ACTIVE_PD_PIN  PIN_PD_UHF
  #define ACTIVE_SQL_PIN PIN_SQL_UHF
  #define ACTIVE_PTT_PIN PIN_PTT_UHF
  #define ACTIVE_DRA_TYPE DRA818_UHF
  #define ACTIVE_LABEL "UHF"
  #define USES_SOFTSERIAL 1
#else
  #define ACTIVE_PD_PIN  PIN_PD_VHF
  #define ACTIVE_SQL_PIN PIN_SQL_VHF
  #define ACTIVE_PTT_PIN PIN_PTT_VHF
  #define ACTIVE_DRA_TYPE DRA818_VHF
  #define ACTIVE_LABEL "VHF"
  #define USES_SOFTSERIAL 0
#endif

// Runtime options
#define SERIAL_BAUD 115200
#define DRA_BAUD 9600
#define DRA_STEP_RETRIES 3
#define DRA_AT_TIMEOUT_MS 700
#define PRE_CONFIG_DELAY_MS 500

#define STATUS_ROW_INTERVAL_MS 1000
#define DIAG_MAX_BYTES_PER_LOOP 32

#define LOW_VOLTAGE_CUTOFF_ENABLE 1
#define LOW_VOLTAGE_CUTOFF_V 3.30f
#define LOW_VOLTAGE_RECOVER_V 3.40f
#define LOW_VOLTAGE_CHECK_MS 5000

#define STUCK_PTT_WATCHDOG_ENABLE 1
#define STUCK_PTT_MAX_TX_MS (7UL * 60UL * 1000UL)

#define HARDWARE_WATCHDOG_ENABLE 1
#define HARDWARE_WATCHDOG_TIMEOUT WDTO_8S

//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIG
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

RadioConfig ACTIVE_CONFIG = {
#if ACTIVE_RADIO_PATH == RADIO_PATH_UHF
  440.8000,
  440.8000,
#else
  144.3900,
  144.3900,
#endif
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
// GLOBALS
//////////////////////////////////////////////////////////////////////////////

SoftwareSerial gUhfSerial(PIN_UART_UHF_RX, PIN_UART_UHF_TX);

#if USES_SOFTSERIAL
DRA818 gRadio((Stream *)&gUhfSerial, ACTIVE_DRA_TYPE);
#else
DRA818 gRadio((Stream *)&Serial, ACTIVE_DRA_TYPE);
#endif

bool gConfigured = false;
bool gSharedReleasedToPi = false;
bool gPdLockOn = false;

bool gLowVoltageCutoffActive = false;
unsigned long gLastVbatCheckMs = 0;
unsigned long gLastStatusMs = 0;
unsigned long gLastSqlLogMs = 0;

int gLastPttSense = -1;
int gLastSqlSense = -1;
uint32_t gPttEdgeCount = 0;
uint32_t gSqlOpenCount = 0;

unsigned long gTxConfirmedSinceMs = 0;
bool gStuckPttAlertEmitted = false;

//////////////////////////////////////////////////////////////////////////////
// HELPERS
//////////////////////////////////////////////////////////////////////////////

void debugPrint(const char *label, const char *msg) {
#if DEBUG_MODE
  Serial.print("[");
  Serial.print(label);
  Serial.print("] ");
  Serial.println(msg);
#else
  (void)label;
  (void)msg;
#endif
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

bool sendATExpect(Stream &port, const char *cmd, const char *expected, unsigned long timeoutMs) {
  drainSerialInput(port);
  port.print(cmd);
  port.print("\r\n");

  char window[128];
  size_t used = 0;
  window[0] = '\0';

  unsigned long start = millis();
  while ((millis() - start) < timeoutMs) {
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

      if (strstr(window, expected) != NULL) {
        return true;
      }
    }
  }

  return false;
}

bool setGroupFallbackRaw(DRA818 &radio, const RadioConfig &config) {
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

  return sendATExpect(*radio.serial, groupCmd, "DMOSETGROUP:0", DRA_AT_TIMEOUT_MS);
}

bool setVolumeFallbackRaw(DRA818 &radio, uint8_t volume) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+DMOSETVOLUME=%u", volume);
  return sendATExpect(*radio.serial, cmd, "DMOSETVOLUME:0", DRA_AT_TIMEOUT_MS);
}

bool setFilterFallbackRaw(DRA818 &radio, bool pre, bool high, bool low) {
  char cmd[40];
  snprintf(cmd, sizeof(cmd), "AT+SETFILTER=%u,%u,%u", pre ? 1 : 0, high ? 1 : 0, low ? 1 : 0);
  return sendATExpect(*radio.serial, cmd, "DMOSETFILTER:0", DRA_AT_TIMEOUT_MS);
}

bool configureRadio(DRA818 &radio, RadioConfig &cfg) {
  bool ok = false;

  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    ok = (radio.handshake() == 1);
  }
  if (!ok) {
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      ok = sendATExpect(*radio.serial, "AT+DMOCONNECT", "DMOCONNECT:0", DRA_AT_TIMEOUT_MS);
    }
  }
  if (!ok) return false;

  ok = false;
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    ok = (radio.group(cfg.bandwidth, cfg.freq_tx, cfg.freq_rx, cfg.ctcss_tx, cfg.squelch, cfg.ctcss_rx) == 1);
  }
  if (!ok) {
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      ok = setGroupFallbackRaw(radio, cfg);
    }
  }
  if (!ok) return false;

  ok = false;
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    ok = (radio.volume(cfg.volume) == 1);
  }
  if (!ok) {
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      ok = setVolumeFallbackRaw(radio, cfg.volume);
    }
  }
  if (!ok) return false;

  ok = false;
  for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
    ok = (radio.filters(cfg.filter_pre, cfg.filter_high, cfg.filter_low) == 1);
  }
  if (!ok) {
    for (uint8_t i = 0; i < DRA_STEP_RETRIES && !ok; ++i) {
      ok = setFilterFallbackRaw(radio, cfg.filter_pre, cfg.filter_high, cfg.filter_low);
    }
  }
  return ok;
}

void applyPdOwnershipState() {
  if (gPdLockOn) {
    pinMode(ACTIVE_PD_PIN, OUTPUT);
    digitalWrite(ACTIVE_PD_PIN, HIGH);
  } else {
    pinMode(ACTIVE_PD_PIN, INPUT_PULLUP);
  }
}

void releaseSharedLinesToPi() {
  if (gSharedReleasedToPi) return;
  applyPdOwnershipState();
  gSharedReleasedToPi = true;
  debugPrint("HANDOFF", "PD released to Pi (pullup mode)");
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
  const float dividerRatio = 2.0f;
  return (adc / 1023.0f) * vcc * dividerRatio;
}

void checkLowVoltageCutoff() {
#if LOW_VOLTAGE_CUTOFF_ENABLE
  unsigned long now = millis();
  if ((now - gLastVbatCheckMs) < LOW_VOLTAGE_CHECK_MS) return;
  gLastVbatCheckMs = now;

  float vbat = readBatteryVoltage();
  if (!gLowVoltageCutoffActive && vbat < LOW_VOLTAGE_CUTOFF_V) {
    gLowVoltageCutoffActive = true;
    pinMode(ACTIVE_PD_PIN, OUTPUT);
    digitalWrite(ACTIVE_PD_PIN, LOW);
    debugPrint("LVCO", "Low voltage cutoff active: PD forced LOW");
  } else if (gLowVoltageCutoffActive && vbat >= LOW_VOLTAGE_RECOVER_V) {
    gLowVoltageCutoffActive = false;
    applyPdOwnershipState();
    debugPrint("LVCO", "Voltage recovered: PD ownership restored");
  }
#endif
}

void updateStuckPttWatchdog(int pttSense, int pdSense) {
#if STUCK_PTT_WATCHDOG_ENABLE
  bool txConfirmed = (pttSense == LOW) && (pdSense == HIGH);
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
    debugPrint("STUCK_PTT", "TX held > 7 min. ATmega can only alert (PTT line is input-only on A6/A7)");
  }
#else
  (void)pttSense;
  (void)pdSense;
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
    if (c == ' ' || c == '\t') Serial.read(); else break;
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

void printStatus() {
  float vbat = readBatteryVoltage();
  int pd = digitalRead(ACTIVE_PD_PIN);
  int ptt = digitalRead(ACTIVE_PTT_PIN);
  int sql = digitalRead(ACTIVE_SQL_PIN);

  Serial.print("[STAT] ms=");
  Serial.print(millis());
  Serial.print(" mod=");
  Serial.print(ACTIVE_LABEL);
  Serial.print(" cfg=");
  Serial.print(gConfigured ? "OK" : "FAIL");
  Serial.print(" pd=");
  Serial.print(pd == HIGH ? "ACTIVE" : "SLEEP");
  Serial.print(" ptt=");
  Serial.print(ptt == LOW ? "TX_REQ" : "IDLE");
  Serial.print(" sql=");
  Serial.print(sql == HIGH ? "OPEN" : "CLOSED");
  Serial.print(" vbat=");
  Serial.println(vbat, 2);
}

void printHelp() {
  Serial.println();
  Serial.println("[DIAG] Commands:");
  Serial.println("  h         help");
  Serial.println("  s         one status row");
  Serial.println("  d t/o/n   PD lock toggle/on/off");
  Serial.println("  mN        volume 1..8");
  Serial.println("  qN        squelch 0..8");
  Serial.println("  fXYZ      filters bits (e.g. f010)");
  Serial.println("  g tx F    set TX frequency MHz");
  Serial.println("  g rx F    set RX frequency MHz");
  Serial.println();
}

void handleSerialDiagCommands() {
  uint8_t processed = 0;
  while (Serial.available() > 0 && processed < DIAG_MAX_BYTES_PER_LOOP) {
    char cmd = (char)Serial.read();
    processed++;
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') continue;

    switch (cmd) {
      case 'h':
      case 'H':
        printHelp();
        break;
      case 's':
      case 'S':
        printStatus();
        break;
      case 'd':
      case 'D': {
        char tok[8];
        if (!readDiagArgToken(tok, sizeof(tok)) || !tok[0]) {
          Serial.println("[DIAG] Usage: d <t|o|n>");
          break;
        }
        if (tok[0] == 't' || tok[0] == 'T') gPdLockOn = !gPdLockOn;
        else if (tok[0] == 'o' || tok[0] == 'O' || tok[0] == '1') gPdLockOn = true;
        else if (tok[0] == 'n' || tok[0] == 'N' || tok[0] == '0') gPdLockOn = false;
        else {
          Serial.println("[DIAG] Usage: d <t|o|n>");
          break;
        }
        applyPdOwnershipState();
        Serial.print("[DIAG] PD lock -> ");
        Serial.println(gPdLockOn ? "ON" : "OFF");
        break;
      }
      case 'm':
      case 'M': {
        char tok[12];
        uint8_t val = 0;
        if (!readDiagArgToken(tok, sizeof(tok)) || !parseUint8Token(tok, 1, 8, val)) {
          Serial.println("[DIAG] Usage: mN or m N (N=1..8)");
          break;
        }
        bool ok = (gRadio.volume(val) == 1) || setVolumeFallbackRaw(gRadio, val);
        if (ok) ACTIVE_CONFIG.volume = val;
        Serial.print("[DIAG] volume ");
        Serial.println(ok ? "OK" : "FAIL");
        break;
      }
      case 'q':
      case 'Q': {
        char tok[12];
        uint8_t val = 0;
        if (!readDiagArgToken(tok, sizeof(tok)) || !parseUint8Token(tok, 0, 8, val)) {
          Serial.println("[DIAG] Usage: qN or q N (N=0..8)");
          break;
        }
        RadioConfig cand = ACTIVE_CONFIG;
        cand.squelch = val;
        bool ok = (gRadio.group(cand.bandwidth, cand.freq_tx, cand.freq_rx, cand.ctcss_tx, cand.squelch, cand.ctcss_rx) == 1) ||
                  setGroupFallbackRaw(gRadio, cand);
        if (ok) ACTIVE_CONFIG = cand;
        Serial.print("[DIAG] squelch ");
        Serial.println(ok ? "OK" : "FAIL");
        break;
      }
      case 'f':
      case 'F': {
        char tok[12];
        bool pre = false;
        bool high = false;
        bool low = false;
        if (!readDiagArgToken(tok, sizeof(tok)) || !parseFilterToken(tok, pre, high, low)) {
          Serial.println("[DIAG] Usage: fXYZ (0/1 bits)");
          break;
        }
        bool ok = (gRadio.filters(pre, high, low) == 1) || setFilterFallbackRaw(gRadio, pre, high, low);
        if (ok) {
          ACTIVE_CONFIG.filter_pre = pre;
          ACTIVE_CONFIG.filter_high = high;
          ACTIVE_CONFIG.filter_low = low;
        }
        Serial.print("[DIAG] filters ");
        Serial.println(ok ? "OK" : "FAIL");
        break;
      }
      case 'g':
      case 'G': {
        char dirTok[12];
        char freqTok[20];
        float mhz = 0.0f;
        if (!readDiagArgToken(dirTok, sizeof(dirTok)) || !readDiagArgToken(freqTok, sizeof(freqTok))) {
          Serial.println("[DIAG] Usage: g <tx|rx> <freq_mhz>");
          break;
        }
        if (!parseFreqToken(freqTok, mhz)) {
          Serial.println("[DIAG] Invalid frequency");
          break;
        }
        RadioConfig cand = ACTIVE_CONFIG;
        if (dirTok[0] == 't' || dirTok[0] == 'T') cand.freq_tx = mhz;
        else if (dirTok[0] == 'r' || dirTok[0] == 'R') cand.freq_rx = mhz;
        else {
          Serial.println("[DIAG] Direction must be tx or rx");
          break;
        }
        bool ok = (gRadio.group(cand.bandwidth, cand.freq_tx, cand.freq_rx, cand.ctcss_tx, cand.squelch, cand.ctcss_rx) == 1) ||
                  setGroupFallbackRaw(gRadio, cand);
        if (ok) ACTIVE_CONFIG = cand;
        Serial.print("[DIAG] freq ");
        Serial.println(ok ? "OK" : "FAIL");
        break;
      }
      default:
        Serial.print("[DIAG] Unknown command: ");
        Serial.println(cmd);
        break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// SETUP / LOOP
//////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(PIN_SQL_VHF, INPUT);
  pinMode(PIN_SQL_UHF, INPUT);
  pinMode(PIN_PTT_VHF, INPUT);
  pinMode(PIN_PTT_UHF, INPUT);

  Serial.begin(SERIAL_BAUD);
  delay(400);

  Serial.println();
  Serial.println("HamWing_RadioConfig (ATmega328P) starting...");
  Serial.print("Active path: ");
  Serial.println(ACTIVE_LABEL);

#if USES_SOFTSERIAL
  gUhfSerial.begin(DRA_BAUD);
  gUhfSerial.listen();
#endif

#if HARDWARE_WATCHDOG_ENABLE
  wdt_enable(HARDWARE_WATCHDOG_TIMEOUT);
#endif

  // Claim PD for startup config.
  pinMode(ACTIVE_PD_PIN, OUTPUT);
  digitalWrite(ACTIVE_PD_PIN, HIGH);
  delay(PRE_CONFIG_DELAY_MS);

  gConfigured = configureRadio(gRadio, ACTIVE_CONFIG);
  if (gConfigured) {
    debugPrint("CFG", "Startup radio configuration succeeded");
    analogWrite(STATUS_LED_PIN, 180);
  } else {
    debugPrint("CFG", "Startup radio configuration FAILED");
    analogWrite(STATUS_LED_PIN, 255);
  }

  releaseSharedLinesToPi();
  printHelp();
  printStatus();
}

void loop() {
#if HARDWARE_WATCHDOG_ENABLE
  wdt_reset();
#endif

  handleSerialDiagCommands();
  checkLowVoltageCutoff();

  int pttSense = digitalRead(ACTIVE_PTT_PIN);
  int sqlSense = digitalRead(ACTIVE_SQL_PIN);
  int pdSense = digitalRead(ACTIVE_PD_PIN);

  if (gLastPttSense < 0) gLastPttSense = pttSense;
  if (gLastSqlSense < 0) gLastSqlSense = sqlSense;

  if (pttSense != gLastPttSense) {
    gLastPttSense = pttSense;
    gPttEdgeCount++;
    debugPrint("PTT", pttSense == LOW ? "PTT asserted (LOW)" : "PTT released (HIGH)");
  }

  if (sqlSense != gLastSqlSense) {
    gLastSqlSense = sqlSense;
    if (sqlSense == HIGH) gSqlOpenCount++;
    debugPrint("SQL", sqlSense == HIGH ? "SQL OPEN" : "SQL CLOSED");
  }

  updateStuckPttWatchdog(pttSense, pdSense);

  if (millis() - gLastStatusMs >= STATUS_ROW_INTERVAL_MS) {
    gLastStatusMs = millis();
    printStatus();
  }

  // Additional low-rate SQL stats line to keep field logs concise.
  if (millis() - gLastSqlLogMs >= 15000UL) {
    gLastSqlLogMs = millis();
    Serial.print("[SQL] open_count=");
    Serial.println(gSqlOpenCount);
  }

  // LED behavior: heartbeat when configured, fast blink when not configured.
  if (gConfigured) {
    analogWrite(STATUS_LED_PIN, (millis() / 8) % 255);
  } else {
    digitalWrite(STATUS_LED_PIN, ((millis() / 100) % 2) ? HIGH : LOW);
  }

  delay(25);
}