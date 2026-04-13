// hamwing_TNC.ino  —  DRA818U one-time programmer for HamWing carrier board
//
// Adafruit Feather 328P
// Adafruit Feather 32u4 Basic
// Adafruit Feather 32u4 Bluefruit LE
//
// Runs once at power-up to configure the DRA818U via its AT command UART,
// then releases the shared PD line so the Raspberry Pi can take over
// runtime control (PTT, PD, HL) without conflict.
//
// ── HamWing wiring (schematic reference) ────────────────────────────────────
//
//  UHF path (ACTIVE_RADIO_PATH = RADIO_PATH_UHF)
//  Arduino             HamWing net      DRA818U pin
//  ──────────────────  ───────────────  ──────────
//  D10 (SoftSerial RX) ←── U_RXD (J5)  ←── TXD
//  D2  (SoftSerial TX) ──► U_TXD (J5)  ──► RXD
//  D3                  ──► PD    (J3)  ──► PD
//  A7  (analog input)  ←── PTT   net   ←── driven by Pi GPIO27
//  A1  (analog input)  ←── SQL   net   ←── DRA818 COS/SQL output
//
//  VHF path (ACTIVE_RADIO_PATH = RADIO_PATH_VHF)
//  Arduino             HamWing net      DRA818V pin
//  ──────────────────  ───────────────  ──────────
//  D0  (Serial RX)     ←── TXD   (J4)  ←── TXD    ← hardware Serial; shared with USB
//  D1  (Serial TX)     ──► RXD   (J4)  ──► RXD
//  D4                  ──► PD    (J3)  ──► PD    ← shared with Pi GPIO4 BCM
//  A6  (analog input)  ←── PTT   net   ←── driven by Pi GPIO27
//  A0  (analog input)  ←── SQL   net   ←── DRA818 COS/SQL output
//
//  NOTE: A6 and A7 (ADC6/ADC7) are analog-input-only on ATmega328P.
//  The Arduino cannot drive them; PTT is controlled solely by the Pi.
//
//  NOTE on VHF Serial Monitor: when VHF path is active, D0/D1 carry DRA818
//  AT traffic.  Serial Monitor output will be interleaved with AT commands.
//  Set the Serial Monitor baud rate to 9600.  No wires need to be
//  disconnected during sketch upload.
//
//  Requires the fatpat DRA818 library in addition to SoftwareSerial:
//    https://github.com/fatpat/arduino-dra818
//
// ── Raspberry Pi control lines (NOT driven by this sketch after setup) ───────
//
//  Pi BCM    Physical   Net          Function
//  ────────  ─────────  ───────────  ──────────────────────────────────────
//  GPIO 4    pin  7     PD           Power-down  (HIGH = active, LOW = sleep)
//  GPIO27    pin 13     PTT          Push-to-talk (LOW = TX keyed)
//  GPIO22    pin 15     HL           Power level  (LOW = low, HIGH = high)
//                                   pi_sstv.py initialises HL LOW and never
//                                   changes it — the payload always runs low power.
//  GPIO12    pin 32     AudioOut L   PWM audio ──┐
//  GPIO13    pin 33     AudioOut R   PWM audio ──┴─► RC filter ──► DRA818 mic
//
//  Audio path (hardware, no firmware involvement):
//    Pi GPIO12/13 ──► R1 270Ω ──► R2 150Ω/C1 33nF LPF ──► C2 10µF ──► DRA818 MIC
//    The 270Ω/150Ω divider attenuates Pi 3.3V audio to a safe mic level.
//    The 33nF cap sets LPF cutoff ≈11kHz.  C2 blocks DC bias.
//
// ── DRA818U AT command reference ────────────────────────────────────────────
//
//  AT+DMOCONNECT                          Handshake        → +DMOCONNECT:0
//  AT+DMOSETGROUP=BW,TXF,RXF,TxCS,SQ,RxCS  Freq/BW/CTCSS  → +DMOSETGROUP:0
//  AT+DMOSETVOLUME=N                      Vol 1–8          → +DMOSETVOLUME:0
//  AT+DMOSETFILTER=PRE,HIGH,LOW           Filters 0/1      → +DMOSETFILTER:0
//
//  Response ":0" = success, ":1" = failure for all SET commands.
// ────────────────────────────────────────────────────────────────────────────

#include <SoftwareSerial.h>
#include <DRA818.h>
#include <string.h>
#include <stdio.h>

// ── HamWing hardware roles ─────────────────────────────────────────────────
// Radio path selector:
//   RADIO_PATH_UHF -> configure UHF path defaults
//   RADIO_PATH_VHF -> configure VHF path defaults
#define RADIO_PATH_UHF 1
#define RADIO_PATH_VHF 0
#define ACTIVE_RADIO_PATH RADIO_PATH_UHF

// Power-down control lines (Arduino drives HIGH to wake radio for programming)
#define PIN_PD_VHF           4   // HamWing VHF PD — shared with Pi GPIO4 BCM
#define PIN_PD_UHF           3   // HamWing UHF PD — D3

// Squelch monitor inputs — analog-only on ATmega328P; use analogRead()
#define PIN_SQL_VHF          A0  // HamWing VHF SQL → ADC0 (PC0)
#define PIN_SQL_UHF          A1  // HamWing UHF SQL → ADC1 (PC1)

// PTT lines — A6/A7 (ADC6/ADC7) are input-only on ATmega328P; Pi drives these
#define PIN_PTT_VHF          A6  // HamWing VHF PTT → ADC6
#define PIN_PTT_UHF          A7  // HamWing UHF PTT → ADC7

// Raspberry Pi runtime role pins (BCM numbers — documented for complete board map)
#define PIN_PI_PTT           27  // Pi GPIO27, HamWing net: PTT
#define PIN_PI_HL            22  // Pi GPIO22, HamWing net: HL
#define PIN_PI_PD            4   // Pi GPIO4,  HamWing net: PD
#define PIN_PI_AUDIO_L       12  // Pi GPIO12, HamWing net: AudioOut L
#define PIN_PI_AUDIO_R       13  // Pi GPIO13, HamWing net: AudioOut R

// Arduino UART pins to HamWing radio connectors
#define PIN_UART_VHF_RX      0   // D0 (hardware Serial RX) ← VHF TXD
#define PIN_UART_VHF_TX      1   // D1 (hardware Serial TX) → VHF RXD
#define PIN_UART_UHF_RX      10  // D10 (SoftwareSerial RX) ← UHF U_RXD
#define PIN_UART_UHF_TX      2   // D2  (SoftwareSerial TX) → UHF U_TXD

#if ACTIVE_RADIO_PATH == RADIO_PATH_UHF
  #define DRA818_RX_PIN          PIN_UART_UHF_RX   // D10
  #define DRA818_TX_PIN          PIN_UART_UHF_TX   // D2
  #define DRA818_PD_PIN          PIN_PD_UHF        // D3
  #define DRA818_SQL_PIN         PIN_SQL_UHF       // A1
  #define DRA818_PTT_PIN         PIN_PTT_UHF       // A7
  #define DRA818_TYPE            DRA818_UHF
  #define ACTIVE_RADIO_LABEL     "UHF"
  #define DRA818_USES_SOFTSERIAL 1   // UHF: SoftwareSerial on D10/D2
  const float FREQ_TX = 434.5000;
  const float FREQ_RX = 434.5000;
#else
  #define DRA818_RX_PIN          PIN_UART_VHF_RX   // D0 — hardware Serial
  #define DRA818_TX_PIN          PIN_UART_VHF_TX   // D1 — hardware Serial
  #define DRA818_PD_PIN          PIN_PD_VHF        // D4
  #define DRA818_SQL_PIN         PIN_SQL_VHF       // A0
  #define DRA818_PTT_PIN         PIN_PTT_VHF       // A6
  #define DRA818_TYPE            DRA818_VHF
  #define ACTIVE_RADIO_LABEL     "VHF"
  #define DRA818_USES_SOFTSERIAL 0   // VHF: hardware Serial on D0/D1
  #warning "VHF path: hardware Serial (D0/D1) carries DRA818 AT traffic. Serial Monitor output will be interleaved."
  const float FREQ_TX = 144.3900;
  const float FREQ_RX = 144.3900;
#endif

// ── Serial ports ─────────────────────────────────────────────────────────────
// Hardware Serial (D0/D1/USB) is used for IDE Serial Monitor logging when the
// UHF path is active.  When VHF is active, D0/D1 carry DRA818 AT traffic and
// log output is interleaved on the same port.
#define LOG_BAUD      9600

// DRA818_STREAM is the Stream used for all DRA818 AT command I/O.
// UHF path: SoftwareSerial on D10/D2 (hardware Serial is free for logging).
// VHF path: hardware Serial on D0/D1 (no separate log port available).
#if DRA818_USES_SOFTSERIAL
  SoftwareSerial dra818Serial(DRA818_RX_PIN, DRA818_TX_PIN);
  #define DRA818_STREAM dra818Serial
#else
  #define DRA818_STREAM Serial
#endif

// ── Radio configuration  —  edit for your mission ────────────────────────────

// Frequency (MHz).  DRA818U range: 400.0000 – 470.0000
// Set TX and RX to the same frequency for simplex SSTV operation.
// FREQ_TX and FREQ_RX are selected by ACTIVE_RADIO_PATH above.

// Bandwidth: 0 = 12.5 kHz narrow,  1 = 25 kHz wide
// Use 1 (wide) for SSTV — the audio spans ~300 Hz to ~2.5 kHz.
const uint8_t BANDWIDTH = 1;

// CTCSS tone code (0 = none, 1–38 = standard tones).
// Use 0 for simplex SSTV with no repeater access tone.
const uint8_t TX_CTCSS  = 0;
const uint8_t RX_CTCSS  = 0;

// Squelch level: 0 = open/monitor, 1–8 = increasing threshold.
// 0 = open squelch is correct for a transmit-only HAB payload.
const uint8_t SQUELCH   = 0;

// Audio output volume: 1 (min) – 8 (max).
// Controls the DRA818 speaker/line-out output level only.
// Does NOT affect TX deviation — TX audio level is set by the
// hardware RC divider (R1/R2) between the Pi and the DRA818 mic input.
const uint8_t VOLUME    = 4;

// Filters: true = enabled, false = disabled.
//   FILTER_PRE:  pre/de-emphasis (designed for voice; rolls off high freqs)
//   FILTER_HIGH: high-pass filter (cuts sub-300 Hz; removes hum)
//   FILTER_LOW:  low-pass filter  (cuts above ~3 kHz)
//
// For SSTV and FSK data ALL three filters should be OFF to maintain
// a flat frequency response across the full SSTV audio passband.
const bool FILTER_PRE   = false;
const bool FILTER_HIGH  = false;
const bool FILTER_LOW   = false;


// ── Timing ──────────────────────────────────────────────────────────────────

// Time to wait after PD goes HIGH before sending the first AT command.
// 200 ms is sufficient for AT command acceptance.
// Note: pi_sstv.py uses RADIO_WAKE_DELAY_SECONDS = 1.0 before keying PTT,
// which is a longer margin appropriate for full TX-carrier readiness.
#define WAKE_DELAY_MS         200
#define SAFE_BOOT_HOLD_MS      25
#define RESPONSE_TIMEOUT_MS  2000
#define CMD_SPACING_MS         50
#define STEP_RETRY_COUNT        2
// ── Status LED ──────────────────────────────────────────────────────────────
// Uses the built-in LED (pin 13 on Uno/Nano) to indicate programming result.
//   Slow blink (1 Hz)  = programming complete OK
//   Fast blink (5 Hz)  = one or more commands failed

#define LED_PIN  LED_BUILTIN
static bool anyFailed = false;


// ── Helpers ──────────────────────────────────────────────────────────────────

void applySafeBootState() {
  // Keep radio in sleep while the MCU starts so boot glitches cannot key RF.
  pinMode(DRA818_PD_PIN, OUTPUT);
  digitalWrite(DRA818_PD_PIN, LOW);
  delay(SAFE_BOOT_HOLD_MS);
}

void releasePdToPi() {
  // Drive LOW before releasing so PD is not left undefined if Pi is late to boot.
  pinMode(DRA818_PD_PIN, OUTPUT);
  digitalWrite(DRA818_PD_PIN, LOW);
  pinMode(DRA818_PD_PIN, INPUT);
}

void wakeRadioForProgramming() {
  pinMode(DRA818_PD_PIN, OUTPUT);
  digitalWrite(DRA818_PD_PIN, HIGH);
  delay(WAKE_DELAY_MS);
}

void printRoleMap() {
  Serial.println(F("[HAMWING] Hardware roles:"));
  Serial.print(F("[HAMWING]   Active path : "));
  Serial.println(F(ACTIVE_RADIO_LABEL));
  Serial.print(F("[HAMWING]   UART RX     : D"));
  Serial.println(DRA818_RX_PIN);
  Serial.print(F("[HAMWING]   UART TX     : D"));
  Serial.println(DRA818_TX_PIN);
  Serial.print(F("[HAMWING]   PD pin      : D"));
  Serial.println(DRA818_PD_PIN);
  Serial.print(F("[HAMWING]   SQL pin     : A"));
  Serial.println(DRA818_SQL_PIN - A0);
  Serial.print(F("[HAMWING]   PTT pin     : A"));
  Serial.println(DRA818_PTT_PIN - A0);
  Serial.println(F("[HAMWING]   Pi runtime : PD=GPIO4, PTT=GPIO27, HL=GPIO22"));
}

void printWiringHint() {
  Serial.print(F("[HAMWING]     Check: D"));
  Serial.print(DRA818_RX_PIN);
  Serial.print(F(" <- DRA818 TXD, D"));
  Serial.print(DRA818_TX_PIN);
  Serial.println(F(" -> DRA818 RXD, power."));
}

void printCtcss(uint8_t code) {
  if (code < 10) {
    Serial.print(F("000"));
  } else {
    Serial.print(F("00"));
  }
  Serial.print(code);
}

bool reportStep(const __FlashStringHelper* label, bool ok) {
  Serial.print('[');
  Serial.print(label);
  Serial.println(ok ? F("] OK") : F("] FAILED"));
  if (!ok) anyFailed = true;
  return ok;
}

bool waitForToken(const char* expected, unsigned long timeout_ms) {
  char resp[96];
  size_t used = 0;
  resp[0] = '\0';
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (DRA818_STREAM.available()) {
      char c = (char)DRA818_STREAM.read();
      if (used < sizeof(resp) - 1) {
        resp[used++] = c;
        resp[used] = '\0';
      } else {
        memmove(resp, resp + 1, sizeof(resp) - 2);
        resp[sizeof(resp) - 2] = c;
        resp[sizeof(resp) - 1] = '\0';
      }
      if (strstr(resp, expected) != NULL) {
        return true;
      }
    }
  }
  return false;
}

bool sendFilterFallback(bool pre, bool high, bool low) {
  while (DRA818_STREAM.available()) DRA818_STREAM.read();

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+DMOSETFILTER=%d,%d,%d", pre ? 1 : 0, high ? 1 : 0, low ? 1 : 0);

  DRA818_STREAM.print(cmd);
  DRA818_STREAM.print("\r\n");
  bool ok = waitForToken("+DMOSETFILTER:0", RESPONSE_TIMEOUT_MS);
  delay(CMD_SPACING_MS);
  return ok;
}

bool configureRadio(DRA818& radio) {
  bool ok = true;

  // 1. Handshake — required first; confirms two-way comms with the module.
  Serial.println(F("[HAMWING] Step 1/4: Handshake"));
  if (!reportStep(F("CONNECT"), radio.handshake() == 1)) {
    return false;
  }

  // 2. Frequency group, bandwidth, CTCSS, squelch.
  //    Library clamps out-of-range values to module limits.
  Serial.println(F("[HAMWING] Step 2/4: Frequency group, bandwidth, CTCSS, squelch"));
  bool groupOk = false;
  for (uint8_t i = 0; i < STEP_RETRY_COUNT && !groupOk; ++i) {
    groupOk = (radio.group(BANDWIDTH, FREQ_TX, FREQ_RX, TX_CTCSS, SQUELCH, RX_CTCSS) == 1);
  }
  ok &= reportStep(F("GROUP"), groupOk);

  // 3. Audio output volume (does not affect TX deviation).
  Serial.println(F("[HAMWING] Step 3/4: Audio volume"));
  bool volumeOk = false;
  for (uint8_t i = 0; i < STEP_RETRY_COUNT && !volumeOk; ++i) {
    volumeOk = (radio.volume(VOLUME) == 1);
  }
  ok &= reportStep(F("VOLUME"), volumeOk);

  // 4. Filters — all OFF preserves flat AF response for SSTV/data.
  Serial.println(F("[HAMWING] Step 4/4: Filters"));
  bool filterOk = false;
  for (uint8_t i = 0; i < STEP_RETRY_COUNT && !filterOk; ++i) {
    filterOk = (radio.filters(FILTER_PRE, FILTER_HIGH, FILTER_LOW) == 1);
  }
  if (!filterOk) {
    Serial.println(F("[HAMWING] Library filter call failed, trying AT+DMOSETFILTER fallback..."));
    for (uint8_t i = 0; i < STEP_RETRY_COUNT && !filterOk; ++i) {
      filterOk = sendFilterFallback(FILTER_PRE, FILTER_HIGH, FILTER_LOW);
    }
  }
  ok &= reportStep(F("FILTER"), filterOk);

  return ok;
}


// ── setup()  —  runs once at power-up ────────────────────────────────────────

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(LOG_BAUD);
  Serial.println(F("\n=============================="));
  Serial.println(F("[HAMWING] DRA818U programmer"));
  Serial.println(F("=============================="));
  printRoleMap();

  applySafeBootState();

  // Wake the DRA818U. PD must be HIGH before the module accepts any commands.
  Serial.println(F("[HAMWING] Waking DRA818U (PD HIGH)..."));
  wakeRadioForProgramming();

  Serial.print(F("[HAMWING] Config — TX: "));
  Serial.print(FREQ_TX, 4);
  Serial.print(F(" MHz  RX: "));
  Serial.print(FREQ_RX, 4);
  Serial.print(F(" MHz  BW: "));
  Serial.println(BANDWIDTH == 1 ? F("25 kHz wide") : F("12.5 kHz narrow"));
  Serial.print(F("[HAMWING] Squelch: "));
  Serial.print(SQUELCH);
  Serial.print(F("  Volume: "));
  Serial.print(VOLUME);
  Serial.print(F("  CTCSS TX/RX: "));
  printCtcss(TX_CTCSS);
  Serial.print('/');
  printCtcss(RX_CTCSS);
  Serial.println();
  Serial.println(F("[HAMWING] Using fatpat DRA818 library calls for setup steps."));
  Serial.println();

  // Constructing DRA818 initialises the radio UART link at 9600 8N1.
  DRA818 dra(&DRA818_STREAM, DRA818_TYPE);

  if (!configureRadio(dra)) {
    // Release PD and report before entering the blink loop.
    releasePdToPi();
    Serial.println(F("[HAMWING] PD released to hi-Z."));
    Serial.println(F("[HAMWING] *** RESULT: FAILED — aborting remaining steps."));
    printWiringHint();
    Serial.println(F("[HAMWING]     LED: fast blink (5 Hz)."));
    return;
  }

  // Release PD to hi-Z so the Raspberry Pi can drive it via BCM GPIO4.
  // If Arduino held it HIGH permanently it would conflict with the Pi
  // asserting LOW to power-down the radio between transmissions.
  releasePdToPi();

  Serial.println(F("[HAMWING] PD released to hi-Z — Raspberry Pi may now take control."));
  Serial.println();
  if (anyFailed) {
    Serial.println(F("[HAMWING] *** RESULT: FAILED — one or more commands did not respond."));
    printWiringHint();
    Serial.println(F("[HAMWING]     Check baud: 9600."));
    Serial.println(F("[HAMWING]     LED: fast blink (5 Hz)."));
  } else {
    Serial.println(F("[HAMWING] *** RESULT: OK — all commands acknowledged."));
    Serial.println(F("[HAMWING]     LED: slow blink (1 Hz)."));
  }
}


// ── loop()  —  blink LED to report programming result ────────────────────────

void loop() {
  // Slow blink = OK,  fast blink = at least one command failed
  int period_ms = anyFailed ? 100 : 1000;
  digitalWrite(LED_PIN, HIGH);
  delay(period_ms);
  digitalWrite(LED_PIN, LOW);
  delay(period_ms);
}