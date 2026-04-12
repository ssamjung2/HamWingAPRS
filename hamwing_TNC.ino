// hamwing_TNC.ino  —  DRA818U one-time programmer for HamWing carrier board
//
// Runs once at power-up to configure the DRA818U via its AT command UART,
// then releases the shared PD line so the Raspberry Pi can take over
// runtime control (PTT, PD, HL) without conflict.
//
// ── HamWing wiring (schematic reference) ────────────────────────────────────
//
//  Arduino             HamWing net     DRA818U pin
//  ─────────────────   ─────────────   ───────────
//  D2  (SoftSerial RX) ←── TXD  (J5)  ←── TXD
//  D3  (SoftSerial TX) ──► RXD  (J5)  ──► RXD
//  D4                  ──► PD   (J3)  ──► PD   ← shared with Pi GPIO4 BCM
//                                               Released to hi-Z after setup()
//
//  NOTE: The DRA818 UART runs on SoftwareSerial (D2/D3) so that the hardware
//  Serial port (D0/D1/USB) stays free for IDE Serial Monitor logging.
//  Rewire J5: DRA818 TXD → D2, DRA818 RXD → D3  (was D0/D1).
//  Set the Serial Monitor baud rate to 115200.  No wires need to be
//  disconnected during sketch upload.
//
//  Requires the "DRA818" library by Jerome LOYET (fatpat) — install via
//  Arduino IDE Library Manager: Sketch → Include Library → Manage Libraries
//  then search for "DRA818" and install.
//
// ── Raspberry Pi control lines (NOT driven by this sketch after setup) ───────
//
//  Pi BCM    Physical   Net          Function
//  ────────  ─────────  ───────────  ──────────────────────────────────────
//  GPIO 4    pin  7     PD           Power-down  (HIGH = active, LOW = sleep)
//  GPIO27    pin 13     PTT          Push-to-talk (LOW = TX keyed)
//  GPIO22    pin 15     HL           Power level  (LOW = low, HIGH = high)
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
//  AT+DMOVERQ                             Version query    → version string
//
//  Response ":0" = success, ":1" = failure for all SET commands.
// ────────────────────────────────────────────────────────────────────────────


// Defining DRA818_DEBUG before the include enables the library's built-in
// AT command trace, which is piped to the Serial Monitor via set_log() below.
#define DRA818_DEBUG
#include <DRA818.h>   // install via Library Manager: search "DRA818" by fatpat

// ── Serial ports ─────────────────────────────────────────────────────────────
// Hardware Serial (D0/D1/USB) is used exclusively for IDE Serial Monitor logging.
#define LOG_BAUD      115200

// SoftwareSerial carries all DRA818U AT command traffic.
#define DRA818_RX_PIN  2   // Arduino D2  ←── DRA818 TXD (J5)
#define DRA818_TX_PIN  3   // Arduino D3  ──► DRA818 RXD (J5)
#define DRA818_BAUD    9600

SoftwareSerial dra818Serial(DRA818_RX_PIN, DRA818_TX_PIN);
DRA818        *dra;         // pointer to the library object; created in setup()

// ── Pin assignments ──────────────────────────────────────────────────────────

#define PIN_PD          4   // PD shared with Pi GPIO4 BCM; released to INPUT after setup


// ── Radio configuration  —  edit for your mission ────────────────────────────

// Frequency (MHz).  DRA818U range: 400.0000 – 470.0000
// Set TX and RX to the same frequency for simplex SSTV operation.
const float FREQ_TX     = 434.5000;
const float FREQ_RX     = 434.5000;

// Bandwidth: DRA818_12K5 = 12.5 kHz narrow,  DRA818_25K = 25 kHz wide
// Use DRA818_25K (wide) for SSTV — the audio spans ~300 Hz to ~2.5 kHz.
const uint8_t BANDWIDTH = DRA818_25K;

// CTCSS tone code: 0 = none, 1–38 = standard CTCSS tones.
// Use 0 for simplex SSTV with no repeater access tone.
const uint8_t TX_CTCSS  = 0;
const uint8_t RX_CTCSS  = 0;

// Squelch level: 0 = open/monitor, 1–8 = increasing threshold.
// 0 = open squelch is correct for a transmit-only HAB payload.
const int SQUELCH       = 0;

// Audio output volume: 1 (min) – 8 (max).
// Controls the DRA818 speaker/line-out output level only.
// Does NOT affect TX deviation — TX audio level is set by the
// hardware RC divider (R1/R2) between the Pi and the DRA818 mic input.
const int VOLUME        = 4;

// Filters: 1 = enabled, 0 = disabled.
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

#define WAKE_DELAY_MS  200   // wait after PD HIGH before first command


// ── Status LED ──────────────────────────────────────────────────────────────
// Uses the built-in LED (pin 13 on Uno/Nano) to indicate programming result.
//   Slow blink (1 Hz)  = programming complete OK
//   Fast blink (5 Hz)  = one or more commands failed

#define LED_PIN  LED_BUILTIN
static bool anyFailed = false;


// ── Helpers ──────────────────────────────────────────────────────────────────
// AT command handling and Serial Monitor logging are provided by the DRA818
// library.  No manual helpers are needed.


// ── setup()  —  runs once at power-up ────────────────────────────────────────

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(LOG_BAUD);
  Serial.println(F("\n=============================="));
  Serial.println(F("[HAMWING] DRA818U programmer"));
  Serial.println(F("=============================="));

  // Wake the DRA818U.  PD must be HIGH before the module accepts any commands.
  Serial.println(F("[HAMWING] Waking DRA818U (PD HIGH)..."));
  pinMode(PIN_PD, OUTPUT);
  digitalWrite(PIN_PD, HIGH);
  delay(WAKE_DELAY_MS);

  Serial.print(F("[HAMWING] Config — TX: "));
  Serial.print(FREQ_TX, 4);
  Serial.print(F(" MHz  RX: "));
  Serial.print(FREQ_RX, 4);
  Serial.print(F(" MHz  BW: "));
  Serial.println(BANDWIDTH == DRA818_25K ? F("25 kHz wide") : F("12.5 kHz narrow"));
  Serial.print(F("[HAMWING] Squelch: "));
  Serial.print(SQUELCH);
  Serial.print(F("  Volume: "));
  Serial.print(VOLUME);
  Serial.print(F("  CTCSS TX/RX: "));
  Serial.print(TX_CTCSS);
  Serial.print('/');
  Serial.println(RX_CTCSS);
  Serial.println(F("[HAMWING] DRA818 library AT trace enabled — raw AT traffic shown below."));
  Serial.println();

  // Initialise the DRA818 library on the SoftwareSerial port.
  // set_log() pipes the library's AT command trace to the Serial Monitor.
  dra818Serial.begin(DRA818_BAUD);
  dra = new DRA818(&dra818Serial, DRA818_UHF);
  dra->set_log(&Serial);

  // 1. Handshake — required first; confirms two-way comms with the module.
  //    The library retries up to 3 times with a 2 s timeout each.
  Serial.println(F("[HAMWING] Step 1/4: Handshake"));
  if (dra->handshake()) {
    Serial.println(F("[HAMWING] Handshake OK"));
  } else {
    Serial.println(F("[HAMWING] *** Handshake FAILED — check D2<-TXD, D3->RXD wiring and power."));
    anyFailed = true;
  }

  // 2. Frequency group, bandwidth, CTCSS, squelch.
  Serial.println(F("[HAMWING] Step 2/4: Frequency group, bandwidth, CTCSS, squelch"));
  if (dra->group(BANDWIDTH, FREQ_TX, FREQ_RX, TX_CTCSS, SQUELCH, RX_CTCSS)) {
    Serial.println(F("[HAMWING] Group OK"));
  } else {
    Serial.println(F("[HAMWING] *** Group FAILED"));
    anyFailed = true;
  }

  // 3. Audio output volume (does not affect TX deviation).
  Serial.println(F("[HAMWING] Step 3/4: Audio volume"));
  if (dra->volume(VOLUME)) {
    Serial.println(F("[HAMWING] Volume OK"));
  } else {
    Serial.println(F("[HAMWING] *** Volume FAILED"));
    anyFailed = true;
  }

  // 4. Filters — all OFF preserves flat AF response for SSTV/data.
  Serial.println(F("[HAMWING] Step 4/4: Filters"));
  if (dra->filters(FILTER_PRE, FILTER_HIGH, FILTER_LOW)) {
    Serial.println(F("[HAMWING] Filters OK"));
  } else {
    Serial.println(F("[HAMWING] *** Filters FAILED"));
    anyFailed = true;
  }

  // Release PD to hi-Z so the Raspberry Pi can drive it via BCM GPIO4.
  // If Arduino held it HIGH permanently it would conflict with the Pi
  // asserting LOW to power-down the radio between transmissions.
  pinMode(PIN_PD, INPUT);

  Serial.println(F("[HAMWING] PD released to hi-Z — Raspberry Pi may now take control."));
  Serial.println();
  if (anyFailed) {
    Serial.println(F("[HAMWING] *** RESULT: FAILED — one or more commands did not respond."));
    Serial.println(F("[HAMWING]     Check: D2 <- DRA818 TXD, D3 -> DRA818 RXD, baud 9600."));
    Serial.println(F("[HAMWING]     LED: fast blink (5 Hz)."));
  } else {
    Serial.println(F("[HAMWING] *** RESULT: OK — all commands acknowledged."));
    Serial.println(F("[HAMWING]     LED: slow blink (1 Hz)."));
  }
}


// ── loop()  —  blink LED to report programming result ────────────────────────

void loop() {
  // Slow blink = OK,  fast blink = at least one command failed
  int period_ms = anyFailed ? 100 : 500;
  digitalWrite(LED_PIN, HIGH);
  delay(period_ms);
  digitalWrite(LED_PIN, LOW);
  delay(period_ms);
}