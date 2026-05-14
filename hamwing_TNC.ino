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
//  D0  (Serial RX) ←── TXD  (J5)  ←── TXD
//  D1  (Serial TX) ──► RXD  (J5)  ──► RXD
//  D4              ──► PD   (J3)  ──► PD       ← shared with Pi GPIO4 BCM
//                                               Released to hi-Z after setup()
//
//  NOTE: Arduino D0/D1 are the hardware serial pins and share the USB
//  programming interface on Uno/Nano.  Disconnect the DRA818 RXD line
//  while uploading this sketch, then reconnect before power-cycling.
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


// ── Pin assignments ──────────────────────────────────────────────────────────

#define PIN_PD          4   // PD shared with Pi GPIO4 BCM; released to INPUT after setup


// ── Radio configuration  —  edit for your mission ────────────────────────────

// Frequency (MHz).  DRA818U range: 400.0000 – 470.0000
// Set TX and RX to the same frequency for simplex SSTV operation.
const float FREQ_TX     = 434.5000;
const float FREQ_RX     = 434.5000;

// Bandwidth: 0 = 12.5 kHz narrow,  1 = 25 kHz wide
// Use 25 kHz (wide) for SSTV — the audio spans ~300 Hz to ~2.5 kHz.
const int BANDWIDTH     = 1;

// CTCSS tone code ("0000" = none, "0001"–"0038" = standard tones).
// Use "0000" for simplex SSTV with no repeater access tone.
const char* TX_CTCSS    = "0000";
const char* RX_CTCSS    = "0000";

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
const int FILTER_PRE    = 0;
const int FILTER_HIGH   = 0;
const int FILTER_LOW    = 0;


// ── Timing ──────────────────────────────────────────────────────────────────

#define DRA818_BAUD          9600
#define WAKE_DELAY_MS         200   // wait after PD HIGH before first command
#define RESPONSE_TIMEOUT_MS  1000   // max ms to wait for each AT response
#define CMD_SPACING_MS         50   // gap between commands


// ── Status LED ──────────────────────────────────────────────────────────────
// Uses the built-in LED (pin 13 on Uno/Nano) to indicate programming result.
//   Slow blink (1 Hz)  = programming complete OK
//   Fast blink (5 Hz)  = one or more commands failed

#define LED_PIN  LED_BUILTIN
static bool anyFailed = false;


// ── Helpers ──────────────────────────────────────────────────────────────────

// Discard any bytes already in the RX buffer.
void flushRx() {
  while (Serial.available()) Serial.read();
}

// Block until the expected token appears in the response stream, or timeout.
// Returns true on match, false on timeout.
bool waitForResponse(const char* expected, unsigned long timeout_ms) {
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (Serial.available()) {
      resp += (char)Serial.read();
      if (resp.indexOf(expected) >= 0) return true;
    }
  }
  return false;
}

// Send one AT command, wait for expected response, return success flag.
bool sendCommand(const char* label, const String& cmd, const char* expected) {
  flushRx();
  Serial.println(cmd);
  bool ok = waitForResponse(expected, RESPONSE_TIMEOUT_MS);
  if (!ok) anyFailed = true;
  delay(CMD_SPACING_MS);
  return ok;
}


// ── setup()  —  runs once at power-up ────────────────────────────────────────

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Wake the DRA818U.  PD must be HIGH before the module accepts any commands.
  // This must happen BEFORE Serial.begin / AT commands.
  pinMode(PIN_PD, OUTPUT);
  digitalWrite(PIN_PD, HIGH);
  delay(WAKE_DELAY_MS);

  Serial.begin(DRA818_BAUD);
  flushRx();

  // 1. Handshake — required first; clears the module's command parser state.
  sendCommand("CONNECT", "AT+DMOCONNECT", "+DMOCONNECT:0");

  // 2. Frequency group, bandwidth, CTCSS, squelch.
  //    BW must be printed as a plain integer (0 or 1), not with a decimal base.
  String groupCmd = "AT+DMOSETGROUP=";
  groupCmd += BANDWIDTH;                   // 0 or 1
  groupCmd += ",";
  groupCmd += String(FREQ_TX, 4);          // e.g. "434.5000"
  groupCmd += ",";
  groupCmd += String(FREQ_RX, 4);
  groupCmd += ",";
  groupCmd += TX_CTCSS;                    // 4-character code e.g. "0000"
  groupCmd += ",";
  groupCmd += SQUELCH;
  groupCmd += ",";
  groupCmd += RX_CTCSS;
  sendCommand("GROUP", groupCmd, "+DMOSETGROUP:0");

  // 3. Audio output volume (does not affect TX deviation).
  sendCommand("VOLUME",
    "AT+DMOSETVOLUME=" + String(VOLUME),
    "+DMOSETVOLUME:0");

  // 4. Filters — all OFF preserves flat AF response for SSTV/data.
  String filterCmd = "AT+DMOSETFILTER=";
  filterCmd += FILTER_PRE;
  filterCmd += ",";
  filterCmd += FILTER_HIGH;
  filterCmd += ",";
  filterCmd += FILTER_LOW;
  sendCommand("FILTER", filterCmd, "+DMOSETFILTER:0");

  // 5. Version query — optional diagnostic; confirms two-way comms.
  flushRx();
  Serial.println("AT+DMOVERQ");
  delay(RESPONSE_TIMEOUT_MS);   // just wait; we don't parse the version string

  // Release PD to hi-Z so the Raspberry Pi can drive it via BCM GPIO4.
  // If Arduino held it HIGH permanently it would conflict with the Pi
  // asserting LOW to power-down the radio between transmissions.
  pinMode(PIN_PD, INPUT);
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