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
//  No third-party libraries required.  Only the built-in SoftwareSerial
//  library (included with every Arduino IDE installation) is used.
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
//  AT+DMOVERQ                             Version query    → version string
//
//  Response ":0" = success, ":1" = failure for all SET commands.
// ────────────────────────────────────────────────────────────────────────────


#include <SoftwareSerial.h>

// ── Serial ports ─────────────────────────────────────────────────────────────
// Hardware Serial (D0/D1/USB) is used exclusively for IDE Serial Monitor logging.
#define LOG_BAUD      115200

// SoftwareSerial carries all DRA818U AT command traffic.
#define DRA818_RX_PIN  2   // Arduino D2  ←── DRA818 TXD (J5)
#define DRA818_TX_PIN  3   // Arduino D3  ──► DRA818 RXD (J5)
#define DRA818_BAUD    9600

SoftwareSerial dra818Serial(DRA818_RX_PIN, DRA818_TX_PIN);

// ── Pin assignments ──────────────────────────────────────────────────────────

#define PIN_PD          4   // PD shared with Pi GPIO4 BCM; released to INPUT after setup


// ── Radio configuration  —  edit for your mission ────────────────────────────

// Frequency (MHz).  DRA818U range: 400.0000 – 470.0000
// Set TX and RX to the same frequency for simplex SSTV operation.
const float FREQ_TX     = 434.5000;
const float FREQ_RX     = 434.5000;

// Bandwidth: 0 = 12.5 kHz narrow,  1 = 25 kHz wide
// Use 1 (wide) for SSTV — the audio spans ~300 Hz to ~2.5 kHz.
const uint8_t BANDWIDTH = 1;

// CTCSS tone code ("0000" = none, "0001"–"0038" = standard tones).
// Use "0000" for simplex SSTV with no repeater access tone.
// Stored as a zero-padded 4-character string to match the AT command format.
const char TX_CTCSS[5]  = "0000";
const char RX_CTCSS[5]  = "0000";

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
// 200 ms is sufficient for AT command acceptance; sendCommand() retries on
// timeout so a tight value here is safe.
// Note: pi_sstv.py uses RADIO_WAKE_DELAY_SECONDS = 1.0 before keying PTT,
// which is a longer margin appropriate for full TX-carrier readiness.
#define WAKE_DELAY_MS         200
#define RESPONSE_TIMEOUT_MS  2000   // max ms to wait for each AT response
#define CMD_SPACING_MS         50   // inter-command gap


// ── Status LED ──────────────────────────────────────────────────────────────
// Uses the built-in LED (pin 13 on Uno/Nano) to indicate programming result.
//   Slow blink (1 Hz)  = programming complete OK
//   Fast blink (5 Hz)  = one or more commands failed

#define LED_PIN  LED_BUILTIN
static bool anyFailed = false;


// ── Helpers ──────────────────────────────────────────────────────────────────

// Discard any stale bytes in the DRA818 receive buffer.
void flushRx() {
  while (dra818Serial.available()) dra818Serial.read();
}

// Block until the expected token appears in the response stream, or timeout.
// Every received byte is mirrored to the Serial Monitor in real time.
// Returns true on match, false on timeout.
bool waitForResponse(const char* expected, unsigned long timeout_ms) {
  String resp = "";
  unsigned long start = millis();
  Serial.print(F("[DRA818] <<< "));
  while (millis() - start < timeout_ms) {
    if (dra818Serial.available()) {
      char c = (char)dra818Serial.read();
      resp += c;
      Serial.write(c);  // mirror to Serial Monitor in real time
      if (resp.indexOf(expected) >= 0) {
        Serial.println();
        return true;
      }
    }
  }
  Serial.println(F("(timeout)"));
  return false;
}

// Send one AT command, log it, wait for the expected response token.
// Returns true on success, false on timeout.
bool sendCommand(const char* label, const String& cmd, const char* expected) {
  flushRx();
  Serial.print(F("[DRA818] >>> "));
  Serial.println(cmd);
  dra818Serial.println(cmd);
  bool ok = waitForResponse(expected, RESPONSE_TIMEOUT_MS);
  Serial.print('[');
  Serial.print(label);
  Serial.println(ok ? F("] OK") : F("] FAILED (no response or unexpected reply)"));
  if (!ok) anyFailed = true;
  delay(CMD_SPACING_MS);
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
  Serial.println(BANDWIDTH == 1 ? F("25 kHz wide") : F("12.5 kHz narrow"));
  Serial.print(F("[HAMWING] Squelch: "));
  Serial.print(SQUELCH);
  Serial.print(F("  Volume: "));
  Serial.print(VOLUME);
  Serial.print(F("  CTCSS TX/RX: "));
  Serial.print(TX_CTCSS);
  Serial.print('/');
  Serial.println(RX_CTCSS);
  Serial.println(F("[HAMWING] AT command trace enabled — raw traffic shown below."));
  Serial.println();

  // Initialise the SoftwareSerial port to the DRA818U.
  dra818Serial.begin(DRA818_BAUD);
  flushRx();

  // 1. Handshake — required first; confirms two-way comms with the module.
  //    If this fails there is no point attempting further commands — bail out
  //    immediately.
  Serial.println(F("[HAMWING] Step 1/4: Handshake"));
  if (!sendCommand("CONNECT", "AT+DMOCONNECT", "+DMOCONNECT:0")) {
    // Release PD and report before entering the blink loop.
    pinMode(PIN_PD, INPUT);
    Serial.println(F("[HAMWING] PD released to hi-Z."));
    Serial.println(F("[HAMWING] *** RESULT: FAILED — aborting remaining steps."));
    Serial.println(F("[HAMWING]     Check: D2 <- DRA818 TXD, D3 -> DRA818 RXD, power."));
    Serial.println(F("[HAMWING]     LED: fast blink (5 Hz)."));
    return;
  }

  // 2. Frequency group, bandwidth, CTCSS, squelch.
  //    BW must be a plain integer (0 or 1); CTCSS is a 4-character string.
  Serial.println(F("[HAMWING] Step 2/4: Frequency group, bandwidth, CTCSS, squelch"));
  {
    String groupCmd = "AT+DMOSETGROUP=";
    groupCmd += BANDWIDTH;
    groupCmd += ',';
    groupCmd += String(FREQ_TX, 4);
    groupCmd += ',';
    groupCmd += String(FREQ_RX, 4);
    groupCmd += ',';
    groupCmd += TX_CTCSS;
    groupCmd += ',';
    groupCmd += SQUELCH;
    groupCmd += ',';
    groupCmd += RX_CTCSS;
    sendCommand("GROUP", groupCmd, "+DMOSETGROUP:0");
  }

  // 3. Audio output volume (does not affect TX deviation).
  Serial.println(F("[HAMWING] Step 3/4: Audio volume"));
  sendCommand("VOLUME", "AT+DMOSETVOLUME=" + String(VOLUME), "+DMOSETVOLUME:0");

  // 4. Filters — all OFF preserves flat AF response for SSTV/data.
  //    AT+DMOSETFILTER=PRE,HIGH,LOW  (1=on, 0=off)
  Serial.println(F("[HAMWING] Step 4/4: Filters"));
  {
    String filterCmd = "AT+DMOSETFILTER=";
    filterCmd += FILTER_PRE  ? 1 : 0;
    filterCmd += ',';
    filterCmd += FILTER_HIGH ? 1 : 0;
    filterCmd += ',';
    filterCmd += FILTER_LOW  ? 1 : 0;
    sendCommand("FILTER", filterCmd, "+DMOSETFILTER:0");
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