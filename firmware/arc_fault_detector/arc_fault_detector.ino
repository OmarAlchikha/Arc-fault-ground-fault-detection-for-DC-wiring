/*
 * DC Arc-Fault Detector — Arduino Mega 2560
 * ==========================================
 *
 * Detects series and parallel arc faults on a low-voltage DC line and
 * opens a MOSFET disconnect before a hazardous condition develops, while
 * riding through load switching, inrush, contact bounce and PWM noise.
 *
 * This file is a line-for-line port of sim/detector.py — the algorithm was
 * developed and Monte-Carlo validated there first.  KEEP THE TWO IN SYNC.
 * Full method rationale: docs/detection-method.md.  Hardware (sensing
 * front end + perfboard switching stage): docs/hardware.md.
 *
 * Signal chain
 * ------------
 *   A0  "i_lf"   : hall current sensor, low-pass filtered (<1.5 kHz).
 *                  2.5 V = 0 A, 400 mV/A.
 *   A1  "hf_env" : broadband noise envelope.  Analog band-pass (8–45 kHz)
 *                  -> precision rectifier -> RC envelope (tau = 1 ms).
 *                  The analog front end does the MHz-class work the AVR
 *                  cannot; the MCU only ever sees slow signals.
 *   A2  "i_ret"  : second hall sensor on the RETURN conductor, for
 *                  residual-current (ground-fault) detection.
 *
 * Sampling: Timer1 CTC interrupt at 12 kHz rotating A0 -> A1 -> A2, so
 * every channel is sampled at 4 kHz.  The ISR never blocks on the ADC: it
 * reads the conversion the PREVIOUS tick started, then starts the next
 * one.  ADC clock is 250 kHz (prescaler 64), so a conversion takes ~52 us
 * and is always finished by the next 83 us tick.  250 kHz is slightly
 * above the datasheet's 200 kHz full-resolution recommendation; the ~1 LSB
 * it costs is irrelevant against our 12-count decision floor, and it buys
 * the timing margin.  analogRead() is NOT used anywhere after setup —
 * the ISR owns the ADC.
 *
 * Decision timing: features per 8 ms window (32 samples/ch), persistence
 * of 9 windows (fast path, current-step corroborated) or 18 windows (slow
 * path) -> typical trip 75–150 ms from arc onset (p95 165 ms in sim).
 */

#include "detector_core.h"   // the algorithm itself — shared with the host
                             // test (test/host_test.cpp) and kept in
                             // lockstep with sim/detector.py

// ---------------------------------------------------------------------------
// Pin map
// ---------------------------------------------------------------------------
const uint8_t PIN_I_LF    = 0;   // ADC ch 0  (A0)  LF current
const uint8_t PIN_HF_ENV  = 1;   // ADC ch 1  (A1)  HF noise envelope
const uint8_t PIN_I_RET   = 2;   // ADC ch 2  (A2)  return-side current (GF)

const uint8_t PIN_TRIP    = 7;   // HIGH = MOSFET ON (line closed).  Driving
                                 // LOW opens the line.  Fail-safe polarity:
                                 // an unpowered/reset MCU pin floats low ->
                                 // gate pulled down by 10k -> line OPEN.
const uint8_t PIN_LED_RUN  = 5;  // heartbeat
const uint8_t PIN_LED_TRIP = 6;  // latched fault indicator
const uint8_t PIN_RESET_BTN = 4; // active-low pushbutton: clear latch

// Algorithm tuning lives in detector_core.h (AF_* constants) so the exact
// same code is validated on the host.  Only board-level policy is here:

// Ground-fault (residual current) protection — simple and independent:
#define GF_THRESH_COUNTS 12   // |i_line - i_return| > ~150 mA sustained
#define GF_PERSIST_WINS  12   // ... for 12 windows (~100 ms) -> trip

#define WIN AF_WIN

// ---------------------------------------------------------------------------
// ISR <-> main loop handoff: double-buffered windows.
// The ISR owns the ADC exclusively.  Each 83 us tick it reads the result of
// the conversion started on the PREVIOUS tick (~52 us, always done by now),
// then points the mux at the next channel in the A0 -> A1 -> A2 rotation
// and starts the next conversion.  ISR cost is a few microseconds.
// ---------------------------------------------------------------------------
volatile uint16_t buf_lf[2][WIN];
volatile uint16_t buf_hf[2][WIN];
volatile uint16_t buf_ret[2][WIN];
volatile uint8_t  buf_active = 0;       // buffer the ISR is filling
volatile uint8_t  buf_idx = 0;          // sample index within window
volatile uint8_t  win_ready = 0;        // flag: a full window awaits the loop
volatile uint8_t  win_overrun = 0;      // loop too slow (should never happen)

static inline void adcStart(uint8_t ch) {
  ADMUX = _BV(REFS0) | (ch & 0x07);     // AVcc reference, channel 0-7
  ADCSRA |= _BV(ADSC);
}

ISR(TIMER1_COMPA_vect) {
  static uint8_t phase = 0;             // channel of the conversion now done
  uint16_t v = ADC;                     // result started on the previous tick

  switch (phase) {
    case 0:                             // A0 result in hand; start A1
      buf_lf[buf_active][buf_idx] = v;
      adcStart(PIN_HF_ENV);
      phase = 1;
      break;
    case 1:                             // A1 result; start A2
      buf_hf[buf_active][buf_idx] = v;
      adcStart(PIN_I_RET);
      phase = 2;
      break;
    default:                            // A2 result; start A0, close the row
      buf_ret[buf_active][buf_idx] = v;
      adcStart(PIN_I_LF);
      phase = 0;
      if (++buf_idx >= WIN) {
        buf_idx = 0;
        if (win_ready) win_overrun = 1; // previous window not consumed yet
        buf_active ^= 1;
        win_ready = 1;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Detector state
// ---------------------------------------------------------------------------
af_state_t af;               // the validated algorithm core's state
bool     tripped   = false;  // board-level latch (arc OR ground fault)

uint8_t  gf_wins   = 0;      // consecutive windows with GF imbalance
uint32_t last_telemetry = 0;
uint32_t last_blink = 0;

void openLine(const __FlashStringHelper *why) {
  digitalWrite(PIN_TRIP, LOW);          // gate low -> MOSFET off -> line open
  digitalWrite(PIN_LED_TRIP, HIGH);
  tripped = true;
  Serial.print(F("TRIP: "));
  Serial.println(why);
}

void setup() {
  af_init(&af);
  pinMode(PIN_TRIP, OUTPUT);
  digitalWrite(PIN_TRIP, LOW);          // start with the line OPEN
  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_TRIP, OUTPUT);
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println(F("DC arc-fault detector — boot"));

  // --- power-on self-test: sanity-check the sensing chain before closing
  //     the line.  A disconnected sensor reads a rail; a healthy idle chain
  //     reads ~512 on current channels and near-zero on the envelope. ---
  delay(100);
  int ilf = analogRead(PIN_I_LF);
  int ret = analogRead(PIN_I_RET);
  int env = analogRead(PIN_HF_ENV);
  if (ilf < 384 || ilf > 640 || ret < 384 || ret > 640) {  // > ±1 A with line open
    Serial.println(F("SELF-TEST FAIL: current channel out of range — staying open"));
    for (;;) { digitalWrite(PIN_LED_TRIP, !digitalRead(PIN_LED_TRIP)); delay(150); }
  }
  if (env > 300) {                      // strong HF with no load connected
    Serial.println(F("SELF-TEST FAIL: HF channel hot at idle — staying open"));
    for (;;) { digitalWrite(PIN_LED_TRIP, !digitalRead(PIN_LED_TRIP)); delay(150); }
  }
  Serial.println(F("self-test OK, closing line"));
  digitalWrite(PIN_TRIP, HIGH);

  // --- hand the ADC to the ISR: 250 kHz ADC clock, first conversion on A0
  //     already in flight when the first timer tick arrives ---
  noInterrupts();
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);   // enable, /64 prescaler
  adcStart(PIN_I_LF);

  // --- Timer1: CTC, 12 kHz interrupt (16 MHz / 1333) ---
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);      // CTC, no prescaler
  OCR1A  = 1332;                        // 16 MHz / (1332+1) = 12.003 kHz
  TIMSK1 = _BV(OCIE1A);
  interrupts();
}

void processWindow(const uint16_t *hf, const uint16_t *lf, const uint16_t *ret) {
  // ---- arc detection: hand the window to the validated core ----
  uint8_t trip = af_process_window(&af, hf, lf);
  if (!tripped) {
    if (trip == AF_TRIP_SLOW) openLine(F("arc signature (slow path)"));
    else if (trip == AF_TRIP_FAST) openLine(F("arc signature + current step (fast path)"));
  }

  // ---- ground fault: sustained line/return imbalance (independent of
  //      the arc logic; window means give plenty of averaging) ----
  uint32_t sum_lf = 0, sum_ret = 0;
  for (uint8_t k = 0; k < WIN; k++) { sum_lf += lf[k]; sum_ret += ret[k]; }
  int16_t imb = (int16_t)(sum_lf / WIN) - (int16_t)(sum_ret / WIN);
  if (imb < 0) imb = -imb;
  if (imb > GF_THRESH_COUNTS) {
    if (gf_wins < 255) gf_wins++;
    if (!tripped && gf_wins >= GF_PERSIST_WINS)
      openLine(F("ground fault (residual current)"));
  } else {
    gf_wins = 0;
  }
}

void loop() {
  // ---- consume a finished window, if any ----
  if (win_ready) {
    uint8_t b = buf_active ^ 1;         // the buffer the ISR just left
    static uint16_t hf[WIN], lf[WIN], ret[WIN];
    noInterrupts();
    for (uint8_t k = 0; k < WIN; k++) {
      hf[k] = buf_hf[b][k]; lf[k] = buf_lf[b][k]; ret[k] = buf_ret[b][k];
    }
    win_ready = 0;
    interrupts();
    processWindow(hf, lf, ret);
  }
  if (win_overrun) {
    win_overrun = 0;
    Serial.println(F("WARN: window overrun"));
  }
  uint32_t now = millis();

  // ---- latched-trip reset button (only resets after button released) ----
  if (tripped && digitalRead(PIN_RESET_BTN) == LOW) {
    delay(50);                          // debounce
    while (digitalRead(PIN_RESET_BTN) == LOW) {}
    af_init(&af);                       // fresh detector state, re-seeds baseline
    gf_wins = 0;
    tripped = false;
    digitalWrite(PIN_LED_TRIP, LOW);
    digitalWrite(PIN_TRIP, HIGH);
    Serial.println(F("reset: line closed"));
  }

  // ---- heartbeat + 1 Hz telemetry ----
  if (now - last_blink >= 500) {
    last_blink = now;
    digitalWrite(PIN_LED_RUN, !digitalRead(PIN_LED_RUN));
  }
  if (now - last_telemetry >= 1000) {
    last_telemetry = now;
    Serial.print(F("base=")); Serial.print((int16_t)(af.baseline >> 8));
    Serial.print(F(" dev=")); Serial.print((int16_t)(af.base_dev >> 8));
    Serial.print(F(" cnt=")); Serial.print(af.counter);
    Serial.print(F(" i="));   Serial.print(af.i_prev - AF_I_ZERO);
    Serial.print(F(" trip=")); Serial.println(tripped);
  }
}
