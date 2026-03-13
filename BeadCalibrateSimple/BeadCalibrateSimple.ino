/*
 * BeadCalibrateSimple.ino
 *
 * Minimal calibration sketch — TCS34725 colour sensor + button only.
 * No servo, no stepper, no hopper motor.
 *
 * ── Wiring ───────────────────────────────────────────────────────────────────
 *   TCS34725  →  Arduino I2C (SDA/SCL) + 3.3V + GND
 *   Button    →  D11 and GND  (uses internal pull-up)
 *
 * ── Usage ────────────────────────────────────────────────────────────────────
 *   1. Flash this sketch.
 *   2. Open Serial Monitor at 115200 baud.
 *   3. Place a bead in front of the sensor.
 *   4. Press the button — 4 CSV rows are printed (one per gain level).
 *   5. Remove bead, place next one, repeat.
 *   6. Copy the Serial output (including the header line) to a .csv file.
 *   7. Run:  python gen_swatches_simple.py  your_file.csv
 *
 * ── CSV columns ───────────────────────────────────────────────────────────────
 *   bead         — sequential number (1-based, increments each button press)
 *   gain         — g1x / g4x / g16x / g60x
 *   R_raw…C_raw  — raw 16-bit RGBC counts
 *   saturated    — 1 if any channel ≥ 60000 (reading unreliable)
 *   R_norm…B_norm — float 0-255, clear-channel-normalised
 *   chroma_r…    — R/(R+G+B) etc.
 *   H, S, L      — HSL in 0..1
 *
 * ── Baud rate: 115200 ────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// IntelliSense stubs — never compiled by the Arduino toolchain.
#ifdef __INTELLISENSE__
  #define F(x) x
  typedef char __FlashStringHelper;
#endif

// ── Pin ───────────────────────────────────────────────────────────────────────
#define BUTTON_PIN  11    // active LOW — connect button between D11 and GND

// ── Sensor settings ───────────────────────────────────────────────────────────
static const uint8_t       INTTIME    = TCS34725_INTEGRATIONTIME_101MS;
static const uint16_t      SAT_THRESH = 60000;
static const unsigned long SETTLE_MS  = 220;  // wait after setGain() — > one integration period

// Gain sweep table — order determines column order in the output.
struct GainRow { tcs34725Gain_t gain; const char* label; };
static const GainRow GAINS[] = {
  { TCS34725_GAIN_1X,  "g1x"  },
  { TCS34725_GAIN_4X,  "g4x"  },
  { TCS34725_GAIN_16X, "g16x" },
  { TCS34725_GAIN_60X, "g60x" },
};
static const int NUM_GAINS = (int)(sizeof(GAINS) / sizeof(GAINS[0]));

// ── Hardware ──────────────────────────────────────────────────────────────────
Adafruit_TCS34725 tcs = Adafruit_TCS34725(INTTIME, TCS34725_GAIN_16X);

// ── State ─────────────────────────────────────────────────────────────────────
static int beadId = 0;

// ── RGB → HSL (h, s, l all in 0..1) ─────────────────────────────────────────
static void rgbToHsl(float r, float g, float b, double &h, double &s, double &l) {
  float rN = r / 255.0f, gN = g / 255.0f, bN = b / 255.0f;
  float mx = max(max(rN, gN), bN);
  float mn = min(min(rN, gN), bN);
  float delta = mx - mn;
  l = (double)((mx + mn) * 0.5f);
  if (delta < 1e-6f) { h = 0.0; s = 0.0; return; }
  s = (l < 0.5) ? (delta / (mx + mn)) : (delta / (2.0f - mx - mn));
  float hRaw;
  if      (mx == rN) hRaw = (gN - bN) / delta + (gN < bN ? 6.0f : 0.0f);
  else if (mx == gN) hRaw = (bN - rN) / delta + 2.0f;
  else               hRaw = (rN - gN) / delta + 4.0f;
  h = (double)(hRaw / 6.0f);
}

// ── Read sensor at current gain and print one CSV row ─────────────────────────
static void readAndPrint(int bead, const char* gainLabel) {
  delay(200);  // let integration complete after settle delay

  uint16_t rawR, rawG, rawB, rawC;
  float    normR, normG, normB;
  tcs.getRawData(&rawR, &rawG, &rawB, &rawC);
  tcs.getRGB(&normR, &normG, &normB);

  bool sat = (rawR >= SAT_THRESH || rawG >= SAT_THRESH ||
              rawB >= SAT_THRESH || rawC >= SAT_THRESH);

  float rgbSum = (float)rawR + (float)rawG + (float)rawB;
  float chR = 0.0f, chG = 0.0f, chB = 0.0f;
  if (rgbSum > 0.0f) {
    chR = (float)rawR / rgbSum;
    chG = (float)rawG / rgbSum;
    chB = (float)rawB / rgbSum;
  }

  double h, s, l;
  rgbToHsl(normR, normG, normB, h, s, l);

  Serial.print(bead);          Serial.print(',');
  Serial.print(gainLabel);     Serial.print(',');
  Serial.print(rawR);          Serial.print(',');
  Serial.print(rawG);          Serial.print(',');
  Serial.print(rawB);          Serial.print(',');
  Serial.print(rawC);          Serial.print(',');
  Serial.print(sat ? 1 : 0);   Serial.print(',');
  Serial.print(normR, 1);      Serial.print(',');
  Serial.print(normG, 1);      Serial.print(',');
  Serial.print(normB, 1);      Serial.print(',');
  Serial.print(chR, 5);        Serial.print(',');
  Serial.print(chG, 5);        Serial.print(',');
  Serial.print(chB, 5);        Serial.print(',');
  Serial.print(h, 5);          Serial.print(',');
  Serial.print(s, 5);          Serial.print(',');
  Serial.print(l, 5);          Serial.println();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!tcs.begin()) {
    Serial.println(F("ERROR: TCS34725 not found. Check wiring. Halting."));
    while (1);
  }

  Serial.println(F("# BeadCalibrateSimple — place bead, press button to scan"));
  Serial.println(F("bead,gain,R_raw,G_raw,B_raw,C_raw,saturated,"
                   "R_norm,G_norm,B_norm,chroma_r,chroma_g,chroma_b,H,S,L"));
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  static bool lastBtn = HIGH;
  bool btn = digitalRead(BUTTON_PIN);

  // Detect falling edge (button press)
  if (lastBtn == HIGH && btn == LOW) {
    delay(30);  // debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      beadId++;
      Serial.print(F("# --- Bead ")); Serial.print(beadId); Serial.println(F(" ---"));

      for (int gi = 0; gi < NUM_GAINS; gi++) {
        tcs.setGain(GAINS[gi].gain);
        delay(SETTLE_MS);
        readAndPrint(beadId, GAINS[gi].label);
      }

      // Restore default gain and wait for button release
      tcs.setGain(TCS34725_GAIN_16X);
      while (digitalRead(BUTTON_PIN) == LOW) {}
    }
  }

  lastBtn = btn;
}
