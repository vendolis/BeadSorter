/*
 * BeadCalibrate.ino
 *
 * Full-machine colour calibration data collector for BeadSorter.
 *
 * Uses IDENTICAL hardware to BeadSorter.ino — hopper feeds beads in, the servo
 * positions each bead at the sensor, and the stepper moves to the next output
 * slot after each bead.  Instead of sorting, it takes NUM_READINGS readings per
 * bead (with a servoWiggleIn() between each to vary the bead's position) and
 * prints everything as CSV on Serial.
 *
 * ── Serial baud rate: 115200 ─────────────────────────────────────────────────
 * (BeadSorter.ino uses 9600 — change your Serial Monitor setting.)
 *
 * ── Usage ────────────────────────────────────────────────────────────────────
 *   1. Flash this sketch.
 *   2. Open Serial Monitor at 115200 baud.
 *   3. Let the machine run — beads are processed automatically.
 *   4. The CSV header is printed once, then one row per reading per bead.
 *      Lines starting with '#' are comments — analyse_beads.py skips them.
 *   5. When done, copy the Serial output to a .csv file and run:
 *        python analyse_beads.py  beads_g16x.csv
 *
 * ── Changing gain ─────────────────────────────────────────────────────────────
 *   To test a different gain, edit CALIB_GAIN and GAIN_LABEL below, reflash,
 *   and collect a second .csv file.  analyse_beads.py compares gains when both
 *   files are concatenated (cat beads_g16x.csv beads_g4x.csv > combined.csv).
 *
 * ── CSV columns ───────────────────────────────────────────────────────────────
 *   bead           — sequential bead number (1-based)
 *   gain           — gain label string (e.g. "g16x")
 *   sample         — reading index within this bead (1 … NUM_READINGS)
 *   R_raw…C_raw    — raw 16-bit RGBC counts from sensor
 *   saturated      — 1 if any channel ≥ SAT_THRESH (reading unreliable)
 *   R_norm…B_norm  — float 0-255, clear-channel-normalised (from getRGB())
 *   chroma_r…      — R/(R+G+B) etc.: ambient-independent 0..1 colour signature
 *   H, S, L        — HSL in 0..1 derived from normalised RGB
 */

#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// IntelliSense stubs — never compiled by the Arduino toolchain.
#ifdef __INTELLISENSE__
  #define F(x) x
  typedef char __FlashStringHelper;
#endif

// ── Inline RGB → HSL ──────────────────────────────────────────────────────────
// Input:  r, g, b in 0..255 (float, from getRGB())
// Output: h, s, l in 0..1   (h is circular: 0 = red, 0.33 = green, 0.67 = blue)
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

// ── Pin / motor constants (identical to BeadSorter.ino) ──────────────────────
#define motorSpeed          255
#define GSM2                5
#define in3                 7
#define in4                 6
#define hopperMotorReverseTime 3000
#define setupPin            11
#define photoLEDPin         9
#define photoSensorPin      A0
#define photoSensorThreshold       600
#define photoSensorCalibMinDiff    100
#define servoAngleIn        36
#define servoAngleOut       62
#define servoAngleWiggle    2
#define servoPin            8

// ── Stepper constants (identical to BeadSorter.ino) ──────────────────────────
#define dirPin              2
#define stepPin             3
#define motorInterfaceType  1
#define stepperMaxSpeed     6000
#define stepperAccel        9000
#define stepperStepsPerRot  200
#define stepperMicroStepping 8
#define numContainerSlots   16
#define stepperMulti        (stepperStepsPerRot * stepperMicroStepping / numContainerSlots)

// Slot where all calibration beads (including rejects) are deposited.
#define REJECT_SLOT         1

// ── Calibration parameters ────────────────────────────────────────────────────

// Wiggle positions taken per bead.  At each position all four gains are read
// before the next wiggle, so total rows per bead = NUM_READINGS × 4.
static const int NUM_READINGS = 10;

// Integration time used for every gain step (uint8_t macro — no typedef in this lib).
static const uint8_t CALIB_INTTIME = TCS34725_INTEGRATIONTIME_101MS;

// Time to wait after setGain() before reading: must be > one integration period.
static const unsigned long GAIN_SETTLE_MS = 220;

// Gain sweep table — all four gains are measured at every wiggle position.
struct GainRow { tcs34725Gain_t gain; const char* label; };
static const GainRow GAINS[] = {
  { TCS34725_GAIN_1X,  "g1x"  },
  { TCS34725_GAIN_4X,  "g4x"  },
  { TCS34725_GAIN_16X, "g16x" },
  { TCS34725_GAIN_60X, "g60x" },
};
static const int NUM_GAINS = (int)(sizeof(GAINS) / sizeof(GAINS[0]));

// Null scan uses 60x (index 3 in GAINS) — same gain used during setup calibration.
static const int NULL_SCAN_GAIN_IDX = 3;

// Saturation warning threshold (same 16-bit space as raw counts).
static const uint16_t SAT_THRESH = 60000;

// Null-scan tolerances: maximum HSL deviation from the empty-tube baseline
// that still counts as "no bead".
// Deliberately tighter than BeadSorter.ino (0.07/0.12/0.12) so that beads
// whose colours happen to sit near the empty-tube reference are NOT rejected.
// Only lower these further if you still see false "empty" readings in the CSV.
static const float NULL_OFFSET_H = 0.03f;
static const float NULL_OFFSET_S = 0.05f;
static const float NULL_OFFSET_L = 0.05f;

// ── Hardware objects ──────────────────────────────────────────────────────────
Adafruit_TCS34725 tcs = Adafruit_TCS34725(CALIB_INTTIME, GAINS[NULL_SCAN_GAIN_IDX].gain);
Servo servo;
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// ── Global state ──────────────────────────────────────────────────────────────
float    nullScanHSL[3] = {0.0f, 0.0f, 0.0f};
int      beadId         = 0;
int      rejectId       = 0;

// Current sensor snapshot — filled by readAllSensorData().
uint16_t rawR, rawG, rawB, rawC;
float    normR, normG, normB;
float    chromaR, chromaG, chromaB;
double   hue, satHSL, litHSL;

// ── Sensor read ───────────────────────────────────────────────────────────────
// Fills all global sensor variables in one call.
void readAllSensorData() {
  delay(200);  // let integration complete (same guard as BeadSorter.ino)
  tcs.getRawData(&rawR, &rawG, &rawB, &rawC);
  tcs.getRGB(&normR, &normG, &normB);
  rgbToHsl(normR, normG, normB, hue, satHSL, litHSL);
  float rgbSum = (float)rawR + (float)rawG + (float)rawB;
  if (rgbSum > 0.0f) {
    chromaR = (float)rawR / rgbSum;
    chromaG = (float)rawG / rgbSum;
    chromaB = (float)rawB / rgbSum;
  } else {
    chromaR = chromaG = chromaB = 0.0f;
  }
}

// ── Null scan ─────────────────────────────────────────────────────────────────
// Returns true when the sensor reading is close enough to the empty-tube
// baseline (i.e. no bead is present).
boolean nullScan() {
  float dh = fabsf((float)hue - nullScanHSL[0]);
  if (dh > 0.5f) dh = 1.0f - dh;
  return (dh                                      <= NULL_OFFSET_H) &&
         (fabsf((float)satHSL - nullScanHSL[1])   <= NULL_OFFSET_S) &&
         (fabsf((float)litHSL - nullScanHSL[2])   <= NULL_OFFSET_L);
}

// ── CSV row output ────────────────────────────────────────────────────────────
void printCSVRow(int bead, const char* gainLabel, int sampleNo) {
  bool sat = (rawR >= SAT_THRESH || rawG >= SAT_THRESH ||
              rawB >= SAT_THRESH || rawC >= SAT_THRESH);
  Serial.print(bead);           Serial.print(',');
  Serial.print(gainLabel);      Serial.print(',');
  Serial.print(sampleNo);       Serial.print(',');
  Serial.print(rawR);           Serial.print(',');
  Serial.print(rawG);           Serial.print(',');
  Serial.print(rawB);           Serial.print(',');
  Serial.print(rawC);           Serial.print(',');
  Serial.print(sat ? 1 : 0);    Serial.print(',');
  Serial.print(normR, 1);       Serial.print(',');
  Serial.print(normG, 1);       Serial.print(',');
  Serial.print(normB, 1);       Serial.print(',');
  Serial.print(chromaR, 5);     Serial.print(',');
  Serial.print(chromaG, 5);     Serial.print(',');
  Serial.print(chromaB, 5);     Serial.print(',');
  Serial.print(hue, 5);         Serial.print(',');
  Serial.print(satHSL, 5);      Serial.print(',');
  Serial.print(litHSL, 5);      Serial.println();
}

// ── Stepper (identical to BeadSorter.ino) ────────────────────────────────────
void moveSorterToPosition(int position) {
  int currentPos     = stepper.currentPosition() / stepperMulti;
  int diffToPosition = position - currentPos;
  if (diffToPosition > 8) {
    position = currentPos + diffToPosition - 16;
  } else if (diffToPosition < -8) {
    position = currentPos + diffToPosition + 16;
  }
  position *= stepperMulti;
  stepper.moveTo(position);
  stepper.runToPosition();
  stepper.setCurrentPosition(stepper.currentPosition() % (16 * stepperMulti));
}

// ── Servo (identical to BeadSorter.ino) ──────────────────────────────────────
void servoFeedIn() {
  int low  = servoAngleIn - servoAngleWiggle;
  int high = servoAngleIn + servoAngleWiggle;
  for (int i = low; i < high; i++) { servo.write(i); delay(100); }
}

void servoWiggleIn() {
  int low  = servoAngleIn - servoAngleWiggle * 2;
  int high = servoAngleIn + servoAngleWiggle * 2;
  for (int c = 0; c < 3; c++) {
    servo.write(low);  delay(100);
    servo.write(high); delay(100);
  }
  servo.write(servoAngleIn);
  delay(100);
}

void servoFeedOut() {
  int low  = servoAngleOut - servoAngleWiggle;
  int high = servoAngleOut + servoAngleWiggle;
  for (int count = 0; count < 3; count++) {
    for (int i = low;  i < high; i++) { servo.write(i); delay(100); }
    for (int i = high; i > low;  i--) { servo.write(i); delay(50);  }
  }
  delay(200);
}

// ── Hopper motor (identical to BeadSorter.ino) ────────────────────────────────
void stopHopperMotor() {
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(GSM2, motorSpeed);
}
void startHopperMotor(bool dir) {
  if (dir) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW); }
  else     { digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); }
  analogWrite(GSM2, motorSpeed);
}
unsigned long timediff(unsigned long t1, unsigned long t2) {
  signed long d = (signed long)t1 - (signed long)t2;
  return (unsigned long)(d < 0 ? -d : d);
}
void handleHopperMotor(bool successfullBead) {
  static unsigned long timestamp = 0;
  static bool direction = false;
  if (!successfullBead) {
    if (timediff(timestamp, millis()) > hopperMotorReverseTime) {
      direction = !direction;
      timestamp = millis();
    }
  } else {
    timestamp = millis();
  }
  int photo = analogRead(photoSensorPin);
  if (photo > photoSensorThreshold) startHopperMotor(direction);
  else                               stopHopperMotor();
}

// ── Photo sensor calibration (identical to BeadSorter.ino) ───────────────────
void calibratePhotoSensor() {
  Serial.println(F("[INIT] Calibrating photo sensor..."));
  int sumOn = 0, sumOff = 0;
  for (int i = 0; i < 3; i++) {
    digitalWrite(photoLEDPin, HIGH); delay(50); sumOn  += analogRead(photoSensorPin);
    digitalWrite(photoLEDPin, LOW);  delay(50); sumOff += analogRead(photoSensorPin);
  }
  int avgOn = sumOn / 3, avgOff = sumOff / 3;
  digitalWrite(photoLEDPin, HIGH);
  Serial.print(F("Photo sensor: LED on=")); Serial.print(avgOn);
  Serial.print(F(", off="));               Serial.print(avgOff);
  Serial.print(F(", diff="));              Serial.println(abs(avgOn - avgOff));
  if (abs(avgOn - avgOff) < photoSensorCalibMinDiff) {
    Serial.println(F("ERROR: Photo sensor calibration failed. Halting."));
    while (1);
  }
  Serial.println(F("Photo sensor OK."));
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== BeadSorter — Calibration Mode ==="));
  Serial.print(F("  Gains: g1x g4x g16x g60x  x"));
  Serial.print(NUM_READINGS);
  Serial.println(F(" wiggle positions"));
  Serial.println();

  pinMode(GSM2,        OUTPUT);
  pinMode(in3,         OUTPUT);
  pinMode(in4,         OUTPUT);
  pinMode(setupPin,    INPUT);
  pinMode(photoLEDPin, OUTPUT);
  digitalWrite(photoLEDPin, HIGH);

  servo.attach(servoPin);
  servo.write(servoAngleIn);

  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAccel);
  stepper.setCurrentPosition(0);
  moveSorterToPosition(REJECT_SLOT);  // start at the collection slot

  calibratePhotoSensor();

  if (!tcs.begin()) {
    Serial.println(F("ERROR: TCS34725 not found. Check wiring. Halting."));
    while (1);
  }
  Serial.println(F("[INIT] TCS34725 found."));

  // Null scan calibration: cycle servo 6 times, then capture empty-tube baseline.
  Serial.println(F("[INIT] Null scan calibration (empty tube)..."));
  for (int i = 0; i < 6; i++) { servoFeedIn(); servoFeedOut(); }
  servoFeedIn();
  readAllSensorData();
  nullScanHSL[0] = (float)hue;
  nullScanHSL[1] = (float)satHSL;
  nullScanHSL[2] = (float)litHSL;
  Serial.print(F("# Null ref: H=")); Serial.print(nullScanHSL[0], 4);
  Serial.print(F("  S="));           Serial.print(nullScanHSL[1], 4);
  Serial.print(F("  L="));           Serial.println(nullScanHSL[2], 4);
  Serial.println(F("[INIT] Ready. Starting collection."));
  Serial.println();

  // CSV header — must match printCSVRow() column order.
  Serial.println(F("bead,gain,sample,"
                   "R_raw,G_raw,B_raw,C_raw,saturated,"
                   "R_norm,G_norm,B_norm,"
                   "chroma_r,chroma_g,chroma_b,"
                   "H,S,L"));

  // Null reference row (bead=0, gain=null_ref) — globals still hold the
  // 16x reading taken above, so we can emit it directly.
  Serial.println(F("# --- Null reference (16x, bead=0) ---"));
  printCSVRow(0, "null_ref", 1);
}

// Sweep all gains at the current bead position and print one CSV row each.
void sweepGains(int bead, int position) {
  for (int gi = 0; gi < NUM_GAINS; gi++) {
    tcs.setGain(GAINS[gi].gain);
    delay(GAIN_SETTLE_MS);
    readAllSensorData();
    printCSVRow(bead, GAINS[gi].label, position);
  }
  // Restore null-scan gain for next null-check.
  tcs.setGain(GAINS[NULL_SCAN_GAIN_IDX].gain);
  delay(GAIN_SETTLE_MS);
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  static bool successfullBead = false;

  handleHopperMotor(successfullBead);

  // Feed bead into sensor position and take null-scan reading at 16x.
  servoFeedIn();
  delay(500);
  servoWiggleIn();
  delay(200);
  readAllSensorData();  // gain is already at GAINS[NULL_SCAN_GAIN_IDX] (16x)

  if (!nullScan()) {
    // ── Bead detected: sweep all gains at each wiggle position ────────────────
    beadId++;
    Serial.print(F("# --- Bead ")); Serial.print(beadId); Serial.println(F(" ---"));

    sweepGains(beadId, 1);  // position 1 — already seated from feed above

    for (int pos = 2; pos <= NUM_READINGS; pos++) {
      servoWiggleIn();
      delay(200);
      sweepGains(beadId, pos);
    }

    successfullBead = true;
  } else {
    // ── Rejected by null scan — print comment + full gain sweep ───────────────
    rejectId++;
    Serial.print(F("# REJECTED ")); Serial.print(rejectId);
    Serial.print(F("  read: H="));  Serial.print((float)hue,    4);
    Serial.print(F(" S="));         Serial.print((float)satHSL, 4);
    Serial.print(F(" L="));         Serial.print((float)litHSL, 4);
    Serial.print(F("  null ref: H=")); Serial.print(nullScanHSL[0], 4);
    Serial.print(F(" S="));            Serial.print(nullScanHSL[1], 4);
    Serial.print(F(" L="));            Serial.println(nullScanHSL[2], 4);

    // Full gain sweep for this reject (bead id is negative to distinguish in CSV).
    sweepGains(-rejectId, 1);

    successfullBead = false;
  }

  // Route all beads (and rejects) into the collection slot, then release.
  moveSorterToPosition(REJECT_SLOT);
  servoFeedOut();
}
