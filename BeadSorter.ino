#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "ColorConverterLib.h"


// Stubs so VS Code IntelliSense resolves AVR-specific macros.
// These lines are never compiled by the Arduino toolchain.
#ifdef __INTELLISENSE__
  #define F(x) x
  #define PSTR(x) x
  #define snprintf_P snprintf
  typedef char __FlashStringHelper;  // makes const __FlashStringHelper* == const char*
  // Simple pass-throughs so IntelliSense resolves SAFE_PRINT / SAFE_PRINTLN calls.
  #define SAFE_PRINT(...)   Serial.print(__VA_ARGS__)
  #define SAFE_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  // Non-blocking serial print helpers.
  // The Arduino AVR core's Serial.write() spin-waits indefinitely when the 64-byte
  // TX buffer is full and the UART cannot drain (e.g. TX line issue, no terminal).
  // These macros skip the print when there is insufficient buffer space, preventing
  // the program from freezing. Output may be dropped but execution never stalls.
  #define SAFE_PRINT(...)   do { if (Serial.availableForWrite() > 16) { Serial.print(__VA_ARGS__);   } } while(0)
  #define SAFE_PRINTLN(...) do { if (Serial.availableForWrite() > 16) { Serial.println(__VA_ARGS__); } } while(0)
#endif

#define dirPin 2 //Stepper
#define stepPin 3 //Stepper
#define motorInterfaceType 1 //Stepper

#define stepperMaxSpeed 6000
#define stepperAccel 9000

//#define stepperMulti 100 //Stepper
#define stepperStepsPerRot 200
#define stepperMicroStepping 8
#define numContainerSlots 16    // in reality there are only 12, but
#define stepperMulti (stepperStepsPerRot*stepperMicroStepping/numContainerSlots)

#define motorSpeed 255 //Container Motor
#define GSM2 5 // Container Motor
#define in3 7 //Container Motor
#define in4 6 //Container Motor
#define hopperMotorReverseTime  3000 //in ms - time of ~ revolution

#define setupPin 11 //Setup

#define photoLEDPin 9 //Photo Sensor LED
#define photoSensorPin A0 //Photo Sensor
#define photoSensorThreshold 600
#define photoSensorCalibMinDiff 100  // minimum on/off difference to consider LED functional

// Number of sortable color slots (bin 15 is reserved for unrecognised beads when full)
#define autoSortMaxColors 11

#define servoAngleIn 36 //Servo
#define servoAngleOut 62
#define servoAngleWiggle 2
#define servoPin 8

// #define DEBUG_PROG 1 // Stepper test
// #define DEBUG_PROG 2 // Color Sensor test
// #define DEBUG_PRINT_PHOTOSENS


// Sensor parameters — change SENSOR_GAIN here to switch gain for all modes.
// Valid gain values: TCS34725_GAIN_1X  TCS34725_GAIN_4X  TCS34725_GAIN_16X  TCS34725_GAIN_60X
#define SENSOR_INTEGRATION_TIME  TCS34725_INTEGRATIONTIME_101MS
#define SENSOR_GAIN              TCS34725_GAIN_16X

Adafruit_TCS34725 tcs = Adafruit_TCS34725(SENSOR_INTEGRATION_TIME, SENSOR_GAIN);

// HSL matching thresholds — independent per channel.
// All values are in the 0..1 range (the ColorConverterLib normalises H, S, L to 0..1).
// H wraps around (0 == 1 == red), so the comparison uses circular distance.
//
// Values below are set to half the minimum inter-bead gap measured during calibration
// (BeadCalibration_20260313_improved.csv, g16x, analyse_beads.py).
// These are the tightest values that avoid confusing any two bead colours.
// If beads fail to match themselves (too many unknowns), loosen them by 2-3×.
float thresholdH = 0.0002;  // hue tolerance
float thresholdS = 0.0011;  // saturation tolerance
float thresholdL = 0.0003;  // lightness tolerance

// Null-scan window: maximum deviation from empty-tube reference in each channel.
float nullScanOffsetH = 0.07;
float nullScanOffsetS = 0.12;
float nullScanOffsetL = 0.12;

Servo servo;
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

bool autoSort = true;
float    resultHSL[3]  = {0.0, 0.0, 0.0};  // H, S, L  — all 0..1
uint16_t resultRaw[4]  = {0, 0, 0, 0};     // R, G, B, C  raw sensor counts
float    resultNorm[3] = {0.0, 0.0, 0.0};  // R_norm, G_norm, B_norm  (0-255, clear-normalised)

float medianHSL[4][3];  // 4-sample accumulation buffer

// storedColors[16][3] — H < 0 marks an empty slot
float storedColors[16][3] = {
  {-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},
  {-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},
  {-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},
  {-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0}
};

bool calibrateNullScan = true;
// If calibrateNullScan=false, set these manually (H, S, L all 0..1).
float nullScanHSL[3] = {0.0, 0.0, 0.0};

uint8_t  autoColorCounter = 0;
uint16_t beadCounter = 0;

const int dynamicContainerArraySize = 16;
// -1 means empty, 666 means blocked (there are only 12 total slots available due to the layout.)
// When starting the program, the arm should be over the middle slot of one of the three on the sides.
// The Unknown color slot (15) will then be the slot to the right of it.
int dynamicContainerArray[dynamicContainerArraySize] = { -1, -1, 666, -1, -1, -1, 666, -1, -1, -1, 666, -1, -1, -1, 666 , -1 };

void setup() {
  Serial.begin(115200);
  SAFE_PRINTLN();
  SAFE_PRINTLN(F("=== BeadSorter starting ==="));

  // Setup Hopper Motor Pins
  SAFE_PRINTLN(F("[INIT] Hopper motor pins..."));
  pinMode(GSM2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  SAFE_PRINTLN(F("[INIT] Setup button pin..."));
  pinMode(setupPin, INPUT);

  SAFE_PRINTLN(F("[INIT] Photo sensor LED pin..."));
  pinMode(photoLEDPin, OUTPUT);
  digitalWrite(photoLEDPin, HIGH); // LED on by default

  SAFE_PRINTLN(F("[INIT] Servo..."));
  servo.attach(servoPin);
  servo.write(servoAngleIn);

  SAFE_PRINT(F("[INIT] Stepper (maxSpeed=")); SAFE_PRINT(stepperMaxSpeed);
  SAFE_PRINT(F(", accel=")); SAFE_PRINT(stepperAccel); SAFE_PRINTLN(F(")..."));
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAccel);
  stepper.setCurrentPosition(0);

#if defined DEBUG_PROG && DEBUG_PROG == 1
  // Full 360 turn, then return to home
  SAFE_PRINTLN(F("[DEBUG] Stepper: full 360 turn..."));
  stepper.moveTo(stepperStepsPerRot * stepperMicroStepping);
  stepper.runToPosition();
  delay(500);
  stepper.moveTo(0);
  stepper.runToPosition();
  delay(500);

  // Visit each output slot once
  SAFE_PRINTLN(F("[DEBUG] Stepper: cycling all output slots..."));
  for (int slot = 0; slot < numContainerSlots; slot++) {
    SAFE_PRINT(F("[DEBUG] Slot ")); SAFE_PRINTLN(slot);
    stepper.moveTo(slot * stepperMulti);
    stepper.runToPosition();
    delay(500);
  }

  // Return to slot 0
  SAFE_PRINTLN(F("[DEBUG] Returning to slot 0..."));
  stepper.moveTo(0);
  stepper.runToPosition();
  SAFE_PRINTLN(F("[DEBUG] Stepper test complete."));
  while (1);
#endif

  // Hold setup button during boot to enter interactive debug mode (2s window)
  {
    SAFE_PRINTLN(F("[INIT] Hold setup button to enter interactive debug mode (2s)..."));
    unsigned long t = millis();
    bool debugRequested = false;
    while (millis() - t < 2000) {
      if (digitalRead(setupPin) == HIGH) {
        SAFE_PRINTLN(F("[Debug] Setup Button registered."));
        debugRequested = true;
        break;
      }
    }
    if (debugRequested) {
      SAFE_PRINTLN(F("[Debug] Entering Debug mode."));
      runInteractiveDebug(); // never returns
      while (1);
    }
    SAFE_PRINTLN(F("[INIT] Continuing normal startup."));
  }

  calibratePhotoSensor();

  if (tcs.begin()) {
    SAFE_PRINTLN(F("[INIT] TCS34725 color sensor found."));
  } else {
    SAFE_PRINTLN(F("ERROR: TCS34725 not found, exiting!"));
    while (1); // Stop program
  }

#if defined DEBUG_PROG && DEBUG_PROG == 2
  servoFeedOut();
  while (1)
  {
    servoFeedIn();
    delay(500);
    servoWiggleIn();
    delay(200);
    readColorSensor();
    printBeadReading();
    delay(800);
    servoFeedOut();
  }
#endif

  SAFE_PRINT(F("[INIT] Mode: "));
  SAFE_PRINTLN(autoSort ? F("autoSort") : F("manual (preset colors)"));
  if (!autoSort) {
    SAFE_PRINTLN(F("[INIT] Importing default color set..."));
    importDefaultColorSet();
    SAFE_PRINTLN(F("[INIT] Default colors imported."));
  }

  if (calibrateNullScan) {
    SAFE_PRINTLN(F("[INIT] Null scan calibration: cycling servo..."));
    for (int i = 0; i < 6; i++) {
      SAFE_PRINT(F("  cycle ")); SAFE_PRINT(i + 1); SAFE_PRINTLN(F("/6"));
      servoFeedIn();
      servoFeedOut();
    }
    servoFeedIn();
    readColorSensor();
    setNullScanValues();
    SAFE_PRINTLN(F("[INIT] Null scan calibration done."));
  } else {
    SAFE_PRINTLN(F("[INIT] Null scan calibration skipped (using hardcoded values)."));
  }

  SAFE_PRINTLN(F("=== Setup complete. Starting continuous color scan. ==="));
  SAFE_PRINTLN();
}

void loop() {
  static bool successfullBead = false;

  while (Serial.available())
  {
    int c = Serial.read();
    switch (c)
    {
      case 't':
      case 'T': printTables(); break;
      default: break;
    }
  }

  handleHopperMotor(successfullBead);

  if (digitalRead(setupPin) == HIGH) {
    while (digitalRead(setupPin) == HIGH) {}
    // for manual colors, use tighter thresholds
    autoSort = false;
    thresholdH = 0.012;
    thresholdS = 0.025;
    thresholdL = 0.025;
    addColor();
  }

  servoFeedIn();
  delay(500);
  servoWiggleIn();
  delay(200);
  readColorSensor();

  if (!nullScan()) {
    SAFE_PRINTLN();
    SAFE_PRINT(F("Bead #")); SAFE_PRINTLN(beadCounter++);
    printBeadReading();
    sortBeadToDynamicArray();
    successfullBead = true;
  } else {
    SAFE_PRINT('.');
    successfullBead = false;
  }

  servoFeedOut();
}

/*
*   print all color tables to serial for debugging.
*/
void printTables() {
  char line[48];

  SAFE_PRINTLN(F("Nullscan (H S L):"));
  snprintf_P(line, sizeof(line), PSTR("%2d H=%.4f S=%.4f L=%.4f"),
             0, nullScanHSL[0], nullScanHSL[1], nullScanHSL[2]);
  SAFE_PRINTLN(line);

  SAFE_PRINTLN(F("Stored colors (H S L):"));
  for (int i = 0; i < 16; i++) {
    if (storedColors[i][0] < 0) {
      snprintf_P(line, sizeof(line), PSTR("%2d (empty)"), i);
    } else {
      snprintf_P(line, sizeof(line), PSTR("%2d H=%.4f S=%.4f L=%.4f"),
                 i, storedColors[i][0], storedColors[i][1], storedColors[i][2]);
    }
    SAFE_PRINTLN(line);
  }

  SAFE_PRINTLN(F("Container array:"));
  for (int i = 0; i < dynamicContainerArraySize; i++) {
    SAFE_PRINT(dynamicContainerArray[i]);
    SAFE_PRINT(' ');
  }
  SAFE_PRINTLN();
}


/*
*  Calibrate photo sensor by toggling the LED 3 times.
*  Compares average readings with LED on vs off.
*  Halts with error if the difference is below photoSensorCalibMinDiff,
*  which indicates a broken LED or blocked line of sight.
*/
void calibratePhotoSensor() {
  SAFE_PRINTLN(F("[INIT] Calibrating photo sensor..."));
  int sumOn  = 0;
  int sumOff = 0;

  for (int i = 0; i < 3; i++) {
    digitalWrite(photoLEDPin, HIGH);
    delay(50);
    sumOn += analogRead(photoSensorPin);

    digitalWrite(photoLEDPin, LOW);
    delay(50);
    sumOff += analogRead(photoSensorPin);
  }

  int avgOn  = sumOn  / 3;
  int avgOff = sumOff / 3;

  digitalWrite(photoLEDPin, HIGH); // restore LED on for normal operation

  SAFE_PRINT(F("Photo sensor calib: LED on="));
  SAFE_PRINT(avgOn);
  SAFE_PRINT(F(", LED off="));
  SAFE_PRINT(avgOff);
  SAFE_PRINT(F(", diff="));
  SAFE_PRINTLN(abs(avgOn - avgOff));

  if (abs(avgOn - avgOff) < photoSensorCalibMinDiff) {
    SAFE_PRINTLN(F("ERROR: Photo sensor calib failed! LED broken or line of sight blocked."));
    while (1); // halt
  }

  SAFE_PRINTLN(F("Photo sensor calibration OK."));
}

/* Logic for the hopper motor
*
*  param successfullBead
*
*/
void handleHopperMotor(bool successfullBead)
{
  static unsigned long timestamp = 0;
  static bool direction = false;

  // reverse motor if no successful beads after timeout
  if (!successfullBead) {
    if (timediff(timestamp, millis()) > hopperMotorReverseTime) {
      direction = !direction;
      timestamp = millis();
    }
  } else {
    timestamp = millis();
  }

  int photoSensor = analogRead(photoSensorPin);
#ifdef DEBUG_PRINT_PHOTOSENS
  SAFE_PRINTLN();
  SAFE_PRINTLN(photoSensor);
#endif
  if (photoSensor > photoSensorThreshold) {   //PhotoSensor detected no beads in the feeding tube
    startHopperMotor(direction);
  } else {                                    //PhotoSensor detected beads in the feeding tube --> stop the motor
    stopHopperMotor();
    SAFE_PRINT(F("Photosensor detected Beads "));
    SAFE_PRINTLN(photoSensor);
  }
}

unsigned long timediff(unsigned long t1, unsigned long t2)
{
  signed long d = (signed long)t1 - (signed long)t2;
  if (d < 0) d = -d;
  return (unsigned long)d;
}


void addColor() {
  clearMedianColors();
  stopHopperMotor();

  SAFE_PRINT(F("Insert Color to register. Press button when done..."));

  while (digitalRead(setupPin) == LOW) {}

  startHopperMotor(false);
  delay(10000);
  stopHopperMotor();
  for (int i = 0; i < 4; i++) {
    SAFE_PRINT(i + 1); SAFE_PRINT(F("/4: "));
    servoFeedIn();
    delay(500);
    servoWiggleIn();
    delay(200);
    readColorSensor();
    addColorToMedianColors(i);
    servoFeedOut();
  }

  calcMedianAndStore();
  startHopperMotor(false);
}

void servoFeedIn() {
  int low  = servoAngleIn - servoAngleWiggle;
  int high = servoAngleIn + servoAngleWiggle;
  for (int i = low; i < high; i++) {
    servo.write(i);
    delay(100);
  }
}

void servoWiggleIn() {
  int low  = servoAngleIn - servoAngleWiggle * 2;
  int high = servoAngleIn + servoAngleWiggle * 2;
  for (int c = 0; c < 3; c++) {
    servo.write(low);
    delay(100);
    servo.write(high);
    delay(100);
  }
  servo.write(servoAngleIn);
  delay(100);
}

void servoFeedOut() {
  int low  = servoAngleOut - servoAngleWiggle;
  int high = servoAngleOut + servoAngleWiggle;
  for (int count = 0; count < 3; count++) {
    for (int i = low; i < high; i++) {
      servo.write(i);
      delay(100);
    }
    for (int i = high; i > low; i--) {
      servo.write(i);
      delay(50);
    }
  }
  delay(200);
}

/*
 * Read the color sensor into resultRaw[], resultNorm[], and resultHSL[].
 *
 * One getRawData() call fills resultRaw[R,G,B,C] with raw 16-bit counts.
 * resultNorm[] is then computed by normalising each channel by the clear
 * channel (×255), matching what the library's getRGB() would return.
 * ColorConverter::RgbToHsl() converts to H, S, L all in [0..1].
 */
void readColorSensor() {
  delay(200);
  tcs.getRawData(&resultRaw[0], &resultRaw[1], &resultRaw[2], &resultRaw[3]);

  if (resultRaw[3] > 0) {
    resultNorm[0] = (float)resultRaw[0] * 255.0f / resultRaw[3];
    resultNorm[1] = (float)resultRaw[1] * 255.0f / resultRaw[3];
    resultNorm[2] = (float)resultRaw[2] * 255.0f / resultRaw[3];
  } else {
    resultNorm[0] = resultNorm[1] = resultNorm[2] = 0.0f;
  }

  double h, s, l;
  ColorConverter::RgbToHsl(
    (uint8_t)constrain((int)resultNorm[0], 0, 255),
    (uint8_t)constrain((int)resultNorm[1], 0, 255),
    (uint8_t)constrain((int)resultNorm[2], 0, 255),
    h, s, l);

  resultHSL[0] = (float)h;
  resultHSL[1] = (float)s;
  resultHSL[2] = (float)l;
}

/*
 * Print all sensor values for the current reading to Serial.
 * Format mirrors the BeadCalibrate CSV columns so readings can be
 * cross-checked directly against calibration data.
 */
void printBeadReading() {
  float rSum = resultNorm[0] + resultNorm[1] + resultNorm[2];
  float chR  = (rSum > 0.0f) ? resultNorm[0] / rSum : 0.0f;
  float chG  = (rSum > 0.0f) ? resultNorm[1] / rSum : 0.0f;
  float chB  = (rSum > 0.0f) ? resultNorm[2] / rSum : 0.0f;
  bool  sat  = (resultRaw[0] >= 65530 || resultRaw[1] >= 65530 ||
                resultRaw[2] >= 65530 || resultRaw[3] >= 65530);

  SAFE_PRINT(F("  raw  R=")); SAFE_PRINT(resultRaw[0]);
  SAFE_PRINT(F(" G="));       SAFE_PRINT(resultRaw[1]);
  SAFE_PRINT(F(" B="));       SAFE_PRINT(resultRaw[2]);
  SAFE_PRINT(F(" C="));       SAFE_PRINT(resultRaw[3]);
  SAFE_PRINTLN(sat ? F(" [SATURATED]") : F(""));

  SAFE_PRINT(F("  norm Rn=")); SAFE_PRINT(resultNorm[0], 1);
  SAFE_PRINT(F(" Gn="));       SAFE_PRINT(resultNorm[1], 1);
  SAFE_PRINT(F(" Bn="));       SAFE_PRINTLN(resultNorm[2], 1);

  SAFE_PRINT(F("  chroma chR=")); SAFE_PRINT(chR, 4);
  SAFE_PRINT(F(" chG="));         SAFE_PRINT(chG, 4);
  SAFE_PRINT(F(" chB="));         SAFE_PRINTLN(chB, 4);

  SAFE_PRINT(F("  HSL   H=")); SAFE_PRINT(resultHSL[0], 4);
  SAFE_PRINT(F(" S="));        SAFE_PRINT(resultHSL[1], 4);
  SAFE_PRINT(F(" L="));        SAFE_PRINTLN(resultHSL[2], 4);
}

void addColorToMedianColors(int noOfTest) {
  medianHSL[noOfTest][0] = resultHSL[0];
  medianHSL[noOfTest][1] = resultHSL[1];
  medianHSL[noOfTest][2] = resultHSL[2];
}

int getNextFreeArrayPlace() {
  int arrayCounter = 0;
  while (arrayCounter < 16 && storedColors[arrayCounter][0] >= 0.0f) {
    arrayCounter++;
  }
  return arrayCounter;
}

void clearMedianColors() {
  memset(medianHSL, 0, sizeof(medianHSL));
}

/*
 * Compute the mean of the 4 accumulated HSL samples and store.
 * Hue uses circular mean (via sin/cos) to handle the 0/1 wrap-around correctly.
 * S and L use a simple arithmetic mean.
 */
void calcMedianAndStore() {
  // Circular mean for hue (0..1 maps to 0..2π)
  float sinSum = 0.0f, cosSum = 0.0f;
  float sSum   = 0.0f, lSum   = 0.0f;
  for (int j = 0; j < 4; j++) {
    float rad = medianHSL[j][0] * 2.0f * (float)M_PI;
    sinSum += sinf(rad);
    cosSum += cosf(rad);
    sSum   += medianHSL[j][1];
    lSum   += medianHSL[j][2];
  }
  float meanRad = atan2f(sinSum, cosSum);
  if (meanRad < 0.0f) meanRad += 2.0f * (float)M_PI;
  resultHSL[0] = meanRad / (2.0f * (float)M_PI);
  resultHSL[1] = sSum / 4.0f;
  resultHSL[2] = lSum / 4.0f;

  int nextColorNo = getNextFreeArrayPlace();
  storeColor(nextColorNo, resultHSL[0], resultHSL[1], resultHSL[2]);

  SAFE_PRINT(F("Stored color to bank #")); SAFE_PRINT(nextColorNo);
  SAFE_PRINT(F("  H=")); SAFE_PRINT(resultHSL[0], 4);
  SAFE_PRINT(F("  S=")); SAFE_PRINT(resultHSL[1], 4);
  SAFE_PRINT(F("  L=")); SAFE_PRINTLN(resultHSL[2], 4);
}

/*
 * Default color set — placeholder only.
 * The previous values were raw RGBC sensor counts that cannot be directly
 * converted to the normalised HSL space used here.
 * Re-register each color manually via the setup-button flow.
 */
void importDefaultColorSet() {
  SAFE_PRINTLN(F("[WARN] Default color set unavailable: raw RGBC values have been"));
  SAFE_PRINTLN(F("       replaced by HSL. Re-register each color with the setup button."));
}

// Returns color name as a flash string pointer — no RAM allocation.
const __FlashStringHelper* getColorNameFromNo(int colorNo) {
  switch (colorNo) {
    case 0:  return F("Color 0");
    case 1:  return F("Color 1");
    case 2:  return F("Color 2");
    case 3:  return F("Color 3");
    case 4:  return F("Color 4");
    case 5:  return F("Color 5");
    case 6:  return F("Color 6");
    case 7:  return F("Color 7");
    case 8:  return F("Color 8");
    case 9:  return F("Color 9");
    case 10: return F("Color 10");
    case 11: return F("Color 11");
    case 12: return F("Color 12");
    default: return F("Unknown");
  }
}

void storeColor(int index, float h, float s, float l) {
  storedColors[index][0] = h;
  storedColors[index][1] = s;
  storedColors[index][2] = l;
}

/*
 * Euclidean distance in HSL space.
 * Hue difference uses the shorter arc on the circular hue axis.
 */
float colorDistanceHSL(float* c1, float* c2)
{
  float dh = fabsf(c1[0] - c2[0]);
  if (dh > 0.5f) dh = 1.0f - dh;
  float ds = c1[1] - c2[1];
  float dl = c1[2] - c2[2];
  return sqrtf(dh*dh + ds*ds + dl*dl);
}

/*
 * Returns true when the sensor reading matches the empty-tube reference,
 * indicating no bead is present.
 */
boolean nullScan() {
  float dh = fabsf(resultHSL[0] - nullScanHSL[0]);
  if (dh > 0.5f) dh = 1.0f - dh;
  return (dh                                         <= nullScanOffsetH) &&
         (fabsf(resultHSL[1] - nullScanHSL[1])       <= nullScanOffsetS) &&
         (fabsf(resultHSL[2] - nullScanHSL[2])       <= nullScanOffsetL);
}

void setNullScanValues() {
  nullScanHSL[0] = resultHSL[0];
  nullScanHSL[1] = resultHSL[1];
  nullScanHSL[2] = resultHSL[2];

  SAFE_PRINT(F("Calib results H=")); SAFE_PRINT(nullScanHSL[0], 4);
  SAFE_PRINT(F(" S="));             SAFE_PRINT(nullScanHSL[1], 4);
  SAFE_PRINT(F(" L="));             SAFE_PRINTLN(nullScanHSL[2], 4);
}

/*
 * Search storedColors for a slot whose HSL values are within the per-channel
 * thresholds of resultHSL.  Hue comparison uses circular distance.
 * Returns the matching index, or -1 if none found.
 */
int findColorInStorage()
{
  for (int i = 0; i < 16; i++) {
    if (storedColors[i][0] < 0.0f) continue;  // empty slot

    float dh = fabsf(resultHSL[0] - storedColors[i][0]);
    if (dh > 0.5f) dh = 1.0f - dh;
    if (dh > thresholdH) continue;

    if (fabsf(resultHSL[1] - storedColors[i][1]) > thresholdS) continue;

    if (fabsf(resultHSL[2] - storedColors[i][2]) > thresholdL) continue;

    return i;
  }
  return -1;
}

void sortBeadToDynamicArray() {
  bool found = false;
  SAFE_PRINTLN(F("Analyzing Results:"));
  clearMedianColors();

  // if we can still store new colors, do up to 4 retries to get a more stable reading for the new color.
  // Otherwise just do one read and sort to best matching color or unknown.
  int maxRetries = (!allContainerFull() && autoColorCounter < autoSortMaxColors) ? 4 : 1;

  for (int retries = 0; retries < maxRetries; retries++) {
    int index = findColorInStorage();
    if (index != -1) {
      SAFE_PRINT(F("Color match #")); SAFE_PRINT(index);
      SAFE_PRINT(F("  H=")); SAFE_PRINT(storedColors[index][0], 4);
      SAFE_PRINT(F("  S=")); SAFE_PRINT(storedColors[index][1], 4);
      SAFE_PRINT(F("  L=")); SAFE_PRINTLN(storedColors[index][2], 4);

      if (!autoSort) {
        SAFE_PRINT(F("Color is ")); SAFE_PRINT(getColorNameFromNo(index)); SAFE_PRINTLN(F("."));
      }

      SAFE_PRINT(F("Color Distance: ")); SAFE_PRINTLN(colorDistanceHSL(storedColors[index], resultHSL), 4);

      int containerNo = getContainerNo(index);
      SAFE_PRINT(F("move stepper to container No:")); SAFE_PRINTLN(containerNo);
      moveSorterToPosition(containerNo);
      found = true;
      break;
    }
    addColorToMedianColors(retries);
    servoWiggleIn();
    readColorSensor();
  }

  if (!found) {
    // Circular mean over all retry readings
    float sinSum = 0.0f, cosSum = 0.0f, sSum = 0.0f, lSum = 0.0f;
    for (int j = 0; j < 4; j++) {
      float rad = medianHSL[j][0] * 2.0f * (float)M_PI;
      sinSum += sinf(rad);
      cosSum += cosf(rad);
      sSum   += medianHSL[j][1];
      lSum   += medianHSL[j][2];
    }
    float meanRad = atan2f(sinSum, cosSum);
    if (meanRad < 0.0f) meanRad += 2.0f * (float)M_PI;
    resultHSL[0] = meanRad / (2.0f * (float)M_PI);
    resultHSL[1] = sSum / 4.0f;
    resultHSL[2] = lSum / 4.0f;

    SAFE_PRINT(F("not found. Avg H=")); SAFE_PRINT(resultHSL[0], 4);
    SAFE_PRINT(F(" S=")); SAFE_PRINT(resultHSL[1], 4);
    SAFE_PRINT(F(" L=")); SAFE_PRINTLN(resultHSL[2], 4);

    if (autoSort) {
      SAFE_PRINTLN(F("autosort!"));
      if (!allContainerFull() && autoColorCounter < autoSortMaxColors) {
        SAFE_PRINT(F("not allContainerFull. StoreColor "));
        SAFE_PRINTLN(autoColorCounter);
        storeColor(autoColorCounter, resultHSL[0], resultHSL[1], resultHSL[2]);
        int containerNo = getContainerNo(autoColorCounter);
        autoColorCounter++;
        moveSorterToPosition(containerNo);
      } else {
        SAFE_PRINTLN(F("All sort slots full -> dumping to bin 15."));
        moveSorterToPosition(dynamicContainerArraySize - 1);
      }
    } else {
      moveSorterToPosition(dynamicContainerArraySize - 1);
    }
  }
}

int getContainerNo(int colorIndex) {
  int containerNo = -1;

  for (int i = 0; i < dynamicContainerArraySize; i++) {
    if (colorIndex == dynamicContainerArray[i]) {
      SAFE_PRINT(F("Color found in container array: ")); SAFE_PRINTLN(dynamicContainerArray[i]);
      containerNo = i;
      break;
    }
  }

  if (containerNo == -1) {
    if (allContainerFull()) {
      containerNo = 15;
    } else {
      containerNo = getNextContainerNo(colorIndex);
    }
  }

  return containerNo;
}

int getNextContainerNo(int colorIndex) {
  SAFE_PRINTLN(F("finding container for new color"));
  int arrayCounter = 0;

  while ((arrayCounter < dynamicContainerArraySize - 1) && (dynamicContainerArray[arrayCounter] > -1)) {
    arrayCounter++;
  }
  dynamicContainerArray[arrayCounter] = colorIndex;

  SAFE_PRINT(F("Return Color Index: ")); SAFE_PRINTLN(colorIndex);
  return arrayCounter;
}

bool allContainerFull() {
  int arrayCounter = 0;
  while (dynamicContainerArray[arrayCounter] > -1) {
    arrayCounter++;
  }
  return arrayCounter >= 15; // 15 is reserved for non sortable
}

void moveSorterToPosition(int position) {
  int currentPos     = stepper.currentPosition() / stepperMulti;
  int diffToPosition = position - currentPos;

  if (diffToPosition > 8) {       // motor can go in negative direction over the zero
    position = currentPos + diffToPosition - 16;
  } else if (diffToPosition < -8) { // motor can go in positive direction over the 15
    position = currentPos + diffToPosition + 16;
  }
  position *= stepperMulti;

  stepper.moveTo(position);
  stepper.runToPosition();

  // correct stepper position into usual range:
  stepper.setCurrentPosition(stepper.currentPosition() % (16 * stepperMulti));
}

void stopHopperMotor() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(GSM2, motorSpeed);
}

void startHopperMotor(bool dir) {
  if (dir) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  analogWrite(GSM2, motorSpeed);
}

// =============================================================================
// INTERACTIVE DEBUG MODE
// Activated by holding the setup button during the 2s boot window.
// Cycles through three test steps; triple press enters Calibration Mode.
// =============================================================================

/*
 * Block until the button is pressed and released.
 * Returns 1 for single, 2 for double, 3 for triple press (each within 500ms).
 */
int waitForButtonPress() {
  while (digitalRead(setupPin) == LOW) {}   // wait for press
  delay(50);                                // debounce
  while (digitalRead(setupPin) == HIGH) {}  // wait for release
  delay(50);
  // Check for a second press within 500ms
  unsigned long t = millis();
  while (millis() - t < 500) {
    if (digitalRead(setupPin) == HIGH) {
      delay(50);
      while (digitalRead(setupPin) == HIGH) {}
      delay(50);
      // Check for a third press within 500ms
      unsigned long t2 = millis();
      while (millis() - t2 < 500) {
        if (digitalRead(setupPin) == HIGH) {
          delay(50);
          while (digitalRead(setupPin) == HIGH) {}
          delay(50);
          return 3;
        }
      }
      return 2;
    }
  }
  return 1;
}

/*
 * Poll for a button press during a timeoutMs window.
 * Returns 0=none, 1=single, 2=double, 3=triple press (each within 500ms).
 */
int checkButton(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (digitalRead(setupPin) == HIGH) {
      delay(50);
      while (digitalRead(setupPin) == HIGH) {}
      delay(50);
      unsigned long t = millis();
      while (millis() - t < 500) {
        if (digitalRead(setupPin) == HIGH) {
          delay(50);
          while (digitalRead(setupPin) == HIGH) {}
          delay(50);
          unsigned long t2 = millis();
          while (millis() - t2 < 500) {
            if (digitalRead(setupPin) == HIGH) {
              delay(50);
              while (digitalRead(setupPin) == HIGH) {}
              delay(50);
              return 3;
            }
          }
          return 2;
        }
      }
      return 1;
    }
  }
  return 0;
}

/*
 * Move stepper to an absolute position while watching for a button press.
 * Returns true if position was reached, false if interrupted.
 * On interrupt the stepper decelerates to a stop and the press is consumed.
 */
bool runStepperTo(long position) {
  stepper.moveTo(position);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    if (digitalRead(setupPin) == HIGH) {
      stepper.stop();           // sets target to deceleration endpoint
      stepper.runToPosition();  // finishes deceleration
      delay(50);
      while (digitalRead(setupPin) == HIGH) {}
      delay(50);
      return false;
    }
  }
  return true;
}

// -----------------------------------------------------------------------------
// DEBUG STEP 1 — Hopper Motor
// Single press : switch direction
// Double press : advance to step 2
// Triple press : enter Calibration Mode
// -----------------------------------------------------------------------------
int debugStep1_Hopper() {
  SAFE_PRINTLN();
  SAFE_PRINTLN(F("--- [DEBUG 1/3] Hopper Motor Test ---"));
  SAFE_PRINTLN(F("  Motor running FORWARD."));
  SAFE_PRINTLN(F("  Single press : switch direction"));
  SAFE_PRINTLN(F("  Double press : next test (Servo / Color Sensor)"));
  SAFE_PRINTLN(F("  Triple press : Calibration Mode"));

  bool dir = false;
  startHopperMotor(dir);

  while (true) {
    int btn = waitForButtonPress();
    if (btn == 1) {
      dir = !dir;
      startHopperMotor(dir);
      SAFE_PRINT(F("  Direction switched -> "));
      SAFE_PRINTLN(dir ? F("REVERSE") : F("FORWARD"));
    } else if (btn == 3) {
      stopHopperMotor();
      SAFE_PRINTLN(F("  Motor stopped. Entering Calibration Mode..."));
      return 4;
    } else {
      stopHopperMotor();
      SAFE_PRINTLN(F("  Motor stopped. Advancing to next test..."));
      return 2;
    }
  }
}

// -----------------------------------------------------------------------------
// DEBUG STEP 2 — Servo / Color Sensor
// Loop: servoFeedOut -> servoFeedIn -> readColorSensor -> print HSL results
// Single press (during/after cycle) : stop at end of current cycle
// Single press (when stopped)       : restart loop
// Double press                      : advance to step 3
// Triple press                      : enter Calibration Mode
// -----------------------------------------------------------------------------
int debugStep2_ServoColor() {
  SAFE_PRINTLN();
  SAFE_PRINTLN(F("--- [DEBUG 2/3] Servo / Color Sensor Test ---"));

  if (!tcs.begin()) {
    SAFE_PRINTLN(F("  ERROR: TCS34725 not found! Cannot run this test."));
    SAFE_PRINTLN(F("  Press button (any) to advance to next test."));
    waitForButtonPress();
    return 3;
  }

  SAFE_PRINTLN(F("  Loop running continuously. Results printed each cycle."));
  SAFE_PRINTLN(F("  Single press : stop after current cycle"));
  SAFE_PRINTLN(F("  Double press : next test (Stepper)"));
  SAFE_PRINTLN(F("  Triple press : Calibration Mode"));
  SAFE_PRINTLN(F("  (when stopped) Single press : restart | Double press : next test | Triple press : Calibration Mode"));

  bool running = true;

  while (true) {
    if (running) {
      SAFE_PRINT(F("[Servo] FeedOut"));
      servoFeedOut();
      delay(500);
      SAFE_PRINT(F("[Servo] FeedIn"));
      servoFeedIn();
      delay(500);
      SAFE_PRINT(F("[Servo] WiggleIn"));
      servoWiggleIn();
      delay(500);
      readColorSensor();
      printBeadReading();

      // Brief window to catch a press at the end of each cycle
      int btn = checkButton(300);
      if (btn == 1) {
        running = false;
        SAFE_PRINTLN(F("  Loop stopped. Single=restart | Double=next test | Triple=Calib Mode."));
      } else if (btn == 3) {
        SAFE_PRINTLN(F("  Entering Calibration Mode..."));
        return 4;
      } else if (btn == 2) {
        SAFE_PRINTLN(F("  Advancing to Stepper test..."));
        return 3;
      }
    } else {
      // Paused — wait for explicit user action
      int btn = waitForButtonPress();
      if (btn == 1) {
        running = true;
        SAFE_PRINTLN(F("  Loop restarted."));
      } else if (btn == 3) {
        SAFE_PRINTLN(F("  Entering Calibration Mode..."));
        return 4;
      } else {
        SAFE_PRINTLN(F("  Advancing to Stepper test..."));
        return 3;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// DEBUG STEP 3 — Stepper
// Runs a full 360 turn then visits all 16 slots with 500ms pauses.
// Single press (during move)  : stop, decelerate to rest
// Single press (when stopped) : save current position as 0, restart cycle
// Double press                : back to step 1
// Triple press                : enter Calibration Mode
// -----------------------------------------------------------------------------
int debugStep3_Stepper() {
  SAFE_PRINTLN();
  SAFE_PRINTLN(F("--- [DEBUG 3/3] Stepper Test ---"));
  SAFE_PRINTLN(F("  Single press during move       : stop"));
  SAFE_PRINTLN(F("  Single press when stopped      : save position as 0, restart cycle"));
  SAFE_PRINTLN(F("  Double press (any time)        : back to Hopper Motor test"));
  SAFE_PRINTLN(F("  Triple press (any time)        : Calibration Mode"));

  while (true) {
    stepper.setCurrentPosition(0);
    bool interrupted = false;

    // Full 360 turn, then return to home
    SAFE_PRINTLN(F("  Starting full 360 turn..."));
    if (!runStepperTo(stepperStepsPerRot * stepperMicroStepping)) {
      interrupted = true;
    } else {
      delay(500);
      SAFE_PRINTLN(F("  Returning to slot 0..."));
      if (!runStepperTo(0)) {
        interrupted = true;
      }
    }

    if (!interrupted) {
      delay(500);
      SAFE_PRINTLN(F("  360 turn done. Starting slot cycle..."));

      for (int slot = 0; slot < numContainerSlots && !interrupted; slot++) {
        SAFE_PRINT(F("  -> Slot ")); SAFE_PRINT(slot);
        SAFE_PRINT(F(" (step pos ")); SAFE_PRINT(slot * stepperMulti); SAFE_PRINTLN(')');

        if (!runStepperTo(slot * stepperMulti)) {
          interrupted = true;
          break;
        }

        // 500ms pause between slots; also detects button presses
        int btn = checkButton(500);
        if (btn == 1) {
          interrupted = true;
        } else if (btn == 3) {
          SAFE_PRINTLN(F("  Entering Calibration Mode..."));
          return 4;
        } else if (btn == 2) {
          SAFE_PRINTLN(F("  Returning to Hopper Motor test..."));
          return 1;
        }
      }

      // Return to slot 0 after visiting all slots
      if (!interrupted) {
        SAFE_PRINTLN(F("  Slot cycle done. Returning to slot 0..."));
        runStepperTo(0);
        delay(500);
      }
    }

    if (interrupted) {
      SAFE_PRINTLN(F("  Movement stopped."));
      SAFE_PRINTLN(F("  Single press : save position as 0, restart cycle"));
      SAFE_PRINTLN(F("  Double press : back to Hopper Motor test"));
      SAFE_PRINTLN(F("  Triple press : Calibration Mode"));
      int btn = waitForButtonPress();
      if (btn == 1) {
        stepper.setCurrentPosition(0);
        SAFE_PRINTLN(F("  Position saved as 0. Restarting cycle..."));
        // continue while(true) -> restart
      } else if (btn == 3) {
        SAFE_PRINTLN(F("  Entering Calibration Mode..."));
        return 4;
      } else {
        SAFE_PRINTLN(F("  Returning to Hopper Motor test..."));
        return 1;
      }
    } else {
      // Full cycle completed normally
      SAFE_PRINTLN(F("  Slot cycle complete!"));
      SAFE_PRINTLN(F("  Single press : restart from 0 | Double press : back to Hopper Motor test | Triple press : Calibration Mode"));
      int btn = waitForButtonPress();
      if (btn == 1) {
        SAFE_PRINTLN(F("  Restarting cycle..."));
        // continue while(true) -> restart
      } else if (btn == 3) {
        SAFE_PRINTLN(F("  Entering Calibration Mode..."));
        return 4;
      } else {
        SAFE_PRINTLN(F("  Returning to Hopper Motor test..."));
        return 1;
      }
    }
  }
}

// =============================================================================
// CALIBRATION MODE
// Entered from debug mode via triple press, or (future) directly at boot.
// Double press : advance to next calibration step
// Triple press : exit back to Debug Mode
// =============================================================================

/*
 * Calibration step 1 — Servo positions.
 * Cycles Out -> In -> WiggleIn -> Out -> ... with each single press.
 * Returns the next calibration step number, or -1 to exit calibration mode.
 */
int calibStep1_Servo() {
  SAFE_PRINTLN();
  SAFE_PRINTLN(F("--- [CALIB 1] Servo Calibration ---"));
  SAFE_PRINTLN(F("  Single press : Out -> In -> WiggleIn -> Out -> ..."));
  SAFE_PRINTLN(F("  Double press : next calibration step"));
  SAFE_PRINTLN(F("  Triple press : back to Debug Mode"));

  // state 0=Out, 1=In, 2=WiggleIn
  int state = 0;
  servo.write(servoAngleOut);
  SAFE_PRINT(F("  Servo -> OUT (angle ")); SAFE_PRINT(servoAngleOut); SAFE_PRINTLN(')');

  while (true) {
    int btn = waitForButtonPress();
    if (btn == 1) {
      state = (state + 1) % 3;
      if (state == 0) {
        servo.write(servoAngleOut);
        SAFE_PRINT(F("  Servo -> OUT (angle "));
        SAFE_PRINT(servoAngleOut);
        SAFE_PRINTLN(')');
      } else if (state == 1) {
        servo.write(servoAngleIn);
        SAFE_PRINT(F("  Servo -> IN  (angle "));
        SAFE_PRINT(servoAngleIn);
        SAFE_PRINTLN(')');
      } else {
        SAFE_PRINTLN(F("  Servo -> WiggleIn"));
        servoWiggleIn();
        SAFE_PRINTLN(F("  WiggleIn done."));
      }
    } else if (btn == 2) {
      return 2;   // advance to next calibration step
    } else {      // triple press
      return -1;  // exit calibration mode
    }
  }
}

/*
 * Calibration mode entry point.
 * Runs calibration steps in sequence; returns when triple press exits to debug.
 */
void runCalibrationMode() {
  SAFE_PRINTLN();
  SAFE_PRINTLN(F("*************************************"));
  SAFE_PRINTLN(F("*       CALIBRATION MODE            *"));
  SAFE_PRINTLN(F("*  Single press : action            *"));
  SAFE_PRINTLN(F("*  Double press : next calib step   *"));
  SAFE_PRINTLN(F("*  Triple press : back to Debug     *"));
  SAFE_PRINTLN(F("*************************************"));

  int step = 1;
  while (true) {
    int next;
    switch (step) {
      case 1: next = calibStep1_Servo(); break;
      default: next = -1; break;
    }
    if (next == -1) {
      SAFE_PRINTLN(F("  Exiting Calibration Mode -> back to Debug Mode."));
      return;
    }
    // No further calibration steps defined yet — wrap back to step 1
    SAFE_PRINTLN(F("  No further calibration steps. Returning to step 1."));
    step = 1;
  }
}

// -----------------------------------------------------------------------------
// ENTRY POINT — called from setup() when button is held at boot.
// Cycles: Hopper (1) -> Servo/Color (2) -> Stepper (3) -> Hopper (1) ...
// Triple press from any step enters Calibration Mode; returns to step 1 after.
// Each step function returns the next step number.
// This function never returns; program halts in debug mode.
// -----------------------------------------------------------------------------
void runInteractiveDebug() {
  // Wait for button to be released before starting
  while (digitalRead(setupPin) == HIGH) {}
  delay(100);

  SAFE_PRINTLN();
  SAFE_PRINTLN(F("*************************************"));
  SAFE_PRINTLN(F("*    INTERACTIVE DEBUG MODE         *"));
  SAFE_PRINTLN(F("*  Single press : action in test    *"));
  SAFE_PRINTLN(F("*  Double press : next / prev test  *"));
  SAFE_PRINTLN(F("*  Triple press : Calibration Mode  *"));
  SAFE_PRINTLN(F("*************************************"));

  int step = 1;
  while (true) {
    switch (step) {
      case 1: step = debugStep1_Hopper();     break;
      case 2: step = debugStep2_ServoColor(); break;
      case 3: step = debugStep3_Stepper();    break;
      case 4: runCalibrationMode(); step = 1; break;
    }
  }
}
