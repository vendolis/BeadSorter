#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// Stubs so VS Code IntelliSense resolves AVR-specific macros.
// These lines are never compiled by the Arduino toolchain.
#ifdef __INTELLISENSE__
  #define F(x) x
  #define PSTR(x) x
  #define snprintf_P snprintf
  typedef char __FlashStringHelper;  // makes const __FlashStringHelper* == const char*
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
#define hopperMotorReverseTime  9000 //in ms - time of ~ revolution

#define setupPin 11 //Setup

#define photoLEDPin 9 //Photo Sensor LED
#define photoSensorPin A0 //Photo Sensor
#define photoSensorThreshold 600
#define photoSensorCalibMinDiff 100  // minimum on/off difference to consider LED functional

#define nullScanOffset 150

#define servoAngleIn 37 //Servo
#define servoAngleOut 61
#define servoAngleWiggle 2
#define servoPin 8

// #define DEBUG_PROG 1 // Stepper test
// #define DEBUG_PROG 2 // Color Sensor test
// #define DEBUG_PRINT_PHOTOSENS


// Parameters: https://learn.adafruit.com/adafruit-color-sensors/program-it
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_16X);
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X);

//Max allowed color difference = thresholdFactor * colorValue + offset (0.08/40)
//dark colors 0.02 + 10
float thresholdFactor = 0.04;
int offset = 40;

Servo servo;
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

bool autoSort = true;
unsigned int resultColor[4] = {0, 0, 0, 0};
unsigned int medianColors[4][4];
unsigned int storedColors[16][4];

bool calibrateNullScan = true;
int nullScanValues[4] = {6618, 1860, 2282, 2116}; //adjust these if no calibration

uint8_t  autoColorCounter = 0;
uint16_t beadCounter = 0;

const int dynamicContainerArraySize = 16;
// -1 means empty, 666 means blocked (there are only 12 total slots available due to the layout.)
// When starting the program, the arm should be over the middle slot of one of the three on the sides.
// The Unknown color slot (15) will then be the slot to the right of it.
int dynamicContainerArray[dynamicContainerArraySize] = { -1, -1, 666, -1, -1, -1, 666, -1, -1, -1, 666, -1, -1, -1, 666 , -1 }; 

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("=== BeadSorter starting ==="));

  // Setup Hopper Motor Pins
  Serial.println(F("[INIT] Hopper motor pins..."));
  pinMode(GSM2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.println(F("[INIT] Setup button pin..."));
  pinMode(setupPin, INPUT);

  Serial.println(F("[INIT] Photo sensor LED pin..."));
  pinMode(photoLEDPin, OUTPUT);
  digitalWrite(photoLEDPin, HIGH); // LED on by default

  Serial.println(F("[INIT] Servo..."));
  servo.attach(servoPin);
  servo.write(servoAngleIn);

  Serial.print(F("[INIT] Stepper (maxSpeed=")); Serial.print(stepperMaxSpeed);
  Serial.print(F(", accel=")); Serial.print(stepperAccel); Serial.println(F(")..."));
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAccel);
  stepper.setCurrentPosition(0);

#if defined DEBUG_PROG && DEBUG_PROG == 1
  // Full 360 turn, then return to home
  Serial.println(F("[DEBUG] Stepper: full 360 turn..."));
  stepper.moveTo(stepperStepsPerRot * stepperMicroStepping);
  stepper.runToPosition();
  delay(500);
  stepper.moveTo(0);
  stepper.runToPosition();
  delay(500);

  // Visit each output slot once
  Serial.println(F("[DEBUG] Stepper: cycling all output slots..."));
  for (int slot = 0; slot < numContainerSlots; slot++) {
    Serial.print(F("[DEBUG] Slot ")); Serial.println(slot);
    stepper.moveTo(slot * stepperMulti);
    stepper.runToPosition();
    delay(500);
  }

  // Return to slot 0
  Serial.println(F("[DEBUG] Returning to slot 0..."));
  stepper.moveTo(0);
  stepper.runToPosition();
  Serial.println(F("[DEBUG] Stepper test complete."));
  while (1);
#endif

  // Hold setup button during boot to enter interactive debug mode (2s window)
  {
    Serial.println(F("[INIT] Hold setup button to enter interactive debug mode (2s)..."));
    unsigned long t = millis();
    bool debugRequested = false;
    while (millis() - t < 2000) {
      if (digitalRead(setupPin) == HIGH) {
        Serial.println(F("[Debug] Setup Button registered."));
        debugRequested = true;
        break;
      }
    }
    if (debugRequested) {
      Serial.println(F("[Debug] Entering Debug mode."));
      runInteractiveDebug(); // never returns
      while (1);
    }
    Serial.println(F("[INIT] Continuing normal startup."));
  }

  calibratePhotoSensor();

  if (tcs.begin()) {
    Serial.println(F("[INIT] TCS34725 color sensor found."));
  } else {
    Serial.println(F("ERROR: TCS34725 not found, exiting!"));
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
    Serial.print(F("Clear:")); Serial.print(resultColor[0]);
    Serial.print(F("\tRed:"));   Serial.print(resultColor[1]);
    Serial.print(F("\tGreen:")); Serial.print(resultColor[2]);
    Serial.print(F("\tBlue:"));  Serial.println(resultColor[3]);
    delay(800);
    servoFeedOut();
  }
#endif

  Serial.print(F("[INIT] Mode: "));
  Serial.println(autoSort ? F("autoSort") : F("manual (preset colors)"));
  if (!autoSort) {
    Serial.println(F("[INIT] Importing default color set..."));
    importDefaultColorSet();
    Serial.println(F("[INIT] Default colors imported."));
  }

  if (calibrateNullScan) {
    Serial.println(F("[INIT] Null scan calibration: cycling servo..."));
    for (int i = 0; i < 6; i++) {
      Serial.print(F("  cycle ")); Serial.print(i + 1); Serial.println(F("/6"));
      servoFeedIn();
      servoFeedOut();
    }
    servoFeedIn();
    readColorSensor();
    setNullScanValues();
    Serial.println(F("[INIT] Null scan calibration done."));
  } else {
    Serial.println(F("[INIT] Null scan calibration skipped (using hardcoded values)."));
  }

  Serial.println(F("=== Setup complete. Starting continuous color scan. ==="));
  Serial.println();
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
    // for manual colors, there are more restricted thresholds
    autoSort = false;
    thresholdFactor = 0.03;
    offset = 20;
    addColor();
  }

  servoFeedIn();
  delay(500);
  servoWiggleIn();
  delay(200);
  readColorSensor();

  if (!nullScan()) {
    Serial.println();
    Serial.print(F("Beads analyzed: ")); Serial.println(beadCounter++);
    Serial.print(F("\tClear:")); Serial.print(resultColor[0]);
    Serial.print(F("\tRed:"));   Serial.print(resultColor[1]);
    Serial.print(F("\tGreen:")); Serial.print(resultColor[2]);
    Serial.print(F("\tBlue:"));  Serial.println(resultColor[3]);
    sortBeadToDynamicArray();
    successfullBead = true;
  } else {
    Serial.print('.');
    successfullBead = false;
  }

  servoFeedOut();
}

/*
*   print all color tables to serial for debugging.
*/
void printTables() {
  char line[40];  // max formatted line is ~35 chars

  Serial.println(F("Nullscan:"));
  snprintf_P(line, sizeof(line), PSTR("%2d c=%5u r=%5u g=%5u b=%5u"),
             0, nullScanValues[0], nullScanValues[1], nullScanValues[2], nullScanValues[3]);
  Serial.println(line);

  Serial.println(F("Stored colors:"));
  for (int i = 0; i < 16; i++) {
    snprintf_P(line, sizeof(line), PSTR("%2d c=%5u r=%5u g=%5u b=%5u"),
               i, storedColors[i][0], storedColors[i][1], storedColors[i][2], storedColors[i][3]);
    Serial.println(line);
  }

  Serial.println(F("Container array:"));
  for (int i = 0; i < dynamicContainerArraySize; i++) {
    Serial.print(dynamicContainerArray[i]);
    Serial.print(' ');
  }
  Serial.println();
}


/*
*  Calibrate photo sensor by toggling the LED 3 times.
*  Compares average readings with LED on vs off.
*  Halts with error if the difference is below photoSensorCalibMinDiff,
*  which indicates a broken LED or blocked line of sight.
*/
void calibratePhotoSensor() {
  Serial.println(F("[INIT] Calibrating photo sensor..."));
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

  Serial.print(F("Photo sensor calib: LED on="));
  Serial.print(avgOn);
  Serial.print(F(", LED off="));
  Serial.print(avgOff);
  Serial.print(F(", diff="));
  Serial.println(abs(avgOn - avgOff));

  if (abs(avgOn - avgOff) < photoSensorCalibMinDiff) {
    Serial.println(F("ERROR: Photo sensor calib failed! LED broken or line of sight blocked."));
    while (1); // halt
  }

  Serial.println(F("Photo sensor calibration OK."));
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
  Serial.println();
  Serial.println(photoSensor);
#endif
  if (photoSensor > photoSensorThreshold) {   //PhotoSensor detected no beads in the feeding tube
    startHopperMotor(direction);
  } else {                                    //PhotoSensor detected beads in the feeding tube --> stop the motor
    stopHopperMotor();
    Serial.print(F("Photosensor detected Beads "));
    Serial.println(photoSensor);
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

  Serial.print(F("Insert Color to register. Press button when done..."));

  while (digitalRead(setupPin) == LOW) {}

  startHopperMotor(false);
  delay(10000);
  stopHopperMotor();
  for (int i = 0; i < 4; i++) {
    Serial.print(i + 1); Serial.print(F("/4: "));
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

void readColorSensor() {
  // Sensor returns R G B and Clear value
  uint16_t clearcol, red, green, blue;
  delay(200); // Farbmessung dauert c. 50ms
  tcs.getRawData(&red, &green, &blue, &clearcol);
  resultColor[0] = (unsigned int)clearcol;
  resultColor[1] = (unsigned int)red;
  resultColor[2] = (unsigned int)green;
  resultColor[3] = (unsigned int)blue;
}

void addColorToMedianColors(int noOfTest) {
  medianColors[noOfTest][0] += resultColor[0];
  medianColors[noOfTest][1] += resultColor[1];
  medianColors[noOfTest][2] += resultColor[2];
  medianColors[noOfTest][3] += resultColor[3];
}

int getNextFreeArrayPlace() {
  int arrayCounter = 0;
  while (
    (arrayCounter < 16) &&
    (storedColors[arrayCounter][0] > 0) &&
    (storedColors[arrayCounter][1] > 0) &&
    (storedColors[arrayCounter][2] > 0) &&
    (storedColors[arrayCounter][3] > 0)) {
    arrayCounter++;
  }
  return arrayCounter;
}

void clearMedianColors() {
  memset(medianColors, 0, sizeof(medianColors));
}

void calcMedianAndStore() { //  this is not a "median" this is a "mean"
  for (int i = 0; i < 4; i++) {
    long temp = 0;
    for (int j = 0; j < 4; j++) {
      temp += medianColors[j][i];
    }
    resultColor[i] = temp / 4;
  }

  int nextColorNo = getNextFreeArrayPlace();
  for (int i = 0; i < 4; i++) {
    storedColors[nextColorNo][i] = resultColor[i];
  }

  Serial.print(F("Stored color to bank #")); Serial.println(nextColorNo);
}

void importDefaultColorSet() {
  storeColor(0,  -1, 1032, 854,  766);  //Orange
  storeColor(1,  -1, 1331, 1321, 918);  //Bright Yellow
  storeColor(2,  -1, 532,  914,  797);  //Green
  storeColor(3,  -1, 512,  845,  967);  //Dark Blue
  storeColor(4,  -1, 1257, 1151, 1129); //Rose
  storeColor(5,  -1, 1265, 1215, 1100); //Skin
  storeColor(6,  -1, 756,  757,  721);  //Red
  storeColor(7,  -1, 1525, 1858, 1735); //White
  storeColor(8,  -1, 512,  742,  700);  //Black
  storeColor(9,  -1, 939,  1462, 1101); //Lime Green
  storeColor(10, -1, 712,  1298, 1121); //Mint Green
  storeColor(11, -1, 1213, 975,  914);  //Coral
  storeColor(12, -1, 1185, 1115, 876);  //Dark Yellow
}

// Returns color name as a flash string pointer — no RAM allocation.
const __FlashStringHelper* getColorNameFromNo(int colorNo) {
  switch (colorNo) {
    case 0:  return F("Orange");
    case 1:  return F("Bright Yellow");
    case 2:  return F("Green");
    case 3:  return F("Dark Blue");
    case 4:  return F("Rose");
    case 5:  return F("Skin");
    case 6:  return F("Red");
    case 7:  return F("White");
    case 8:  return F("Black");
    case 9:  return F("Lime Green");
    case 10: return F("Mint Green");
    case 11: return F("Coral");
    case 12: return F("Dark Yellow");
    default: return F("Unknown");
  }
}

void storeColor(int index, int Clear, int red, int green, int blue) {
  storedColors[index][0] = Clear;
  storedColors[index][1] = red;
  storedColors[index][2] = green;
  storedColors[index][3] = blue;
}

unsigned int colorDistance(unsigned int color1[], unsigned int color2[])
{
  long long sum = 0;
  for (int i = 0; i < 4; i++) {
    long long d = (long)color1[i] - (long)color2[i];
    sum += d * d;
  }
  return (unsigned int)sqrt(sum);
}

boolean nullScan() {
  return (
    (resultColor[0] > (nullScanValues[0] - nullScanOffset) && resultColor[0] < (nullScanValues[0] + nullScanOffset)) &&
    (resultColor[1] > (nullScanValues[1] - nullScanOffset) && resultColor[1] < (nullScanValues[1] + nullScanOffset)) &&
    (resultColor[2] > (nullScanValues[2] - nullScanOffset) && resultColor[2] < (nullScanValues[2] + nullScanOffset)) &&
    (resultColor[3] > (nullScanValues[3] - nullScanOffset) && resultColor[3] < (nullScanValues[3] + nullScanOffset)));
}

void setNullScanValues() {
  nullScanValues[0] = resultColor[0];
  nullScanValues[1] = resultColor[1];
  nullScanValues[2] = resultColor[2];
  nullScanValues[3] = resultColor[3];

  Serial.print(F("Calib results: "));
  Serial.print(nullScanValues[0]); Serial.print(' ');
  Serial.print(nullScanValues[1]); Serial.print(' ');
  Serial.print(nullScanValues[2]); Serial.print(' ');
  Serial.println(nullScanValues[3]);
}

int findColorInStorage()
{
  for (int i = 0; i < 16; i++) {
    int threshold  = thresholdFactor * resultColor[0] + offset;
    int upperLimit = resultColor[0] + threshold;
    int lowerLimit = resultColor[0] - threshold;
    if (storedColors[i][0] >= lowerLimit && storedColors[i][0] <= upperLimit) {
      threshold  = thresholdFactor * resultColor[1] + offset;
      upperLimit = resultColor[1] + threshold;
      lowerLimit = resultColor[1] - threshold;
      if (storedColors[i][1] >= lowerLimit && storedColors[i][1] <= upperLimit) {
        threshold  = thresholdFactor * resultColor[2] + offset;
        upperLimit = resultColor[2] + threshold;
        lowerLimit = resultColor[2] - threshold;
        if (storedColors[i][2] >= lowerLimit && storedColors[i][2] <= upperLimit) {
          threshold  = thresholdFactor * resultColor[3] + offset;
          upperLimit = resultColor[3] + threshold;
          lowerLimit = resultColor[3] - threshold;
          if (storedColors[i][3] >= lowerLimit && storedColors[i][3] <= upperLimit) {
            return i;
          }
        }
      }
    }
  }
  return -1;
}

void sortBeadToDynamicArray() {
  bool found = false;
  Serial.println(F("Analyzing Results:"));
  clearMedianColors();

  for (int retries = 0; retries < 4; retries++) {
    int index = findColorInStorage();
    if (index != -1) {
      Serial.print(F("Color is #")); Serial.println(storedColors[index][0]);
      if (autoSort) {
        Serial.print(F("Color is R:")); Serial.print(storedColors[index][1]);
        Serial.print(F(" G:"));         Serial.print(storedColors[index][2]);
        Serial.print(F(" B:"));         Serial.println(storedColors[index][3]);
      } else {
        Serial.print(F("Color is #")); Serial.print(getColorNameFromNo(index)); Serial.println(F("."));
      }

      Serial.print(F("Color Distance is: ")); Serial.println(colorDistance(storedColors[index], resultColor));

      int containerNo = getContainerNo(index);
      Serial.print(F("move stepper to container No:")); Serial.println(containerNo);
      moveSorterToPosition(containerNo);
      found = true;
      break;
    }
    addColorToMedianColors(retries);
    servoWiggleIn();
    readColorSensor();
  }

  if (!found) {
    char line[32];
    // generate mean value of measurements for storage
    for (int i = 0; i < 4; i++) {
      long temp = 0;
      for (int j = 0; j < 4; j++) {
        temp += medianColors[j][i];
        snprintf_P(line, sizeof(line), PSTR("i=%d,j=%d t=%ld m=%u"), i, j, temp, medianColors[j][i]);
        Serial.println(line);
      }
      resultColor[i] = temp / 4;
      snprintf_P(line, sizeof(line), PSTR("store=%u"), resultColor[i]);
      Serial.println(line);
    }

    Serial.println(F("not found"));

    if (autoSort) {
      Serial.println(F("autosort!"));
      if (!allContainerFull()) {
        Serial.print(F("not allContainerFull. StoreColor "));
        Serial.println(autoColorCounter);
        storeColor(autoColorCounter, resultColor[0], resultColor[1], resultColor[2], resultColor[3]);
        int containerNo = getContainerNo(autoColorCounter);
        autoColorCounter++;
        moveSorterToPosition(containerNo);
      } else {
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
      Serial.print(F("Color found in container array: ")); Serial.println(dynamicContainerArray[i]);
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
  Serial.println(F("finding container for new color"));
  int arrayCounter = 0;

  while ((arrayCounter < dynamicContainerArraySize - 1) && (dynamicContainerArray[arrayCounter] > -1)) {
    arrayCounter++;
  }
  dynamicContainerArray[arrayCounter] = colorIndex;

  Serial.print(F("Return Color Index: ")); Serial.println(colorIndex);
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
  Serial.println();
  Serial.println(F("--- [DEBUG 1/3] Hopper Motor Test ---"));
  Serial.println(F("  Motor running FORWARD."));
  Serial.println(F("  Single press : switch direction"));
  Serial.println(F("  Double press : next test (Servo / Color Sensor)"));
  Serial.println(F("  Triple press : Calibration Mode"));

  bool dir = false;
  startHopperMotor(dir);

  while (true) {
    int btn = waitForButtonPress();
    if (btn == 1) {
      dir = !dir;
      startHopperMotor(dir);
      Serial.print(F("  Direction switched -> "));
      Serial.println(dir ? F("REVERSE") : F("FORWARD"));
    } else if (btn == 3) {
      stopHopperMotor();
      Serial.println(F("  Motor stopped. Entering Calibration Mode..."));
      return 4;
    } else {
      stopHopperMotor();
      Serial.println(F("  Motor stopped. Advancing to next test..."));
      return 2;
    }
  }
}

// -----------------------------------------------------------------------------
// DEBUG STEP 2 — Servo / Color Sensor
// Loop: servoFeedOut -> servoFeedIn -> readColorSensor -> print results
// Single press (during/after cycle) : stop at end of current cycle
// Single press (when stopped)       : restart loop
// Double press                      : advance to step 3
// Triple press                      : enter Calibration Mode
// -----------------------------------------------------------------------------
int debugStep2_ServoColor() {
  Serial.println();
  Serial.println(F("--- [DEBUG 2/3] Servo / Color Sensor Test ---"));

  if (!tcs.begin()) {
    Serial.println(F("  ERROR: TCS34725 not found! Cannot run this test."));
    Serial.println(F("  Press button (any) to advance to next test."));
    waitForButtonPress();
    return 3;
  }

  Serial.println(F("  Loop running continuously. Results printed each cycle."));
  Serial.println(F("  Single press : stop after current cycle"));
  Serial.println(F("  Double press : next test (Stepper)"));
  Serial.println(F("  Triple press : Calibration Mode"));
  Serial.println(F("  (when stopped) Single press : restart | Double press : next test | Triple press : Calibration Mode"));

  bool running = true;

  while (true) {
    if (running) {
      Serial.print(F("[Servo] FeedOut"));
      servoFeedOut();
      delay(500);
      Serial.print(F("[Servo] FeedIn"));
      servoFeedIn();
      delay(500);
      Serial.print(F("[Servo] WiggleIn"));
      servoWiggleIn();
      delay(500);
      readColorSensor();
      Serial.print(F("  C:")); Serial.print(resultColor[0]);
      Serial.print(F("  R:")); Serial.print(resultColor[1]);
      Serial.print(F("  G:")); Serial.print(resultColor[2]);
      Serial.print(F("  B:")); Serial.println(resultColor[3]);

      // Brief window to catch a press at the end of each cycle
      int btn = checkButton(300);
      if (btn == 1) {
        running = false;
        Serial.println(F("  Loop stopped. Single=restart | Double=next test | Triple=Calib Mode."));
      } else if (btn == 3) {
        Serial.println(F("  Entering Calibration Mode..."));
        return 4;
      } else if (btn == 2) {
        Serial.println(F("  Advancing to Stepper test..."));
        return 3;
      }
    } else {
      // Paused — wait for explicit user action
      int btn = waitForButtonPress();
      if (btn == 1) {
        running = true;
        Serial.println(F("  Loop restarted."));
      } else if (btn == 3) {
        Serial.println(F("  Entering Calibration Mode..."));
        return 4;
      } else {
        Serial.println(F("  Advancing to Stepper test..."));
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
  Serial.println();
  Serial.println(F("--- [DEBUG 3/3] Stepper Test ---"));
  Serial.println(F("  Single press during move       : stop"));
  Serial.println(F("  Single press when stopped      : save position as 0, restart cycle"));
  Serial.println(F("  Double press (any time)        : back to Hopper Motor test"));
  Serial.println(F("  Triple press (any time)        : Calibration Mode"));

  while (true) {
    stepper.setCurrentPosition(0);
    bool interrupted = false;

    // Full 360 turn, then return to home
    Serial.println(F("  Starting full 360 turn..."));
    if (!runStepperTo(stepperStepsPerRot * stepperMicroStepping)) {
      interrupted = true;
    } else {
      delay(500);
      Serial.println(F("  Returning to slot 0..."));
      if (!runStepperTo(0)) {
        interrupted = true;
      }
    }

    if (!interrupted) {
      delay(500);
      Serial.println(F("  360 turn done. Starting slot cycle..."));

      for (int slot = 0; slot < numContainerSlots && !interrupted; slot++) {
        Serial.print(F("  -> Slot ")); Serial.print(slot);
        Serial.print(F(" (step pos ")); Serial.print(slot * stepperMulti); Serial.println(')');

        if (!runStepperTo(slot * stepperMulti)) {
          interrupted = true;
          break;
        }

        // 500ms pause between slots; also detects button presses
        int btn = checkButton(500);
        if (btn == 1) {
          interrupted = true;
        } else if (btn == 3) {
          Serial.println(F("  Entering Calibration Mode..."));
          return 4;
        } else if (btn == 2) {
          Serial.println(F("  Returning to Hopper Motor test..."));
          return 1;
        }
      }

      // Return to slot 0 after visiting all slots
      if (!interrupted) {
        Serial.println(F("  Slot cycle done. Returning to slot 0..."));
        runStepperTo(0);
        delay(500);
      }
    }

    if (interrupted) {
      Serial.println(F("  Movement stopped."));
      Serial.println(F("  Single press : save position as 0, restart cycle"));
      Serial.println(F("  Double press : back to Hopper Motor test"));
      Serial.println(F("  Triple press : Calibration Mode"));
      int btn = waitForButtonPress();
      if (btn == 1) {
        stepper.setCurrentPosition(0);
        Serial.println(F("  Position saved as 0. Restarting cycle..."));
        // continue while(true) -> restart
      } else if (btn == 3) {
        Serial.println(F("  Entering Calibration Mode..."));
        return 4;
      } else {
        Serial.println(F("  Returning to Hopper Motor test..."));
        return 1;
      }
    } else {
      // Full cycle completed normally
      Serial.println(F("  Slot cycle complete!"));
      Serial.println(F("  Single press : restart from 0 | Double press : back to Hopper Motor test | Triple press : Calibration Mode"));
      int btn = waitForButtonPress();
      if (btn == 1) {
        Serial.println(F("  Restarting cycle..."));
        // continue while(true) -> restart
      } else if (btn == 3) {
        Serial.println(F("  Entering Calibration Mode..."));
        return 4;
      } else {
        Serial.println(F("  Returning to Hopper Motor test..."));
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
  Serial.println();
  Serial.println(F("--- [CALIB 1] Servo Calibration ---"));
  Serial.println(F("  Single press : Out -> In -> WiggleIn -> Out -> ..."));
  Serial.println(F("  Double press : next calibration step"));
  Serial.println(F("  Triple press : back to Debug Mode"));

  // state 0=Out, 1=In, 2=WiggleIn
  int state = 0;
  servo.write(servoAngleOut);
  Serial.print(F("  Servo -> OUT (angle ")); Serial.print(servoAngleOut); Serial.println(')');

  while (true) {
    int btn = waitForButtonPress();
    if (btn == 1) {
      state = (state + 1) % 3;
      if (state == 0) {
        servo.write(servoAngleOut);
        Serial.print(F("  Servo -> OUT (angle "));
        Serial.print(servoAngleOut);
        Serial.println(')');
      } else if (state == 1) {
        servo.write(servoAngleIn);
        Serial.print(F("  Servo -> IN  (angle "));
        Serial.print(servoAngleIn);
        Serial.println(')');
      } else {
        Serial.println(F("  Servo -> WiggleIn"));
        servoWiggleIn();
        Serial.println(F("  WiggleIn done."));
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
  Serial.println();
  Serial.println(F("*************************************"));
  Serial.println(F("*       CALIBRATION MODE            *"));
  Serial.println(F("*  Single press : action            *"));
  Serial.println(F("*  Double press : next calib step   *"));
  Serial.println(F("*  Triple press : back to Debug     *"));
  Serial.println(F("*************************************"));

  int step = 1;
  while (true) {
    int next;
    switch (step) {
      case 1: next = calibStep1_Servo(); break;
      default: next = -1; break;
    }
    if (next == -1) {
      Serial.println(F("  Exiting Calibration Mode -> back to Debug Mode."));
      return;
    }
    // No further calibration steps defined yet — wrap back to step 1
    Serial.println(F("  No further calibration steps. Returning to step 1."));
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

  Serial.println();
  Serial.println(F("*************************************"));
  Serial.println(F("*    INTERACTIVE DEBUG MODE         *"));
  Serial.println(F("*  Single press : action in test    *"));
  Serial.println(F("*  Double press : next / prev test  *"));
  Serial.println(F("*  Triple press : Calibration Mode  *"));
  Serial.println(F("*************************************"));

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
