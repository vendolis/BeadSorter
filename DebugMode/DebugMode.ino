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
// Loop: servoFeedOut -> servoFeedIn -> readColorSensor -> print HSL results
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
      printBeadReading();

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
