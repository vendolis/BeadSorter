# BeadSorter

An Arduino-based automated bead sorting machine that uses a color sensor to identify beads and route them into separate containers by color.

---

## Physical Stages

The machine is divided into numbered physical stages, each with corresponding 3D-printed parts:

| Stage | Name | Description |
|-------|------|-------------|
| 0 | Hopper | Bulk bead storage reservoir |
| 1 | Sequentializer | DC motor agitates beads into a single-file tube |
| 3 | Analyzer | Servo-controlled gate presents one bead at a time to the color sensor |
| 4 | Dispatcher | Stepper-driven rotating carousel directs the bead to the correct output |
| 5 | Outputs | Ring of output containers (up to 15 color bins + 1 overflow) |

---

## Hardware

### Components

| Component | Purpose |
|-----------|---------|
| Arduino (Nano/Uno) | Main controller |
| TCS34725 color sensor | Reads RGBC values of each bead via I2C |
| Servo motor | Gate in the analyzer stage — swings in/out to present a bead to the sensor |
| Stepper motor (200 steps/rev) | Rotates the output carousel to the correct container |
| L298N / H-bridge | Drives the DC hopper motor |
| DC motor (hopper) | Agitates beads from the hopper into the sequentializer tube |
| Photoresistor / LDR | Detects whether beads are present in the feeding tube |
| Push button | Setup / calibration trigger |

### Pin Assignments

| Pin | Name | Function |
|-----|------|----------|
| D2 | `dirPin` | Stepper direction |
| D3 | `stepPin` | Stepper step pulse |
| D5 | `GSM2` | Hopper motor PWM speed |
| D6 | `in4` | Hopper motor direction B |
| D7 | `in3` | Hopper motor direction A |
| D8 | `servoPin` | Servo PWM |
| D11 | `setupPin` | Setup/calibration button (INPUT) |
| A0 | `photoSensorPin` | Photoresistor (analog) |
| SDA/SCL | I2C | TCS34725 color sensor |

---

## Software Architecture

### Libraries Required

- `Servo.h` — Arduino servo control
- `AccelStepper.h` — Stepper motor with acceleration/deceleration
- `Wire.h` — I2C communication
- `Adafruit_TCS34725.h` — TCS34725 color sensor driver

### Main Loop Flow

```
loop()
  │
  ├─ Handle serial commands (e.g. 't' to print tables)
  │
  ├─ handleHopperMotor()        ← keep beads flowing; reverse if stuck
  │
  ├─ Check setup button         ← if pressed: enter manual color registration
  │
  ├─ servoFeedIn()              ← swing servo to "in" position, presenting bead
  │
  ├─ readColorSensor()          ← read RGBC from TCS34725
  │
  ├─ nullScan()?
  │    ├─ YES (no bead) → print ".", servoFeedOut(), loop
  │    └─ NO  (bead detected)
  │         ├─ sortBeadToDynamicArray()
  │         │    ├─ findColorInStorage()   ← up to 4 retries with wiggle
  │         │    ├─ getContainerNo()       ← map color index → carousel slot
  │         │    └─ moveSorterToPosition() ← rotate stepper to that slot
  │         └─ servoFeedOut()             ← release bead into carousel
```

### Startup Sequence

1. Initialise all pins and peripherals.
2. If `calibrateNullScan = true`: wiggle the servo 6 times (to settle), then read the empty-sensor baseline and store it as the null-scan reference.
3. If `autoSort = false`: load the 13 preset default colors.
4. Begin the main loop.

---

## Operating Modes

### Auto-Sort Mode (`autoSort = true`, default)

The machine learns colors on the fly:
- Every new color that doesn't match anything stored is automatically assigned to the next free container slot.
- Up to **15 distinct colors** can be learned and sorted.
- When all 15 slots are full, unrecognised beads go to the last container (slot 15 = overflow).
- Thresholds: `thresholdFactor = 0.04`, `offset = 40`.

### Manual / Preset Mode (`autoSort = false`)

Activated by pressing the setup button during runtime (or set at compile time):
- Uses 13 hardcoded reference colors (Orange, Yellow, Green, Dark Blue, Rose, Skin, Red, White, Black, Lime Green, Mint Green, Coral, Dark Yellow).
- Tighter matching thresholds: `thresholdFactor = 0.03`, `offset = 20`.
- New colors can still be registered manually — press the setup button, load a single color type into the hopper, then press again to scan 4 samples and compute a mean reference.

---

## Color Matching

### Sensor Configuration

```cpp
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_16X);
```

The sensor returns four 16-bit channels: **Clear (C), Red (R), Green (G), Blue (B)**.

### Null Scan (Empty-Sensor Detection)

On startup the empty-sensor RGBC values are recorded as `nullScanValues[]`. Each loop iteration, if the measured RGBC is within ±`nullScanOffset` (150) of these baseline values on all four channels, `nullScan()` returns `true` — meaning no bead is present.

### Color Identification (`findColorInStorage`)

Each stored color is compared channel by channel. For each channel the allowed window is:

```
threshold = thresholdFactor × measured_value + offset
window    = [measured − threshold, measured + threshold]
```

A stored color matches only if **all four channels** fall within their respective windows. The first match found wins.

### Retry Logic

If no match is found immediately, the servo wiggles the bead (`servoWiggleIn`) and re-reads up to **4 times**. The four readings are averaged (mean) before a final unmatched bead is stored as a new auto-color.

### Color Distance (Euclidean)

`colorDistance()` computes the 4D Euclidean distance between two RGBC vectors. This is logged to Serial for debugging but is not used in the primary matching logic (which uses the per-channel threshold approach above).

---

## Stepper / Carousel

| Parameter | Value |
|-----------|-------|
| Steps per revolution | 200 |
| Microstepping | 16× |
| Container slots defined | 16 (12 physically used) |
| Steps per slot | `200 × 16 / 16 = 200` |
| Max speed | 6000 steps/s |
| Acceleration | 9000 steps/s² |

`moveSorterToPosition()` always takes the **shortest path** — if the forward distance is more than 8 slots it wraps around in reverse, and vice versa. After moving, the absolute position is normalised back into the 0–15 slot range to prevent integer overflow over long runs.

### Container Array

`dynamicContainerArray[16]` maps color indices to physical carousel slots. Pre-seeded with sentinel value `666` at slots 1, 5, 9, 13 (reserved/dividers) and `-1` (free) elsewhere. Slot 15 is the overflow bin.

---

## Hopper Motor Control

The DC hopper motor runs continuously while the photosensor reports no beads in the tube (sensor value > `photoSensorThreshold` = 600).

If no bead has been successfully sorted for more than `hopperMotorReverseTime` (9 s), the motor **reverses direction** to dislodge jams. The direction resets on each successful bead.

---

## Servo Positions

| Position | Angle | Description |
|----------|-------|-------------|
| In | 28° ± 2° | Bead held under sensor |
| Out | 51° ± 2° | Bead released to carousel |
| Wiggle | ±4° around `In` | Repositions bead for re-reading |

`servoFeedOut()` cycles the "out" wiggle **3 times** to ensure the bead is fully ejected.

---

## Serial Interface

Baud rate: **9600**

| Input | Action |
|-------|--------|
| `t` or `T` | Print null-scan values, all stored colors, and the container array |

### Output Format

- `.` — no bead detected (null scan)
- `Photosensor detected Beads <value>` — tube is full, hopper stopped
- `Beads analyzed: N` followed by RGBC values — bead detected
- `Color is #<name>` — matched color (manual mode) or RGBC (auto mode)
- `Color Distance is: <d>` — Euclidean distance to matched reference
- `move stepper to container No: <n>` — target slot
- `not found` + `autosort!` — new color being learned

---

## Configuration Reference

All key parameters are `#define` constants at the top of [BeadSorter.ino](BeadSorter.ino):

| Constant | Default | Description |
|----------|---------|-------------|
| `stepperMaxSpeed` | 6000 | Stepper max steps/s |
| `stepperAccel` | 9000 | Stepper acceleration steps/s² |
| `stepperStepsPerRot` | 200 | Motor steps per revolution |
| `stepperMicroStepping` | 16 | Driver microstepping factor |
| `numContainerSlots` | 16 | Logical slot count |
| `motorSpeed` | 255 | Hopper motor PWM (0–255) |
| `hopperMotorReverseTime` | 9000 | ms before reversing hopper on jam |
| `photoSensorThreshold` | 600 | ADC level above which tube is empty |
| `nullScanOffset` | 150 | ±tolerance for empty-sensor detection |
| `servoAngleIn` | 28° | Servo scan position |
| `servoAngleOut` | 51° | Servo release position |
| `servoAngleWiggle` | 2° | Servo dither amplitude |
| `thresholdFactor` | 0.04 | Color match window scaling factor |
| `offset` | 40 | Color match minimum window (counts) |
| `calibrateNullScan` | true | Auto-calibrate empty sensor on boot |
| `autoSort` | true | Learn colors automatically |

---

## Debug Modes

Uncomment a `DEBUG_PROG` define to enable a debug loop at startup:

| Value | Mode |
|-------|------|
| `1` | Stepper test: repeatedly moves to position 1000 and back |
| `2` | Color sensor test: prints live RGBC readings every 400 ms |

`DEBUG_PRINT_PHOTOSENS` — if defined, prints the raw photosensor ADC value every loop iteration.

---

## Known Issues / Notes

- `calcMedianAndStore()` computes a **mean**, not a median (acknowledged in a comment).
- The `updateDetectedColorFromTempStoredColor` / `updateStoredColorCount` running-average update feature is commented out.

### VS Code IntelliSense warning

IntelliSense may report *"PCH warning: header stop cannot be in a linkage block"* on the last `#include` line. This is **not a real compiler error** — it is caused by `Arduino.h` using `extern "C"` blocks internally which confuse the MSVC-style PCH engine. It is suppressed by setting `"intelliSenseMode": "gcc-x86"` in [.vscode/c_cpp_properties.json](.vscode/c_cpp_properties.json), which matches the actual AVR-GCC toolchain.
