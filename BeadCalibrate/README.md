# BeadSorter Colour Calibration Toolchain

Two files work together to find the best colour-space and sensor settings for
your bead set before tuning `BeadSorter.ino`.

| File | What it does |
|---|---|
| `BeadCalibrate/BeadCalibrate.ino` | Runs the full machine, collects colour data for each bead, outputs CSV |
| `analyse_beads.py` | Reads that CSV, compares colour spaces and gains, suggests thresholds |

---

## Hardware

Identical wiring to `BeadSorter.ino` — no changes needed.
The stepper carousel is **not moved** during calibration; all beads fall into
whichever slot is currently positioned under the chute.

---

## BeadCalibrate.ino

### What it does

For every bead that passes through the sensor:

1. Feeds the bead in with the servo (`servoFeedIn`).
2. Checks whether a bead is actually present (null scan at 16× gain).
3. If a bead is detected, takes **NUM\_READINGS wiggle positions**.
   At each position it reads all **4 gain settings** (1×, 4×, 16×, 60×)
   before moving to the next wiggle — so each bead produces
   `NUM_READINGS × 4` CSV rows.
4. Releases the bead (`servoFeedOut`) and waits for the next one.

### Setup

1. Flash `BeadCalibrate.ino` to the Arduino.
2. Open Serial Monitor at **115200 baud** (BeadSorter.ino uses 9600 — change
   the setting in your IDE).
3. Let the machine run. Beads are fed automatically by the hopper.
4. When all beads have been processed, copy the full Serial output and save it
   as a `.csv` file (e.g. `beads.csv`).

> **Tip:** Lines starting with `#` are comments — the header line, bead
> markers, rejected-bead diagnostics. `analyse_beads.py` skips them
> automatically.

### Configuration

All tuning constants are at the top of the sketch under
`// ── Calibration parameters ──`:

| Constant | Default | Description |
|---|---|---|
| `NUM_READINGS` | `10` | Wiggle positions per bead |
| `CALIB_INTTIME` | `TCS34725_INTEGRATIONTIME_101MS` | Integration time for every gain step |
| `GAIN_SETTLE_MS` | `220` | ms to wait after `setGain()` before reading (must be > integration period) |
| `NULL_SCAN_GAIN_IDX` | `2` (= 16×) | Which entry in `GAINS[]` is used for the null-scan check |
| `SAT_THRESH` | `60000` | Raw count above which a channel is flagged as saturated |
| `NULL_OFFSET_H/S/L` | `0.03 / 0.05 / 0.05` | How close a reading must be to the empty-tube reference to be rejected |

The gain sweep table is:

```cpp
{ TCS34725_GAIN_1X,  "g1x"  },
{ TCS34725_GAIN_4X,  "g4x"  },
{ TCS34725_GAIN_16X, "g16x" },   // ← also used for null scan
{ TCS34725_GAIN_60X, "g60x" },
```

### Null scan

At startup the servo cycles 6 times with an empty tube and records the baseline
HSL reading at 16×. During the main loop, each bead is checked against this
baseline before the gain sweep starts. A reading is **rejected** (no bead)
only when all three channels are within their `NULL_OFFSET` of the baseline
simultaneously (AND logic).

Rejected beads print a diagnostic line:

```
# REJECTED  read: H=0.1300 S=0.2280 L=0.3190  null ref: H=0.1316 S=0.2317 L=0.3216
```

If legitimate beads are being rejected, reduce `NULL_OFFSET_H/S/L` further.
If empty-tube readings are being accepted as beads, increase them.

### CSV output format

```
bead,gain,sample,R_raw,G_raw,B_raw,C_raw,saturated,R_norm,G_norm,B_norm,chroma_r,chroma_g,chroma_b,H,S,L
```

| Column | Range | Description |
|---|---|---|
| `bead` | 1 … N | Sequential bead number |
| `gain` | g1x / g4x / g16x / g60x | Gain setting for this row |
| `sample` | 1 … NUM\_READINGS | Wiggle position index |
| `R_raw` … `C_raw` | 0 … 65535 | Raw 16-bit RGBC counts from sensor |
| `saturated` | 0 / 1 | 1 if any channel ≥ SAT\_THRESH |
| `R_norm` … `B_norm` | 0 … 255 | Clear-channel-normalised float RGB (from `getRGB()`) |
| `chroma_r` … `chroma_b` | 0 … 1 | R/(R+G+B) etc. — ambient-independent colour signature |
| `H`, `S`, `L` | 0 … 1 | HSL derived from normalised RGB; H is circular (0 = red) |

---

## analyse_beads.py

### Requirements

Python 3.6+, standard library only (no pip installs needed).

### Usage

```bash
# Analyse a single collection run:
python analyse_beads.py beads.csv

# Restrict to one gain only:
python analyse_beads.py beads.csv --gain g16x

# Show more closest-pair rows:
python analyse_beads.py beads.csv --top 20
```

### What it reports

#### 1 — Saturation summary

How many rows per gain setting had a saturated channel.
A high saturation percentage at 60× means that gain clips signal and loses
colour information — prefer a lower gain for those beads.

#### 2 — Per-bead mean values

For each gain setting, a table of mean H, S, L, chroma\_r, chroma\_g, chroma\_b
per bead (saturated rows excluded).  Also reports average intra-bead noise
(standard deviation) per channel — lower is more stable.

#### 3 — Colour-space separability

For each gain × colour-space combination, the **minimum pairwise distance**
between any two beads is computed.  A higher minimum means the beads are
easier to separate.

| Space | What it measures |
|---|---|
| `HSL` | Euclidean distance with circular hue |
| `HS` | Same but ignoring L — tests whether lightness helps |
| `Chroma2` | 2-D chromaticity (chroma\_r, chroma\_g) |
| `Chroma3` | 3-D chromaticity (all three fractions) |
| `NormRGB` | Euclidean distance in normalised RGB (0–255 scale) |

The space with the highest minimum pairwise distance is the best discriminator
for your specific bead set.

#### 4 — Closest pairs

The N hardest-to-separate bead pairs in the best colour space, sorted by
distance.  Pairs flagged `← may collide` have a distance below 0.040 and
may be misclassified by `BeadSorter.ino` depending on threshold settings.

#### 5 — Threshold suggestions

Half the minimum per-channel inter-bead gap — the tightest threshold that
does not split any bead pair in that channel.  Reported for both HSL and
Chroma2:

```
Suggested thresholds for BeadSorter.ino (half the min inter-bead gap):
  HSL:       thresholdH = 0.0182  thresholdS = 0.0310  thresholdL = 0.0120
  Chroma2:   threshold  = 0.0240  (Euclidean distance in chroma_r / chroma_g plane)
```

#### 6 — Cross-gain comparison

A summary table showing separability scores across all gains and colour spaces,
followed by the recommended gain and colour space combination.

### Interpreting the results

| Finding | What to do |
|---|---|
| High saturation % at g60x | Don't use 60× in BeadSorter.ino for these beads |
| `Chroma2` score > `HSL` score | Consider switching BeadSorter.ino to store chroma\_r / chroma\_g instead of H/S/L |
| High intra-bead `std_H` | Hue is noisy for those beads (low saturation); rely more on S or chromaticity |
| Two beads always close in every space | They may physically be the same colour; no threshold can separate them |
| Suggested thresholdH is very small (< 0.010) | Hue alone won't separate all beads; the AND-logic of H+S+L helps |

### Applying the results to BeadSorter.ino

Update the threshold constants near the top of `BeadSorter.ino`:

```cpp
float thresholdH = 0.020;   // ← replace with suggested value
float thresholdS = 0.040;   // ← replace with suggested value
float thresholdL = 0.040;   // ← replace with suggested value
```

If the recommended gain differs from the current 16×, update the sensor
initialisation line:

```cpp
// Example: switch to 4×
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS,
                                           TCS34725_GAIN_4X);
```

---

## Swatch image generators

Three helper scripts produce visual colour-swatch sheets from calibration data.
They are independent of each other — use whichever matches your run.

### gen_swatches.py — simple accepted-bead sheet

Reads a **clean CSV** (saturated rows already removed, or use the raw CSV and
let it filter by `saturated == 0`).  Shows only accepted beads; no null
reference, no rejects.  Useful as a quick sanity-check after a first
calibration run.

```bash
# Edit the two constants at the top of the file before running:
#   CSV_FILE = "beads_clean.csv"   ← your clean CSV path
#   GAIN     = "g4x"               ← which gain to visualise
python gen_swatches.py
# Writes: bead_swatches_g4x.png
```

**Requires:** Pillow (`pip install pillow`).
**Input:** clean CSV only.

---

### gen_swatches_white.py — white-background run with clustered rejects

Produces a three-section sheet:

1. **Null reference** — the empty-tube baseline swatch.
2. **Accepted beads** — mean NormRGB colour + ΔH/ΔS/ΔL vs null ref.
3. **Rejected bead clusters** — groups of near-identical rejections
   (within 0.02 H / 0.03 S / 0.04 L of each other are merged into one
   cluster swatch).  A red border flags clusters whose mean HSL sits
   within the null-scan tolerances (0.03/0.05/0.05) of the null
   reference — those are "too close to empty tube" rejections.

```bash
# Edit at the top of the file:
#   CSV_RAW   = "BeadCalibration_20260313_whitebackground.csv"  ← raw serial output
#   CSV_CLEAN = "beads_white_clean.csv"                         ← clean CSV
#   GAIN      = "g16x"
python gen_swatches_white.py
# Writes: bead_swatches_white_bg.png
```

**Requires:** Pillow.
**Inputs:** raw serial-output CSV **and** clean CSV.  The raw file is parsed
for `# Null ref:` and `# REJECTED … read: … null ref: …` comment lines
emitted by `BeadCalibrate.ino`.

Use this variant when you want to understand **recurring reject patterns**
(e.g. a single bead colour that consistently fails the null scan).

---

### gen_swatches_largewin.py — individual-reject diagnostic sheet

Identical three-section layout to `gen_swatches_white.py`, but the third
section shows **every individual rejection** as its own swatch rather than
grouping them.  The `NULL_OFF` constant `(0.03, 0.05, 0.05)` matches
`NULL_OFFSET_H/S/L` in `BeadCalibrate.ino` exactly; swatches whose ΔH/ΔS/ΔL
all fall within those offsets get a red border.

```bash
# Pass the raw serial-output CSV directly; no separate clean CSV needed:
python gen_swatches_largewin.py BeadCalibration_20260313_largerWindow.csv
# Writes: bead_swatches_largewin.png

# Custom output name:
python gen_swatches_largewin.py BeadCalibration_20260313_improved.csv my_swatches.png

# Different gain:
python gen_swatches_largewin.py BeadCalibration_20260313_improved.csv my_swatches.png --gain g4x
```

**Requires:** Pillow.
**Input:** raw serial-output CSV only (serves as both data source and comment parser).

Use this variant when you want to inspect **each individual rejection event**
(e.g. to see whether a bead is consistently rejected or only occasionally,
and what its exact HSL values were each time).

---

### Which script to use?

| Situation | Script |
|---|---|
| First look at colours from a new run | `gen_swatches.py` |
| Many beads are failing the null scan; identify the recurring pattern | `gen_swatches_white.py` |
| Debug specific rejection events one by one | `gen_swatches_largewin.py` |

### Dependency on BeadCalibrate.ino comment format

`gen_swatches_white.py` and `gen_swatches_largewin.py` parse the raw
serial-output file using regular expressions that match two specific comment
formats emitted by `BeadCalibrate.ino`:

```
# Null ref: H=0.1316  S=0.2317  L=0.3216
# REJECTED 3  read: H=0.1300 S=0.2280 L=0.3190  null ref: H=0.1316 S=0.2317 L=0.3216
```

If you modify the `Serial.print` statements in `BeadCalibrate.ino` that
produce these lines, you must update the regex patterns in both scripts
accordingly.

---

## Full workflow

```
1. Flash BeadCalibrate.ino
2. Open Serial Monitor at 115200 baud
3. Run all beads through the machine
4. Copy Serial output → save as beads.csv  (this is the "raw" file)
5. python analyse_beads.py beads.csv
6. Note recommended gain and thresholds
7. (Optional) Run a swatch script to visualise colours:
     python gen_swatches.py            # quick look, accepted beads only
     python gen_swatches_white.py      # clustered rejects + null ref
     python gen_swatches_largewin.py   # individual rejects + null ref
8. Update BeadSorter.ino with new gain and threshold values
9. Re-flash BeadSorter.ino and verify sorting accuracy
```
