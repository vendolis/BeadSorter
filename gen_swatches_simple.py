#!/usr/bin/env python3
"""
gen_swatches_simple.py

Generate a calibration swatch image from BeadCalibrateSimple CSV output.

Layout:
  rows    = beads  (one row per button press)
  columns = gains  (g1x | g4x | g16x | g60x)

Saturated readings are shown in dark grey with a "SATURATED" label.

Usage:
    python gen_swatches_simple.py <input.csv> [output.png]

    If output.png is omitted, the image is saved next to the CSV file
    with _swatches.png appended.

Dependencies:
    pip install Pillow
"""

import csv, sys, math, os
from PIL import Image, ImageDraw, ImageFont

# ── Config ────────────────────────────────────────────────────────────────────
GAIN_ORDER = ["g1x", "g4x", "g16x", "g60x"]

SWATCH_W = 160
SWATCH_H = 100
TEXT_H   = 68     # metadata text area below each swatch
PADDING  = 10     # gap between cells
CELL_W   = SWATCH_W + PADDING
CELL_H   = SWATCH_H + TEXT_H + PADDING

MARGIN   = 36     # left/right/bottom margin
HEADER_H = 80     # space for title + gain column labels
ROW_LABEL_W = 46  # width reserved for the bead-number label on the left

BG           = (240, 240, 240)
SAT_FILL     = (55, 55, 55)
SAT_TXT      = (255, 80, 80)
MISSING_FILL = (210, 210, 210)

# ── Args ──────────────────────────────────────────────────────────────────────
if len(sys.argv) < 2:
    print("Usage: gen_swatches_simple.py <input.csv> [output.png]")
    sys.exit(1)

csv_file = sys.argv[1]
if len(sys.argv) > 2:
    out_file = sys.argv[2]
else:
    base = os.path.splitext(csv_file)[0]
    out_file = base + "_swatches.png"

# ── Load CSV ──────────────────────────────────────────────────────────────────
# data[bead_id][gain_label] = dict of sensor values
data = {}

with open(csv_file, newline="", encoding="utf-8") as fh:
    clean_lines = (line for line in fh if not line.lstrip().startswith("#"))
    reader = csv.DictReader(clean_lines)
    for row in reader:
        try:
            bead = int(row["bead"])
            if bead <= 0:
                continue
            gain = row["gain"].strip()
            if gain not in GAIN_ORDER:
                continue
            if bead not in data:
                data[bead] = {}
            data[bead][gain] = {
                "R":   float(row["R_norm"]),
                "G":   float(row["G_norm"]),
                "B":   float(row["B_norm"]),
                "sat": int(row["saturated"]),
                "H":   float(row["H"]),
                "S":   float(row["S"]),
                "L":   float(row["L"]),
                "chR": float(row["chroma_r"]),
                "chG": float(row["chroma_g"]),
                "chB": float(row["chroma_b"]),
            }
        except (KeyError, ValueError):
            pass

beads = sorted(data.keys())
if not beads:
    print("No valid bead data found in CSV.")
    sys.exit(1)

print(f"Loaded {len(beads)} beads from {csv_file}")

# ── Image dimensions ──────────────────────────────────────────────────────────
n_cols = len(GAIN_ORDER)
n_rows = len(beads)

content_w = ROW_LABEL_W + n_cols * CELL_W
IMG_W     = MARGIN + content_w + MARGIN
IMG_H     = HEADER_H + n_rows * CELL_H + MARGIN

img  = Image.new("RGB", (IMG_W, IMG_H), BG)
draw = ImageDraw.Draw(img)

# ── Fonts ─────────────────────────────────────────────────────────────────────
try:
    fnt_title = ImageFont.truetype("arial.ttf", 20)
    fnt_label = ImageFont.truetype("arial.ttf", 13)
    fnt_small = ImageFont.truetype("arial.ttf", 11)
except OSError:
    fnt_title = ImageFont.load_default()
    fnt_label = fnt_title
    fnt_small = fnt_title

# ── Title ─────────────────────────────────────────────────────────────────────
draw.text(
    (MARGIN, 12),
    f"Bead calibration swatches — {len(beads)} beads  |  {os.path.basename(csv_file)}",
    fill=(20, 20, 20),
    font=fnt_title,
)

# ── Column headers (gain labels) ──────────────────────────────────────────────
for ci, gain in enumerate(GAIN_ORDER):
    cx = MARGIN + ROW_LABEL_W + ci * CELL_W + CELL_W // 2
    bbox = draw.textbbox((0, 0), gain, font=fnt_label)
    tw = bbox[2] - bbox[0]
    draw.text((cx - tw // 2, 46), gain, fill=(40, 40, 40), font=fnt_label)

# ── Draw swatches ─────────────────────────────────────────────────────────────
for ri, bead in enumerate(beads):
    y0 = HEADER_H + ri * CELL_H

    # Row label
    row_label = f"#{bead}"
    bbox = draw.textbbox((0, 0), row_label, font=fnt_label)
    th = bbox[3] - bbox[1]
    draw.text(
        (MARGIN, y0 + (SWATCH_H - th) // 2),
        row_label,
        fill=(40, 40, 40),
        font=fnt_label,
    )

    for ci, gain in enumerate(GAIN_ORDER):
        x0  = MARGIN + ROW_LABEL_W + ci * CELL_W
        sx0 = x0 + PADDING // 2
        sy0 = y0
        sx1 = sx0 + SWATCH_W
        sy1 = sy0 + SWATCH_H

        if gain not in data[bead]:
            # Missing reading (e.g. CSV truncated mid-bead)
            draw.rectangle([sx0, sy0, sx1, sy1], fill=MISSING_FILL, outline=(140, 140, 140))
            draw.text((sx0 + 4, sy0 + 4), "no data", fill=(100, 100, 100), font=fnt_small)
            continue

        r = data[bead][gain]
        sat = bool(r["sat"])
        r8  = min(255, max(0, round(r["R"])))
        g8  = min(255, max(0, round(r["G"])))
        b8  = min(255, max(0, round(r["B"])))

        fill   = SAT_FILL if sat else (r8, g8, b8)
        luma   = 0.299 * r8 + 0.587 * g8 + 0.114 * b8
        txt_fg = (255, 255, 255) if (luma < 140 or sat) else (0, 0, 0)

        draw.rectangle([sx0, sy0, sx1, sy1], fill=fill, outline=(80, 80, 80), width=1)

        if sat:
            draw.text((sx0 + 4, sy0 + 4), "SATURATED", fill=SAT_TXT, font=fnt_small)
        else:
            rgb_str = f"RGB {r8} {g8} {b8}"
            bbox2   = draw.textbbox((0, 0), rgb_str, font=fnt_small)
            tw2     = bbox2[2] - bbox2[0]
            draw.text(
                (sx0 + (SWATCH_W - tw2) // 2, sy0 + SWATCH_H // 2 - 7),
                rgb_str,
                fill=txt_fg,
                font=fnt_small,
            )

        # Metadata below swatch
        ty = sy1 + 4
        draw.text(
            (sx0, ty),
            f"H={r['H']:.3f}  S={r['S']:.3f}  L={r['L']:.3f}",
            fill=(50, 50, 50),
            font=fnt_small,
        )
        draw.text(
            (sx0, ty + 15),
            f"chR={r['chR']:.3f}  chG={r['chG']:.3f}  chB={r['chB']:.3f}",
            fill=(50, 50, 50),
            font=fnt_small,
        )

# ── Save ──────────────────────────────────────────────────────────────────────
img.save(out_file, dpi=(150, 150))
print(f"Saved {out_file}  ({IMG_W}×{IMG_H} px)")
