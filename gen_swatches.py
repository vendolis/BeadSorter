#!/usr/bin/env python3
"""Generate a colour-swatch image for all beads at a given gain."""

import csv, math, collections, statistics, sys
from PIL import Image, ImageDraw, ImageFont

CSV_FILE = "beads_clean.csv"
GAIN     = "g4x"

# ── Load data ─────────────────────────────────────────────────────────────────
rows = []
with open(CSV_FILE, newline="", encoding="utf-8") as fh:
    clean = (l for l in fh if not l.lstrip().startswith("#"))
    for row in csv.DictReader(clean):
        try:
            if row["gain"].strip() != GAIN:
                continue
            if int(row["saturated"]) != 0:
                continue
            rows.append({
                "bead":    int(row["bead"]),
                "R_norm":  float(row["R_norm"]),
                "G_norm":  float(row["G_norm"]),
                "B_norm":  float(row["B_norm"]),
                "chroma_r": float(row["chroma_r"]),
                "chroma_g": float(row["chroma_g"]),
                "chroma_b": float(row["chroma_b"]),
                "H":       float(row["H"]),
                "S":       float(row["S"]),
                "L":       float(row["L"]),
            })
        except (KeyError, ValueError):
            pass

by_bead = collections.defaultdict(list)
for r in rows:
    by_bead[r["bead"]].append(r)

beads = sorted(by_bead.keys())
means = {}
for b, rs in by_bead.items():
    means[b] = {
        "R": statistics.mean(r["R_norm"] for r in rs),
        "G": statistics.mean(r["G_norm"] for r in rs),
        "B": statistics.mean(r["B_norm"] for r in rs),
        "chR": statistics.mean(r["chroma_r"] for r in rs),
        "chG": statistics.mean(r["chroma_g"] for r in rs),
        "chB": statistics.mean(r["chroma_b"] for r in rs),
        "H": statistics.mean(r["H"] for r in rs),
        "S": statistics.mean(r["S"] for r in rs),
        "L": statistics.mean(r["L"] for r in rs),
    }

# ── Layout ────────────────────────────────────────────────────────────────────
COLS        = 6
SWATCH_W    = 180
SWATCH_H    = 140
TEXT_H      = 90        # text area below each swatch block
CELL_W      = SWATCH_W + 20
CELL_H      = SWATCH_H + TEXT_H + 10
MARGIN      = 30
ROWS        = math.ceil(len(beads) / COLS)

IMG_W = COLS * CELL_W + 2 * MARGIN
IMG_H = ROWS * CELL_H + 2 * MARGIN + 60   # +60 for title

BG = (245, 245, 245)
img = Image.new("RGB", (IMG_W, IMG_H), BG)
draw = ImageDraw.Draw(img)

# Fonts — fall back to default if no TTF available
try:
    fnt_title = ImageFont.truetype("arial.ttf", 22)
    fnt_label = ImageFont.truetype("arial.ttf", 14)
    fnt_small = ImageFont.truetype("arial.ttf", 12)
except OSError:
    fnt_title = ImageFont.load_default()
    fnt_label = fnt_title
    fnt_small = fnt_title

# Title
title = f"Bead colour swatches  —  gain {GAIN}  ({len(beads)} beads)"
draw.text((MARGIN, 14), title, fill=(30, 30, 30), font=fnt_title)

# ── Draw each bead ────────────────────────────────────────────────────────────
for idx, bead in enumerate(beads):
    col = idx % COLS
    row = idx // COLS

    x0 = MARGIN + col * CELL_W
    y0 = MARGIN + 60 + row * CELL_H

    m = means[bead]
    r8 = min(255, max(0, round(m["R"])))
    g8 = min(255, max(0, round(m["G"])))
    b8 = min(255, max(0, round(m["B"])))
    colour = (r8, g8, b8)

    # Swatch rectangle with thin border
    sx0, sy0 = x0 + 10, y0
    sx1, sy1 = sx0 + SWATCH_W, sy0 + SWATCH_H
    draw.rectangle([sx0, sy0, sx1, sy1], fill=colour, outline=(80, 80, 80), width=1)

    # Decide text colour for contrast
    luma = 0.299 * r8 + 0.587 * g8 + 0.114 * b8
    txt_col = (255, 255, 255) if luma < 140 else (0, 0, 0)

    # Bead number centred in swatch
    label = f"#{bead}"
    bbox = draw.textbbox((0, 0), label, font=fnt_label)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]
    draw.text((sx0 + (SWATCH_W - tw) // 2, sy0 + (SWATCH_H - th) // 2 - 6),
              label, fill=txt_col, font=fnt_label)

    # RGB line inside swatch
    rgb_str = f"RGB {r8} {g8} {b8}"
    bbox2 = draw.textbbox((0, 0), rgb_str, font=fnt_small)
    tw2 = bbox2[2] - bbox2[0]
    draw.text((sx0 + (SWATCH_W - tw2) // 2,
               sy0 + (SWATCH_H - th) // 2 + th),
              rgb_str, fill=txt_col, font=fnt_small)

    # Metadata below swatch
    ty = sy1 + 6
    draw.text((sx0, ty),      f"H={m['H']:.3f}  S={m['S']:.3f}  L={m['L']:.3f}",
              fill=(50, 50, 50), font=fnt_small)
    draw.text((sx0, ty + 16), f"chR={m['chR']:.3f}  chG={m['chG']:.3f}",
              fill=(50, 50, 50), font=fnt_small)
    draw.text((sx0, ty + 32), f"chB={m['chB']:.3f}",
              fill=(50, 50, 50), font=fnt_small)

out = "bead_swatches_g4x.png"
img.save(out, dpi=(150, 150))
print(f"Saved {out}  ({IMG_W}x{IMG_H} px, {len(beads)} beads)")
