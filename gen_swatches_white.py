#!/usr/bin/env python3
"""Generate colour swatches for the white-background calibration run.
Produces two panels:
  1. Accepted beads (g16x, best gain)
  2. Rejected beads (HSL only, converted back to RGB for display)
Plus a null-reference swatch for comparison.
"""

import csv, math, re, collections, statistics
from PIL import Image, ImageDraw, ImageFont

CSV_RAW  = "BeadCalibration_20260313_whitebackground.csv"
CSV_CLEAN = "beads_white_clean.csv"
GAIN     = "g16x"   # best gain for this run

# ── HSL→RGB helper ────────────────────────────────────────────────────────────
def hsl_to_rgb255(h, s, l):
    """Convert HSL (0-1 range) to (R,G,B) 0-255."""
    if s == 0:
        v = round(l * 255)
        return (v, v, v)
    def _f(p, q, t):
        t %= 1.0
        if t < 1/6: return p + (q - p) * 6 * t
        if t < 1/2: return q
        if t < 2/3: return p + (q - p) * (2/3 - t) * 6
        return p
    q = l * (1 + s) if l < 0.5 else l + s - l * s
    p = 2 * l - q
    r = _f(p, q, h + 1/3)
    g = _f(p, q, h)
    b = _f(p, q, h - 1/3)
    return (round(r*255), round(g*255), round(b*255))

# ── Load accepted bead data ────────────────────────────────────────────────────
rows = []
with open(CSV_CLEAN, newline="", encoding="utf-8") as fh:
    clean = (l for l in fh if not l.lstrip().startswith("#"))
    for row in csv.DictReader(clean):
        try:
            if row["gain"].strip() != GAIN: continue
            if int(row["saturated"]) != 0:  continue
            rows.append({
                "bead":   int(row["bead"]),
                "R_norm": float(row["R_norm"]),
                "G_norm": float(row["G_norm"]),
                "B_norm": float(row["B_norm"]),
                "H": float(row["H"]), "S": float(row["S"]), "L": float(row["L"]),
                "chroma_r": float(row["chroma_r"]),
                "chroma_g": float(row["chroma_g"]),
                "chroma_b": float(row["chroma_b"]),
            })
        except (KeyError, ValueError):
            pass

by_bead = collections.defaultdict(list)
for r in rows:
    by_bead[r["bead"]].append(r)

accepted_means = {}
for b, rs in sorted(by_bead.items()):
    accepted_means[b] = {
        "R": statistics.mean(r["R_norm"] for r in rs),
        "G": statistics.mean(r["G_norm"] for r in rs),
        "B": statistics.mean(r["B_norm"] for r in rs),
        "H": statistics.mean(r["H"] for r in rs),
        "S": statistics.mean(r["S"] for r in rs),
        "L": statistics.mean(r["L"] for r in rs),
        "chR": statistics.mean(r["chroma_r"] for r in rs),
        "chG": statistics.mean(r["chroma_g"] for r in rs),
        "chB": statistics.mean(r["chroma_b"] for r in rs),
    }

# ── Parse rejects from raw file ────────────────────────────────────────────────
null_ref = None
rejects = []
raw_lines = open(CSV_RAW, encoding="utf-8").readlines()

for line in raw_lines:
    # Null ref
    m = re.search(r'Null ref: H=([\d.]+)\s+S=([\d.]+)\s+L=([\d.]+)', line)
    if m:
        null_ref = {"H": float(m.group(1)), "S": float(m.group(2)), "L": float(m.group(3))}

    # Rejects
    m = re.search(r'REJECTED.*?read: H=([\d.]+) S=([\d.]+) L=([\d.]+)'
                  r'.*null ref.*H=([\d.]+) S=([\d.]+) L=([\d.]+)', line)
    if m:
        rH, rS, rL, nH, nS, nL = [float(x) for x in m.groups()]
        rejects.append({
            "H": rH, "S": rS, "L": rL,
            "dH": abs(rH - nH), "dS": abs(rS - nS), "dL": abs(rL - nL),
        })

# Group near-identical rejects (within 0.01 in all channels) to find clusters
clusters = []
used = [False] * len(rejects)
for i, r in enumerate(rejects):
    if used[i]: continue
    grp = [r]
    used[i] = True
    for j, r2 in enumerate(rejects):
        if used[j]: continue
        if abs(r["H"]-r2["H"])<0.02 and abs(r["S"]-r2["S"])<0.03 and abs(r["L"]-r2["L"])<0.04:
            grp.append(r2)
            used[j] = True
    clusters.append(grp)

reject_means = []
for grp in sorted(clusters, key=lambda g: g[0]["H"]):
    rm = {
        "H": statistics.mean(r["H"] for r in grp),
        "S": statistics.mean(r["S"] for r in grp),
        "L": statistics.mean(r["L"] for r in grp),
        "dH": statistics.mean(r["dH"] for r in grp),
        "dS": statistics.mean(r["dS"] for r in grp),
        "dL": statistics.mean(r["dL"] for r in grp),
        "n": len(grp),
    }
    reject_means.append(rm)

# ── Layout constants ──────────────────────────────────────────────────────────
SWATCH_W = 175
SWATCH_H = 130
TEXT_H   = 100
CELL_W   = SWATCH_W + 20
CELL_H   = SWATCH_H + TEXT_H + 10
MARGIN   = 30
COLS     = 6

def needed_rows(n): return math.ceil(n / COLS)

section_heights = {
    "accepted": needed_rows(len(accepted_means)) * CELL_H + 90,
    "rejects":  needed_rows(len(reject_means))  * CELL_H + 90,
    "nullref":  CELL_H + 60,
}
IMG_W = COLS * CELL_W + 2 * MARGIN
IMG_H = sum(section_heights.values()) + MARGIN * 2 + 20

BG = (240, 240, 240)
img = Image.new("RGB", (IMG_W, IMG_H), BG)
draw = ImageDraw.Draw(img)

try:
    fnt_title  = ImageFont.truetype("arial.ttf", 20)
    fnt_sec    = ImageFont.truetype("arialbd.ttf", 16)
    fnt_label  = ImageFont.truetype("arial.ttf", 14)
    fnt_small  = ImageFont.truetype("arial.ttf", 12)
except OSError:
    fnt_title = fnt_sec = fnt_label = fnt_small = ImageFont.load_default()

def draw_swatch(x0, y0, colour, top_label, sub_lines, border=(80,80,80)):
    r8, g8, b8 = colour
    draw.rectangle([x0+10, y0, x0+10+SWATCH_W, y0+SWATCH_H],
                   fill=colour, outline=border, width=2)
    luma = 0.299*r8 + 0.587*g8 + 0.114*b8
    tc = (255,255,255) if luma < 140 else (0,0,0)
    bb = draw.textbbox((0,0), top_label, font=fnt_label)
    tw = bb[2]-bb[0]; th = bb[3]-bb[1]
    draw.text((x0+10 + (SWATCH_W-tw)//2, y0 + (SWATCH_H-th)//2 - 8),
              top_label, fill=tc, font=fnt_label)
    ty = y0 + SWATCH_H + 6
    for line in sub_lines:
        draw.text((x0+12, ty), line, fill=(40,40,40), font=fnt_small)
        ty += 15

def section_header(y, title, subtitle="", bg=(60,90,130)):
    draw.rectangle([0, y, IMG_W, y+38], fill=bg)
    draw.text((MARGIN, y+6), title, fill=(255,255,255), font=fnt_sec)
    if subtitle:
        draw.text((MARGIN + 300, y+10), subtitle, fill=(210,210,210), font=fnt_small)
    return y + 48

# ── Draw null ref ─────────────────────────────────────────────────────────────
cy = MARGIN
draw.text((MARGIN, cy), "White-background calibration run — colour swatches",
          fill=(20,20,20), font=fnt_title)
cy += 34

cy = section_header(cy, "NULL REFERENCE  (empty tube baseline at g16×)",
                    f"H={null_ref['H']:.4f}  S={null_ref['S']:.4f}  L={null_ref['L']:.4f}",
                    bg=(80, 80, 80))
nr_col = hsl_to_rgb255(null_ref["H"], null_ref["S"], null_ref["L"])
draw_swatch(MARGIN, cy, nr_col, "NULL REF",
            [f"H={null_ref['H']:.4f}  S={null_ref['S']:.4f}",
             f"L={null_ref['L']:.4f}",
             f"RGB {nr_col[0]} {nr_col[1]} {nr_col[2]}",
             f"(empty tube colour)"],
            border=(200,50,50))
cy += CELL_H + 10

# ── Draw accepted beads ───────────────────────────────────────────────────────
cy = section_header(cy,
    f"ACCEPTED BEADS  ({len(accepted_means)} beads, gain {GAIN})",
    f"Only {len(accepted_means)} of expected ~30 beads passed the null-scan check",
    bg=(40, 110, 60))

for idx, (bead, m) in enumerate(sorted(accepted_means.items())):
    col = idx % COLS
    row = idx // COLS
    x0 = MARGIN + col * CELL_W
    y0 = cy + row * CELL_H
    r8 = min(255, max(0, round(m["R"])))
    g8 = min(255, max(0, round(m["G"])))
    b8 = min(255, max(0, round(m["B"])))
    dH = abs(m["H"] - null_ref["H"])
    dS = abs(m["S"] - null_ref["S"])
    dL = abs(m["L"] - null_ref["L"])
    draw_swatch(x0, y0, (r8,g8,b8),
        f"Bead #{bead}",
        [f"RGB {r8} {g8} {b8}",
         f"H={m['H']:.3f}  S={m['S']:.3f}  L={m['L']:.3f}",
         f"chR={m['chR']:.3f}  chG={m['chG']:.3f}",
         f"\u0394H={dH:.3f}  \u0394S={dS:.3f}  \u0394L={dL:.3f}"])
cy += needed_rows(len(accepted_means)) * CELL_H + 14

# ── Draw rejected bead clusters ───────────────────────────────────────────────
cy = section_header(cy,
    f"REJECTED BEADS  ({len(rejects)} total, grouped into {len(reject_means)} clusters)",
    "Colours computed from H/S/L via inverse HSL conversion",
    bg=(150, 60, 40))

for idx, rm in enumerate(reject_means):
    col = idx % COLS
    row = idx // COLS
    x0 = MARGIN + col * CELL_W
    y0 = cy + row * CELL_H
    colour = hsl_to_rgb255(rm["H"], rm["S"], rm["L"])
    r8,g8,b8 = colour
    # Flag if this cluster is suspiciously close to null ref
    too_close = rm["dH"] < 0.03 and rm["dS"] < 0.05 and rm["dL"] < 0.05
    border = (200, 50, 50) if too_close else (80, 80, 80)
    draw_swatch(x0, y0, (r8,g8,b8),
        f"n={rm['n']}",
        [f"RGB {r8} {g8} {b8}",
         f"H={rm['H']:.3f}  S={rm['S']:.3f}",
         f"L={rm['L']:.3f}",
         f"\u0394H={rm['dH']:.3f}  \u0394S={rm['dS']:.3f}  \u0394L={rm['dL']:.3f}",
         "\u25c4 too close to null" if too_close else ""],
        border=border)
cy += needed_rows(len(reject_means)) * CELL_H

out = "bead_swatches_white_bg.png"
img.save(out, dpi=(150,150))
print(f"Saved {out}  ({IMG_W}x{IMG_H} px)")
print(f"  Accepted: {len(accepted_means)} beads")
print(f"  Rejects:  {len(rejects)} total → {len(reject_means)} clusters")
