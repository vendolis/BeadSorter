#!/usr/bin/env python3
"""
Colour-swatch sheet for BeadCalibration_20260313_largerWindow.csv
  • Null reference  (1 swatch)
  • Each accepted bead at g16x  (individual swatches, NormRGB colour)
  • Each individual reject  (individual swatches, colour from HSL)
"""
import csv, re, math, collections, statistics
from PIL import Image, ImageDraw, ImageFont

CSV_RAW   = "BeadCalibration_20260313_largerWindow.csv"
CSV_CLEAN = "beads_largewin_clean.csv"
GAIN      = "g16x"

# ── HSL → RGB (all values in 0-1 range, returns 0-255 tuple) ─────────────────
def hsl_to_rgb255(h, s, l):
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
    return (round(_f(p, q, h + 1/3) * 255),
            round(_f(p, q, h)       * 255),
            round(_f(p, q, h - 1/3) * 255))

def clamp255(v): return min(255, max(0, round(v)))

def luma(r, g, b): return 0.299*r + 0.587*g + 0.114*b

def text_colour(r, g, b):
    return (255, 255, 255) if luma(r, g, b) < 145 else (20, 20, 20)

# ── Load accepted bead data (g16x) ───────────────────────────────────────────
rows = []
with open(CSV_CLEAN, newline="", encoding="utf-8") as fh:
    clean = (l for l in fh if not l.lstrip().startswith("#"))
    for row in csv.DictReader(clean):
        try:
            if row["gain"].strip() != GAIN: continue
            if int(row["saturated"]): continue
            rows.append({
                "bead":    int(row["bead"]),
                "R_norm":  float(row["R_norm"]),
                "G_norm":  float(row["G_norm"]),
                "B_norm":  float(row["B_norm"]),
                "H": float(row["H"]), "S": float(row["S"]), "L": float(row["L"]),
                "chR": float(row["chroma_r"]),
                "chG": float(row["chroma_g"]),
                "chB": float(row["chroma_b"]),
            })
        except (KeyError, ValueError):
            pass

by_bead = collections.defaultdict(list)
for r in rows:
    by_bead[r["bead"]].append(r)

bead_means = {}
for b, rs in sorted(by_bead.items()):
    bead_means[b] = {
        "R": statistics.mean(r["R_norm"] for r in rs),
        "G": statistics.mean(r["G_norm"] for r in rs),
        "B": statistics.mean(r["B_norm"] for r in rs),
        "H": statistics.mean(r["H"]      for r in rs),
        "S": statistics.mean(r["S"]      for r in rs),
        "L": statistics.mean(r["L"]      for r in rs),
        "chR": statistics.mean(r["chR"]  for r in rs),
        "chG": statistics.mean(r["chG"]  for r in rs),
        "chB": statistics.mean(r["chB"]  for r in rs),
    }

# ── Parse rejects and null ref from raw file ──────────────────────────────────
null_ref = None
rejects  = []
for line in open(CSV_RAW, encoding="utf-8"):
    m = re.search(r'Null ref: H=([\d.]+)\s+S=([\d.]+)\s+L=([\d.]+)', line)
    if m:
        null_ref = {"H": float(m.group(1)), "S": float(m.group(2)), "L": float(m.group(3))}

    m = re.search(
        r'REJECTED.*?read: H=([\d.]+) S=([\d.]+) L=([\d.]+)'
        r'.*null ref.*H=([\d.]+) S=([\d.]+) L=([\d.]+)', line)
    if m:
        rH, rS, rL, nH, nS, nL = [float(x) for x in m.groups()]
        rejects.append({
            "H": rH, "S": rS, "L": rL,
            "dH": abs(rH-nH), "dS": abs(rS-nS), "dL": abs(rL-nL),
        })

# ── Layout ────────────────────────────────────────────────────────────────────
COLS     = 6
SW_W     = 170   # swatch width
SW_H     = 120   # swatch height
TEXT_H   = 108   # text block below swatch
PAD      = 16    # horizontal gap between cells
CELL_W   = SW_W + PAD
CELL_H   = SW_H + TEXT_H + 8
MARGIN   = 28
SEC_HDR  = 44    # section header bar height
TITLE_H  = 40

def rows_needed(n): return max(1, math.ceil(n / COLS))

n_beads   = len(bead_means)
n_rejects = len(rejects)

IMG_W = COLS * CELL_W + 2 * MARGIN
IMG_H = (TITLE_H + MARGIN
         + SEC_HDR + CELL_H            # null ref
         + 14
         + SEC_HDR + rows_needed(n_beads)   * CELL_H + 14
         + SEC_HDR + rows_needed(n_rejects) * CELL_H + 14
         + MARGIN)

BG = (235, 235, 235)
img  = Image.new("RGB", (IMG_W, IMG_H), BG)
draw = ImageDraw.Draw(img)

try:
    F_TITLE = ImageFont.truetype("arialbd.ttf", 21)
    F_SEC   = ImageFont.truetype("arialbd.ttf", 15)
    F_LBL   = ImageFont.truetype("arialbd.ttf", 14)
    F_SM    = ImageFont.truetype("arial.ttf",   12)
    F_XS    = ImageFont.truetype("arial.ttf",   11)
except OSError:
    F_TITLE = F_SEC = F_LBL = F_SM = F_XS = ImageFont.load_default()

# ── Drawing helpers ───────────────────────────────────────────────────────────
def section_bar(y, title, note="", bg=(55, 95, 145)):
    draw.rectangle([0, y, IMG_W, y + SEC_HDR - 1], fill=bg)
    draw.text((MARGIN, y + 7), title, fill=(255, 255, 255), font=F_SEC)
    if note:
        nb = draw.textbbox((0,0), note, font=F_XS)
        nw = nb[2] - nb[0]
        draw.text((IMG_W - nw - MARGIN, y + 14), note, fill=(200, 220, 255), font=F_XS)
    return y + SEC_HDR

def draw_cell(x0, y0, fill_rgb, line1, lines_below, border_col=(70,70,70), border_w=1):
    r, g, b = fill_rgb
    # swatch
    draw.rectangle([x0, y0, x0+SW_W, y0+SW_H], fill=(r,g,b),
                   outline=border_col, width=border_w)
    # centre label inside swatch
    tc = text_colour(r, g, b)
    bb = draw.textbbox((0,0), line1, font=F_LBL)
    tw, th = bb[2]-bb[0], bb[3]-bb[1]
    draw.text((x0 + (SW_W-tw)//2, y0 + (SW_H-th)//2),
              line1, fill=tc, font=F_LBL)
    # text block below
    ty = y0 + SW_H + 5
    for txt in lines_below:
        draw.text((x0 + 2, ty), txt, fill=(35, 35, 35), font=F_SM)
        ty += 15

# ── Title ─────────────────────────────────────────────────────────────────────
cy = MARGIN
draw.text((MARGIN, cy),
          "BeadSorter — LargerWindow calibration run   (gain g16x)",
          fill=(20, 20, 20), font=F_TITLE)
cy += TITLE_H

# ── Section 1: Null reference ─────────────────────────────────────────────────
cy = section_bar(cy, "NULL REFERENCE  (empty-tube baseline)",
                 f"H={null_ref['H']:.4f}  S={null_ref['S']:.4f}  L={null_ref['L']:.4f}",
                 bg=(70, 70, 70))

nr_rgb = hsl_to_rgb255(null_ref["H"], null_ref["S"], null_ref["L"])
draw_cell(MARGIN, cy, nr_rgb,
          "NULL REF",
          [f"H={null_ref['H']:.4f}  S={null_ref['S']:.4f}",
           f"L={null_ref['L']:.4f}",
           f"RGB {nr_rgb[0]} {nr_rgb[1]} {nr_rgb[2]}",
           "(empty tube)"],
          border_col=(180, 50, 50), border_w=2)
cy += CELL_H + 14

# ── Section 2: Accepted beads ─────────────────────────────────────────────────
cy = section_bar(cy,
    f"ACCEPTED BEADS  ({n_beads})",
    f"NormRGB colour  ·  {GAIN}  ·  mean of 10 samples",
    bg=(35, 110, 55))

for idx, (bead, m) in enumerate(sorted(bead_means.items())):
    col = idx % COLS
    row = idx // COLS
    x0 = MARGIN + col * CELL_W
    y0 = cy + row * CELL_H
    r8 = clamp255(m["R"]); g8 = clamp255(m["G"]); b8 = clamp255(m["B"])
    dH = abs(m["H"] - null_ref["H"])
    dS = abs(m["S"] - null_ref["S"])
    dL = abs(m["L"] - null_ref["L"])
    draw_cell(x0, y0, (r8, g8, b8),
        f"Bead #{bead}",
        [f"RGB {r8} {g8} {b8}",
         f"H={m['H']:.4f}  S={m['S']:.4f}",
         f"L={m['L']:.4f}",
         f"chR={m['chR']:.4f}  chG={m['chG']:.4f}",
         f"\u0394H={dH:.4f}  \u0394S={dS:.4f}  \u0394L={dL:.4f}"])

cy += rows_needed(n_beads) * CELL_H + 14

# ── Section 3: Individual rejects ─────────────────────────────────────────────
cy = section_bar(cy,
    f"REJECTED BEADS  ({n_rejects} individual rejections)",
    "Colour reconstructed from H/S/L only  ·  red border = all deltas within null-offset",
    bg=(150, 55, 35))

NULL_OFF = (0.03, 0.05, 0.05)   # H, S, L offsets from BeadCalibrate.ino README

for idx, rj in enumerate(rejects):
    col = idx % COLS
    row = idx // COLS
    x0 = MARGIN + col * CELL_W
    y0 = cy + row * CELL_H
    rgb = hsl_to_rgb255(rj["H"], rj["S"], rj["L"])
    too_close = (rj["dH"] < NULL_OFF[0] and
                 rj["dS"] < NULL_OFF[1] and
                 rj["dL"] < NULL_OFF[2])
    border = (200, 40, 40) if too_close else (70, 70, 70)
    draw_cell(x0, y0, rgb,
        f"Rej #{idx+1}",
        [f"H={rj['H']:.4f}  S={rj['S']:.4f}",
         f"L={rj['L']:.4f}",
         f"\u0394H={rj['dH']:.4f}  \u0394S={rj['dS']:.4f}",
         f"\u0394L={rj['dL']:.4f}",
         "\u25c4 within null offsets" if too_close else "outside null offsets"],
        border_col=border, border_w=2)

cy += rows_needed(n_rejects) * CELL_H + 14

out = "bead_swatches_largewin.png"
img.save(out, dpi=(150, 150))
print(f"Saved {out}  ({IMG_W}\u00d7{IMG_H} px)")
print(f"  Null ref : H={null_ref['H']:.4f} S={null_ref['S']:.4f} L={null_ref['L']:.4f}")
print(f"  Beads    : {n_beads}")
print(f"  Rejects  : {n_rejects}")
