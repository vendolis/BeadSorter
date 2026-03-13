#!/usr/bin/env python3
"""
analyse_beads.py — BeadSorter colour-space analysis tool
=========================================================

Reads the CSV produced by BeadCalibrate.ino and answers:
  1. Which TCS34725 gain setting gives clean, stable readings (no saturation,
     low intra-bead variance)?
  2. Which colour space — HSL, Chromaticity, Normalized RGB — best separates
     the scanned bead colours (maximum minimum pairwise distance)?
  3. What per-channel thresholds should go into BeadSorter.ino?

Usage:
  python analyse_beads.py  beads.csv
  python analyse_beads.py  beads.csv  --gain g16x
  python analyse_beads.py  beads.csv  --top 15

Output sections:
  • Saturation summary (per gain)
  • Per-bead mean values table (per gain)
  • Pairwise distance table — closest pairs first (per gain × colour space)
  • Cross-gain separability comparison
  • Threshold recommendations for BeadSorter.ino

Lines starting with '#' in the CSV are treated as comments and ignored.

Colour spaces compared
----------------------
  HSL       — H (circular), S, L  (same as BeadSorter.ino currently uses)
  HS        — H (circular), S only  (tests whether L helps or hurts)
  Chroma2   — chroma_r, chroma_g  (ambient-independent 2-D chromaticity)
  Chroma3   — chroma_r, chroma_g, chroma_b  (3-D; redundant since r+g+b=1
              but included for completeness)
  NormRGB   — R_norm, G_norm, B_norm  (clear-normalised, 0-255 scale)

Notes
-----
  - Chroma2 is usually best for this sensor because it is immune to absolute
    brightness variation and captures hue+saturation in a single 2-D metric.
  - HSL is good when hue spans a wide arc; it degrades for low-saturation
    colours where hue is poorly defined / noisy.
  - 16x gain often saturates bright reds/whites; if saturation % is high,
    prefer 4x or 1x for those beads.
"""

import sys
import csv
import math
import collections
import itertools
import argparse
import statistics

# ── Circular hue helpers ──────────────────────────────────────────────────────

def hue_dist(h1: float, h2: float) -> float:
    """Shortest angular distance between two hue values in [0, 1]."""
    d = abs(h1 - h2)
    return min(d, 1.0 - d)


def circular_mean_hue(hues) -> float:
    """Compute the circular mean of hue values in [0, 1]."""
    sin_sum = sum(math.sin(h * 2.0 * math.pi) for h in hues)
    cos_sum = sum(math.cos(h * 2.0 * math.pi) for h in hues)
    angle = math.atan2(sin_sum, cos_sum)
    if angle < 0.0:
        angle += 2.0 * math.pi
    return angle / (2.0 * math.pi)


# ── Distance functions (one per colour space) ─────────────────────────────────

def dist_hsl(a: dict, b: dict) -> float:
    dh = hue_dist(a["H"], b["H"])
    return math.sqrt(dh**2 + (a["S"] - b["S"])**2 + (a["L"] - b["L"])**2)


def dist_hs(a: dict, b: dict) -> float:
    """HSL without L — tests whether lightness adds discriminating power."""
    dh = hue_dist(a["H"], b["H"])
    return math.sqrt(dh**2 + (a["S"] - b["S"])**2)


def dist_chroma2(a: dict, b: dict) -> float:
    """2-D chromaticity: chroma_r and chroma_g only (chroma_b = 1 - r - g)."""
    return math.sqrt((a["chroma_r"] - b["chroma_r"])**2 +
                     (a["chroma_g"] - b["chroma_g"])**2)


def dist_chroma3(a: dict, b: dict) -> float:
    """3-D chromaticity: all three normalised colour fractions."""
    return math.sqrt((a["chroma_r"] - b["chroma_r"])**2 +
                     (a["chroma_g"] - b["chroma_g"])**2 +
                     (a["chroma_b"] - b["chroma_b"])**2)


def dist_normrgb(a: dict, b: dict) -> float:
    """Euclidean distance in clear-normalised RGB space (0-255 scale)."""
    return math.sqrt((a["R_norm"] - b["R_norm"])**2 +
                     (a["G_norm"] - b["G_norm"])**2 +
                     (a["B_norm"] - b["B_norm"])**2)


# Ordered dict so the table columns stay consistent.
COLOR_SPACES = {
    "HSL":     dist_hsl,
    "HS":      dist_hs,
    "Chroma2": dist_chroma2,
    "Chroma3": dist_chroma3,
    "NormRGB": dist_normrgb,
}

# ── CSV parsing ───────────────────────────────────────────────────────────────

def parse_csv(path: str) -> list:
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        # Filter comment lines before handing to DictReader.
        clean = (line for line in fh if not line.lstrip().startswith("#"))
        reader = csv.DictReader(clean)
        for row in reader:
            try:
                entry = {
                    "bead":     int(row["bead"]),
                    "gain":     row["gain"].strip(),
                    "sample":   int(row["sample"]),
                    "R_raw":    int(row["R_raw"]),
                    "G_raw":    int(row["G_raw"]),
                    "B_raw":    int(row["B_raw"]),
                    "C_raw":    int(row["C_raw"]),
                    "sat":      int(row["saturated"]) != 0,
                    "R_norm":   float(row["R_norm"]),
                    "G_norm":   float(row["G_norm"]),
                    "B_norm":   float(row["B_norm"]),
                    "chroma_r": float(row["chroma_r"]),
                    "chroma_g": float(row["chroma_g"]),
                    "chroma_b": float(row["chroma_b"]),
                    "H":        float(row["H"]),
                    "S":        float(row["S"]),
                    "L":        float(row["L"]),
                }
                rows.append(entry)
            except (KeyError, ValueError):
                pass  # skip malformed rows silently
    return rows

# ── Per-bead statistics ───────────────────────────────────────────────────────

SCALAR_FIELDS = ["R_norm", "G_norm", "B_norm",
                 "chroma_r", "chroma_g", "chroma_b",
                 "S", "L"]


def compute_bead_means(rows: list, gain: str) -> dict:
    """
    Return {bead_id: mean_dict} for the given gain, excluding saturated rows.
    mean_dict contains mean values for every colour field plus 'n' (sample count).
    """
    filtered = [r for r in rows if r["gain"] == gain and not r["sat"]]
    by_bead = collections.defaultdict(list)
    for r in filtered:
        by_bead[r["bead"]].append(r)

    means = {}
    for bead, rs in sorted(by_bead.items()):
        m = {"bead": bead, "n": len(rs)}
        m["H"] = circular_mean_hue([r["H"] for r in rs])
        for f in SCALAR_FIELDS:
            m[f] = statistics.mean(r[f] for r in rs)
        # Intra-bead noise (std dev); needs ≥ 2 samples.
        if len(rs) >= 2:
            m["std_H"] = statistics.stdev(r["H"] for r in rs)
            for f in SCALAR_FIELDS:
                m[f"std_{f}"] = statistics.stdev(r[f] for r in rs)
        means[bead] = m
    return means


# ── Pairwise analysis ─────────────────────────────────────────────────────────

def pairwise_distances(means: dict, dist_fn) -> list:
    """Return sorted list of (distance, bead_a, bead_b) for all pairs."""
    pairs = [
        (dist_fn(means[a], means[b]), a, b)
        for a, b in itertools.combinations(sorted(means.keys()), 2)
    ]
    pairs.sort()
    return pairs


def suggest_hsl_thresholds(means: dict) -> tuple:
    """
    Return (thH, thS, thL) as half the minimum per-channel inter-bead gap.
    This is the tightest threshold that does not split any pair.

    Note: this is computed per-channel independently (AND-logic matching).
    A pair separated by one channel won't be penalised in another channel's gap.
    """
    beads = sorted(means.keys())
    min_dH = min_dS = min_dL = float("inf")
    for a, b in itertools.combinations(beads, 2):
        dh = hue_dist(means[a]["H"], means[b]["H"])
        ds = abs(means[a]["S"] - means[b]["S"])
        dl = abs(means[a]["L"] - means[b]["L"])
        if dh < min_dH:
            min_dH = dh
        if ds < min_dS:
            min_dS = ds
        if dl < min_dL:
            min_dL = dl
    return min_dH / 2.0, min_dS / 2.0, min_dL / 2.0


def suggest_chroma_threshold(means: dict) -> float:
    """Return half the minimum 2-D chromaticity distance between any pair."""
    beads = sorted(means.keys())
    min_d = min(
        dist_chroma2(means[a], means[b])
        for a, b in itertools.combinations(beads, 2)
    )
    return min_d / 2.0


# ── Formatting helpers ────────────────────────────────────────────────────────

def sep(char="─", width=62):
    print(char * width)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Analyse BeadCalibrate.ino CSV — find best gain + colour space.")
    ap.add_argument("csv_file", help="CSV file produced by BeadCalibrate.ino")
    ap.add_argument("--gain", default=None,
                    help="Restrict analysis to one gain label, e.g. g16x")
    ap.add_argument("--top", type=int, default=10,
                    help="Number of closest bead pairs to list (default 10)")
    args = ap.parse_args()

    rows = parse_csv(args.csv_file)
    if not rows:
        print("No data found.  Check path and CSV format.", file=sys.stderr)
        sys.exit(1)

    all_gains   = sorted({r["gain"] for r in rows})
    all_beads   = sorted({r["bead"] for r in rows})
    gains_todo  = [args.gain] if args.gain else all_gains

    print(f"\nLoaded {len(rows)} rows | {len(all_beads)} beads | "
          f"gains: {', '.join(all_gains)}")

    # ── 1. Saturation summary ──────────────────────────────────────────────────
    sep("═")
    print("SATURATION SUMMARY  (saturated rows are excluded from analysis)")
    sep("═")
    print(f"  {'Gain':8s}  {'Saturated':>12s}  {'Total':>8s}  {'%':>5s}")
    sep()
    for gain in all_gains:
        total = [r for r in rows if r["gain"] == gain]
        sat   = [r for r in total if r["sat"]]
        pct   = 100.0 * len(sat) / max(len(total), 1)
        flag  = "  ← SATURATED" if pct > 20 else ""
        print(f"  {gain:8s}  {len(sat):>12d}  {len(total):>8d}  {pct:>4.0f}%{flag}")
    print()

    # Collect best-colour-space score per gain for cross-gain table.
    cross_gain_scores = {}  # gain → {cs_name: min_dist}

    # ── 2. Per-gain deep analysis ──────────────────────────────────────────────
    for gain in gains_todo:
        sep("═")
        print(f"GAIN: {gain}")
        sep("═")

        means = compute_bead_means(rows, gain)
        if len(means) < 2:
            print(f"  Fewer than 2 unsaturated beads at gain {gain}. Skipping.\n")
            continue

        # ── 2a. Per-bead averages ──────────────────────────────────────────────
        print(f"\nPer-bead mean values  (n = samples used, excl. saturated)")
        sep()
        hdr = (f"  {'Bead':>5}  {'H':>7}  {'S':>7}  {'L':>7}  "
               f"{'chR':>7}  {'chG':>7}  {'chB':>7}  {'n':>3}")
        print(hdr)
        sep()
        for bead, m in sorted(means.items()):
            print(f"  {bead:5d}  {m['H']:7.4f}  {m['S']:7.4f}  {m['L']:7.4f}  "
                  f"{m['chroma_r']:7.4f}  {m['chroma_g']:7.4f}  {m['chroma_b']:7.4f}  {m['n']:3d}")

        # ── 2b. Intra-bead noise ──────────────────────────────────────────────
        noise_key = "std_H"
        noisy = {b: m for b, m in means.items() if noise_key in m}
        if noisy:
            avg_std_H = statistics.mean(m["std_H"] for m in noisy.values())
            avg_std_S = statistics.mean(m.get("std_S", 0) for m in noisy.values())
            avg_std_L = statistics.mean(m.get("std_L", 0) for m in noisy.values())
            avg_std_chR = statistics.mean(m.get("std_chroma_r", 0) for m in noisy.values())
            avg_std_chG = statistics.mean(m.get("std_chroma_g", 0) for m in noisy.values())
            print(f"\n  Average intra-bead std dev (noise):")
            print(f"    H={avg_std_H:.5f}  S={avg_std_S:.5f}  L={avg_std_L:.5f}"
                  f"  chR={avg_std_chR:.5f}  chG={avg_std_chG:.5f}")

        # ── 2c. Pairwise distances per colour space ───────────────────────────
        print(f"\nColour-space separability  (min inter-bead distance — higher = better)")
        sep()
        cs_scores = {}
        best_cs_name = None
        best_cs_score = -1.0
        for cs_name, dist_fn in COLOR_SPACES.items():
            pairs = pairwise_distances(means, dist_fn)
            min_d, ba, bb = pairs[0]
            cs_scores[cs_name] = min_d
            flag = " ◄ BEST" if False else ""  # placeholder; filled below
            print(f"  {cs_name:8s}  min dist = {min_d:.5f}  (closest pair: beads {ba} vs {bb})")
            if min_d > best_cs_score:
                best_cs_score = min_d
                best_cs_name  = cs_name

        cross_gain_scores[gain] = cs_scores
        print(f"\n  ► Best colour space at this gain: {best_cs_name}"
              f"  (min pair dist = {best_cs_score:.5f})")

        # ── 2d. Closest pairs in the best colour space ────────────────────────
        best_pairs = pairwise_distances(means, COLOR_SPACES[best_cs_name])
        n_show = min(args.top, len(best_pairs))
        print(f"\n  Closest {n_show} bead pairs in {best_cs_name} space:")
        sep()
        for dist, ba, bb in best_pairs[:n_show]:
            warn = "  ← may collide" if dist < 0.040 else ""
            print(f"    beads {ba:3d} vs {bb:3d}  dist = {dist:.5f}{warn}")

        # ── 2e. Threshold suggestions ─────────────────────────────────────────
        thH, thS, thL = suggest_hsl_thresholds(means)
        thCh = suggest_chroma_threshold(means)
        print(f"\n  Suggested thresholds for BeadSorter.ino  (half the min inter-bead gap):")
        print(f"    HSL:       thresholdH = {thH:.4f}  "
              f"thresholdS = {thS:.4f}  thresholdL = {thL:.4f}")
        print(f"    Chroma2:   threshold  = {thCh:.4f}  "
              f"(Euclidean distance in chroma_r / chroma_g plane)")
        print()

    # ── 3. Cross-gain comparison ───────────────────────────────────────────────
    if len(gains_todo) > 1 and len(cross_gain_scores) >= 2:
        sep("═")
        print("CROSS-GAIN COMPARISON  (min pairwise distance — higher = better)")
        sep("═")
        cs_names = list(COLOR_SPACES.keys())
        header = f"  {'Gain':8s}" + "".join(f"  {n:>8s}" for n in cs_names)
        print(header)
        sep()
        for gain in gains_todo:
            if gain not in cross_gain_scores:
                continue
            row_str = f"  {gain:8s}"
            row_str += "".join(
                f"  {cross_gain_scores[gain].get(n, 0.0):>8.4f}"
                for n in cs_names
            )
            print(row_str)

        # Best overall combination.
        best_gain = max(
            cross_gain_scores,
            key=lambda g: max(cross_gain_scores[g].values(), default=0.0)
        )
        best_cs_all = max(
            COLOR_SPACES,
            key=lambda cs: cross_gain_scores.get(best_gain, {}).get(cs, 0.0)
        )
        gain_enum = {
            "g1x":  "TCS34725_GAIN_1X",
            "g4x":  "TCS34725_GAIN_4X",
            "g16x": "TCS34725_GAIN_16X",
            "g60x": "TCS34725_GAIN_60X",
        }
        print()
        sep("═")
        print(f"RECOMMENDATION")
        sep("═")
        print(f"  Best gain:        {best_gain}")
        print(f"  Best colour space: {best_cs_all}")
        print()
        print(f"  If switching gain in BeadSorter.ino:")
        print(f"    Adafruit_TCS34725 tcs = Adafruit_TCS34725(")
        print(f"        TCS34725_INTEGRATIONTIME_101MS,")
        print(f"        {gain_enum.get(best_gain, best_gain)});")

        # Print best-gain thresholds one more time.
        best_means = compute_bead_means(rows, best_gain)
        if len(best_means) >= 2:
            thH, thS, thL = suggest_hsl_thresholds(best_means)
            thCh = suggest_chroma_threshold(best_means)
            print()
            print(f"  HSL thresholds at {best_gain}:")
            print(f"    float thresholdH = {thH:.4f};")
            print(f"    float thresholdS = {thS:.4f};")
            print(f"    float thresholdL = {thL:.4f};")
            print()
            print(f"  Chroma2 threshold at {best_gain}:  {thCh:.4f}")
            print(f"  (Store chroma_r and chroma_g per colour and compare with"
                  f" dist = sqrt(dCR²+dCG²))")
        print()


if __name__ == "__main__":
    main()
