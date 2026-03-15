"""
parse_sorting_log.py  —  BeadSorter log → CSV

Usage:
    python parse_sorting_log.py <log_file> [options]

Options:
    --containers  Comma-separated list of containers to include (default: 0-14)
                  Use "all" to include container 15 as well.
    --no-nullscans  Exclude null scan rows from output.
    --out         Output CSV path (default: same folder as log, same stem + .csv)

Examples:
    python parse_sorting_log.py SortingLog_20260314185000.txt
    python parse_sorting_log.py SortingLog_20260314185000.txt --containers all  # includes 0-15
    python parse_sorting_log.py SortingLog_20260314185000.txt --containers 0,1,2
    python parse_sorting_log.py SortingLog_20260314185000.txt --out my_output.csv
"""

import argparse
import csv
import re
import sys
from collections import Counter
from pathlib import Path


# ── Helpers ──────────────────────────────────────────────────────────────────

def parse_raw(block):
    m = re.search(r'raw\s+R=(\d+) G=(\d+) B=(\d+) C=(\d+)', block)
    return (m.group(1), m.group(2), m.group(3), m.group(4)) if m else ('', '', '', '')


def parse_hsl(block):
    m = re.search(r'HSL\s+H=([\d.]+) S=([\d.]+) L=([\d.]+)', block)
    return (m.group(1), m.group(2), m.group(3)) if m else ('', '', '')


# ── Parsers ───────────────────────────────────────────────────────────────────

def parse_beads(content, containers):
    """Return list of dicts for bead events that match the requested containers."""
    rows = []
    for block in re.split(r'\n(?=Bead #\d+)', content):
        bead_m = re.search(r'Bead #(\d+)', block)
        if not bead_m:
            continue
        bead_no = int(bead_m.group(1))

        cont_m  = re.search(r'container array: (\d+)', block)
        store_m = re.search(r'StoreColor (\d+)', block)
        bin15_m = re.search(r'All sort slots full -> dumping to bin 15', block)

        if cont_m:
            container = int(cont_m.group(1))
        elif store_m:
            container = int(store_m.group(1))
        elif bin15_m:
            container = 15
        else:
            continue  # no container assignment found

        if container not in containers:
            continue

        R, G, B, C = parse_raw(block)
        H, S, L    = parse_hsl(block)
        rows.append({
            'type':      'bead',
            'bead_no':   bead_no,
            'container': container,
            'R': R, 'G': G, 'B': B, 'C': C,
            'H': H, 'S': S, 'L': L,
        })

    rows.sort(key=lambda r: r['bead_no'])
    return rows


def parse_nullscans(content):
    """Return list of dicts for every 'No bead detected (null scan)' event."""
    rows = []
    parts = re.split(r'No bead detected \(null scan\):', content)
    for part in parts[1:]:
        block = part.split('\n\n')[0]
        R, G, B, C = parse_raw(block)
        H, S, L    = parse_hsl(block)
        rows.append({
            'type':      'nullscan',
            'bead_no':   '',
            'container': '',
            'R': R, 'G': G, 'B': B, 'C': C,
            'H': H, 'S': S, 'L': L,
        })
    return rows


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description='Parse a BeadSorter sorting log into a CSV.')
    p.add_argument('log_file', help='Path to the .txt sorting log')
    p.add_argument('--containers', default='0-14',
                   help='Containers to include: "all", a range like "0-14", '
                        'or a comma-separated list like "0,1,2" (default: 0-14)')
    p.add_argument('--no-nullscans', action='store_true',
                   help='Exclude null scan rows from output')
    p.add_argument('--out', default=None,
                   help='Output CSV path (default: <log_stem>.csv next to log)')
    return p.parse_args()


def resolve_containers(spec):
    """Parse the --containers argument into a set of ints."""
    if spec.lower() == 'all':
        containers = set(range(0, 16))
        return containers
    containers = set()
    for part in spec.split(','):
        part = part.strip()
        if '-' in part:
            lo, hi = part.split('-', 1)
            containers.update(range(int(lo), int(hi) + 1))
        else:
            containers.add(int(part))
    return containers


def main():
    args = parse_args()

    log_path = Path(args.log_file)
    if not log_path.exists():
        print(f"Error: file not found: {log_path}", file=sys.stderr)
        sys.exit(1)

    out_path = Path(args.out) if args.out else log_path.with_suffix('.csv')

    containers = resolve_containers(args.containers)

    print(f"Reading: {log_path}")
    content = log_path.read_text(encoding='utf-8', errors='replace')

    bead_rows     = parse_beads(content, containers)
    nullscan_rows = [] if args.no_nullscans else parse_nullscans(content)

    rows = bead_rows + nullscan_rows

    fields = ['type', 'bead_no', 'container', 'R', 'G', 'B', 'C', 'H', 'S', 'L']
    with open(out_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    # Summary
    print(f"Bead rows: {len(bead_rows)}")
    if bead_rows:
        dist = Counter(r['container'] for r in bead_rows)
        for c in sorted(dist):
            print(f"  Container {c}: {dist[c]} beads")
    print(f"Null scan rows: {len(nullscan_rows)}")
    print(f"Total rows: {len(rows)}")
    print(f"Written to: {out_path}")


if __name__ == '__main__':
    main()
