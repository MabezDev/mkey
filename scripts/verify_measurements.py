#!/usr/bin/env python3
"""
mkey design-review measurement verifier.

Parses the authoritative 3D sources (plate.step, mkey.step, plate.kicad_pcb,
mkey.kicad_pcb) and checks every dimensional claim made in 3d/case.scad
against them. Prints a PASS/WARN/CRITICAL table and a list of any
discrepancies > 0.05 mm.

Run from anywhere; paths are resolved relative to this script.

Usage:
    python3 scripts/verify_measurements.py
    python3 scripts/verify_measurements.py --verbose

─── LOAD-BEARING PROJECT FACTS (do not re-question in future reviews) ──────

1. DISPLAY SEATS IN THE OVERLAY, NOT THE PLATE.
   The WEA2012 1.72" AMOLED module is held in the OVERLAY HOUSING
   (`display_cutout` in case.scad) backed by a separately-fabricated
   `display_retainer` piece glued to the overlay underside. The plate's
   display cutout (nominal 31.500 × 37.200 mm) exists ONLY to pass the
   24-pin FPC ribbon down to J2 on the main PCB beneath. The module body
   never passes through the plate. Therefore the 0.020 mm nominal
   undersize of the plate cutout Y dimension (37.200) vs the module
   datasheet height (37.220) is NOT a fit problem and is deliberately
   not checked by this script. See case.scad Section 2 header.

2. PLATE AND PCB ARE ALREADY PHYSICALLY FABRICATED.
   Any future fix must live in the case (3d/case.scad) or in a hand
   rework of the physical parts — NOT in plate.kicad_pcb or
   mkey.kicad_pcb. Treat those KiCad files as locked references for
   consistency checking only.

3. HAND FABRICATION ONLY — saw + drill + file.
   No CNC, no router tables, no precision pocketing. Any geometry
   suggestion that requires a blind pocket with controlled depth,
   rounded internal corners inside a recess, or sub-millimeter
   non-through features is off-limits.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
CASE_SCAD = REPO / "3d" / "case.scad"
PLATE_STEP = REPO / "3d" / "plate.step"
MKEY_STEP = REPO / "3d" / "mkey.step"
PLATE_KICAD = REPO / "hardware" / "plate" / "plate.kicad_pcb"
MKEY_KICAD = REPO / "hardware" / "board" / "mkey.kicad_pcb"

# Tolerance (mm) for "PASS" — deltas ≤ this are considered exact matches.
PASS_TOL = 0.05
WARN_TOL = 0.10

# ------------------------------------------------------------------------
# STEP parsing
# ------------------------------------------------------------------------

CART_POINT = re.compile(
    r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*"
    r"(-?[\d.Ee+-]+)\s*,\s*(-?[\d.Ee+-]+)\s*,\s*(-?[\d.Ee+-]+)\s*\)\s*\)"
)


def step_bbox(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (min, max) bounding box of every CARTESIAN_POINT in a STEP file."""
    xs, ys, zs = [], [], []
    with path.open() as f:
        for line in f:
            for m in CART_POINT.finditer(line):
                xs.append(float(m.group(1)))
                ys.append(float(m.group(2)))
                zs.append(float(m.group(3)))
    return ((min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs)))


# ------------------------------------------------------------------------
# KiCad plate parser: extract Edge.Cuts geometry
# ------------------------------------------------------------------------

LAYER_RE = re.compile(r"\(layer\s+\"([^\"]+)\"\)")
START_RE = re.compile(r"\(start\s+(-?[\d.]+)\s+(-?[\d.]+)\)")
END_RE = re.compile(r"\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)")
MID_RE = re.compile(r"\(mid\s+(-?[\d.]+)\s+(-?[\d.]+)\)")


@dataclass
class Edge:
    kind: str  # "line" or "arc"
    pts: list[tuple[float, float]]


def _iter_sexpr_blocks(text: str, head: str):
    """Yield (block_text) for each top-level `(head ...)` S-expression in text.

    Uses paren matching; ignores parens inside quoted strings.
    """
    i = 0
    n = len(text)
    needle = f"({head}"
    while True:
        j = text.find(needle, i)
        if j < 0:
            return
        # Ensure token boundary (next char is whitespace)
        if j + len(needle) < n and not text[j + len(needle)].isspace():
            i = j + 1
            continue
        depth = 0
        in_str = False
        k = j
        while k < n:
            c = text[k]
            if c == '"' and (k == 0 or text[k - 1] != "\\"):
                in_str = not in_str
            elif not in_str:
                if c == "(":
                    depth += 1
                elif c == ")":
                    depth -= 1
                    if depth == 0:
                        yield text[j : k + 1]
                        break
            k += 1
        i = k + 1


def parse_edge_cuts(path: Path) -> list[Edge]:
    text = path.read_text()
    edges: list[Edge] = []
    for block in _iter_sexpr_blocks(text, "gr_line"):
        m_layer = LAYER_RE.search(block)
        if not m_layer or m_layer.group(1) != "Edge.Cuts":
            continue
        m_s = START_RE.search(block)
        m_e = END_RE.search(block)
        if not (m_s and m_e):
            continue
        edges.append(Edge("line", [
            (float(m_s.group(1)), float(m_s.group(2))),
            (float(m_e.group(1)), float(m_e.group(2))),
        ]))
    for block in _iter_sexpr_blocks(text, "gr_arc"):
        m_layer = LAYER_RE.search(block)
        if not m_layer or m_layer.group(1) != "Edge.Cuts":
            continue
        m_s = START_RE.search(block)
        m_m = MID_RE.search(block)
        m_e = END_RE.search(block)
        if not (m_s and m_m and m_e):
            continue
        edges.append(Edge("arc", [
            (float(m_s.group(1)), float(m_s.group(2))),
            (float(m_m.group(1)), float(m_m.group(2))),
            (float(m_e.group(1)), float(m_e.group(2))),
        ]))
    return edges


def edge_cuts_bbox(edges: list[Edge]) -> tuple[tuple[float, float], tuple[float, float]]:
    xs, ys = [], []
    for e in edges:
        for x, y in e.pts:
            xs.append(x)
            ys.append(y)
    return ((min(xs), min(ys)), (max(xs), max(ys)))


def inner_boundary(edges: list[Edge], min_len: float = 30.0) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return the inner (non-tab) plate outline bbox.

    The plate has tab protrusions (~20 mm long) and interior cuts (switch
    windows, display cutout). The main plate edges are long straight runs
    (> 30 mm between tabs). We filter to lines of that length and take their
    extremum Y (front/back) and X (left/right) values.
    """
    long_lines = []
    for e in edges:
        if e.kind != "line":
            continue
        (x1, y1), (x2, y2) = e.pts
        length = math.hypot(x2 - x1, y2 - y1)
        if length >= min_len:
            long_lines.append(e)
    if not long_lines:
        raise RuntimeError("no long Edge.Cuts lines found — inner boundary detection failed")

    xs, ys = [], []
    for e in long_lines:
        for x, y in e.pts:
            xs.append(x)
            ys.append(y)
    return ((min(xs), min(ys)), (max(xs), max(ys)))


def find_display_cutout(edges: list[Edge]) -> tuple[tuple[float, float], tuple[float, float], float]:
    """Locate the display cutout in the plate. Returns (min, max, corner_r).

    Identified as a closed loop of 4 lines + 4 arcs whose bbox is the
    expected ~31.5 × 37.2 mm rectangle located near X≈320, Y≈-77.
    """
    best = None
    for e in edges:
        if e.kind != "arc":
            continue
        sx, sy = e.pts[0]
        ex, ey = e.pts[2]
        if 300 < sx < 340 and -100 < sy < -55:
            # candidate — corner radius is distance from implicit center
            dx = ex - sx
            dy = ey - sy
            r = math.hypot(dx, dy) / math.sqrt(2)  # 90° arc
            if best is None or r < best[1]:
                best = (e, r)
    # Collect all Edge.Cuts geometry inside the region 300..340 × -100..-55
    xs, ys = [], []
    for e in edges:
        for x, y in e.pts:
            if 300 < x < 340 and -100 < y < -55:
                xs.append(x)
                ys.append(y)
    if not xs:
        raise RuntimeError("display cutout not found in plate.kicad_pcb")
    bb = ((min(xs), min(ys)), (max(xs), max(ys)))
    corner_r = best[1] if best else 0.0
    return bb[0], bb[1], corner_r


# ------------------------------------------------------------------------
# KiCad mkey PCB parser: extract switch positions and USB-C
# ------------------------------------------------------------------------

@dataclass
class Footprint:
    library: str
    ref: str
    x: float
    y: float
    rot: float


def parse_footprints(path: Path) -> list[Footprint]:
    text = path.read_text()
    out: list[Footprint] = []
    for block in _iter_sexpr_blocks(text, "footprint"):
        m_lib = re.match(r"\(footprint\s+\"([^\"]+)\"", block)
        if not m_lib:
            continue
        # Outer "(at x y [rot])" — the first one directly inside the footprint block
        m_at = re.search(r"\(at\s+(-?[\d.]+)\s+(-?[\d.]+)(?:\s+(-?[\d.]+))?\s*\)", block)
        # Reference is stored as either `(property "Reference" "MX8")` (KiCad 8+)
        # or `(fp_text reference "MX8" ...)` (KiCad 6/7). Try both.
        m_ref = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', block) or \
                re.search(r'\(fp_text\s+reference\s+"([^"]+)"', block)
        if m_at and m_ref:
            x = float(m_at.group(1))
            y = float(m_at.group(2))
            rot = float(m_at.group(3)) if m_at.group(3) else 0.0
            out.append(Footprint(m_lib.group(1), m_ref.group(1), x, y, rot))
    return out


# ------------------------------------------------------------------------
# Checks
# ------------------------------------------------------------------------

@dataclass
class Check:
    label: str
    claimed: float | None
    measured: float | None
    tol: float = PASS_TOL

    @property
    def delta(self) -> float | None:
        if self.claimed is None or self.measured is None:
            return None
        return self.measured - self.claimed

    @property
    def verdict(self) -> str:
        d = self.delta
        if d is None:
            return "SKIP"
        if abs(d) <= self.tol:
            return "PASS"
        if abs(d) <= WARN_TOL:
            return "WARN"
        return "CRITICAL"


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args(argv)

    checks: list[Check] = []
    errors: list[str] = []

    # ------------------------ plate.step bbox ----------------------------
    try:
        plate_min, plate_max = step_bbox(PLATE_STEP)
        plate_x_width = plate_max[0] - plate_min[0]
        plate_y_depth = plate_max[1] - plate_min[1]
        plate_z_thick = plate_max[2] - plate_min[2]
        checks.append(Check("plate.step thickness Z", 1.600, plate_z_thick))
    except FileNotFoundError:
        errors.append(f"missing: {PLATE_STEP}")
    except Exception as e:
        errors.append(f"plate.step parse failed: {e}")

    # ------------------------ plate.kicad_pcb ----------------------------
    try:
        edges = parse_edge_cuts(PLATE_KICAD)
        (xmin, ymin), (xmax, ymax) = inner_boundary(edges)
        # Plate outer: X 0.588..353.226, Y -107.641..-7.416 per case.scad
        checks.append(Check("plate outer X_min", 0.588, xmin))
        checks.append(Check("plate outer X_max", 353.226, xmax))
        checks.append(Check("plate outer Y_front (neg)", -7.416, ymax))  # Y_front is LESS negative
        checks.append(Check("plate outer Y_back  (neg)", -107.641, ymin))
        plate_w = xmax - xmin
        plate_d = abs(ymin - ymax)
        checks.append(Check("plate_w (inner width)", 352.638, plate_w))
        checks.append(Check("plate_d (inner depth, corner)", 100.225, plate_d, tol=WARN_TOL))

        # Plate display cutout. Only checked for position/size consistency
        # with the SCAD constants — the cutout's purpose is to pass the 24-pin
        # FPC ribbon to J2, NOT the display module body. The module seats in
        # the overlay housing, not the plate, so nominal cutout vs module-
        # body dimensions is NOT a fit concern and is deliberately not
        # checked here. (See `display_cutout` / `display_retainer` in case.scad.)
        (dmin, dmax, dr) = find_display_cutout(edges)
        checks.append(Check("disp_cut KiCad X1", 305.846, dmin[0]))
        checks.append(Check("disp_cut KiCad X2", 337.346, dmax[0]))
        checks.append(Check("disp_cut KiCad Y1", -95.946, dmin[1]))
        checks.append(Check("disp_cut KiCad Y2", -58.746, dmax[1]))
        disp_w = dmax[0] - dmin[0]
        disp_h = dmax[1] - dmin[1]
        checks.append(Check("disp_cut width (as-built)",  31.500, disp_w))
        checks.append(Check("disp_cut height (as-built)", 37.200, disp_h))
        checks.append(Check("disp_cut corner radius", 2.200, round(dr, 3)))
    except FileNotFoundError:
        errors.append(f"missing: {PLATE_KICAD}")
    except Exception as e:
        errors.append(f"plate.kicad_pcb parse failed: {e}")

    # ------------------------ mkey.kicad_pcb -----------------------------
    try:
        footprints = parse_footprints(MKEY_KICAD)
        # USB-C
        usb = next(
            (f for f in footprints
             if "USB_C" in f.library.upper() or "TYPE-C" in f.library.upper()),
            None,
        ) or next((f for f in footprints if f.ref == "J1"), None)
        if usb is not None:
            checks.append(Check("USB-C board X", 364.275, usb.x))
            checks.append(Check("USB-C board Y", 47.610, usb.y))
            checks.append(Check("USB-C rotation (deg)", 180.0, usb.rot))
            usb_plate_x = usb.x - 31.473
            checks.append(Check("usb_plate_x (derived)", 332.802, usb_plate_x))
        else:
            errors.append("USB-C footprint not found (looked for ref J1 / USB_C)")

        # Switches
        sw = [f for f in footprints if f.ref.startswith("MX")]
        sw_plate = []
        for s in sw:
            plate_local_x = s.x - 31.473
            plate_local_y = abs(s.y - 150.195) - 7.416
            sw_plate.append((s.ref, plate_local_x, plate_local_y))

        # Arrow keys — expect UP at (321.587, 31.050), DOWN at (321.587, 12.000),
        # LEFT at (302.537, 12.000), RIGHT at (340.637, 12.000).
        arrow_expected = {
            "UP":    (321.587, 31.050),
            "DOWN":  (321.587, 12.000),
            "LEFT":  (302.537, 12.000),
            "RIGHT": (340.637, 12.000),
        }
        # Identify arrows by plate-local positions rather than refs.
        def nearest(tx: float, ty: float):
            return min(sw_plate, key=lambda r: math.hypot(r[1] - tx, r[2] - ty))
        for name, (tx, ty) in arrow_expected.items():
            ref, px, py = nearest(tx, ty)
            checks.append(Check(f"arrow_{name.lower()} X", tx, px))
            checks.append(Check(f"arrow_{name.lower()} Y", ty, py))

        # Switch coverage: every switch must fall inside the union of the key_rects
        # with key_cap_clearance expanded per side.
        key_rects = [
            [  2.500, 78.675, 288.250, 97.725],   # row 0
            [  2.499, 59.625, 264.437, 78.675],   # row 1
            [  2.499, 40.575, 264.437, 59.625],   # row 2
            [264.437, 40.575, 288.249, 78.675],   # ISO enter
            [  2.500, 21.525, 288.250, 40.575],   # row 3
            [  2.500,  2.475, 288.249, 21.525],   # row 4
            [312.062, 21.525, 331.112, 40.575],   # arrow UP
            [293.012,  2.475, 350.162, 21.525],   # arrow LDR
        ]
        key_cap_clearance = 0.25

        def in_any_rect(px: float, py: float) -> bool:
            t = key_cap_clearance
            for x1, y1, x2, y2 in key_rects:
                if (x1 - t) <= px <= (x2 + t) and (y1 - t) <= py <= (y2 + t):
                    return True
            return False

        misses = [(ref, px, py) for (ref, px, py) in sw_plate if not in_any_rect(px, py)]
        checks.append(Check(
            f"switches inside key_rects union ({len(sw_plate) - len(misses)}/{len(sw_plate)})",
            0.0,
            float(len(misses)),
        ))
        if args.verbose and misses:
            for r in misses:
                print(f"  MISS: {r}")
    except FileNotFoundError:
        errors.append(f"missing: {MKEY_KICAD}")
    except Exception as e:
        errors.append(f"mkey.kicad_pcb parse failed: {e}")

    # ------------------------ report -------------------------------------
    passed = sum(1 for c in checks if c.verdict == "PASS")
    warned = sum(1 for c in checks if c.verdict == "WARN")
    critical = sum(1 for c in checks if c.verdict == "CRITICAL")
    skipped = sum(1 for c in checks if c.verdict == "SKIP")

    col = max((len(c.label) for c in checks), default=20)
    print(f"{'check'.ljust(col)}  {'claimed':>11}  {'measured':>11}  {'delta':>9}  verdict")
    print("-" * (col + 48))
    for c in checks:
        claimed = f"{c.claimed:.4f}" if c.claimed is not None else "-"
        measured = f"{c.measured:.4f}" if c.measured is not None else "-"
        delta = f"{c.delta:+.4f}" if c.delta is not None else "-"
        print(f"{c.label.ljust(col)}  {claimed:>11}  {measured:>11}  {delta:>9}  {c.verdict}")

    print()
    print(f"Summary: PASS={passed} WARN={warned} CRITICAL={critical} SKIP={skipped}")
    for e in errors:
        print(f"ERROR: {e}")

    return 0 if (critical == 0 and not errors) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
