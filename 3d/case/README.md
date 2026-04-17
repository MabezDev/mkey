# mKey Case — 3D Printing Build Guide

## Prerequisites

- [OpenSCAD](https://openscad.org/downloads.html) (CLI must be in `$PATH`)

## Quick Start

Export print-ready STLs with breakaway supports for 3D printing:

```bash
cd 3d/case
./build_stl.sh --supports
```

This produces `mkey_tray.stl` and `mkey_overlay.stl` in the current directory,
oriented for optimal printing (overlay flipped cosmetic-face-down, tray
bottom-down with cross-braces).

## Build Options

| Flag | Description |
|------|-------------|
| `--piece tray\|overlay\|both` | Which piece(s) to export (default: `both`) |
| `--supports` | Add internal anti-warp rib walls to the tray cavity |
| `--fn N` | Circle resolution override (default: `128`; file preview default is `48`) |
| `--outdir DIR` | Output directory (default: `.`) |

## Print Orientation

The build script sets `PRINT_MODE=true`, which transforms the parts for
optimal 3D printing:

- **Overlay**: The 5-degree wedge tilt is removed so the part lies perfectly flat.
  The cosmetic face (key openings / display window) is flipped **down** toward
  the build plate — this gives the best surface finish on both SLA resin and FDM.

- **Tray**: Printed bottom-down (the bottom is already flat at Z=0). With
  `--supports`, five internal rib walls are added inside the cavity, running
  front-to-back. These grow simultaneously with the tray walls during the
  SLA build, creating T-sections that resist the 363 mm long walls from
  bowing under cure shrinkage. After post-cure: score each rib along the
  front/back wall necks (in the Y direction, from inside the open cavity) —
  do NOT try to saw down from above, the 0.5 mm gap between rib top and
  wall top is too tight for a flush-cut blade. Snap out in segments using
  the pre-modelled perforations, then lightly sand the witness marks
  parallel to the wall surface (not into the thin neck stub).

## Design Parameters (case.scad)

The main toggles in `case.scad` control which features are included:

| Toggle | Default | Purpose |
|--------|---------|---------|
| `SHOW_TRAY` | `true` | Render the tray (bottom + walls) |
| `SHOW_OVERLAY` | `true` | Render the overlay (top surface) |
| `PRINT_MODE` | `false` | Re-orient pieces for 3D print export |
| `PRINT_SUPPORTS` | `false` | Add internal anti-warp rib walls to tray (print mode only) |
| `ENABLE_OVERLAY_RABBET` | `true` | Locating tongue/recess for X/Y registration |
| `ENABLE_MAGNET_POCKETS` | `true` | Blind magnet pockets for overlay hold-down |
| `ENABLE_DECORATIVE_TRIMS` | `true` | Debossed logos, pinstripe, initials |

## Fabrication Methods

**SLA resin (recommended)** — JLC3DP or equivalent. All tolerances and
minimum feature sizes are tuned for SLA's +/-0.2 mm accuracy. Use
`--supports` to add braces for the tray.

**MJF/SLS nylon** — NOT recommended. Loses the 1.0 mm shelf_frame_x display
window detail.

**FDM** — NOT suitable. Layer lines destroy the display cut tolerance and the
1.0 mm shelf frame sits below FDM's 1.6 mm wall minimum.

**Hand-cut hardwood** — Disable print-specific features:
```bash
# In case.scad or via -D flags:
ENABLE_DECORATIVE_TRIMS = false;
PRINT_MODE = false;
PRINT_SUPPORTS = false;
```

## Assembly

### Gasket install order

The plate is gasket-mounted: each of the 9 tabs is sandwiched between two
gasket strips inside its wall slot. Order matters — the plate cannot be
seated if the top gaskets are placed first.

1. **Bottom gaskets first.** Press a gasket strip into the bottom of each
   of the 9 slots before the plate goes in. These rest at the bottom of
   each slot, below where the tab will sit.
2. **Drop the plate in.** Lower the plate so every tab descends through
   the open slot top and lands on its bottom gasket.
3. **Top gaskets second.** Press a second gasket strip onto the top of
   each tab, filling the remaining slot height.
4. **Overlay caps the stack.** Seating the overlay on the wall tops
   closes the open-top slots and compresses the top gaskets against the
   tabs, which compresses the tabs against the bottom gaskets.

### Magnet polarity

The overlay is held to the tray by six Ø2×1 mm N52 disc magnet pairs —
one per tray pocket and one per matching overlay pocket. Each pair must
**attract**, not repel, or the overlay will kick itself off the seat.

1. **Pick a reference magnet** from the bag and mark one face (dab of
   paint, sharpie dot). Set it aside.
2. **Tray magnets** — install every magnet with the **same face up** as
   the reference. Before each press-fit, hold the magnet face-up next to
   the reference: if they repel, flip it.
3. **Overlay magnets** — install with the **opposite face up**. When the
   overlay is flipped onto the tray, each overlay magnet then presents
   the attracting pole downward toward its tray mate. Test by holding
   the magnet with its eventual-up face touching the reference's
   marked face: it should **attract**.
4. **Retain with thin CA glue.** Pockets are press-fit (magnet Ø2.0 +
   JLC3DP ±0.3 mm hole slop → nominal Ø2.5 pocket), but a drop of CA
   prevents walk-out under thermal cycling or drop shock.

**Test before any glue cures.** With all 12 magnets placed but still
removable, lower the overlay onto the tray dry (no gaskets, no plate).
All six pairs should click together. If any pair repels, pop that
magnet out and flip it — catching this before the CA sets is much
cheaper than after.
