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
  `--supports`, four internal rib walls are added inside the cavity, running
  front-to-back. These grow simultaneously with the tray walls during the
  SLA build, creating T-sections that resist the 363 mm long walls from
  bowing under cure shrinkage. After post-cure: score each rib at the
  front/back wall necks with a flush-cut saw, snap out, and lightly sand
  the witness marks on the inner wall faces.

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
