# mkey scripts

Design-review and verification tooling for the mkey keyboard.

## `verify_measurements.py`

Cross-checks every dimensional constant in `3d/case.scad` against the
authoritative source-of-truth files:

- `hardware/plate/plate.kicad_pcb` (plate outline, tab positions, display cutout)
- `hardware/board/mkey.kicad_pcb` (switch positions, USB-C, ESP32 footprint)
- `3d/plate.step` (plate thickness, exported from KiCad)

Parses all of them in pure stdlib Python — no CadQuery / FreeCAD / OCP
dependency. Run from anywhere:

```sh
python3 scripts/verify_measurements.py
python3 scripts/verify_measurements.py --verbose
```

Reports a PASS/WARN/CRITICAL table per check. Any non-PASS row is a real
discrepancy between the SCAD claim and the physical source file that needs
investigation.

Also audits that all 66 MX switches on the board fall inside the SCAD
`key_rects[]` union (with `key_cap_clearance` expanded) — any switch outside
the union would mean the case has no opening for that key.

## Load-bearing project facts (please read before proposing changes)

These are intentionally documented here so future design reviews don't
re-surface them:

1. **Display seats in the overlay housing, NOT in the plate.** The WEA2012
   AMOLED module is held in `display_cutout` (overlay through-cut) backed
   by the separately-fabricated `display_retainer` piece. Only the 24-pin
   FPC ribbon passes through the plate's display cutout to reach J2 on the
   PCB below. The nominal 0.020 mm Y-undersize of the plate cutout vs the
   module datasheet (31.5×37.200 plate vs 31.5×37.220 module) is IRRELEVANT
   — not a fit issue. Do not flag it.

2. **The plate and main PCB are already physically fabricated.** Any fix
   must live in the case (`3d/case.scad`) or in a hand rework of the
   physical parts. The `.kicad_pcb` files are locked references for
   consistency checking only; don't edit them to "fix" case review findings.

3. **Hand fabrication only — saw + drill + file.** No CNC, no router, no
   precision pocketing. Blind pockets with controlled depth, rounded
   interior corners inside recesses, and sub-millimeter non-through
   features are all off-limits as design suggestions.

See `3d/case.scad` Section 2 for the display arrangement details and the
structural/tolerance analysis that led to the current design.
