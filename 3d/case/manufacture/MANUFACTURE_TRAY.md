# mKey Tray — Manufacture Notes

SLA standard resin (ABS-like or tough resin preferred for warp resistance).

**X-axis scale compensation MANDATORY on this piece.** 363 mm length; uncompensated drift prevents assembly. Confirm in pre-build screenshots.

## Orientation

Tilt the tray **27 deg about the short (Y) axis**, back wall (USB side) raised, front edge lowest. Apply a secondary **Z-rotation of ~15 deg** so the long axis runs diagonally across the build plate, minimizing cross-section area per layer.

The cavity opening faces upward and away from the FEP — supports attach to the outer bottom surface and outer wall surfaces only.

Raise the tilted part so the lowest point sits **5.0 mm above the raft** surface.

## Supports

- **Global preset:** Heavy. Upper tip 0.7 mm, contact depth 0.4 mm, lower/pillar 1.0 mm.
- **Pitch:** 2.0 mm within 10 mm of any edge/corner/thin wall; 3.0 mm elsewhere.
- **Corner clusters:** 4 heavy supports within 5 mm of each corner.
- **Underside ribs** (pre-modelled, permanent): support along rib crests at 3.0 mm pitch.
- **Cross-bracing:** enabled.
- **Double-border raft ring:** primary at part perimeter, secondary ~3 mm inside.
- **No supports inside cavity.** Anti-warp ribs and transverse rib are pre-modelled.
- Run island detection; manually add heavy-tip supports to any flagged unsupported islands.

## Raft

Skate-style, 1.0 mm thick, 120% area ratio. Perforated/anti-suction if available. 10 transition layers.

## Slicer Settings

| Parameter | Value |
|---|---|
| Layer height | 50 um |
| Bottom layers | 8 at ~5x normal exposure |
| Lift distance | 8 mm TSMC (4 mm @ 30 mm/min + 4 mm @ 120 mm/min) |
| Retract speed | 150 mm/min |
| Normal lift speed | 60–90 mm/min |
| Rest after retract | 3 s (4 s for tough/ABS-like resin) |
| Anti-aliasing | >= 2x |
| Vat temperature | 27–30 deg C |
| FEP condition | Fresh |

## Drain Holes

6 drain holes (3.0 mm dia) are pre-modelled through the tray floor at the four corners and two midpoints. Flush with IPA syringe during wash to clear uncured resin from the cavity.

## Post-Processing

1. **IPA wash** (99%, <= 5 min) with supports still attached. Syringe-flush drain holes.
2. **Air dry** 10–15 min, drain holes facing down.
3. **UV post-cure** cosmetic-face-up on flat acrylic sheet, supports still attached. 60 deg C, 30–45 min. Optional water-submerged cure for first 15–20 min. Place flat weight across tray wall tops during cure.
4. **Support removal** after full cure only. Flush cutters, then sand 400 → 800 grit.
5. **Remove internal ribs**: score breakaway necks with flush cutters, snap, sand witness marks.

## Submission Note

> Part is pre-supported in STL. Use provided orientation. Do not re-orient or re-support.
