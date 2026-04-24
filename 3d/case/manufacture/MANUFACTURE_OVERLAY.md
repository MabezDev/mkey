# mKey Overlay — Manufacture Notes

SLA standard resin (ABS-like or tough resin preferred for warp resistance).

**X-axis scale compensation MANDATORY on this piece.** 363 mm length; uncompensated drift prevents assembly. Confirm in pre-build screenshots.

## Orientation

Tilt the overlay **25 deg about the short (Y) axis**, back edge raised, front edge lowest. **Cosmetic face (key openings, display window) pointing DOWN** toward the build plate — preserves best surface finish. Apply secondary **Z-rotation ~15 deg** diagonal.

Raise the tilted overlay so the lowest point sits **5.0 mm above the raft**.

## Supports

- **Global preset:** Heavy. Upper tip 0.7 mm, contact depth 0.4 mm, lower/pillar 1.0 mm.
- **Pitch:** 2.0 mm along the two long frame rails and within 10 mm of display window frame exterior. 3.0 mm elsewhere.
- **Corner clusters:** 4 heavy supports within 5 mm of each corner.
- **Cross-bracing:** enabled.
- **Double-border raft ring:** primary at part perimeter, secondary ~3 mm inside.

### Exclusion Zones — NO supports on:

- Inside any key opening (pre-modelled tie bars bridge these)
- On the display window glass area or the 1.0 mm shelf frame
- On any decorative feature (pinstripe groove, top logo)
- Inside magnet or screw pockets on the upward-facing side

The diagonal and cross tie bars (pre-modelled, breakaway) create closed cells within the key-opening void — these are self-supporting in the tilted orientation.

A breakaway reinforcement flange around the display window is pre-modelled on the pocket side to protect the 1.0 mm picture-frame rib during peel. Remove post-cure with flush cutters and sand the pocket floor smooth.

Verify in slicer that no support pillars pass through the display window opening.

## Raft

Skate-style, 1.0 mm thick, 120% area ratio. Perforated/anti-suction if available. 10 transition layers.

## Slicer Settings

| Parameter | Value |
|---|---|
| Layer height | 50 um |
| Bottom layers | 8 at ~5x normal exposure |
| Lift distance | 8 mm TSMC (4 mm @ 30 mm/min + 4 mm @ 120 mm/min) |
| Retract speed | 150 mm/min |
| Normal lift speed | 60–90 mm/min (reduce to 45 mm/min for first 20 layers after raft transition) |
| Rest after retract | 3 s (4 s for tough/ABS-like resin) |
| Anti-aliasing | >= 2x |
| Vat temperature | 27–30 deg C |
| FEP condition | Fresh |

**First 20 layers after raft transition:** reduce lift speed to 45 mm/min. The overlay's first contact is the thin frame corners — slower lift prevents supports from pulling corners away before the frame rail fills in.

## Post-Processing

1. **IPA wash** (99%, <= 5 min) with supports still attached. Syringe-flush display pocket and magnet pockets.
2. **Air dry** 10–15 min.
3. **UV post-cure** cosmetic-face-up on flat acrylic sheet, supports still attached. 60 deg C, 30–45 min. Optional water-submerged cure for first 15–20 min.
4. **Support removal** after full cure only. Flush cutters, then sand 400 → 800 → 1200 grit on cosmetic face. Optional UV-stable acrylic clear-coat spray to blend sanded spots.
5. **Remove display window flange**: flush cutters from pocket side, sand pocket floor smooth.
6. **Remove tie bars**: flex downward to snap at breakaway necks. Sand witness marks inside key openings (hidden by keycaps).

## Submission Note

> Part is pre-supported in STL. Use provided orientation. Do not re-orient or re-support.
