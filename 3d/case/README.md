# mKey Case — 3D Printing Build Guide

## Prerequisites

- [OpenSCAD](https://openscad.org/downloads.html) (CLI must be in `$PATH`)

## Quick Start

Export print-ready STLs for 3D printing:

```bash
cd 3d/case
./build_stl.sh
```

This produces `mkey_tray.stl` and `mkey_overlay.stl` in the current directory,
oriented for optimal printing (overlay flipped cosmetic-face-down, tray
bottom-down). Add supports in your slicer (Lychee, ChiTuBox, etc.).

> [!NOTE]
> **Run `./build_stl.sh` after every parameter or geometry change.** The
> build verifies that all assertions pass and that CGAL produces a valid
> mesh at production `$fn`. A clean OpenSCAD preview at low `$fn` does not
> guarantee a clean export — some CGAL errors only surface at `$fn=128`.

## Build Options

| Flag | Description |
|------|-------------|
| `--piece tray\|overlay\|both` | Which piece(s) to export (default: `both`) |
| `--fn N` | Circle resolution override (default: `128`; file preview default is `48`) |
| `--outdir DIR` | Output directory (default: `.`) |

## Print Orientation

The build script sets `PRINT_MODE=true`, which transforms the parts for
optimal 3D printing:

- **Overlay**: The 5-degree wedge tilt is removed so the part lies perfectly flat.
  The cosmetic face (key openings / display window) is flipped **down** toward
  the build plate — this gives the best surface finish on both SLA resin and FDM.

- **Tray**: Printed bottom-down (the bottom is already flat at Z=0). Add
  supports in the slicer — place them on outer walls and bottom only, not
  inside the cavity.

## Design Parameters (case.scad)

The main toggles in `case.scad` control which features are included:

| Toggle | Default | Purpose |
|--------|---------|---------|
| `SHOW_TRAY` | `true` | Render the tray (bottom + walls) |
| `SHOW_OVERLAY` | `true` | Render the overlay (top surface) |
| `PRINT_MODE` | `false` | Re-orient pieces for 3D print export |
| `ENABLE_OVERLAY_RABBET` | `true` | Locating tongue/recess for X/Y registration |
| `ENABLE_MAGNET_POCKETS` | `true` | Blind magnet pockets for overlay hold-down |
| `ENABLE_SCREW_INSERTS` | `false` | M1.4 bolt + glued hex nut overlay retention (mutually exclusive with magnets) |
| `ENABLE_DECORATIVE_TRIMS` | `true` | Debossed logos, pinstripe, initials |

## JLC3DP SLA submission notes

> [!IMPORTANT]
> **X-AXIS SCALE COMPENSATION IS A HARD BUILD GATE.** Do not upload the
> STLs unless you have pasted the full order-note block below into the
> JLC3DP manufacturer-instructions field AND confirmed on the pre-build
> screenshot that JLC3DP applied the compensation on both pieces.
> Skipping this does not produce a looser fit — it produces parts that
> **physically cannot assemble**. The plate PCB is fabricated and frozen
> at its true size; the printed case without X-scale comp drifts up to
> ±1.09 mm over the 363 mm case length, which blows past every outer
> tolerance budget in the design (slot play, USB cheek, magnet margins).
>
> The geometry has belt-and-braces room for ±0.3 mm of residual post-
> compensation drift, not the raw ±1.09 mm uncompensated drift.

Paste the following instructions into the JLC3DP order-note / manufacturer-
instructions field when uploading `mkey_tray.stl` and `mkey_overlay.stl`.
Missing any of these will compromise fit or finish.

> Two-piece keyboard case, SLA standard resin.
>
> **MANDATORY — X-AXIS SCALE COMPENSATION ON BOTH PIECES:** Apply JLC3DP's
> per-batch X-axis scale compensation on BOTH `mkey_tray.stl` and
> `mkey_overlay.stl`. The case is 363 mm long and mates to a
> pre-fabricated FR4 PCB at tab positions spanning the full length; any
> asymmetric or uncompensated shrinkage will prevent assembly. Please
> confirm the compensation in your pre-build screenshots.
>
> **TRAY (mkey_tray.stl):**
> - Orient bottom-face DOWN on the build plate (the flat ~363 × 111 mm face).
>   Walls up, cavity open upward.
> - Factory supports on the outer side walls and outer bottom are acceptable.
>   **Do not add factory supports inside the cavity.**
> - The outer back wall carries a debossed "mKey" logo centred
>   horizontally, vertically between the bottom chamfer and the wall top
>   (~43 × 9 mm footprint). Please keep factory support contact points
>   clear of that debossed region if possible.
>
> **OVERLAY (mkey_overlay.stl):**
> - Orient cosmetic-face DOWN on the build plate (the face with the key
>   openings and display window). The underside, which carries the magnet
>   pockets (or hex nut pockets if using screw retention), faces up.
> - Place factory supports **only on the outer vertical side walls.** No
>   point supports on the downward-facing cosmetic face (key openings,
>   display window rim, pinstripe groove) and no supports inside the magnet
>   pockets, hex nut pockets, or display pocket on the up-facing underside. A raft under the
>   cosmetic face is acceptable provided the pinstripe groove and key
>   openings drain during post-processing (we IPA-flush these on our end).
> - **RAFT KEEP-OUT under the display window:** Please keep the raft
>   contact area clear of the ~32 × 38 mm region centred on the display
>   window cutout on the down-facing cosmetic face. The picture-frame
>   around the window is a 1.0 × 1.5 mm cross-grain rib — raft peel
>   forces can fracture it during support removal. An island of bare
>   build-plate under the window (with supports only on the surrounding
>   slab) is preferred.
>
> Please confirm orientations AND the X-axis scale compensation in the
> pre-build screenshots before starting the build.

**Why X-scale compensation is mandatory:** JLC3DP SLA has ±0.3% length
tolerance above 100 mm. At the 363 mm case X-axis that's ±1.09 mm worst
case, which would exceed the 0.6 mm slot tolerance on the outermost plate
tabs and shift the USB cutout off the receptacle body. With compensation,
residual drift is expected to stay inside the designed tolerances.

## Hand-finishing (post-delivery)

Order of operations when the parts arrive from JLC3DP:

1. **Inspect and photograph** factory support locations before removing
   anything — useful if a part has to be reordered.
2. **IPA-flush the resin-trap zones** with isopropanol and a soft brush
   *before* the final UV cure, if the factory hasn't already. These are
   recesses that sat against the build plate and may hold uncured resin:
    - Tray bottom underside: the `SM / 2026` owner-initials deboss.
    - Overlay cosmetic face: the pinstripe groove and (if enabled) the
      `DECO_LOGO_TOP` deboss.
3. **Snap off factory supports** from the tray bottom raft, tray outer
   walls, and overlay outer vertical walls. Expect witness nubbins —
   sand flat.
4. **Flush-sand the tray bottom** (raft side) with 120 → 240 → 400 grit,
   checking `SM / 2026` deboss legibility under raking light. Stop when
   legible; the 1.0 mm deboss depth tolerates ~0.3 mm of surface removal
   before legibility degrades.
5. **Light-sand the overlay outer vertical walls** (400 grit) to clean
   support nubbins. The cosmetic face should need only 800–1200 grit
   touch-up where any micro-support witness remains.
6. **Break the window edge** on the overlay with a fine file: the 1.0 mm
   shelf_frame_x on the display window long edges is at the brittle-
   fracture boundary under direct fingernail pressure during cleaning.
   A 0.2–0.3 mm edge break at all four window corners and edges
   multiplies its real-world durability.

## Fabrication Methods

**SLA resin (recommended)** — JLC3DP or equivalent. All tolerances and
minimum feature sizes are tuned for SLA's +/-0.2 mm accuracy. Add
supports in your slicer.

**MJF/SLS nylon** — NOT recommended. Loses the 1.0 mm shelf_frame_x display
window detail.

**FDM** — NOT suitable. Layer lines destroy the display cut tolerance and the
1.0 mm shelf frame sits below FDM's 1.6 mm wall minimum.

**Hand-cut hardwood** — Disable print-specific features:
```bash
# In case.scad or via -D flags:
ENABLE_DECORATIVE_TRIMS = false;
PRINT_MODE = false;
```

## Assembly

### Gasket install order

The plate is gasket-mounted: each of the 9 tabs is sandwiched between two
gasket strips inside its wall slot. Order matters — the plate cannot be
seated if the top gaskets are placed first.

1. **Bottom gaskets first.** Press a gasket strip into the bottom of each
   of the 9 slots before the plate goes in. These rest at the bottom of
   each slot, below where the tab will sit.
2. **Drop the plate in, front-first.** With 9 tabs engaging 9 slots
   simultaneously, a pure vertical descent is harder than a two-stage
   descent. Engage the 4 front tabs first (lower end of the tilted case),
   then **lower the plate straight down** — the 3 back tabs and 2 side
   tabs drop into their slots simultaneously. Do NOT pivot about the
   front tabs: a rotational descent swings the side tabs through ~3 mm
   of Z and can bind them against the wall top, which has only 1.5 mm
   of open-top margin. The 0.6 mm per-side slot tolerance accommodates
   the straight-down motion.
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
4. **Glue each magnet FLUSH with the pocket mouth (critical).** The
   pockets are 2.0 mm deep for 1.0 mm magnets — if a magnet sinks to
   the pocket floor, the 1 mm recess on each side creates a 2 mm air
   gap that reduces holding force by ~90%. Place a flat reference plate
   (glass, steel straightedge) over the pocket mouth during CA cure so
   the magnet cures at the surface, not the bottom.
5. **Retain with thin CA glue (mandatory, not optional).** The 1.25 mm
   nominal cheek between pocket and outer wall (1.05 mm worst-case after
   SLA shrink) is too thin to rely on press-fit alone — any walk-out
   during thermal cycling or drop shock can chip the cheek. Use gel-type
   CA so it doesn't wick into the pocket and onto the seam, and press a
   strip of painter's tape across the pocket mouths during cure to prevent
   lift.

**Test before any glue cures.** With all 12 magnets placed but still
removable, lower the overlay onto the tray dry (no gaskets, no plate).
All six pairs should click together. If any pair repels, pop that
magnet out and flip it — catching this before the CA sets is much
cheaper than after.

### Screw + nut retention (alternative to magnets)

When `ENABLE_SCREW_INSERTS=true` (and `ENABLE_MAGNET_POCKETS=false`), the
overlay is secured by four M1.4 bolts threading up from inside the tray
into hex nuts glued into the overlay. Nothing is visible externally — bolt
heads sit in pockets on the tray underside (hidden against the desk),
and the overlay top surface is flush.

#### Hardware (4 of each)

| Qty | Part | Specification | Key dimensions |
|-----|------|---------------|----------------|
| 4 | Hex nut | M1.4 DIN 934 | 3.0 mm across-flats, 1.2 mm thick |
| 4 | Socket head cap screw | M1.4 × 14 mm | 2.6 mm head dia, 1.4 mm head height |

Same bolt length at all 4 positions. The bolt threads up from the tray
underside through the wall column and into the overlay nut. The through-
hole is a 4-segment stepped bore — no post-print drilling required:
- **Pocket** (3.4 mm dia, from tray bottom upward): bolt head sits here
- **Mid** (2.0 mm dia, intermediate): head catches on pocket→mid step
- **Lower clearance** (2.5 mm dia, 5.2 mm): wider pass-through
- **Upper clearance** (2.0 mm dia, 2.0 mm from wall top): shaft alignment

**Bolt length note:** M1.4 × 14 mm. The overlay nut pocket is 1.4 mm
deep — the 1.2 mm nut sits flush with the overlay underside (0.2 mm
tolerance headroom). 14 mm is the only standard M1.4 SHCS length that
simultaneously satisfies the mid-bore and back-pocket 3× depth rules;
12 mm fails the back pocket and 16 mm fails the mid bore.

Each segment stays within JLC3DP SLA depth limits (max 3× diameter) even
under worst-case ±0.3 mm hole-tolerance shrink.
At the back corners, the 10 mm chamfer removes the tray bottom; the
pocket starts from the chamfer face (deboss).

#### Nut installation

1. **Dry-fit first.** Drop each nut into its hex pocket on the overlay
   underside and confirm it sits flush with the surface. The pocket is
   only 0.2 mm deeper than the nut, so the nut should sit essentially
   flush by geometry. The hex shape should prevent rotation by hand.
   If a nut is tight, ream the pocket lightly with a needle file — do
   not force it.
2. **Glue with gel-type CA.** Apply a thin ring of gel CA around the
   pocket wall, press the nut in, and hold for 10 seconds. Wipe any
   squeeze-out immediately — excess CA on the overlay seating face will
   prevent the overlay from sitting flat on the tray.
3. **Do not use heat-set inserts.** SLA resin warps under soldering-iron
   temperatures. Glued hex nuts are the correct fastener for this process.

#### Final assembly

1. Complete the gasket + plate install (see above).
2. Seat the overlay on the tray wall tops (nuts face down, aligning
   with the clearance holes in the tray walls).
3. Flip the assembled case upside-down.
4. Insert the M1.4 × 14 mm bolts into all 4 pocket holes from below
   and tighten into the overlay nuts with a 1.25 mm hex key. Finger-
   tight plus a quarter turn is sufficient. **Hand-torque only — do
   NOT use a torque driver or electric screwdriver.** The bolt head
   bears on a 0.3 mm radial annulus between the Ø3.4 mm pocket and
   Ø2.0 mm mid segments (~2.2 mm² contact area); over-tightening will
   crush that step into the SLA resin and wander the pocket, which is
   not serviceable. If an overlay feels loose after the quarter turn,
   STOP and inspect — you will not fix it by tightening harder.
5. Flip the case back upright. The bolt heads sit in the pocket
   recesses on the tray underside, hidden against the desk.
