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
  `--supports`, six internal rib walls are added inside the cavity, running
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
| `ENABLE_SCREW_INSERTS` | `false` | M1.4 bolt + glued hex nut overlay retention (mutually exclusive with magnets) |
| `ENABLE_DECORATIVE_TRIMS` | `true` | Debossed logos, pinstripe, initials |

## JLC3DP SLA submission notes

Paste the following instructions into the JLC3DP order-note / manufacturer-
instructions field when uploading `mkey_tray.stl` and `mkey_overlay.stl`.
Missing any of these will compromise fit or finish.

> Two-piece keyboard case, SLA standard resin.
>
> **TRAY (mkey_tray.stl):**
> - Orient bottom-face DOWN on the build plate (the flat ~363 × 111 mm face).
>   Walls up, cavity open upward.
> - Internal anti-warp ribs are already modelled inside the cavity — **do
>   not add factory supports inside the cavity.** Factory supports on the
>   outer side walls and outer bottom are acceptable.
> - The outer left and right side walls each carry a debossed "mKey" logo
>   centred at mid-height, mid-length (~30 × 7 mm footprint). Please keep
>   factory support contact points clear of that debossed region if possible.
> - **Apply X-axis scale compensation on this piece** — it is a 363 mm long
>   part with critical tab-alignment slot features at both X extremes.
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
> - **Apply X-axis scale compensation on this piece as well** — it must
>   register into the tray's locating rabbet at both X extremes; asymmetric
>   scaling would prevent the two pieces mating.
>
> Please confirm orientations in the prep screenshots before starting the
> build.

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

### Anti-warp rib breakaway (tray)

The tray ships with 6 internal ribs, each cut with scored necks at the
front wall, back wall, and floor. Expected removal sequence:

1. Enter the tray cavity from the open top with a #11 craft blade.
2. For each rib, score the X-side notches at the front-wall neck
   (Y ≈ 4.8–6.8 mm) and back-wall neck (Y ≈ 104–106 mm) if the factory
   hasn't left them pre-thinned — usually the necks print thin enough to
   snap directly.
3. Work a flush-cutter into each wall-neck perforation gap (2 mm Z-height)
   and snip the thin webs (3 per wall). Do front wall first, then back.
4. With the rib free at both walls, twist it around the floor axis — the
   6 × 13.5 × 1 mm floor webs shear one at a time as the 4 mm punched
   perforations propagate the crack.
5. Lift the rib out, flush-sand the 30 total floor stubs with a sanding
   stick parallel to the wall (not perpendicular — the stick will catch
   adjacent stubs otherwise).

**Never saw down from above.** The 0.5 mm gap between rib top and wall top
is too tight for a flush-cut blade and risks nicking the overlay seat.

### Ship with ribs attached

The tray's long-axis stiffness without ribs is marginal for shipping
impact (the 363 mm × 23 mm cavity U-section racks easily under drop
loads). Snap the ribs out only at assembly time, not before transport.

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
2. **Drop the plate in, front-first.** With 9 tabs engaging 9 slots
   simultaneously, a pure vertical descent is harder than a two-stage
   descent. Engage the 4 front tabs first (lower end of the tilted case),
   then pivot the plate back and down so the 3 back tabs and 2 side tabs
   drop into their slots together. The 0.6 mm per-side slot tolerance
   and 1.5 mm open-top margin accommodate this motion.
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
4. **Retain with thin CA glue (mandatory, not optional).** The 1.15 mm
   nominal cheek between pocket and outer wall (0.95 mm worst-case after
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
| 4 | Socket head cap screw | M1.4 × 12 mm | 2.6 mm head dia, 1.4 mm head height |

Same bolt length at all 4 positions. The bolt threads up from the tray
underside through the wall column and into the overlay nut. The through-
hole is a 3-segment stepped bore — no post-print drilling required:
- **Pocket** (3.0 mm dia, from tray bottom upward): bolt head sits here
- **Mid** (2.5 mm dia, intermediate): extends the printable depth
- **Clearance** (2.0 mm dia, from wall top downward): shaft exits here

Each segment stays within JLC3DP SLA depth limits (max 3× diameter).
At the back corners, the 10 mm chamfer removes the tray bottom; the
pocket starts from the chamfer face (deboss).

#### Nut installation

1. **Dry-fit first.** Drop each nut into its hex pocket on the overlay
   underside and confirm it sits flush or slightly below the surface.
   The hex shape should prevent rotation by hand. If a nut is tight,
   ream the pocket lightly with a needle file — do not force it.
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
4. Insert the M1.4 × 12 mm bolts into all 4 pocket holes from below
   and tighten into the overlay nuts with a 1.25 mm hex key. Finger-
   tight plus a quarter turn is sufficient.
5. Flip the case back upright. The bolt heads sit in the pocket
   recesses on the tray underside, hidden against the desk.
