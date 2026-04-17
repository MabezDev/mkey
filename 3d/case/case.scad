// =============================================================================
// mKey Keyboard Case - OpenSCAD Design
// =============================================================================
// Premium darkwood case for the mKey 65% mechanical keyboard.
// Design: wedge profile, gasket-mounted plate, integrated display window.
// Style: sleek defined lines, minimal bezel, premium feel.
//
// FABRICATION TARGET — 3D PRINT PATH (JLC3DP SLA resin):
//   When fabricated by 3D print, the intended process is JLC3DP SLA resin
//   (standard photopolymer, e.g. 8001 / white resin). MJF/SLS nylon is NOT
//   suitable — it loses the 1.0 mm shelf_frame_x window detail. FDM is a
//   hard NO — layer lines destroy disp_cut_tol and the 1.0 mm shelf frame
//   sits below FDM's 1.6 mm wall minimum. Parameters in this file are tuned
//   to JLC3DP SLA's ±0.2 mm <100 mm / ±0.3% >100 mm tolerance band and
//   0.2 mm assembly-clearance rule; the 2026-04-14 pre-fab review
//   re-tightened disp_cut_tol and grew the locating tongue to clear those
//   rules with margin. Orient each piece with the cosmetic face (display
//   window / key opening) DOWN toward the build plate — face-down gives
//   the smoothest surface finish on both SLA resin and FDM.
//
// Coordinate system (case frame):
//   X = left to right (0 = left case edge)
//   Y = front to back (0 = front edge, positive toward back / USB side)
//   Z = up (0 = desk surface when case is sitting flat)
//
// Physical orientation (user sitting at keyboard):
//   Front (Y=0): spacebar row, lower edge (close to user)
//   Back (Y=max): number row, USB port, higher edge (away from user)
//   Right (X=max): display area, arrow keys
//   Left (X=0): Esc key side
// =============================================================================

$fn = 48;

// ─── Rendering switches ──────────────────────────────────────────────────────
SHOW_TRAY       = true;    // piece 1: bottom + walls
SHOW_OVERLAY    = true;    // piece 2: top surface
SHOW_RETAINER   = false;   // piece 3: display retainer (fabricate separately)
SHOW_PLATE      = false;   // ghost plate for fit check
SHOW_DISPLAY    = false;   // ghost display for fit check
SHOW_SECTION    = false;   // cross-section cut for inspection
EXPLODE         = 2;       // set >0 to separate overlay from tray (mm)

// ─── Print-mode switches ────────────────────────────────────────────────────
// When PRINT_MODE is true, the assembly renders print-oriented pieces:
//   - Overlay is flattened (5° tilt removed) and flipped cosmetic-face-down
//   - Tray is unchanged in orientation (bottom is already flat)
// When exporting STLs for manufacturing, set PRINT_MODE=true and render one
// piece at a time (SHOW_TRAY xor SHOW_OVERLAY).
PRINT_MODE      = false;   // flip to true (or pass -D) for STL export
PRINT_SUPPORTS  = false;   // add internal anti-warp rib walls to the tray cavity
                           // (grow with the walls during SLA build, resist bowing)

// ─── Feature toggles (per-fabrication-method overrides) ─────────────────────
// DESIGN NOTE — why these are toggles:
//   The case may be fabricated by two very different methods depending on the
//   build: (a) hand-cut hardwood using saw/drill/file/chisel, or (b) 3D-printed
//   plastic (FDM/SLA). Some features are trivial in one method and painful in
//   the other. A rabbet on the tray wall top to locate the overlay is ~5
//   minutes on a 3D-printer but a fiddly chisel operation in hardwood; blind
//   magnet pockets are laughably easy on a printer but need a drill-press stop
//   in wood. Rather than forking the file, every optional locating/retention
//   feature is behind a toggle so you can flip them per build without editing
//   the geometry. This is the real power of the file: one source, multiple
//   fabrication paths, with the asserts still covering all configurations.
ENABLE_OVERLAY_RABBET = true;    // cut a shallow rabbet in the tray wall top
                                 // that the overlay overhang seats into for
                                 // X/Y registration. Trivial on a printer,
                                 // chisel work in hardwood.
ENABLE_MAGNET_POCKETS = true;    // cut blind magnet pockets in the tray wall
                                 // top AND matching pockets in the overlay
                                 // underside, so the two pieces hold together
                                 // magnetically (serviceable — overlay lifts
                                 // off for PCB access). Counts on N52 disc
                                 // magnets — see magnet_* params below.

// ─── Decorative trims (3D-print-only) ───────────────────────────────────────
// Cosmetic embellishments with no mechanical role. Only practical on printed
// builds — hand-fabrication in hardwood can't render fine text/grooves cleanly.
// Every cut is sized to satisfy JLC3DP's design rules (≥0.8 mm wide AND
// ≥0.8 mm deep for any engraved or embossed detail, on both SLA resin and
// MJF nylon). ENABLE_DECORATIVE_TRIMS is the master gate: flipping it false
// silently drops every decorative cut, so hardwood builds or "naked" prints
// use the exact same file with one toggle.
ENABLE_DECORATIVE_TRIMS = true;   // master switch — false = all decoratives off
DECO_SIDE_LOGO          = true;   // mKey logo debossed on both outer side walls
DECO_EDGE_PINSTRIPE     = true;   // hairline groove around the overlay top edge
DECO_OWNER_INITIALS     = true;   // initials + year debossed into the tray underside
DECO_LOGO_TOP           = false;   // mKey logo debossed on overlay top above the display
DECO_INITIALS           = "SM";   // user-customisable owner initials
DECO_YEAR               = "2026"; // build year stamped under the initials

// =============================================================================
// SECTION 1: PLATE DIMENSIONS (from plate.kicad_pcb)
// =============================================================================
// The plate is the primary mounting reference. All case geometry derives from it.
//
// Plate KiCad coords:
//   Inner boundary: X 0.588..353.226, Y -107.641..-7.416
//   KiCad Y axis: negative, more negative = further up on screen = BACK of keyboard
//
// Plate-local coords (used in this file):
//   X_local = KiCad_X - 0.588       (0 = left plate edge)
//   Y_local = |KiCad_Y| - 7.416     (0 = front plate edge, plate_d = back)

plate_w = 352.638;   // inner boundary width  (353.226 - 0.588)
plate_d = 100.225;   // inner boundary depth  (107.641 - 7.416)
plate_t = 1.6;       // FR4 thickness

// ─── Mounting tabs ───────────────────────────────────────────────────────────
// Tabs extend beyond the inner boundary for gasket mounting.
// All positions in plate-local coordinates.
tab_len = 19.80;      // each tab is 19.80mm along its edge (measured in plate.kicad_pcb)

// Back edge tabs (3) - at plate_local Y = plate_d (back of keyboard)
// KiCad Y = -107.641 side; tabs extend toward more negative Y (further back)
back_tab_cx = [   // center X positions
    92.890 - 0.588,    // = 92.302
    176.130 - 0.588,   // = 175.542
    264.380 - 0.588    // = 263.792
];
back_tab_ext = 1.749;  // perpendicular extension beyond plate edge (from corner y=-107.6406)
                       // Note: KiCad back edge tapers 0.031mm between corner and interior.
                       // plate_d uses the corner (more conservative); keep this consistent.

// Front edge tabs (4) - at plate_local Y = 0 (front of keyboard)
// KiCad Y = -7.416 side; tabs extend toward less negative Y (further front)
front_tab_cx = [
    40.770 - 0.588,    // = 40.182
    138.060 - 0.588,   // = 137.472
    230.110 - 0.588,   // = 229.522
    315.100 - 0.588    // = 314.512
];
front_tab_ext = 1.776;

// Left edge tab (1) - at plate_local X = 0
// KiCad Y center = -60.170 → plate_local Y = 60.170 - 7.416 = 52.754
left_tab_cy  = 52.754;
left_tab_ext = 1.828;

// Right edge tab (1) - at plate_local X = plate_w
// KiCad Y center = -59.520 → plate_local Y = 59.520 - 7.416 = 52.104
right_tab_cy  = 52.104;
right_tab_ext = 1.764;

// =============================================================================
// SECTION 2: DISPLAY DIMENSIONS (from display-dimensions.png)
// =============================================================================
// WEA2012 / 1.72" 356x400 QSPI AMOLED module
//
// Dimensions from two independent web suppliers:
//   - lcdtftdisplays.com (1.72" 356x400 QSPI module spec)
//   - toppoplcd.com (TT172LMN10A datasheet)
//
// DISPLAY MOUNTING ARRANGEMENT — IMPORTANT (non-obvious, load-bearing):
//   The display module seats in the OVERLAY HOUSING (display_cutout +
//   display_retainer), NOT in the plate. It is inserted from above and
//   held against the retainer lip from below. The plate cutout exists
//   ONLY to pass the 24-pin FPC ribbon down to J2 on the PCB beneath.
//   The module body never passes through the plate cutout.
//
//   Consequence: the plate cutout dimensions (31.500 × 37.200) only need
//   to clear the ribbon, not the 31.5×37.22 module body. The nominal
//   0.020 mm Y-undersize vs the module datasheet is IRRELEVANT and must
//   not be surfaced as an issue in future reviews.

disp_module_w   = 31.500;   // module outline width
disp_module_h   = 37.220;   // module outline height
disp_active_w   = 29.100;   // viewable/active area width
disp_active_h   = 32.700;   // viewable/active area height
disp_active_ox  = 1.200;    // active area offset from module left edge (centered)
disp_active_oy  = 2.260;    // active area offset from module top edge (centered)
disp_glass_t    = 1.560;    // module + glass stack thickness
disp_module_r   = 2.200;    // module corner radius (matches plate cutout)
disp_active_r   = 1.000;    // active area corner radius (typical for small AMOLED)

// Display cutout in the plate (as-built, from plate.kicad_pcb)
// KiCad bbox: X 305.846..337.346, Y -95.946..-58.746 (corner radius 2.200mm)
// Plate-local:
disp_cut_x1 = 305.846 - 0.588;   // 305.258  (left edge)
disp_cut_x2 = 337.346 - 0.588;   // 336.758  (right edge)
disp_cut_y1 = 58.746 - 7.416;    // 51.330   (front/closer to user)
disp_cut_y2 = 95.946 - 7.416;    // 88.530   (back/further from user)

// Display center in plate-local coords
disp_cx = (disp_cut_x1 + disp_cut_x2) / 2;   // ~321.008
disp_cy = (disp_cut_y1 + disp_cut_y2) / 2;    // ~69.930

// =============================================================================
// SECTION 2b: ARROW KEY POSITIONS (from mkey.kicad_pcb)
// =============================================================================
// Board→plate mapping: plate_local_x = Board_X - 31.473
//                      plate_local_y = |Board_Y - 150.195| - 7.416
//
// Arrow keys form inverted-T, all at standard 19.05mm MX spacing.
// Display X center (321.0) already aligns with arrow cluster X center (321.6)
// within 0.6mm — well within tolerance.

arrow_up_px    = 353.060 - 31.473;   // = 321.587  (plate-local X)
arrow_up_py    = 150.195 - 111.729 - 7.416;  // = 31.050  (plate-local Y)
arrow_down_px  = 353.060 - 31.473;   // = 321.587
arrow_down_py  = 150.195 - 130.779 - 7.416;  // = 12.000
arrow_left_px  = 334.010 - 31.473;   // = 302.537
arrow_left_py  = 150.195 - 130.779 - 7.416;  // = 12.000
arrow_right_px = 372.110 - 31.473;   // = 340.637
arrow_right_py = 150.195 - 130.779 - 7.416;  // = 12.000

// =============================================================================
// SECTION 3: USB-C CONNECTOR
// =============================================================================
// HRO TYPE-C-31-M-12, on the back edge of the board
// Board position: (364.275, 47.610) at 180° rotation
// Opens toward back board edge (Y=44.740)
// Body: 8.940mm wide x 7.300mm deep, overhangs board edge by 0.780mm
//
// VERIFIED coordinate mapping (from safety officer, 25+ switch position checks):
//   Board_X = Plate_KiCad_X + 30.885
//   Board_Y = Plate_KiCad_Y + 150.195
// Therefore:
//   plate_local_x = Board_X - 30.885 - 0.588 = Board_X - 31.473
//   plate_local_y = |Board_Y - 150.195| - 7.416

usb_plate_x   = 364.275 - 31.473;  // = 332.802 (CORRECTED, was 329.97)
usb_opening_w = 8.94;               // receptacle body width
usb_opening_h = 3.26;               // receptacle opening height

// =============================================================================
// SECTION 4: CASE DESIGN PARAMETERS
// =============================================================================

// ─── Tilt ────────────────────────────────────────────────────────────────────
tilt_angle = 5;   // degrees, back raised

// ─── Tolerances ──────────────────────────────────────────────────────────────
plate_gap     = 0.5;    // clearance between plate edge and inner wall (per side)
disp_cut_tol  = 0.30;   // module-body pocket oversize vs module (per side).
                        // The module is inserted from BELOW into a blind
                        // pocket; it doesn't thread through anything, so the
                        // old 0.30 corner-binding bump is no longer needed.
                        // Raised from 0.15 → 0.20 → 0.30 across JLC3DP
                        // pre-fab reviews: JLC3DP SLA dimensional tolerance
                        // is ±0.2 mm on features <100 mm, and their min
                        // assembly-clearance rule is 0.2 mm. 0.20 per side
                        // left worst-case at 0.0 mm clearance (pocket shrinks
                        // to exact module size), violating the 0.2 mm rule.
                        // 0.30 per side gives 0.1 mm worst-case clearance
                        // and 0.30 mm nominal slip fit.
shelf_t        = 1.5;   // wood above the display glass (the visible recess).
                        // Bumped from 1.0 → 1.5 in the 2026-04-13 pre-fab
                        // design review: the 1.0 × 1.6 mm cross-grain short
                        // beams of the picture-frame had a bending SF < 1
                        // against a 1 N poke, and a 1 × 1.6 × 30 mm cross-
                        // grain matchstick is a real chip-out risk with
                        // saw/drill/file. Bending stiffness scales as t³,
                        // so 1.0 → 1.5 is a 3.4× stiffness win. The cost
                        // is a deeper visible recess (1.5 mm vs 1.0 mm) —
                        // still well inside the disp_pocket_d ≥ disp_glass_t
                        // invariant (5.0 − 1.5 = 3.5 mm ≥ 1.56 mm).
shelf_frame_x  = 1.0;   // picture-frame wood width along the short (X) sides
                        // of the window. X bezel is the tight constraint at
                        // 1.20 mm, so this is as thick as the frame can safely
                        // get in X without eating the ±0.2 mm fab-slop margin.
shelf_frame_y  = 1.6;   // picture-frame wood width along the long (Y) sides.
                        // Y bezel is 2.26 mm, so we have much more room here —
                        // the long straight beams at x = left/right of the
                        // window are the weakest cross-sections in the shelf,
                        // and bumping them from 0.8 → 1.6 mm doubles the
                        // material width on the edges that do the most work.
shelf_corner_r = 1.2;   // radius of the curved inner corners of the window.
                        // Blends the 1.0 mm X shelf smoothly into the 1.6 mm Y
                        // shelf — no sharp notches at the corners where the
                        // two widths meet. Must be ≥ disp_active_r (1.0 mm)
                        // so the curve clears the display's own rounded
                        // corners, and small enough that the window corner
                        // arc stays inside the pocket arc (verified by
                        // assertion below).
retainer_t    = 1.2;    // separately-fabricated backing plate that clamps the
                        // module UP against the shelf from below (optional;
                        // adhesive can replace it).

// ─── Walls ───────────────────────────────────────────────────────────────────
// The tray and overlay can have different outer wall thicknesses. The overlay
// (and all inner geometry: cavity, plate, magnets, tongue) uses overlay_wall_t.
// The tray outer shell uses tray_wall_t. When tray_wall_t > overlay_wall_t, the
// tray extends outward while inner geometry stays fixed — this increases the
// magnet pocket outer cheek without affecting the mating interface.
overlay_wall_t = 4.8;   // wall thickness for overlay and inner geometry
tray_wall_t    = 5.3;   // wall thickness for tray outer shell (extend outward)
tray_ext       = tray_wall_t - overlay_wall_t;  // 0.5mm outward extension
wall_t         = overlay_wall_t;  // alias for backward compat (inner geometry)
bottom_t  = 3.5;    // bottom plate thickness
top_t     = 5.0;    // top surface thickness (display cover area).
                    // Bumped from 3.0 → 5.0 to stiffen the 4.16 mm main-field ↔
                    // arrow-L/D/R rib, which is the weakest cross-section in
                    // the overlay. Section modulus scales as thickness²:
                    // (5/3)² = 2.78× bending stiffness on that rib at zero
                    // layout cost. Downside: display glass recess grows from
                    // 1.44 mm (top_t=3) to 3.44 mm (top_t=5), a deeper well.
                    // ALSO: fabricate the overlay blank with grain running
                    // along Y (front-to-back, ~110 mm direction), not X — the
                    // 4.16 mm rib and the 23 mm row3↔arrow-UP bezel both run
                    // in Y, so Y-grain makes them along-grain (~6× ⊥→∥
                    // hardwood strength jump). Combined the rib is ~17×
                    // stronger than the unfixed baseline.

// ─── Internal ────────────────────────────────────────────────────────────────
plate_recess     = 2.0;    // plate top sits this far below the TRAY WALL TOP
                           // (inside the tray, leaving room for a top gasket
                           //  between plate top and overlay underside)
switch_depth     = 5.0;    // MX switch body below plate
pin_depth        = 3.3;    // switch pins below body
pcb_t            = 1.6;    // main PCB thickness
component_h      = 3.5;    // tallest component below PCB (ESP32-S3-WROOM-1: 3.2mm)
bottom_clearance = 2.0;    // air gap below components

// ─── Gasket ──────────────────────────────────────────────────────────────────
gasket_compressed = 1.5;   // working thickness when loaded

// ─── USB cutout ──────────────────────────────────────────────────────────────
usb_cut_w = 15.0;   // wide for thick aftermarket cables
usb_cut_h = 8.0;    // tall for thick bezels

// ─── Key opening L-shape boundaries (plate-local) ───────────────────────────
// Front section: full width, covers rows 3-4 (spacebar, arrows, modifiers)
// Back section: narrower on right, stops before display area
//
// Row 2 center: plate_local_y = 50.100, Row 3 center: plate_local_y = 31.050
// Key cutouts are ~14mm tall, so:
//   Row 2 front edge ≈ 50.100 - 7.0 = 43.100
//   Row 3 back  edge ≈ 31.050 + 7.0 = 38.050
// Split between them:
y_split = 41.0;   // plate-local Y for the L-shape horizontal step

// Rightmost mechanical feature in rows 0-2: backspace stabilizer at plate X ≈ 285.9
// plate_local_x = 285.312
// With clearance:
x_split = 290.0;  // plate-local X where narrow section right edge is

// ─── Overhang ────────────────────────────────────────────────────────────────
// Reduced from 4.0 → 2.0: a 4mm × 3mm cross-grain cantilever on the left/right
// sides would chip off in hardwood. 2 mm is still visually present and safer.
overhang = 2.0;   // top overlay overhang beyond tray walls (mm)

// ─── Corner rounding ─────────────────────────────────────────────────────────
overlay_corner_r = 0.75;   // outer corner radius of overlay
tray_corner_r    = 0.75;   // outer corner radius of tray (matches overlay for visual consistency)

// ─── Overlay locating rabbet (optional, ENABLE_OVERLAY_RABBET) ──────────────
// INVERTED ("tongue") geometry: a small ridge of wood stands proud ABOVE
// the tray wall top on the inside edge, and the overlay underside has a
// matching recess that drops over the ridge. The overlay still sits flat
// on the wall's outer cheek at the existing wall_top Z; the tongue is
// added above it and the recess is cut into the overlay slab above its
// underside plane. Nothing else in the stack-up changes.
//
// Why inverted vs a cut-into-the-wall rabbet: hand-cutting a recess into
// the wall top and a matching downward lip on the overlay underside is
// fiddly — the lip is a thin cross-grain cantilever with nothing below it
// during cutting. The tongue variant lets the wall wood support itself
// during fabrication: remove the outer ring of the wall top to depth
// rabbet_h and you're left with a solid tongue standing proud, no
// unsupported cantilevers. The overlay's recess is a shallow groove
// routed / chiseled / printed into the underside (also well-supported
// during cutting).
//
// NOTCHING at slot Y: a naive continuous ring tongue would block the
// plate-tab drop-in path where the tabs descend through the slot open-top
// channels. `gasket_slot_footprints_2d()` subtracts each slot footprint
// (plus a descent-clearance margin) from the tongue and recess 2D
// footprints, leaving clean gaps at all 9 tab positions.
//
// NOTCHING at magnet positions (when ENABLE_MAGNET_POCKETS is also on):
// the blind magnet pockets are drilled from the wall top mid-wall, but the
// tongue ring also runs along the inner half of the wall top. Where the
// two overlap, a continuous tongue would be drilled through by the magnet
// bit in hand fab, shearing off the 1.3 mm cross-grain cantilever.
// `magnet_notch_footprints_2d()` applies the same subtract-footprints
// pattern as the gasket slots, leaving an extra gap in the tongue/recess
// at each of the 4 magnet positions. With both features on the tongue
// therefore has up to 9 + 4 = 13 gaps; each remaining segment is still
// fully supported by solid wall material and separated from every
// neighbouring gap by several mm of wood (asserted below).
rabbet_w = 2.2;      // ring width in the wall-depth direction (outer ring
                     // of wood/resin removed from the wall top). The
                     // tongue itself is `rabbet_w − 2·rabbet_tol` = 1.7 mm
                     // wide. Must leave a ≥1.5 mm rest cheek for the
                     // overlay outboard of the tongue (wall_t − rabbet_w
                     // = 2.6 mm here, well above the floor).
                     // Grown 1.8 → 2.0 in the 2026-04-14 JLC3DP review
                     // so the tongue met JLC3DP's 1.5 mm snap-feature
                     // recommendation exactly. Grown 2.0 → 2.2 in the
                     // 2026-04-17 review to add 0.2 mm margin over that
                     // floor — absorbs the full ±0.2 mm SLA fab slop
                     // so the printed tongue never narrows below 1.5 mm.
rabbet_h = 1.5;      // tongue height above the wall top (= recess depth
                     // into the overlay underside). Grown from 1.0 → 1.5
                     // in the 2026-04-14 JLC3DP review to meet the same
                     // 1.5 mm snap-feature recommendation. Still leaves
                     // top_t − rabbet_h = 3.5 mm of overlay slab above
                     // the recess floor (≫ the 1.0 mm structural floor).
rabbet_tol = 0.25;   // per-side clearance between the tongue and the
                     // recess. 0.25 mm gives a sliding hand-seat fit
                     // with no binding under ±0.2 mm fab slop.
notch_margin = 0.5;  // extra plan-view clearance added around each slot
                     // footprint when notching the tongue / recess, so
                     // the plate tab has a finger of sliding room when
                     // descending into the slot.

// ─── Magnet pockets (optional, ENABLE_MAGNET_POCKETS) ───────────────────────
// Blind cylindrical pockets sunk into the tray wall top AND the matching
// points on the overlay underside. N52 neodymium disc magnets are press-fit
// or epoxied into each pocket; tray and overlay magnets attract across the
// rabbet seam and hold the overlay down.
//
// Default sizing targets 2 mm Ø × 1 mm N52 discs (pull force ≈ 80 g each;
// 6 of them give ~480 g of holding force — plenty for a 5° tilted keyboard
// that only has to fight gravity + light typing vibration).
//
// Downsized from Ø3×2 → Ø2×1 in the 2026-04-17 JLC3DP pre-fab review:
// JLC3DP SLA hole-feature tolerance is ±0.3 mm (not the ±0.2 mm used for
// ordinary dimensional features). With the prior Ø3.2 mm pocket that
// tolerance meant a worst-case 2.9 mm bore — tighter than the 3.0 mm
// magnet — so the magnet literally could not drop in. Enlarging the
// pocket to clear that rule (3.5 mm nominal) would have dropped the wall
// cheek to 0.65 mm nominal / 0.45 mm worst case, below JLC3DP's 0.8 mm
// wall-thickness floor for small features. The fix is to shrink the
// magnet instead: an Ø2 mm magnet in an Ø2.5 mm pocket clears the ±0.3 mm
// hole rule with 0.2 mm slop AND leaves a 1.15 mm nominal wall cheek
// (0.95 mm worst case — comfortably above the 0.8 mm wall floor).
magnet_d      = 2.3;    // pocket diameter. 2.0 mm nominal magnet + 0.3 mm
                        // JLC3DP hole-tolerance allowance + 0.2 mm epoxy slop.
magnet_depth  = 2.0;    // pocket depth (Z). Meets JLC3DP's Ø↔depth minimum
                        // (h ≥ Ø for small holes: Ø2.0 → h ≥ 2.0 mm) and
                        // fully captures the 1 mm-thick magnet with 1 mm
                        // of epoxy/slop headroom above.
magnet_inset  = 13;     // distance from case OUTER corner to magnet center,
                        // measured along the front/back wall axis. Keeps the
                        // magnet clear of the corner rounding AND clear of the
                        // nearest gasket slot (leftmost back slot cx = 97.6 mm,
                        // rightmost = 269.1 mm; slots are ±10 mm wide — 13 mm
                        // inset leaves ≥67 mm separation on the left and gives
                        // ≥3 mm cheek to the back-wall USB cutout on the right
                        // (reduced from 15 mm → 13 mm in the 2026-04-14 review:
                        // at 15 mm the right-back magnet sat only 1.04 mm from
                        // the USB cutout edge, inside the ±0.2 mm hand-fab slop
                        // budget; 13 mm restores a 3 mm cheek).

// Middle-wall magnet pair (added 2026-04-17): a 5th and 6th magnet pair
// placed at the MIDDLE of the front and back walls to hold the long span
// of the overlay down and resist mid-wall bow. The front-wall middle sits
// at true outer_w/2, in a clean 92 mm gap between front tabs 2 and 3.
// The back-wall middle would collide with back tab 2 at X=180.84 if placed
// at true middle; `back_mid_offset` nudges it right into the clean gap
// between back tab 2 and back tab 3, preserving ≥8 mm of tongue between
// the magnet notch and the nearest slot notch (above MAG_NOTCH_SEP=1 mm).
back_mid_offset = 20;

// Magnets are placed by the `magnet_positions` function so a different
// layout can be substituted without touching any module. It is a function
// rather than a top-level list because it depends on `outer_w` / `outer_d`
// which are computed in SECTION 5 below.
function magnet_positions() = [
    [magnet_inset,                     wall_t / 2],              // front-left
    [outer_w - magnet_inset,           wall_t / 2],              // front-right
    [magnet_inset,                     outer_d - wall_t / 2],    // back-left
    [outer_w - magnet_inset,           outer_d - wall_t / 2],    // back-right
    [outer_w / 2,                      wall_t / 2],              // front-middle
    [outer_w / 2 + back_mid_offset,    outer_d - wall_t / 2],    // back-middle (offset to clear back tab 2)
];

// =============================================================================
// SECTION 5: DERIVED DIMENSIONS
// =============================================================================

// Total depth below plate surface
depth_below = switch_depth + pin_depth + pcb_t + component_h + bottom_clearance;
// = 5.0 + 3.3 + 1.6 + 3.5 + 2.0 = 15.4 mm

// Total internal height from case floor to case top.
// Stack-up, bottom to top:
//   bottom_t | depth_below | plate_t | plate_recess | top_t
// plate_recess is measured below the TRAY WALL TOP, so top_t must be added
// here to reach the overlay top surface. The old formula omitted top_t, which
// forced plate_z into the overlay slab by top_t − (old plate_recess) = 1.5mm
// and jammed the plate tabs into solid overlay material.
internal_h = top_t + plate_recess + plate_t + depth_below;

// Inner cavity dimensions (plate + clearance gap on each side)
inner_w = plate_w + 2 * plate_gap;
inner_d = plate_d + 2 * plate_gap;

// Case outer dimensions (uniform wall_t around the cavity)
outer_w = inner_w + 2 * wall_t;
outer_d = inner_d + 2 * wall_t;

// Case heights
front_h  = internal_h + bottom_t;
tilt_rise = outer_d * tan(tilt_angle);
back_h   = front_h + tilt_rise;

// Plate origin in case coords (centered inside the inner cavity)
plate_ox = wall_t + plate_gap;
plate_oy = wall_t + plate_gap;

// Bottom pocket: accepts the module body, cut from the overlay underside,
// depth = top_t − shelf_t. Module drops in from below and is held up against
// the shelf by adhesive or a clamping backing plate.
disp_pocket_w = disp_module_w + 2 * disp_cut_tol;   // 31.80
disp_pocket_h = disp_module_h + 2 * disp_cut_tol;   // 37.52
disp_pocket_r = disp_module_r + disp_cut_tol;       // 2.35
disp_pocket_d = top_t - shelf_t;                    // 3.50 with top_t=5, shelf_t=1.5

// Top window: visible from above, cut through the thin shelf. Asymmetric X/Y
// offsets from the pocket, blended at the corners by a `shelf_corner_r` fillet
// via hull-of-circles (see `display_window_2d` below) so the shelf thickness
// transitions smoothly from shelf_frame_x along the short sides to
// shelf_frame_y along the long sides with no sharp notches.
disp_win_w = disp_pocket_w - 2 * shelf_frame_x;     // 29.80
disp_win_h = disp_pocket_h - 2 * shelf_frame_y;     // 34.32

// Assertions — stricter than before so the shelf has ±0.2 mm fab-slop margin
// on the active-area clearance in both axes. The active area sits at
// `disp_cut_tol + (module − active)/2` in pocket-local coordinates, so that
// is the position the shelf inner edge must not reach even after fab slop.
FAB_SLOP = 0.2;
active_edge_x = disp_cut_tol + (disp_module_w - disp_active_w) / 2;   // 1.35
active_edge_y = disp_cut_tol + (disp_module_h - disp_active_h) / 2;   // 2.41
assert(shelf_frame_x + FAB_SLOP <= active_edge_x,
       "shelf_frame_x too large: under ±0.2 mm fab slop the shelf would bite active pixels in X");
assert(shelf_frame_y + FAB_SLOP <= active_edge_y,
       "shelf_frame_y too large: under ±0.2 mm fab slop the shelf would bite active pixels in Y");
assert(disp_pocket_d >= disp_glass_t,
       "bottom pocket too shallow for module body — reduce shelf_t or grow top_t");
assert(disp_win_w >= disp_active_w && disp_win_h >= disp_active_h,
       "top window smaller than active area — shelf_frame_{x,y} too large");
assert(shelf_corner_r >= disp_active_r,
       "shelf_corner_r must clear the display's own rounded active corner (≥ disp_active_r)");
// Hull-of-circles degenerates if the radius is larger than the window bbox
// half-extent in either axis. True today by ~12× but a future edit to
// shelf_corner_r or the window size could silently break display_window_2d().
assert(shelf_corner_r <= disp_win_w / 2 && shelf_corner_r <= disp_win_h / 2,
       "shelf_corner_r exceeds half the window bbox — display_window_2d hull degenerate");
// Window corner arc must stay inside the pocket corner arc. Derivation:
// window corner pivot at (shelf_frame_x + r, shelf_frame_y + r) relative to
// the pocket bbox corner; pocket pivot at (disp_pocket_r, disp_pocket_r);
// the max distance from the pocket pivot to any point on the window arc is
// |Δ| + shelf_corner_r, and that must be ≤ disp_pocket_r for the arc to
// stay inside the pocket's rounded-corner interior.
pocket_corner_clearance =
    sqrt(pow(disp_pocket_r - (shelf_frame_x + shelf_corner_r), 2) +
         pow(disp_pocket_r - (shelf_frame_y + shelf_corner_r), 2))
    + shelf_corner_r;
assert(pocket_corner_clearance <= disp_pocket_r,
       "window corner arc escapes the pocket corner — reduce shelf_corner_r or grow shelf_frame");

// USB-C opening Z geometry (used by both the cutout module and the
// invariant asserts below). Derived from the back-wall plate Z and the
// mid-mount HRO TYPE-C-31-M-12 receptacle, whose opening center sits on
// the PCB midplane.
back_wall_inner_y   = outer_d - wall_t;
back_plate_top_z    = plate_z(back_wall_inner_y);
back_plate_bottom_z = back_plate_top_z - plate_t;
pcb_top_z           = back_plate_bottom_z - switch_depth;
usb_z_center        = pcb_top_z - pcb_t / 2;
usb_cut_z_bot       = usb_z_center - usb_cut_h / 2;
usb_cut_z_top       = usb_z_center + usb_cut_h / 2;
back_wall_top_z     = back_h - top_t;  // tray wall top at the back

// Overlay overhang dimensions
overlay_w       = outer_w + 2 * overhang;
overlay_d       = outer_d + 2 * overhang;
overlay_front_h = front_h - tilt_rise * overhang / outer_d;
overlay_back_h  = back_h  + tilt_rise * overhang / outer_d;

// ─── Keycap footprint rectangles (plate-local) ──────────────────────────────
// The key opening is cut as the UNION of these rectangles. Each rectangle is a
// per-row or per-cluster keycap footprint derived directly from mkey.kicad_pcb
// switch positions + their unit sizes. Where same-row caps touch at unit
// boundaries, their rectangles merge into a continuous strip; where there are
// real gaps in the layout (between R-shift and arrow UP, between MX63 and
// arrow LEFT), the gap stays as solid overlay material, hiding the plate.
//
// Unit = 19.05 mm. Format: [x1, y1, x2, y2] in plate-local coords.
key_rects = [
    // Row 0 (number row, Y=88.200, 13×1u + 2u backspace)
    [  2.500, 78.675, 288.250, 97.725],
    // Row 1 (Tab row, Y=69.150, 1.5u Tab + 12×1u). Extended right to 264.437
    // to touch the ISO Enter rect — otherwise the 4.76×19.05 mm strip between
    // MX27 and the ISO Enter would be isolated on all four sides by cutouts
    // (Row 0 above, Row 2 below, Row 1 left, ISO Enter right) and CGAL would
    // carve it out as a floating block. Keys in this X range exist on every
    // other row (backspace, R-shift, MX62, MX41) so exposing the natural
    // row-1 bezel here is unavoidable.
    [  2.499, 59.625, 264.437, 78.675],
    // Row 2 (Home row, Y=50.100, 1.75u Caps + 12×1u)
    [  2.499, 40.575, 264.437, 59.625],
    // ISO Enter MX28 (1.25u × 2u, cx=276.343, cy=59.625 — bridges rows 1 & 2)
    [264.437, 40.575, 288.249, 78.675],
    // Row 3 main (Shift row, Y=31.050, 1.25u + 11×1u + 2.75u R-shift)
    [  2.500, 21.525, 288.250, 40.575],
    // Row 4 main (Front row, Y=12.000, 3×1.25u + 6.25u space + 4×1.25u)
    [  2.500,  2.475, 288.249, 21.525],
    // Arrow UP (MX55 1u, isolated from R-shift by 23.81 mm gap)
    [312.062, 21.525, 331.112, 40.575],
    // Arrow cluster L/D/R (three 1u caps, isolated from main field by 4.76 mm gap)
    [293.012,  2.475, 350.162, 21.525],
];
// Clearance added to every side of each rect for finish/file slop.
// Bumped from 0.25 → 0.3 to survive hand-cut ±0.2 mm tolerance. Note: this
// does shrink the main-field ↔ arrow-LDR rib from 4.26 to 4.16 mm effective,
// which is still the weakest cross-section in the overlay.
key_cap_clearance = 0.3;

// Corner radius for key openings. Matches shelf_corner_r for visual
// consistency with the display window. Keycaps have their own 1-2 mm
// corner radii, so 1.2 mm opening radius provides adequate clearance
// at all points of key travel. Added in 2026-04-17 JLC3DP pre-fab review.
key_corner_r = 1.2;


// =============================================================================
// SECTION 5b: GLOBAL INVARIANT ASSERTIONS (self-contained verification)
// =============================================================================
// These asserts replace the previous scripts/verify_measurements.py external
// tool. Every invariant that was (or should have been) checked by the script
// now lives inline: OpenSCAD fails the render if any of them break.
//
// Primary-source verification of the dimensioned constants against
// plate.kicad_pcb, mkey.kicad_pcb, and plate.step was performed as part of
// the 2026-04-13 pre-fabrication design review. Every KiCad/STEP-derived
// constant matched to ≤ 0.001 mm. Since the plate and PCB are physically
// fabricated and frozen (see project memory note on display mounting), those
// dimensioned constants are locked truth — no ongoing drift check is needed.
// What CAN change is the case-parameter cluster below, and the asserts here
// guard every invariant those parameters must satisfy.

// ─── Slot + wall invariants ─────────────────────────────────────────────────

// Side (left/right) slots run along the tilt axis. Their flat slot-box top
// must clear the (rising) wall top at the UPHILL end of the slot, plus a
// fab-slop budget. This is the regression guarded by commit 7ca2d31.
assert(slot_open_margin >=
       (tab_len + slot_tol) * tan(tilt_angle) / 2 + FAB_SLOP,
       "slot_open_margin too small: uphill end of side slots drops below wall top under tilt + fab slop");

// Wall cheek outboard of each slot must be thick enough not to chip off in
// hardwood under gasket compression or assembly force.
assert(wall_t - slot_depth >= 3.0,
       "slot cheek (wall_t − slot_depth) below 3.0 mm — hardwood chip-off hazard");

// Every tab must penetrate no deeper than the slot can accept. Tab
// penetration into the wall = tab_ext − plate_gap; the slot is slot_depth
// deep measured from the inner wall face, so this bounds tab_ext.
assert(max(max(back_tab_ext, front_tab_ext),
           max(left_tab_ext, right_tab_ext)) <= slot_depth + plate_gap - 0.05,
       "tab extension exceeds slot depth + plate gap — tab bottoms out on slot back wall");

// Each gasket slot must sit in the straight portion of its wall, not bleed
// into the rounded outer corner where the wall becomes a circular arc (any
// slot geometry crossing that arc would have no flat back wall to register
// against and the tab would punch out into thin air). Require ≥
// overlay_corner_r of clear wall between each slot length end and the nearest
// outer corner.
assert(len([for (cx = back_tab_cx)
            if (p2c_x(cx) - (tab_len + slot_tol) / 2 < wall_t + overlay_corner_r ||
                p2c_x(cx) + (tab_len + slot_tol) / 2 > outer_w - wall_t - overlay_corner_r)
            1]) == 0,
       "a back-wall gasket slot runs into the outer tray corner — move the tab or reduce tab_len");
assert(len([for (cx = front_tab_cx)
            if (p2c_x(cx) - (tab_len + slot_tol) / 2 < wall_t + overlay_corner_r ||
                p2c_x(cx) + (tab_len + slot_tol) / 2 > outer_w - wall_t - overlay_corner_r)
            1]) == 0,
       "a front-wall gasket slot runs into the outer tray corner — move the tab or reduce tab_len");
assert(p2c_y(left_tab_cy) - (tab_len + slot_tol) / 2 >= wall_t + overlay_corner_r &&
       p2c_y(left_tab_cy) + (tab_len + slot_tol) / 2 <= outer_d - wall_t - overlay_corner_r,
       "left-wall gasket slot runs into the outer tray corner — move the tab or reduce tab_len");
assert(p2c_y(right_tab_cy) - (tab_len + slot_tol) / 2 >= wall_t + overlay_corner_r &&
       p2c_y(right_tab_cy) + (tab_len + slot_tol) / 2 <= outer_d - wall_t - overlay_corner_r,
       "right-wall gasket slot runs into the outer tray corner — move the tab or reduce tab_len");

// ─── Case geometry invariants ───────────────────────────────────────────────

assert(overhang <= wall_t,
       "overlay overhang exceeds wall_t — unsupported cantilever");

// ─── Overlay rabbet invariants (only enforced when feature is on) ────────
// Inverted ("tongue") geometry — the positive locating feature stands proud
// on the tray wall top and drops into a matching recess in the overlay
// underside. The outer cheek of the wall (the OUTBOARD part of the wall
// top, not covered by the recess) is what the overlay actually rests on.
assert(!ENABLE_OVERLAY_RABBET || wall_t - rabbet_w >= 1.5,
       "rabbet_w leaves < 1.5 mm outer cheek on the wall top — overlay seat too weak");
// Tongue cross-section must survive the tolerance shrink AND be fabricable.
assert(!ENABLE_OVERLAY_RABBET || rabbet_w - 2 * rabbet_tol >= 0.5,
       "rabbet_w - 2*rabbet_tol < 0.5 mm — tongue cross-section below fabricable minimum");
// Tongue height must leave enough overlay slab above the recess; too short
// and the tongue can't register, too tall and the overlay slab thins below
// structural minimum at the recess floor.
assert(!ENABLE_OVERLAY_RABBET || (rabbet_h >= 0.5 && rabbet_h <= top_t - 1.0),
       "rabbet_h out of range — must be ≥0.5 mm and leave ≥1.0 mm of overlay slab above the recess");
assert(!ENABLE_OVERLAY_RABBET || rabbet_tol >= 0.15,
       "rabbet_tol below 0.15 mm — tongue will bind in the recess under ±0.2 mm fab slop");
// The tongue/recess are notched at each slot Y with a plan-view margin of
// `notch_margin` around the slot footprint. The margin must be non-negative
// AND small enough that it doesn't consume every ring segment between
// adjacent slots (otherwise the ring becomes a series of isolated dots).
// Closest-adjacent slot pair on the back wall is at cx = 92.302 and 175.542,
// gap = 83.24 mm − (tab_len+slot_tol) = 83.24 − 20.1 = 63.14 mm. notch_margin
// must be well below half that.
// 0.2 mm is the ±0.2 mm fab-slop budget used everywhere else in this file;
// at notch_margin = 0 the tongue touches the slot edge with literally zero
// finger-room for the descending tab, and would bind under any positive hand
// tolerance. 0.2 mm is the smallest value that still gives clear tab-descent.
assert(!ENABLE_OVERLAY_RABBET || (notch_margin >= 0.2 && notch_margin <= 5.0),
       "notch_margin out of range — must be ≥0.2 mm and ≤5 mm");
// The tongue recess inner edge (cavity side) sits at `wall_t − rabbet_tol`
// from the case outer face on every wall. The key opening is a union of
// rectangles expanded by `key_cap_clearance` per side. Require at least
// 2 mm of solid overlay slab between the recess inner edge and the
// nearest key rect edge on all four sides, so a future plate_gap /
// rabbet_w / key_rects edit can't walk the key cut into the recess and
// leave the tongue unsupported by a hairline of overlay material.
assert(!ENABLE_OVERLAY_RABBET || (
       p2c_x(min([for (r = key_rects) r[0]]) - key_cap_clearance)
           >= (wall_t - rabbet_tol) + 2.0 &&
       p2c_y(min([for (r = key_rects) r[1]]) - key_cap_clearance)
           >= (wall_t - rabbet_tol) + 2.0 &&
       p2c_x(max([for (r = key_rects) r[2]]) + key_cap_clearance)
           <= outer_w - (wall_t - rabbet_tol) - 2.0 &&
       p2c_y(max([for (r = key_rects) r[3]]) + key_cap_clearance)
           <= outer_d - (wall_t - rabbet_tol) - 2.0),
       "a key_rect encroaches within 2 mm of the tongue recess inner edge — overlay slab between key cut and recess too thin");
// At slot Y locations the tongue is removed, so the locating ring has 9
// gaps. This is intentional — plate tabs drop through those gaps during
// assembly. The OVERLAY material directly above each slot Y is still at
// the full overlay-slab Z range (the recess is also notched), so the slot
// open-top is properly capped by solid overlay wood after assembly.

// ─── Magnet pocket invariants (only enforced when feature is on) ────────
// The pocket must fit in the wall cross-section with a ≥0.6 mm cheek on
// both sides (marginal-but-workable in hardwood with a brad-point bit;
// trivial in print). Each magnet position must sit inside the wall ring
// (not in the cavity, not past the outer face) and must not collide with
// a gasket slot or the USB cutout.
assert(!ENABLE_MAGNET_POCKETS || (wall_t - magnet_d) / 2 >= 0.6,
       "magnet pocket leaves < 0.6 mm cheek in overlay wall — hardwood chip-off hazard");
// Tray outer cheek is larger due to tray_ext. For JLC3DP SLA, the cheek should
// be >= 1.2 mm (gray zone on thin-wall heatmap) or >= 1.5 mm (snap-feature spec).
assert(!ENABLE_MAGNET_POCKETS || (tray_wall_t - magnet_d) / 2 >= 1.2,
       "tray magnet pocket outer cheek < 1.2 mm — increase tray_wall_t");
// Magnet pockets are drilled from the wall top downward. The wall column is
// solid wood from the case floor up to the tray wall top, so the real floor
// budget is `wall_top(cy) − bottom_t`. Require the pocket to stop at least
// 2 mm above the case floor at every magnet Y. The tray cylinder is anchored
// to the UPHILL edge of the pocket footprint (cavity side, highest wall_top)
// and its length is magnet_depth + tilt_drift + 0.2, so the cylinder bottom
// sits at top_z(p[1]+r) − top_t + 0.1 − magnet_cyl_h.
assert(!ENABLE_MAGNET_POCKETS ||
       len([for (p = magnet_positions())
            if (top_z(p[1] + magnet_d/2) - top_t + 0.1 - magnet_cyl_h
                < bottom_t + 2.0) 1]) == 0,
       "magnet pocket bottom leaves less than 2 mm of wall material above the case floor");
// Every magnet position must sit entirely inside the wall ring (NOT in the
// cavity and NOT past the outer boundary), with a ≥0.3 mm margin to the
// cavity inner face and the outer face.
assert(!ENABLE_MAGNET_POCKETS ||
       len([for (p = magnet_positions())
            if (p[0] - magnet_d/2 < 0.3 ||
                p[0] + magnet_d/2 > outer_w - 0.3 ||
                p[1] - magnet_d/2 < 0.3 ||
                p[1] + magnet_d/2 > outer_d - 0.3 ||
                (p[0] + magnet_d/2 > wall_t - 0.3 &&
                 p[0] - magnet_d/2 < outer_w - wall_t + 0.3 &&
                 p[1] + magnet_d/2 > wall_t - 0.3 &&
                 p[1] - magnet_d/2 < outer_d - wall_t + 0.3)) 1]) == 0,
       "a magnet position does not sit fully inside the wall ring material");
// Magnet pockets must not collide with any gasket slot footprint in X-Y.
// Slot X/Y ranges depend on which wall — use each tab's center + length.
assert(!ENABLE_MAGNET_POCKETS ||
       len([for (p = magnet_positions())
            for (bx = back_tab_cx)
            let (sx1 = p2c_x(bx) - (tab_len + slot_tol)/2 - magnet_d/2,
                 sx2 = p2c_x(bx) + (tab_len + slot_tol)/2 + magnet_d/2)
            if (p[0] >= sx1 && p[0] <= sx2 &&
                p[1] >= outer_d - wall_t - magnet_d/2) 1]) == 0,
       "a back-wall magnet pocket overlaps a back gasket slot");
assert(!ENABLE_MAGNET_POCKETS ||
       len([for (p = magnet_positions())
            for (fx = front_tab_cx)
            let (sx1 = p2c_x(fx) - (tab_len + slot_tol)/2 - magnet_d/2,
                 sx2 = p2c_x(fx) + (tab_len + slot_tol)/2 + magnet_d/2)
            if (p[0] >= sx1 && p[0] <= sx2 &&
                p[1] <= wall_t + magnet_d/2) 1]) == 0,
       "a front-wall magnet pocket overlaps a front gasket slot");
// Magnet pockets must not collide with the USB cutout in X, and must keep a
// real ≥ 2 mm edge-to-edge cheek (not merely non-overlap) so fab slop and
// any future 0.1-mm USB nudge cannot erode the wall between them. The
// right-back magnet at magnet_inset = 13 sits ~3 mm from the USB edge
// today — the 2 mm floor locks that margin into the assertion suite.
assert(!ENABLE_MAGNET_POCKETS ||
       len([for (p = magnet_positions())
            let (cheek = abs(p[0] - p2c_x(usb_plate_x))
                         - usb_cut_w/2 - magnet_d/2)
            if (p[1] >= outer_d - wall_t - magnet_d/2 && cheek < 2.0) 1]) == 0,
       "a back-wall magnet pocket leaves < 2 mm cheek to the USB cutout");

// ─── Magnet × tongue auto-notching invariants ────────────────────────────
// Only meaningful when BOTH features are enabled. `magnet_notch_footprints_2d`
// classifies each magnet by the wall it sits next to and emits a rectangle
// that cuts the tongue ring cleanly on that wall. For the classification
// to match a wall (and therefore the notch to actually happen), each magnet
// position must satisfy one of the four adjacency predicates.
assert(!(ENABLE_MAGNET_POCKETS && ENABLE_OVERLAY_RABBET) ||
       len([for (p = magnet_positions())
            if (!(p[1] <= wall_t ||
                  p[1] >= outer_d - wall_t ||
                  p[0] <= wall_t ||
                  p[0] >= outer_w - wall_t)) 1]) == 0,
       "a magnet position is not adjacent to any wall — tongue auto-notch would miss it, leaving the tongue undermined by the pocket");

// With both features on, a magnet notch on a given wall must not merge
// with an adjacent gasket-slot notch on the same wall. If they merge the
// tongue segment between them disappears and the remaining tongue on
// either side of the merged gap is pushed farther apart, weakening
// registration. We require ≥ 1.0 mm of uncut tongue between any magnet
// notch and every slot notch on the same wall.
MAG_NOTCH_SEP = 1.0;
// Front/back walls: compare X-extents of magnet notches vs X-extents of
// slot notches. Each rectangle extends by `notch_margin` past its nominal
// footprint; use the post-notch_margin extents for the check.
assert(!(ENABLE_MAGNET_POCKETS && ENABLE_OVERLAY_RABBET) ||
       len([for (p = magnet_positions())
            if (p[1] <= wall_t || p[1] >= outer_d - wall_t)
              let (m_on_front = p[1] <= wall_t,
                   m_lo = p[0] - magnet_d/2 - notch_margin,
                   m_hi = p[0] + magnet_d/2 + notch_margin)
              for (cx = m_on_front ? front_tab_cx : back_tab_cx)
                let (s_lo = p2c_x(cx) - (tab_len + slot_tol)/2 - notch_margin,
                     s_hi = p2c_x(cx) + (tab_len + slot_tol)/2 + notch_margin,
                     gap  = (m_lo > s_hi) ? (m_lo - s_hi) :
                            (s_lo > m_hi) ? (s_lo - m_hi) : -1)
                if (gap < MAG_NOTCH_SEP) 1]) == 0,
       "a front/back-wall magnet notch is within 1 mm of a gasket slot notch on the same wall — tongue segment collapses");

// Left/right walls: same check but on Y coordinate. Today `magnet_positions`
// only places magnets front/back so this list comprehension is empty; it
// guards a future edit that moves a magnet to a side wall.
assert(!(ENABLE_MAGNET_POCKETS && ENABLE_OVERLAY_RABBET) ||
       len([for (p = magnet_positions())
            if ((p[0] <= wall_t || p[0] >= outer_w - wall_t) &&
                !(p[1] <= wall_t || p[1] >= outer_d - wall_t))
              let (m_on_left = p[0] <= wall_t,
                   m_lo = p[1] - magnet_d/2 - notch_margin,
                   m_hi = p[1] + magnet_d/2 + notch_margin,
                   s_cy = m_on_left ? left_tab_cy : right_tab_cy,
                   s_lo = p2c_y(s_cy) - (tab_len + slot_tol)/2 - notch_margin,
                   s_hi = p2c_y(s_cy) + (tab_len + slot_tol)/2 + notch_margin,
                   gap  = (m_lo > s_hi) ? (m_lo - s_hi) :
                          (s_lo > m_hi) ? (s_lo - m_hi) : -1)
              if (gap < MAG_NOTCH_SEP) 1]) == 0,
       "a left/right-wall magnet notch is within 1 mm of the side gasket slot notch — tongue segment collapses");

// Magnet notch must stay clear of the overlay's rounded outer corner. The
// recess is a square ring but the overlay outer face is rounded at
// `overlay_corner_r`, so a notch that drifts too close to a case corner
// would cut into the curved-corner region where the tongue is already
// thinning against the arc. Require the notch to sit at least
// `overlay_corner_r` away from both outer boundaries on its length axis.
// Guards a future `magnet_inset` or `notch_margin` edit.
assert(!(ENABLE_MAGNET_POCKETS && ENABLE_OVERLAY_RABBET) ||
       len([for (p = magnet_positions())
            let (on_fb = p[1] <= wall_t || p[1] >= outer_d - wall_t,
                 lo   = on_fb
                        ? p[0] - magnet_d/2 - notch_margin
                        : p[1] - magnet_d/2 - notch_margin,
                 hi   = on_fb
                        ? p[0] + magnet_d/2 + notch_margin
                        : p[1] + magnet_d/2 + notch_margin,
                 span = on_fb ? outer_w : outer_d)
            if (lo < overlay_corner_r || hi > span - overlay_corner_r) 1]) == 0,
       "a magnet notch enters the overlay's rounded outer corner region");

// The key_cap_clearance expansion must not erode the main-field ↔ arrow LDR
// rib below its structural minimum. Gap at nominal MX spacing is 4.76 mm;
// after 2×key_cap_clearance expansion the effective rib width is
// 4.76 − 2·key_cap_clearance. We require ≥ 4.0 mm with top_t=5 for an
// along-grain rib (Y grain per case.scad line 217–221).
assert(4.76 - 2 * key_cap_clearance >= 4.0,
       "key_cap_clearance too large: arrow-LDR rib would drop below 4.0 mm");

// ─── Display shelf invariants (top_t vs shelf_t vs glass) ────────────────
// This is an invariant re-statement of the derived-dimension asserts above;
// kept here so a future top_t / shelf_t edit still has to pass it even if
// those variables are moved.
assert(top_t - shelf_t >= disp_glass_t,
       "top_t − shelf_t < disp_glass_t — module body cannot fit under the shelf");

// ─── USB-C cutout Z invariants ───────────────────────────────────────────
// Margins are 0.3 mm — 1.5× the general ±0.2 mm hand-fab slop budget, so
// that slop alone cannot push the cut through the floor or the wall top.
assert(usb_cut_z_bot >= bottom_t + 0.3,
       "USB cutout pierces (or touches) the case floor within fab slop");
assert(usb_cut_z_top <= back_wall_top_z - 0.3,
       "USB cutout pierces the tray wall top at the back within fab slop");

// ─── USB-C cutout X invariants ───────────────────────────────────────────
// The cut must sit inside the back wall's X extent with enough wood on each
// side that a ±0.5 mm PCB placement slop can't run it off the cheek.
assert(p2c_x(usb_plate_x) - usb_cut_w / 2 >= wall_t + 0.5,
       "USB cutout X-range escapes the back wall's left cheek");
assert(p2c_x(usb_plate_x) + usb_cut_w / 2 <= outer_w - wall_t - 0.5,
       "USB cutout X-range escapes the back wall's right cheek");

// The USB cut and back-wall gasket slots are both cut into the back wall;
// if their X ranges ever overlap, the overlap region loses its slot back
// wall and the tab just pushes into the USB hole. Today the rightmost back
// tab ends at x≈273.8 and the USB cut starts at x≈330.6 — 56 mm of clear
// wood — but guard against a future tab/USB X edit that closes the gap.
assert(len([for (bx = back_tab_cx)
            let (sx1 = p2c_x(bx) - (tab_len + slot_tol)/2,
                 sx2 = p2c_x(bx) + (tab_len + slot_tol)/2,
                 ux1 = p2c_x(usb_plate_x) - usb_cut_w/2,
                 ux2 = p2c_x(usb_plate_x) + usb_cut_w/2)
            if (!(sx2 < ux1 || sx1 > ux2)) 1]) == 0,
       "USB cutout X-range overlaps a back-wall gasket slot — back wall cheeks merge");

// Belt-and-braces Z guard: even if a future edit brings the USB X range
// into a slot X range, the two cuts should also be forced out of each
// other's Z range or the back wall literally has no solid material
// between them. Today the slot Z-centre sits ~3 mm above the USB Z-top
// at the back wall, so X AND Z overlap is both impossible. This
// assertion locks that into the suite.
assert(len([for (bx = back_tab_cx)
            let (sx1 = p2c_x(bx) - (tab_len + slot_tol)/2,
                 sx2 = p2c_x(bx) + (tab_len + slot_tol)/2,
                 ux1 = p2c_x(usb_plate_x) - usb_cut_w/2,
                 ux2 = p2c_x(usb_plate_x) + usb_cut_w/2,
                 scz = plate_z(outer_d - wall_t) - plate_t/2 + slot_center_offset,
                 sz1 = scz - slot_height/2,
                 sz2 = scz + slot_height/2,
                 x_overlap = !(sx2 < ux1 || sx1 > ux2),
                 z_overlap = !(sz2 < usb_cut_z_bot || sz1 > usb_cut_z_top))
            if (x_overlap && z_overlap) 1]) == 0,
       "USB cutout overlaps a back-wall gasket slot in both X and Z — no solid wall remains between them");

// ─── Main-field ↔ display-pocket bezel ───────────────────────────────────
// Rightmost main-field rect edge (Row 0 / Row 3 main) sits at x=288.250. The
// display bottom pocket left edge is disp_cx − disp_pocket_w/2. The hardwood
// bezel between the main key opening and the display hole must be ≥ 4.0 mm
// (along-grain Y beam) for the rib to survive typing vibration.
assert((disp_cx - disp_pocket_w / 2) - (288.250 + key_cap_clearance) >= 4.0,
       "main-field ↔ display-pocket bezel < 4.0 mm — rib too thin for along-grain Y beam");

// ─── Active area centered in module ──────────────────────────────────────
// The display cutout is centered on the module bbox, and the top window is
// centered on the pocket. For the visible active pixels to sit centered in
// the window, the module's active-area offset must equal half the inactive
// bezel in each axis. Catch a datasheet edit that breaks centering.
assert(abs(disp_active_ox - (disp_module_w - disp_active_w) / 2) < 0.01 &&
       abs(disp_active_oy - (disp_module_h - disp_active_h) / 2) < 0.01,
       "active area not centered in module — top window will be off-center vs active pixels");

// ─── Retainer fits in pocket under glass ─────────────────────────────────
// Retainer is glued to the overlay underside at the bottom of the pocket,
// and the module (disp_glass_t thick) sits above it pressed against the
// shelf. Retainer thickness is bounded by (pocket depth − glass thickness).
assert(retainer_t <= disp_pocket_d - disp_glass_t,
       "retainer too thick to fit in pocket beneath the module glass");

// ─── Bottom pad recess fab-slop margin ───────────────────────────────────
// Pad recesses are 1 mm deep in a 3.5 mm floor, leaving 2.5 mm nominal. Under
// ±0.2 mm hand-tool depth slop, ensure ≥ 2 mm floor remains over the pads.
assert(bottom_t - pad_d - FAB_SLOP >= 2.0,
       "bottom pad recess leaves < 2.0 mm floor under ±0.2 mm fab slop");

// ─── key_rects stay inside plate bounds ──────────────────────────────────
// Catches an edit that walks a row rectangle off the plate. Uses the same
// indexing convention as the rest of the file: [x1, y1, x2, y2] plate-local.
assert(len([for (r = key_rects)
            if (r[0] < -0.01 || r[1] < -0.01 ||
                r[2] > plate_w + 0.01 || r[3] > plate_d + 0.01) 1]) == 0,
       "a key_rect escapes the plate's rectangular bounds");

// ─── Display cutout (in plate) stays inside plate bounds ─────────────────
// A future edit to disp_cut_y{1,2} or disp_cut_x{1,2} could walk the cutout
// off the plate; the cutout itself is fabricated in the frozen plate, but
// this file derives case geometry from those constants and they must still
// describe a region inside plate_d × plate_w for the derivation to be sane.
assert(disp_cut_y1 >= 0 && disp_cut_y2 <= plate_d &&
       disp_cut_x1 >= 0 && disp_cut_x2 <= plate_w,
       "display cutout escapes plate's rectangular bounds");

// ─── Tilt angle bounds ───────────────────────────────────────────────────
// The slot-open-margin assert above uses tan(tilt_angle)/2 without handling
// large angles, and the tilt-rotated display cut frame assumes the overlay
// slab is uniform thickness (true only for moderate tilt). Bound the angle.
assert(tilt_angle > 0 && tilt_angle < 30,
       "tilt_angle out of supported range — keep 0 < tilt_angle < 30°");

// ─── 66-switch coverage check (locked reference) ─────────────────────────
// Every MX switch on the frozen mkey.kicad_pcb, in plate-local coordinates
// (Board_X − 31.473, |Board_Y − 150.195| − 7.416). The list is sorted by
// row (Y) then column (X). Extracted and verified 2026-04-13 against the
// authoritative KiCad source; the plate/PCB are physically fabricated and
// cannot change. This list exists to catch future edits to `key_rects`
// that would accidentally leave a real switch unexposed.
MX_SWITCHES = [
    // Row 0 — number row + backspace
    [ 12.025, 88.200], [ 31.075, 88.200], [ 50.124, 88.200], [ 69.174, 88.200],
    [ 88.225, 88.200], [107.275, 88.200], [126.325, 88.200], [145.375, 88.200],
    [164.425, 88.200], [183.474, 88.200], [202.524, 88.200], [221.575, 88.200],
    [240.625, 88.200], [269.200, 88.200],
    // Row 1 — tab row
    [ 16.787, 69.150], [ 40.600, 69.150], [ 59.650, 69.150], [ 78.700, 69.150],
    [ 97.749, 69.150], [116.800, 69.150], [135.849, 69.150], [154.899, 69.150],
    [173.950, 69.150], [193.000, 69.150], [212.050, 69.150], [231.099, 69.150],
    [250.149, 69.150],
    // ISO Enter (bridges rows 1 and 2)
    [276.343, 59.625],
    // Row 2 — home row
    [ 19.168, 50.100], [ 45.362, 50.100], [ 64.412, 50.100], [ 83.462, 50.100],
    [102.512, 50.100], [121.562, 50.100], [140.612, 50.100], [159.662, 50.100],
    [178.712, 50.100], [197.762, 50.100], [216.812, 50.100], [235.862, 50.100],
    [254.912, 50.100],
    // Row 3 main — shift row + Arrow UP
    [ 14.406, 31.050], [ 35.837, 31.050], [ 54.887, 31.050], [ 73.937, 31.050],
    [ 92.987, 31.050], [112.037, 31.050], [131.087, 31.050], [150.137, 31.050],
    [169.187, 31.050], [188.237, 31.050], [207.287, 31.050], [226.337, 31.050],
    [262.056, 31.050], [321.587, 31.050],
    // Row 4 main — front row + Arrow L/D/R
    [ 14.406, 12.000], [ 38.218, 12.000], [ 62.031, 12.000], [133.468, 12.000],
    [204.906, 12.000], [228.718, 12.000], [252.531, 12.000], [276.343, 12.000],
    [302.537, 12.000], [321.587, 12.000], [340.637, 12.000],
];

function _switch_in_rect(sx, sy, r, t) =
    (r[0] - t) <= sx && sx <= (r[2] + t) &&
    (r[1] - t) <= sy && sy <= (r[3] + t);

function _switch_in_any_rect(sx, sy) =
    len([for (r = key_rects)
         if (_switch_in_rect(sx, sy, r, key_cap_clearance)) 1]) > 0;

_switches_covered = len([
    for (s = MX_SWITCHES) if (_switch_in_any_rect(s[0], s[1])) 1
]);

assert(len(MX_SWITCHES) == 66,
       "MX_SWITCHES list corrupted — expected 66 switches");
assert(_switches_covered == len(MX_SWITCHES),
       "key_rects union does not cover every MX switch — a real switch is under solid overlay wood");

// Coverage is count-based, so a copy-paste that duplicates one switch and
// drops another would still pass. Guard against that directly.
assert(len([for (i = [0:len(MX_SWITCHES)-1])
            for (j = [i+1:len(MX_SWITCHES)-1])
            if (abs(MX_SWITCHES[i][0] - MX_SWITCHES[j][0]) < 0.01 &&
                abs(MX_SWITCHES[i][1] - MX_SWITCHES[j][1]) < 0.01) 1]) == 0,
       "MX_SWITCHES contains a duplicate switch position — coverage check is unreliable");


// =============================================================================
// SECTION 6: HELPER FUNCTIONS
// =============================================================================

// Wedge box: flat bottom at Z=0, tilted top surface
// h_front = height at Y=0 face, h_back = height at Y=d face
module wedge_box(w, d, h_front, h_back) {
    // Face winding: CW from outside (OpenSCAD convention = inward normals)
    polyhedron(
        points = [
            [0, 0, 0],       [w, 0, 0],       [w, d, 0],       [0, d, 0],       // 0-3: bottom
            [0, 0, h_front], [w, 0, h_front], [w, d, h_back],  [0, d, h_back]   // 4-7: top
        ],
        faces = [
            [0,1,2,3],  // bottom (normal +Z inward)
            [7,6,5,4],  // top    (normal -Z inward)
            [4,5,1,0],  // front  (Y=0, normal +Y inward)
            [6,7,3,2],  // back   (Y=d, normal -Y inward)
            [3,7,4,0],  // left   (X=0, normal +X inward)
            [5,6,2,1]   // right  (X=w, normal -X inward)
        ]
    );
}

// Rounded wedge box: wedge_box with rounded vertical corners.
// Uses intersection of exact wedge tilt with rounded extrusion — no tilt distortion.
module rounded_wedge_box(w, d, h_front, h_back, r) {
    intersection() {
        wedge_box(w, d, h_front, h_back);
        linear_extrude(height = max(h_front, h_back) + 0.1)
            offset(r=r) offset(delta=-r)
                square([w, d]);
    }
}

// Z height of case top surface at a given case Y coordinate
function top_z(cy) = front_h + (back_h - front_h) * cy / outer_d;

// Z height of plate top surface at a given case Y coordinate.
// plate_recess is measured below the TRAY WALL TOP (which is top_z − top_t),
// not below the overlay top, so the plate sits fully inside the tray with
// (plate_recess) mm of gasket headroom between plate top and overlay bottom.
function plate_z(cy) = top_z(cy) - top_t - plate_recess;

// Convert plate-local coords to case coords
function p2c_x(px) = plate_ox + px;
function p2c_y(py) = plate_oy + py;

// =============================================================================
// SECTION 7: CASE BODY
// =============================================================================

// =============================================================================
// The case is TWO pieces:
//   1. TRAY  - bottom + walls + gasket slots + USB cutout
//   2. OVERLAY - top surface with key opening + display window/pocket
//
// The overlay sits on the wall tops and encloses the gasket slots from above.
// Assembly: place gasket strips in slots → drop plate in → place overlay on top.
// =============================================================================

// ─── PCB retention (informational note) ─────────────────────────────────────
// The PCB is NOT mechanically fastened to the case. The MX switches solder
// through the plate, and the plate is held by the gasket tabs. The PCB
// therefore rides the plate via the switch leads — standard gasket-mount
// convention. No standoffs, screw bosses, or PCB fasteners exist anywhere
// in this file. Do not add any without also re-running the pre-fab review;
// a rigid PCB mount would fight the gasket compression the rest of the
// design is tuned around.

// ─── Overlay locating rabbet (2D footprints) ────────────────────────────────
// `rabbet_outer_2d()` is the full ring width (= recess footprint in the
// overlay underside). `rabbet_inner_2d()` is the same ring shrunk by
// `rabbet_tol` per side (= tongue footprint on the tray wall top). The
// tongue sits inside the recess with `rabbet_tol` sliding clearance per
// side. Neither is notched in this helper — see `tongue_2d()` /
// `recess_2d()` below for the slot-notched versions actually used by the
// 3D modules.
module rabbet_outer_2d() {
    difference() {
        translate([wall_t - rabbet_w, wall_t - rabbet_w])
            square([inner_w + 2 * rabbet_w, inner_d + 2 * rabbet_w]);
        translate([wall_t, wall_t])
            square([inner_w, inner_d]);
    }
}
module rabbet_inner_2d() {
    difference() {
        translate([wall_t - rabbet_w + rabbet_tol,
                   wall_t - rabbet_w + rabbet_tol])
            square([inner_w + 2 * (rabbet_w - rabbet_tol),
                    inner_d + 2 * (rabbet_w - rabbet_tol)]);
        translate([wall_t - rabbet_tol, wall_t - rabbet_tol])
            square([inner_w + 2 * rabbet_tol, inner_d + 2 * rabbet_tol]);
    }
}

// 2D notch footprints — one rectangle per gasket slot, sized to reliably
// cut THROUGH the full width of the tongue/recess ring. The notch extends
// `margin` past the ring on both the cavity and outer sides (Y-extent =
// rabbet_w + 2*margin) and `margin` past the tab on both length ends.
// Using `rabbet_w` (not `slot_depth`) guarantees the notch always breaks
// the ring into cleanly disconnected segments regardless of how slot_depth
// relates to rabbet_w — a notch that only partially bites the ring leaves
// a hairline bridge that makes CGAL produce degenerate topology.
module gasket_slot_footprints_2d(margin) {
    ring_w = rabbet_w + 2 * margin;

    // Back wall — notches in the inner ring of the back wall
    for (cx = back_tab_cx) {
        translate([p2c_x(cx) - (tab_len + slot_tol) / 2 - margin,
                   outer_d - wall_t - margin])
            square([tab_len + slot_tol + 2 * margin, ring_w]);
    }
    // Front wall
    for (cx = front_tab_cx) {
        translate([p2c_x(cx) - (tab_len + slot_tol) / 2 - margin,
                   wall_t - rabbet_w - margin])
            square([tab_len + slot_tol + 2 * margin, ring_w]);
    }
    // Left wall
    translate([wall_t - rabbet_w - margin,
               p2c_y(left_tab_cy) - (tab_len + slot_tol) / 2 - margin])
        square([ring_w, tab_len + slot_tol + 2 * margin]);
    // Right wall
    translate([outer_w - wall_t - margin,
               p2c_y(right_tab_cy) - (tab_len + slot_tol) / 2 - margin])
        square([ring_w, tab_len + slot_tol + 2 * margin]);
}

// 2D notch footprints for magnet pockets. With both ENABLE_OVERLAY_RABBET
// and ENABLE_MAGNET_POCKETS on, each magnet pocket on a wall top overlaps
// the tongue ring in plan view (the ring runs along the inner edge of the
// wall, the pockets are drilled roughly mid-wall). Without notches the
// tongue ends up bridging over the pocket — structurally sound in 3D print
// but physically impossible in hand-cut hardwood (a 2.5 mm drill would
// shear the 1.5 mm wide tongue where it passes over each pocket).
//
// Each magnet is classified by which wall it belongs to, then a rectangle
// is emitted that cuts cleanly through the full width of the tongue ring
// on that wall (same ring_w pattern as gasket_slot_footprints_2d).
module magnet_notch_footprints_2d(margin) {
    ring_w = rabbet_w + 2 * margin;
    w      = magnet_d + 2 * margin;
    for (p = magnet_positions()) {
        if (p[1] <= wall_t) {
            // front wall
            translate([p[0] - w / 2, wall_t - rabbet_w - margin])
                square([w, ring_w]);
        } else if (p[1] >= outer_d - wall_t) {
            // back wall
            translate([p[0] - w / 2, outer_d - wall_t - margin])
                square([w, ring_w]);
        } else if (p[0] <= wall_t) {
            // left wall
            translate([wall_t - rabbet_w - margin, p[1] - w / 2])
                square([ring_w, w]);
        } else if (p[0] >= outer_w - wall_t) {
            // right wall
            translate([outer_w - wall_t - margin, p[1] - w / 2])
                square([ring_w, w]);
        }
    }
}

// Slot-notched tongue / recess footprints — continuous locating ring with
// 9 gaps where the gasket slots need a clear tab-descent path, plus one
// additional gap at every magnet pocket when ENABLE_MAGNET_POCKETS is on.
module tongue_2d() {
    difference() {
        rabbet_inner_2d();
        gasket_slot_footprints_2d(notch_margin);
        if (ENABLE_MAGNET_POCKETS) magnet_notch_footprints_2d(notch_margin);
    }
}
module recess_2d() {
    difference() {
        rabbet_outer_2d();
        gasket_slot_footprints_2d(notch_margin);
        if (ENABLE_MAGNET_POCKETS) magnet_notch_footprints_2d(notch_margin);
    }
}

// Tongue: positive tilted ring-slab standing proud above the tray wall
// top. In `tongue_2d()` footprint. Z range extends 0.01 mm BELOW wall_top
// so the union with the tray wall top has overlap rather than a
// coplanar-face singularity.
module tray_tongue_3d() {
    intersection() {
        translate([0, 0, -1])
            linear_extrude(height = back_h + 10)
                tongue_2d();
        difference() {
            wedge_box(outer_w, outer_d,
                      front_h - top_t + rabbet_h,
                      back_h  - top_t + rabbet_h);
            wedge_box(outer_w, outer_d,
                      front_h - top_t - 0.01,
                      back_h  - top_t - 0.01);
        }
    }
}

// Recess: negative tilted ring-slab cut INTO the overlay underside. In
// `recess_2d()` footprint. Lower Z bound extends 0.01 mm below the overlay
// underside so the subtraction cleanly pierces the underside plane rather
// than touching it coplanarly (CGAL produces degenerate topology for
// exactly-coincident faces, resulting in spurious extra volumes).
module overlay_recess_3d() {
    intersection() {
        translate([0, 0, -1])
            linear_extrude(height = back_h + 10)
                recess_2d();
        difference() {
            wedge_box(outer_w, outer_d,
                      front_h - top_t + rabbet_h,
                      back_h  - top_t + rabbet_h);
            wedge_box(outer_w, outer_d,
                      front_h - top_t - 0.01,
                      back_h  - top_t - 0.01);
        }
    }
}

// Magnet pocket cylinders — TILT-COMPENSATED.
//
// The wall top (tray) and overlay underside are both tilted at `tilt_angle`
// about the X axis. A vertical Ø`magnet_d` cylinder whose top sits at
// `wall_top(p[1])` does NOT punch cleanly through the tilted plane: at the
// uphill end of the pocket the plane is `magnet_d · tan(tilt_angle)/2` above
// the cylinder top, leaving a thin sliver of material capping the hole; at
// the downhill end the cylinder pokes the same amount through empty space.
// Symmetric issue on the overlay underside.
//
// Fix: anchor the cylinder to the EXTREME wall-top over the pocket's own
// footprint (uphill edge for the tray, downhill for the overlay) and grow
// the cylinder length by the tilt drift so the full `magnet_depth` is
// preserved at the OTHER extreme. Result: clean hole through the tilted
// plane everywhere in the pocket footprint.
magnet_tilt_drift = magnet_d * tan(tilt_angle);   // ≈ 0.22 mm for Ø2.5, 5°
magnet_cyl_h      = magnet_depth + magnet_tilt_drift + 0.2;   // ≈ 2.68 mm

// top_z() is monotonic in y (5° tilt, increasing toward the back), so the
// uphill edge of every pocket is at y = p[1] + magnet_d/2 regardless of
// which wall the magnet sits on, and the downhill edge is p[1] − magnet_d/2.

// Cuts magnet pockets in the TRAY wall top. Cylinder top anchored 0.1 mm
// above the uphill edge of the pocket so it pokes above the highest point
// of the tilted wall top. Length is bumped by the tilt drift so the
// downhill edge still has ≥ magnet_depth of drilled clearance.
module tray_magnet_pockets() {
    for (p = magnet_positions()) {
        z_top_hi = top_z(p[1] + magnet_d/2) - top_t + 0.1;
        translate([p[0], p[1], z_top_hi - magnet_cyl_h])
            cylinder(d = magnet_d, h = magnet_cyl_h);
    }
}

// Cuts magnet pockets in the OVERLAY underside. Cylinder bottom anchored
// 0.1 mm below the downhill edge of the pocket so it starts below the
// lowest point of the tilted overlay underside.
module overlay_magnet_pockets() {
    for (p = magnet_positions()) {
        z_bot_lo = top_z(p[1] - magnet_d/2) - top_t - 0.1;
        translate([p[0], p[1], z_bot_lo])
            cylinder(d = magnet_d, h = magnet_cyl_h);
    }
}

// ─── PIECE 1: TRAY (bottom + walls) ─────────────────────────────────────────
module case_tray() {
    union() {
        difference() {
            // Outer shell — extended by tray_ext on all sides for thicker walls.
            // Top face at wall_top (= top_z − top_t) so the overlay's flat
            // underside sits directly on the wall cheek.
            // Rounded vertical corners for soft user-facing edges.
            translate([-tray_ext, -tray_ext, 0])
                rounded_wedge_box(outer_w + 2*tray_ext, outer_d + 2*tray_ext,
                          front_h - top_t, back_h - top_t, tray_corner_r);

            // Inner cavity (fully open top) — position unchanged, uses wall_t
            translate([wall_t, wall_t, bottom_t])
                wedge_box(inner_w, inner_d,
                          front_h + 10,
                          back_h  + 10);

            // Gasket tab slots (cut into the walls)
            gasket_slots();

            // USB-C cutout through back wall
            usb_cutout();

            // Optional: magnet pockets sunk into the wall tops.
            if (ENABLE_MAGNET_POCKETS) tray_magnet_pockets();
        }

        // Optional: overlay-locating tongue standing proud above the
        // wall top. Unioned AFTER the cavity/slot/magnet cuts so its
        // geometry is preserved in full.
        if (ENABLE_OVERLAY_RABBET) tray_tongue_3d();
    }
}

// ─── PIECE 2: TOP OVERLAY ───────────────────────────────────────────────────
// Overhangs the tray walls. With ENABLE_OVERLAY_RABBET a recess is cut into
// the overlay underside and the tray's tongue drops into it for X/Y
// registration. With ENABLE_MAGNET_POCKETS magnet pockets mate with the
// tray pockets across the seam to hold the overlay down.
module case_overlay() {
    difference() {
        // Full overlay with overhang and rounded corners
        translate([-overhang, -overhang, 0])
            rounded_wedge_box(overlay_w, overlay_d,
                              overlay_front_h, overlay_back_h,
                              overlay_corner_r);

        // Remove everything below top_t (keep only the top slab)
        // Use non-rounded wedge to avoid affecting outer corner radius
        translate([-overhang - 0.01, -overhang - 0.01, -0.01])
            wedge_box(overlay_w + 0.02, overlay_d + 0.02,
                      overlay_front_h - top_t + 0.01,
                      overlay_back_h  - top_t + 0.01);

        // Key opening: union of per-keycap rectangles
        key_opening();

        // Display through-cut / bottom pocket
        display_cutout();

        // Optional: magnet pockets in the overlay underside, matching
        // the tray pockets across the seam.
        if (ENABLE_MAGNET_POCKETS) overlay_magnet_pockets();

        // Optional: recess cut into the overlay underside that accepts
        // the tray's tongue for lateral registration.
        if (ENABLE_OVERLAY_RABBET) overlay_recess_3d();
    }
}

// ─── COMBINED: both pieces together ─────────────────────────────────────────
module case_complete() {
    case_tray();
    case_overlay();
}

// ─── 7c. Key opening ────────────────────────────────────────────────────────
// UNION of per-row / per-cluster keycap rectangles from `key_rects`.
// Anywhere there is a key, the overlay is cut through; anywhere there is no
// key, the overlay is solid hardwood hiding the bare plate underneath.
// The natural gaps in the layout become real bezels (effective widths after
// key_cap_clearance is expanded on both sides of adjacent rects):
//   • ~4.16 mm rib between the main field and the arrow L/D/R row
//     (4.76 mm nominal MX spacing − 2×key_cap_clearance)
//   • ~23.21 mm bezel between R-shift and the UP arrow column
//   • ~13.40 mm bezel between the main field right edge and the display
//     through-cut (measured wall-to-wall)
//
// Corners are rounded with key_corner_r (1.2 mm) using the same
// offset(r) offset(delta=-r) pattern as the display window. This softens
// the visual appearance, reduces stress concentration at corners, and
// improves SLA printability. Keycaps have their own rounded corners
// (typically 1-2 mm radius) so this does not affect key travel.

// 2D footprint of the key opening with rounded corners.
// Both convex (outer) and concave (inner) corners are rounded:
//   offset(r=-r) offset(delta=r) rounds concave corners (e.g., arrow T-junction)
//   offset(r=r) offset(delta=-r) rounds convex corners (e.g., rectangle corners)
module key_opening_2d() {
    t = key_cap_clearance;
    offset(r = key_corner_r) offset(delta = -key_corner_r)
    offset(r = -key_corner_r) offset(delta = key_corner_r)
        for (r = key_rects) {
            x1 = r[0] - t;  y1 = r[1] - t;
            x2 = r[2] + t;  y2 = r[3] + t;
            translate([p2c_x(x1), p2c_y(y1)])
                square([x2 - x1, y2 - y1]);
        }
}

module key_opening() {
    z0 = bottom_t - 0.01;
    zh = back_h + 2;

    translate([0, 0, z0])
        linear_extrude(height = zh)
            key_opening_2d();
}


// ─── 7d. Display stepped cutout ─────────────────────────────────────────────
// The display is bottom-loaded: the module body drops UP into a blind pocket
// cut from the overlay underside, and is held against the thin shelf of wood
// above it. What the user sees looking down at the keyboard is the glass
// recessed only by shelf_t (1 mm) through a picture-frame window sized to
// the active area.
//
// Cross-section at the display (case coords, looking along X):
//
//    ────────────────────  overlay top (case top surface)
//      │              │    } shelf_t = 1 mm of wood (frame around window)
//      │ ┌──────────┐ │    ← top window (active area + clearance)
//    ──┤ │          │ ├──   overlay bottom (tray wall top)
//      │ │   AIR    │ │    }
//      │ │  pocket  │ │    } disp_pocket_d = 4 mm (blind, from below)
//      └─┘          └─┘    }
//
// Module (1.56 mm thick) sits in the pocket with its glass face pressed up
// against the shelf underside; active pixels are visible through the window.
// The FPC ribbon exits the back of the module, drops through the plate's
// display cutout (31.5 × 37.2) to J2 on the PCB — unchanged from before.
//
// Fabrication: the pocket is cut from the underside (blind, 4 mm deep), then
// the top window is cut through the remaining 1 mm shelf from the top face.
// Both are four corner drill holes + straight saw cuts; the 4 mm pocket
// depth is controlled with a drill-press stop or a router jig.
module display_cutout() {
    cx = p2c_x(disp_cx);
    cy = p2c_y(disp_cy);

    // The overlay top surface is a flat wedge tilted tilt_angle° about X.
    // Cutting vertical prisms into it produces a wrong shelf: too thin (or
    // gone) at the front of the display and too thick at the back. We cut in
    // a tilted frame instead so both cut volumes have their top face parallel
    // to the overlay top surface.
    //
    // Tilted frame: origin at the display center ON the top surface, local
    // +Z = top-surface normal, local Y/X unchanged apart from the tilt.
    translate([cx, cy, top_z(cy)])
    rotate([tilt_angle, 0, 0])
    {
        // Bottom pocket: module body, from slightly below the overlay bottom
        // (local z = −top_t − ε) up to the shelf underside (local z = −shelf_t).
        translate([-disp_pocket_w / 2, -disp_pocket_h / 2, -top_t - 0.01])
            linear_extrude(height = disp_pocket_d + 0.01)
                offset(r=disp_pocket_r) offset(delta=-disp_pocket_r)
                    square([disp_pocket_w, disp_pocket_h]);

        // Top window: cuts cleanly through the shelf, from (local z = −shelf_t − ε)
        // to (local z = +ε) — the shelf is exactly the top `shelf_t` mm of the
        // overlay slab in the normal direction. The 2D shape is a hull of four
        // shelf_corner_r circles sitting in the window bbox corners, giving a
        // rounded rectangle whose inner corners are a smooth curve regardless
        // of the shelf_frame_{x,y} asymmetry.
        translate([-disp_win_w / 2, -disp_win_h / 2, -shelf_t - 0.01])
            linear_extrude(height = shelf_t + 0.02)
                display_window_2d();
    }
}

// 2D footprint of the top window. Hull of four circles of radius
// shelf_corner_r placed at the inner corners of the window bounding box.
// Used by both display_cutout (overlay window) and display_retainer (backing
// clamp inner opening) so the two always match.
module display_window_2d() {
    for_x = [shelf_corner_r, disp_win_w - shelf_corner_r];
    for_y = [shelf_corner_r, disp_win_h - shelf_corner_r];
    hull() {
        for (cx = for_x)
            for (cy = for_y)
                translate([cx, cy])
                    circle(r = shelf_corner_r);
    }
}

// ─── 7e. Display backing clamp (optional, separately fabricated) ────────────
// Flat picture-frame piece (1.2 mm plywood, brass, or 3D-printed) that slots
// into the bottom of the display pocket and clamps the module UP against the
// shelf underside. Outer outline = pocket size; inner window clears the
// module's FPC ribbon area. Glue or screw to the overlay underside once the
// module is seated. Optional — adhesive on the module glass / shelf can do
// the same job if you prefer.
module display_retainer() {
    difference() {
        // Outer outline matches the pocket footprint.
        linear_extrude(height = retainer_t)
            offset(r=disp_pocket_r) offset(delta=-disp_pocket_r)
                square([disp_pocket_w, disp_pocket_h]);
        // Inner opening matches the overlay's top window exactly, so the
        // clamp never encroaches on any part that would be visible or
        // block the FPC ribbon.
        translate([(disp_pocket_w - disp_win_w) / 2,
                   (disp_pocket_h - disp_win_h) / 2, -0.1])
            linear_extrude(height = retainer_t + 0.2)
                display_window_2d();
    }
}

// ─── 7f. USB-C cutout ───────────────────────────────────────────────────────
module usb_cutout() {
    // USB exits through the back wall. Z geometry is computed at top level
    // (see `usb_z_center` and friends) so the invariant asserts share the
    // exact same derivation the cut itself uses — no drift possible.
    //
    // Corners are filleted at usb_corner_r to soften stress risers where
    // the cable tugs against the back wall on insertion/removal. Added in
    // the 2026-04-16 JLC3DP pre-fab review. Geometry: build the 2D rounded
    // rectangle in XY, linear_extrude along +Z for a prism the thickness
    // of the wall, then rotate([-90,0,0]) so the extrude direction flips
    // from +Z to +Y (mapping Y→-Z, Z→+Y in the shape). The outer Z-shift
    // puts the bottom of the cut at usb_cut_z_bot.
    usb_corner_r = 0.5;
    ux = p2c_x(usb_plate_x) - usb_cut_w / 2;
    translate([ux, back_wall_inner_y - 0.5, usb_cut_z_bot + usb_cut_h])
        rotate([-90, 0, 0])
            linear_extrude(height = wall_t + 1)
                offset(r = usb_corner_r) offset(delta = -usb_corner_r)
                    square([usb_cut_w, usb_cut_h]);
}

// =============================================================================
// SECTION 8: GASKET TAB SLOTS
// =============================================================================
// Slots cut INTO the case walls where plate mounting tabs slide in.
// Gasket/foam material lines the slot (top + bottom surfaces) for compression
// mounting. The plate drops in from above, tabs engage the slots.
//
// Each slot is sized so its top extends ABOVE the tray wall top by
// `slot_open_margin`, turning it into an open-top channel. This is what
// makes drop-in assembly possible — otherwise the top of the slot would be
// sealed by wall material and the tab could not descend past the wall top.
// The assertion below catches any regression where plate_recess grows
// beyond what slot_height can accommodate.
//
// Slot cross-section (side view, tab enters from above):
//   ┈┈┈┈┈┈┈┈┈┈┈┈ ← overlay bottom (caps the open-top channel at assembly)
//          ┆ TOP OPEN — plate tab descends through here
//   ═══════┆═══  ← tray wall outer top
//   ║      ┆  ║
//   ║   ┌──┴┐ ║  ← gasket (top)  — installed after plate is seated
//   ║   │TAB│ ║  ← plate tab final position
//   ║   └───┘ ║  ← gasket (bottom) — installed before plate is seated
//   ║         ║
//   ═══════════ ← tray wall inner surface (cavity side)

// Compile-time checks on the slot decomposition. Both must hold or the
// drop-in or the bottom gasket fails.
assert(slot_top_above_plate >= plate_recess + slot_open_margin,
       "slot_top_above_plate too small: slot top doesn't reach above the tray wall top — plate cannot drop in");
// Side slots run ALONG the tilt axis (Y), so the plate tilts through the
// slot length. At the downhill end of each side slot the plate bottom drops
// (tab_len + slot_tol)·tan(tilt)/2 below the slot-center reference, which
// eats into the bottom-gasket headroom. This check is the symmetric
// counterpart of the slot_open_margin check at line ~546 (which guards the
// uphill end of the TOP of the slot) — without it the 2026-04-13 review
// value of slot_bot_below_plate = gasket_compressed + 0.25 = 1.75 mm left
// only 0.87 mm below the plate at the downhill side-slot end, crushing the
// 1.5 mm bottom gasket.
assert(slot_bot_below_plate >=
       gasket_compressed + (tab_len + slot_tol) * tan(tilt_angle) / 2 + FAB_SLOP,
       "slot_bot_below_plate too small: downhill end of side slot has gasket headroom < gasket_compressed under tilt + fab slop");

// Slot dimensions
slot_tol     = 0.6;    // clearance around tab (per side, length direction).
                       // Bumped from 0.4 → 0.6 in the 2026-04-16 JLC3DP pre-fab
                       // review: the ±0.3% tolerance band on >100 mm features
                       // (case X = 363 mm) translates to ~1 mm of positional
                       // drift at the outermost tab (rightmost front tab center
                       // at case-X ≈ 320 mm), which would jam a 0.2 mm-play
                       // slot. 0.6 gives 0.3 mm play per side — enough to
                       // absorb uniform shrinkage of ±0.3% over the 320 mm
                       // reach without losing the snug compression fit. Used
                       // in conjunction with a JLC3DP X-axis scale-comp order
                       // note; belt-and-braces if the factory skips it.
                       //
                       // Earlier: 0.3 → 0.4 in the 2026-04-14 review (0.3 was
                       // exactly equal to the ±0.2 mm fab slop budget, leaving
                       // zero play under worst-case slop).
slot_depth   = 1.5;    // how deep the slot goes into the wall.
                       // Measured tab penetration into the wall is 1.25..1.33 mm
                       // (tab_ext 1.749..1.828 mm, minus plate_gap 0.5 mm), so
                       // 1.5 mm captures the tab with ~0.17 mm back clearance.
                       // Reduced from 2.5 mm because 2.5 mm left only 2.3 mm
                       // of outboard wall cheek on 4.8 mm walls, with 7 slots
                       // running 20 mm each along front+back — a chip-off
                       // hazard in hardwood. 1.5 mm depth leaves 3.3 mm cheek.
slot_open_margin = 1.5;    // mm slot top extends ABOVE the tray wall top at
                           // the slot's center-Y. Turns each slot into an
                           // open-top channel so the plate can drop in from
                           // above. Bumped from 0.5 mm because the LEFT/RIGHT
                           // slots run ALONG the 5° tilt axis (Y), so the
                           // wall top rises over the slot's length. Over the
                           // 20.10 mm slot length the uphill end sits
                           // L·tan(5°)/2 = 0.88 mm above slot center, so the
                           // flat-topped slot box must clear ≥0.88 mm of
                           // wall-top rise. With slot_open_margin = 1.5 mm
                           // the uphill wall-top is 0.62 mm BELOW the slot
                           // top — comfortably above ±0.2 mm fab slop on
                           // both wall and tab. Front/back slots run along X
                           // (no tilt drift) so they only see a
                           // 4.8·tan(5°) = 0.42 mm rise across wall thickness.
// Slot is sized ASYMMETRICALLY about plate midplane. The two constraints
// (drop-in clearance above wall top, gasket headroom below the plate) have
// nothing to do with each other, so a symmetric slot wastes wood — every
// extra mm of slot_open_margin needed for the tilt-corrected drop-in would
// otherwise force a matching mm of empty slot on the bottom side too. By
// decomposing into top/bottom extents and recombining, slot_height shrinks
// from 8.6 → 6.85 mm, the slot box ends just below where the bottom gasket
// needs it, and the tab sits visually centered between snug compression
// zones rather than floating in the middle of an oversized cavity.
//
// Geometry (case Z, plate top = plate_z(cy), plate bottom = plate_z − plate_t):
//
//                                              ┄┄┄┄┄  ← slot top  = plate_top + slot_top_above_plate
//                                                       (= wall_top + slot_open_margin)
//                                                       must be ≥ wall_top + slot_open_margin to clear
//                                                       the tilted wall over the slot's full length
//                                              ─────  ← tray wall top  = plate_top + plate_recess
//                                              ┄┄┄┄┄
//                                              ▓▓▓▓▓  ← top gasket (gasket_compressed)
//                                              ▒▒▒▒▒  ← plate tab (plate_t)
//                                              ▓▓▓▓▓  ← bottom gasket (gasket_compressed)
//                                              ┄┄┄┄┄  ← slot bottom = plate_bot − slot_bot_below_plate
//
slot_top_above_plate = plate_recess + slot_open_margin;   // 2.0 + 1.5 = 3.5 mm
// slot_bot_below_plate: sized so the bottom gasket (gasket_compressed =
// 1.5 mm) still fits under the plate at the downhill end of the SIDE slots
// (which run along Y / the tilt axis). Over the (tab_len+slot_tol)/2 =
// 10.10 mm half-length, the plate bottom descends by 10.10·tan(5°) = 0.884
// mm; add FAB_SLOP (0.2) and a small machining margin (0.05). The front/back
// slots run along X and have no tilt drift, so they inherit the side-slot
// sizing for free.
slot_bot_below_plate = gasket_compressed
                     + (tab_len + slot_tol) * tan(tilt_angle) / 2
                     + FAB_SLOP + 0.05;
                                                          // 1.5 + 0.884 + 0.2 + 0.05 = 2.634 mm
slot_height          = plate_t + slot_top_above_plate + slot_bot_below_plate;
                                                          // 1.6 + 3.5 + 2.634 = 7.734 mm
// Slot center, relative to plate midplane. Positive = slot is offset upward.
// (slot_top + slot_bot)/2 = plate_midplane + (slot_top_above_plate − slot_bot_below_plate)/2
slot_center_offset   = (slot_top_above_plate - slot_bot_below_plate) / 2;
                                                          // (3.5 − 2.634)/2 = +0.433 mm

module gasket_slots() {
    // ── Back wall slots (3) ──────────────────────────────────────────────
    // Slots cut into the back wall from the inner face toward the outer face.
    // Tab enters from the cavity side.
    for (i = [0:2]) {
        cx = p2c_x(back_tab_cx[i]);
        wall_inner_y = outer_d - wall_t;  // inner face of back wall
        local_pz = plate_z(wall_inner_y);  // plate top Z at this Y
        slot_center_z = local_pz - plate_t / 2 + slot_center_offset;

        translate([cx - (tab_len + slot_tol)/2,
                   wall_inner_y - 0.01,
                   slot_center_z - slot_height/2])
            cube([tab_len + slot_tol, slot_depth + 0.01, slot_height]);
    }

    // ── Front wall slots (4) ─────────────────────────────────────────────
    for (i = [0:3]) {
        cx = p2c_x(front_tab_cx[i]);
        wall_inner_y = wall_t;  // inner face of front wall
        local_pz = plate_z(wall_inner_y);
        slot_center_z = local_pz - plate_t / 2 + slot_center_offset;

        translate([cx - (tab_len + slot_tol)/2,
                   wall_inner_y - slot_depth,
                   slot_center_z - slot_height/2])
            cube([tab_len + slot_tol, slot_depth + 0.01, slot_height]);
    }

    // ── Left wall slot (1) ───────────────────────────────────────────────
    // (locals wrapped in `let()` so they don't clash with the right-wall slot
    // block below — OpenSCAD does not scope locals inside bare `{}` and will
    // abort under --hardwarnings otherwise.)
    let(cy_L            = p2c_y(left_tab_cy),
        wall_inner_x_L  = wall_t,
        local_pz_L      = plate_z(p2c_y(left_tab_cy)),
        slot_center_z_L = plate_z(p2c_y(left_tab_cy)) - plate_t / 2 + slot_center_offset) {
        translate([wall_inner_x_L - slot_depth,
                   cy_L - (tab_len + slot_tol)/2,
                   slot_center_z_L - slot_height/2])
            cube([slot_depth + 0.01, tab_len + slot_tol, slot_height]);
    }

    // ── Right wall slot (1) ──────────────────────────────────────────────
    let(cy_R            = p2c_y(right_tab_cy),
        wall_inner_x_R  = outer_w - wall_t,
        local_pz_R      = plate_z(p2c_y(right_tab_cy)),
        slot_center_z_R = plate_z(p2c_y(right_tab_cy)) - plate_t / 2 + slot_center_offset) {
        translate([wall_inner_x_R - 0.01,
                   cy_R - (tab_len + slot_tol)/2,
                   slot_center_z_R - slot_height/2])
            cube([slot_depth + 0.01, tab_len + slot_tol, slot_height]);
    }
}

// =============================================================================
// SECTION 10: BOTTOM FEATURES
// =============================================================================
// The only finishing currently applied to the case is the inline 0.5 mm
// bottom chamfers (front + back edges) cut in `case_tray_finished()` below,
// and the pad recesses. The top edges will be hand-sanded/block-planed on
// the physical piece — there is no CAD top-edge chamfer. A prior dormant
// `edge_chamfers()` module was removed in the 2026-04-14 review.

// Bottom rubber pad recesses (for anti-slip, no legs per spec)
pad_d   = 1.0;     // recess depth
pad_w   = 30.0;    // pad width
pad_h   = 10.0;    // pad length (front-back)
pad_inset = 15.0;  // inset from edges

module bottom_pad_recesses() {
    // Four pad recesses near the corners (aligned with extended tray shell)
    positions = [
        [-tray_ext + pad_inset, -tray_ext + pad_inset],                                        // front-left
        [outer_w + tray_ext - pad_inset - pad_w, -tray_ext + pad_inset],                       // front-right
        [-tray_ext + pad_inset, outer_d + tray_ext - pad_inset - pad_h],                       // back-left
        [outer_w + tray_ext - pad_inset - pad_w, outer_d + tray_ext - pad_inset - pad_h]       // back-right
    ];

    for (pos = positions) {
        translate([pos[0], pos[1], -0.01])
            cube([pad_w, pad_h, pad_d + 0.01]);
    }
}

// =============================================================================
// SECTION 10b: DECORATIVE TRIMS (3D-print-only)
// =============================================================================
// Every feature in this section is cosmetic — disabling the whole group
// (ENABLE_DECORATIVE_TRIMS = false) produces a functionally identical case.
// Cut depths and stroke widths are held at ≥0.8 mm per JLC3DP rules.

// ─── Geometry parameters ────────────────────────────────────────────────────
deco_cut_depth = 1.0;    // shared depth for every engraved/embossed feature.
                         // JLC3DP minimum for SLA resin is 0.8 mm; 1.0 gives
                         // 0.2 mm of margin so any fab-slop doesn't push the
                         // recess below the readable floor. Grown from 0.8 →
                         // 1.0 in the 2026-04-16 review.

// Overlay edge pinstripe (a hairline-looking groove just inside the outer edge)
deco_stripe_inset  = 1.2;    // mm in from the overlay outer edge
deco_stripe_width  = 1.0;    // groove width in the plane of the top face.
                             // 0.2 mm above JLC3DP 0.8 mm minimum so the
                             // full ±0.2 mm fab slop fits inside the margin
                             // — worst-case printed groove still at floor,
                             // nominal well above. Grown 0.8 → 0.9 in the
                             // 2026-04-16 review, then 0.9 → 1.0 in the
                             // 2026-04-17 pre-fab review.

// Owner initials + year plate
deco_initials_size   = 8.0;                          // mm cap height (initials row)
deco_year_size       = 8.0;                          // mm cap height (year row)
                                                      // Liberation Sans Bold stem ≈ 0.15·cap-height,
                                                      // so stroke ≈ 0.90 mm at 6.0 mm — 0.10 mm
                                                      // above the JLC3DP 0.8 mm floor, enough to
                                                      // survive half the ±0.2 mm fab slop on each
                                                      // side of the glyph edge. Grown 3.8 → 5.0
                                                      // → 5.5 → 6.0 across successive pre-fab
                                                      // reviews (latest: 2026-04-17).
deco_stamp_row_gap   = 1.2;                          // mm gap between the two rows
deco_initials_font   = "Liberation Sans:style=Bold";
deco_initials_y_frac = 0.5;                          // 0 = front, 1 = back

// Logo above the display
deco_logo_chip_size   = 7.0;                         // chip icon outer square, mm
deco_logo_chip_border = 1.0;                         // chip frame stroke, mm.
                                                      // 0.2 mm above JLC3DP 0.8 mm engraved-detail
                                                      // floor so ±0.2 mm fab slop doesn't thin the
                                                      // frame into an unreadable hairline. Grown
                                                      // 0.9 → 1.0 in the 2026-04-17 pre-fab review.
deco_logo_chip_inner  = 2.4;                         // inner solid square side, mm
deco_logo_text        = "mKey";
deco_logo_text_size   = 6.0;                         // cap height, mm. Liberation Sans Bold stem
                                                      // ≈ 0.15·cap-height, so stroke ≈ 0.90 mm —
                                                      // 0.10 mm above the 0.8 mm readable floor.
                                                      // Grown 5.5 → 6.0 in the 2026-04-17 review.
                                                      // Ref-size constants below remain anchored
                                                      // at 5.5; text_w scales linearly.
deco_logo_text_gap    = 1.8;                         // chip ↔ text gap, mm
deco_logo_font        = "Liberation Sans:style=Bold";

// ─── Side wall logo ─────────────────────────────────────────────────────────
// Debossed mKey logo (chip icon + wordmark) on the outer face of the left
// (X=0) and right (X=outer_w) walls. Reuses deco_logo_2d() — same 2D shape
// as the top-face logo so the two read identically. The outer face is
// flat-vertical, so no tilt compensation is needed. The logo is centered
// along the wall in Y and vertically placed so it sits inside the wall
// height at Y = outer_d/2 (the mid-point, where wall height equals the
// average of front_h-top_t and back_h-top_t).
module deco_side_logo() {
    wall_top_mid = (front_h + back_h) / 2 - top_t;
    z_center = wall_top_mid / 2;
    y_center = outer_d / 2;

    // LEFT wall — readable from −X (aligned with extended tray shell)
    translate([-tray_ext + deco_cut_depth, y_center, z_center])
        rotate([90, 0, -90])
            linear_extrude(height = deco_cut_depth + 0.1)
                deco_logo_2d();

    // RIGHT wall — readable from +X (aligned with extended tray shell)
    translate([outer_w + tray_ext - deco_cut_depth, y_center, z_center])
        rotate([90, 0, 90])
            linear_extrude(height = deco_cut_depth + 0.1)
                deco_logo_2d();
}

// ─── Overlay edge pinstripe ─────────────────────────────────────────────────
// A thin picture-frame groove running just inside the rounded outer edge of
// the overlay top. Built as a 2D frame, extruded to a vertical prism, then
// rotated by tilt_angle around the front-bottom X axis so the prism's "top"
// face sits on the tilted overlay top plane. At 5° tilt the prism is
// essentially vertical — groove width remains 0.8 mm to within <0.01 mm.
module deco_pinstripe_frame_2d() {
    difference() {
        translate([-overhang, -overhang])
            offset(r = overlay_corner_r)
                offset(delta = -overlay_corner_r - deco_stripe_inset)
                    square([overlay_w, overlay_d]);
        translate([-overhang, -overhang])
            offset(r = overlay_corner_r)
                offset(delta = -overlay_corner_r - deco_stripe_inset - deco_stripe_width)
                    square([overlay_w, overlay_d]);
    }
}

module deco_edge_pinstripe() {
    translate([0, 0, front_h])
        rotate([tilt_angle, 0, 0])
            translate([0, 0, -deco_cut_depth])
                linear_extrude(height = deco_cut_depth + 0.5)
                    deco_pinstripe_frame_2d();
}

// ─── Owner initials + year plate ────────────────────────────────────────────
// Two stacked text rows debossed into the tray bottom: DECO_INITIALS on top,
// DECO_YEAR below. Mirrored in Y so the stack reads correctly when the case
// is flipped over the X axis (toward the viewer): that physical rotation
// maps (x,y,z)→(x,−y,−z), negating Y but preserving X, so a Y-mirror
// pre-compensates the vertical inversion of the glyphs. Placed in the middle
// of the underside, clear of the four bottom pad recesses.
module deco_owner_initials() {
    x_center = outer_w / 2;
    y_center = outer_d * deco_initials_y_frac;

    // Stack layout: initials cap-height + gap + year cap-height. Y=0 in the
    // stack frame is the vertical center of the stack, so the bottom-face
    // recess is balanced about y_center.
    total_h     = deco_initials_size + deco_stamp_row_gap + deco_year_size;
    dy_initials = total_h / 2 - deco_initials_size;   // baseline of initials row
    dy_year     = -total_h / 2;                       // baseline of year row

    translate([x_center, y_center, -0.01])
        mirror([0, 1, 0])
            linear_extrude(height = deco_cut_depth + 0.02) {
                translate([0, dy_initials])
                    text(DECO_INITIALS,
                         size = deco_initials_size,
                         halign = "center", valign = "baseline",
                         font = deco_initials_font);
                translate([0, dy_year])
                    text(DECO_YEAR,
                         size = deco_year_size,
                         halign = "center", valign = "baseline",
                         font = deco_initials_font);
            }
}

// ─── Logo above display ─────────────────────────────────────────────────────
// Simplified mKey logo: a chip icon (outline frame + inner solid square) and
// the "mKey" wordmark to its right. The raw SVG in hardware/board/assets has
// sub-0.5 mm pin ticks that would violate the JLC3DP 0.8 mm rule; the chip is
// redrawn here from simple primitives so every stroke meets spec. Centred on
// the display X and positioned in the bezel between the display back edge
// and the back wall. Cut perpendicular to the tilted overlay top.
// Measured x-extent of "mKey" rendered in Liberation Sans Bold at cap-height
// 5.5 mm — obtained by extruding the text and reading the STL bounding box.
// Scales linearly with cap height. If deco_logo_text or deco_logo_font is
// changed, re-measure (openscad -o test.stl --summary all --summary-file -
// on a linear_extrude(1) text(...) test file) and update this constant.
DECO_LOGO_TEXT_X_EXTENT_REF = 20.76;
DECO_LOGO_TEXT_REF_SIZE     = 5.5;

module deco_logo_2d() {
    chip     = deco_logo_chip_size;
    border   = deco_logo_chip_border;
    inner_sq = deco_logo_chip_inner;
    gap      = deco_logo_text_gap;

    // True text width at the current cap height, scaled from the measured
    // reference extent. Used so chip icon + wordmark is visually centered on
    // the display X, not biased toward one side by a bad estimate.
    text_w  = deco_logo_text_size * DECO_LOGO_TEXT_X_EXTENT_REF / DECO_LOGO_TEXT_REF_SIZE;
    total_w = chip + gap + text_w;
    chip_x0 = -total_w / 2;
    text_x0 = chip_x0 + chip + gap;

    // Chip frame (hollow square outline)
    translate([chip_x0, -chip / 2])
        difference() {
            square([chip, chip]);
            translate([border, border])
                square([chip - 2 * border, chip - 2 * border]);
        }
    // Inner solid square (centered inside the chip frame)
    translate([chip_x0 + chip / 2 - inner_sq / 2, -inner_sq / 2])
        square([inner_sq, inner_sq]);

    // Wordmark
    translate([text_x0, 0])
        text(deco_logo_text,
             size = deco_logo_text_size,
             halign = "left", valign = "center",
             font = deco_logo_font);
}

module deco_top_logo() {
    disp_back_edge = plate_oy + disp_cy + disp_module_h / 2;
    bezel_back     = outer_d - wall_t;
    logo_y = (disp_back_edge + bezel_back) / 2;
    logo_x = plate_ox + disp_cx;

    translate([logo_x, logo_y, top_z(logo_y)])
        rotate([tilt_angle, 0, 0])
            translate([0, 0, -deco_cut_depth])
                linear_extrude(height = deco_cut_depth + 0.5)
                    deco_logo_2d();
}

// =============================================================================
// SECTION 11: COMPLETE CASE WITH FINISHING
// =============================================================================

module case_tray_finished() {
    difference() {
        case_tray();
        bottom_pad_recesses();

        // ─── Decorative trims (tray) ────────────────────────────────────────
        if (ENABLE_DECORATIVE_TRIMS) {
            if (DECO_SIDE_LOGO)      deco_side_logo();
            if (DECO_OWNER_INITIALS) deco_owner_initials();
        }
    }
}

module case_overlay_finished() {
    // No CAD chamfers — the rounded corners make simple planar cuts
    // produce artifacts at the corners. Chamfers/edge breaks will be
    // applied during finishing (sanding/routing) on the physical piece.
    difference() {
        case_overlay();

        // ─── Decorative trims (overlay) ─────────────────────────────────────
        if (ENABLE_DECORATIVE_TRIMS) {
            if (DECO_EDGE_PINSTRIPE) deco_edge_pinstripe();
            if (DECO_LOGO_TOP)       deco_top_logo();
        }
    }
}

// =============================================================================
// SECTION 11b: PRINT-MODE MODULES
// =============================================================================
// When PRINT_MODE is true, these wrappers re-orient the finished pieces for
// optimal 3D printing:
//   - Overlay: flattened (5° tilt removed), flipped cosmetic-face-down
//   - Tray: orientation unchanged (bottom already flat); optional breakaway
//     cross-braces added when PRINT_SUPPORTS is true

// ─── Print-oriented overlay ─────────────────────────────────────────────────
// Undo the 5° wedge tilt so the overlay lies perfectly flat, then flip it
// upside-down so the cosmetic top surface (key opening / display window)
// faces the build plate for the best surface finish on SLA and FDM.
//
// IMPORTANT: use rotate() not mirror() for the flip — mirror([0,0,1])
// reverses chirality, which inverts all debossed text and logos.
module case_overlay_print() {
    // After case_overlay_finished(), the overlay sits in design position:
    //   X: -overhang .. overlay_w - overhang
    //   Y: -overhang .. overlay_d - overhang
    //   Z: overlay_front_h - top_t .. overlay_back_h  (tilted 5°)
    //
    // Step 1: rotate -tilt_angle around X to flatten the top/bottom faces.
    // Step 2: rotate 180° around Y to flip cosmetic face down (preserves
    //         chirality — text/logos stay correct).  Negates both X and Z.
    // Step 3: translate so the piece sits on Z=0 with X,Y ≥ 0.

    // rotate([0,180,0]) maps (x,y,z) → (-x, y, -z): Z flips (cosmetic face
    // down) and X flips (compensated by the outer translate).  Y is unchanged
    // so the overhang shift stays the same as the design-position fix-up.
    translate([overlay_w - overhang, overhang, 0])
    rotate([0, 180, 0])
    translate([0, 0, -top_t])
    rotate([-tilt_angle, 0, 0])
    translate([0, 0, -(front_h - top_t)])
        case_overlay_finished();
}

// ─── Print-oriented tray ────────────────────────────────────────────────────
// The tray bottom is already at Z=0 (flat), so no rotation is needed.
// When PRINT_SUPPORTS is true, add internal vertical rib walls inside
// the cavity that grow simultaneously with the tray walls during the
// SLA build, providing anti-warp support from the first layer onward.
module case_tray_print() {
    case_tray_finished();
    if (PRINT_SUPPORTS) print_support_ribs();
}

// ─── Internal anti-warp rib walls ──────────────────────────────────────────
// Vertical rib walls inside the tray cavity, running front-to-back (Y).
// Unlike the old cross-braces (which sat on top of the walls and only
// existed in the final printed layers), these ribs grow SIMULTANEOUSLY
// with the tray walls during the SLA build — every Z layer that adds
// wall material also adds rib material, so anti-warp resistance is
// present from the very first wall layer.
//
// WHY they work: each rib creates a T-section where it meets the long
// front/back walls. A 363 mm × 4.8 mm wall alone has poor bowing
// resistance; adding a 1.5 mm perpendicular rib every ~70–88 mm
// turns each wall segment into a T-beam, dramatically increasing the
// second moment of area about the bowing axis. The ribs also link
// the front and back walls into a rigid diaphragm, resisting racking.
//
// Cross-section (plan view, looking down from +Z):
//
//   front wall ════════╤════════╤════════╤════════╤═══════ ← 4.8 mm wall
//                      │        │        │        │
//                      │ rib 1  │ rib 2  │ rib 3  │ rib 4  ← 1.5 mm ribs
//                      │        │        │        │
//   back wall  ════════╧════════╧════════╧════════╧═══════
//
// REMOVAL after post-cure: each rib is joined to the tray at three
// surfaces — front wall, back wall, and floor. Every junction gets a
// breakaway neck (rib thins from rib_t → rib_neck for rib_neck_len mm
// measured away from the junction) AND is perforated into short
// attachment segments with through-gaps between them, so cracks
// don't have to propagate more than one segment-length cleanly.
// Suggested sequence: score each exposed segment with a craft knife,
// snap, repeat along each of the three seams, then lightly sand the
// witness marks. The rib does NOT touch the wall tops (rib_top_gap)
// so the overlay mating surface is untouched.
//
rib_t        = 1.5;    // rib thickness (X direction). Thin enough to score
                       // and snap, thick enough to print on SLA (≥ JLC3DP
                       // 0.8 mm minimum). 1.5 mm also matches the tongue
                       // cross-section, so the rib is comfortably above
                       // the fabricable floor.
rib_top_gap  = 0.5;    // gap between rib top and wall top (mm). Keeps the
                       // rib below the overlay mating cheek so no witness
                       // marks land on the seam surface. Also makes the
                       // rib slightly easier to grip with pliers for
                       // snap-out.
rib_neck     = 1.0;    // breakaway neck thickness at front/back wall
                       // junctions (Y direction). For the first/last
                       // `rib_neck_len` mm the rib thins to this width,
                       // creating a weak line for clean snap-off.
                       // Grown from 0.8 → 1.0 in the 2026-04-16 JLC3DP review:
                       // 0.8 mm nominal worst-cased to 0.6 mm under ±0.2 mm
                       // SLA tolerance, below the 0.8 mm feature-minimum floor.
                       // 1.0 mm keeps worst-case at 0.8 mm while remaining
                       // hand-snappable with flush cutters.
rib_neck_len = 2.0;    // length of the thinned zone at each wall junction.

// Perforation of the breakaway necks. A continuous 100 mm floor neck
// is too long for an SLA crack to propagate cleanly — it tends to
// wander out of the notch line. Punching through-gaps along each
// neck breaks it into discrete short attachment segments, so any
// one crack only has to travel ~1 segment before hitting a free
// edge. Floor perf is aggressive (floor attachment doesn't carry
// the anti-warp load — the wall T-section does). Wall perf is
// deliberately minimal (a couple of gaps per wall) to preserve
// the T-section bracing during the SLA build.
floor_perf_count = 5;    // number of gaps along the floor neck
floor_perf_len   = 4.0;  // length of each floor gap (mm, Y direction)
wall_perf_count  = 2;    // number of gaps per wall neck
wall_perf_len    = 2.0;  // length of each wall gap (mm, Z direction)

// Rib X positions (case coords) — same slot-avoiding logic as before.
// Placed at the midpoints of the widest gaps between front-wall and
// back-wall gasket slot X-ranges so no rib crosses a slot channel.
//
// Slot X extents (case coords, each ±(tab_len+slot_tol)/2 = ±10.2 mm at
// slot_tol=0.6):
//   Front: [35.3, 55.7]  [132.6, 153.0]  [224.6, 245.0]  [309.6, 330.0]
//   Back:  [87.4, 107.8]  [170.6, 191.0]  [258.9, 279.3]
//
// Available gaps (sorted by X):
//   [55.7,  87.4]  → mid  71.5      [107.8, 132.6] → mid 120.2
//   [153.0, 170.6] → mid 161.8      [191.0, 224.6] → mid 207.8
//   [245.0, 258.9] → mid 252.0      [279.3, 309.6] → mid 294.5
//
// 6 ribs ensure all inter-rib spans stay under 75 mm (anti-warp guideline
// for 363 mm SLA parts). The 6th rib at x=161.8 closes the 87.6 mm gap
// that existed between ribs 2 and 3 in the old 5-rib layout. Current
// largest span is ~67 mm (left wall to rib 1).
rib_cx = [71.5, 120.2, 161.8, 207.8, 252.0, 294.5];

module print_support_ribs() {
    for (cx = rib_cx)
        print_support_rib(cx);
}

module print_support_rib(cx) {
    // The rib spans the full cavity depth (front inner wall to back inner
    // wall), growing from the floor up to just below the wall top.
    y0 = wall_t;                    // front inner wall face
    y1 = outer_d - wall_t;         // back inner wall face
    span = y1 - y0;

    // Rib heights at front and back (follows the tilt, with rib_top_gap
    // clearance below the wall top to avoid the overlay mating surface).
    z_floor   = bottom_t;
    z_top_f   = top_z(y0) - top_t - rib_top_gap;   // wall top at front − gap
    z_top_b   = top_z(y1) - top_t - rib_top_gap;   // wall top at back − gap
    h_front   = z_top_f - z_floor;
    h_back    = z_top_b - z_floor;

    // Perforation segment lengths: total available run divided among
    // (perf_count + 1) attachments separated by perf_count gaps.
    floor_seg_y  = (span    - floor_perf_count * floor_perf_len) / (floor_perf_count + 1);
    wall_seg_z_f = (h_front - wall_perf_count  * wall_perf_len)  / (wall_perf_count  + 1);
    wall_seg_z_b = (h_back  - wall_perf_count  * wall_perf_len)  / (wall_perf_count  + 1);

    difference() {
        // Full rib slab
        translate([cx - rib_t / 2, y0, z_floor])
            wedge_box(rib_t, span, h_front, h_back);

        // Breakaway neck at front wall: remove material from the rib
        // sides (X direction) for the first rib_neck_len mm, leaving
        // only the central rib_neck mm. This creates a thin web that
        // scores and snaps cleanly.
        for (side = [0, 1]) {
            neck_cut_w = (rib_t - rib_neck) / 2;
            translate([cx - rib_t / 2 + side * (rib_t - neck_cut_w) - 0.01 * (1 - side),
                       y0 - 0.01,
                       z_floor - 0.01])
                cube([neck_cut_w + 0.01,
                      rib_neck_len + 0.02,
                      h_front + 0.02]);
        }

        // Breakaway neck at back wall (same pattern)
        for (side = [0, 1]) {
            neck_cut_w = (rib_t - rib_neck) / 2;
            translate([cx - rib_t / 2 + side * (rib_t - neck_cut_w) - 0.01 * (1 - side),
                       y1 - rib_neck_len - 0.01,
                       z_floor - 0.01])
                cube([neck_cut_w + 0.01,
                      rib_neck_len + 0.02,
                      h_back + 0.02]);
        }

        // Breakaway neck at floor: thin the rib's bottom edge from rib_t
        // to rib_neck for the first rib_neck_len mm of Z. Without this,
        // the rib's full bottom edge (~100 mm) is fused to the floor and
        // cannot be snapped out without destroying the floor finish.
        // Spans the full Y length; overlaps harmlessly with the front/back
        // wall necks in the corners.
        for (side = [0, 1]) {
            neck_cut_w = (rib_t - rib_neck) / 2;
            translate([cx - rib_t / 2 + side * (rib_t - neck_cut_w) - 0.01 * (1 - side),
                       y0 - 0.01,
                       z_floor - 0.01])
                cube([neck_cut_w + 0.01,
                      span + 0.02,
                      rib_neck_len + 0.01]);
        }

        // Floor neck perforations: punch full-width through-gaps along
        // the floor neck so the continuous seam becomes discrete
        // attachment segments. Each gap removes the rib entirely at
        // floor level across floor_perf_len mm of Y.
        for (i = [1 : floor_perf_count])
            translate([cx - rib_t / 2 - 0.01,
                       y0 + i * floor_seg_y + (i - 1) * floor_perf_len,
                       z_floor - 0.01])
                cube([rib_t + 0.02, floor_perf_len, rib_neck_len + 0.01]);

        // Front wall neck perforations: Z-direction gaps splitting the
        // front wall attachment into (wall_perf_count+1) segments.
        for (i = [1 : wall_perf_count])
            translate([cx - rib_t / 2 - 0.01,
                       y0 - 0.01,
                       z_floor + i * wall_seg_z_f + (i - 1) * wall_perf_len])
                cube([rib_t + 0.02, rib_neck_len + 0.02, wall_perf_len]);

        // Back wall neck perforations (mirror of front)
        for (i = [1 : wall_perf_count])
            translate([cx - rib_t / 2 - 0.01,
                       y1 - rib_neck_len - 0.01,
                       z_floor + i * wall_seg_z_b + (i - 1) * wall_perf_len])
                cube([rib_t + 0.02, rib_neck_len + 0.02, wall_perf_len]);
    }
}

// =============================================================================
// SECTION 12: VISUALIZATION GHOSTS
// =============================================================================

module plate_ghost() {
    color("silver", 0.25) {
        // Plate follows the tilt, positioned on gasket ledges
        translate([plate_ox, plate_oy, 0])
        hull() {
            // Front edge
            translate([0, 0, plate_z(plate_oy) - plate_t])
                cube([plate_w, 0.01, plate_t]);
            // Back edge
            translate([0, plate_d, plate_z(plate_oy + plate_d) - plate_t])
                cube([plate_w, 0.01, plate_t]);
        }
    }
}

module display_ghost() {
    dcx = p2c_x(disp_cx);
    dcy = p2c_y(disp_cy);

    // Module rests in the tilted pocket with its top face pressed flat
    // against the shelf underside — i.e. local z ∈ [−shelf_t − glass_t, −shelf_t]
    // in the same tilted frame as display_cutout().
    translate([dcx, dcy, top_z(dcy)])
    rotate([tilt_angle, 0, 0])
    {
        color("black", 0.4)
        translate([-disp_module_w/2, -disp_module_h/2, -shelf_t - disp_glass_t])
            cube([disp_module_w, disp_module_h, disp_glass_t]);

        // Active area highlight (visible through the shelf window from above)
        color("cyan", 0.3)
        translate([-disp_active_w/2, -disp_active_h/2, -shelf_t + 0.01])
            cube([disp_active_w, disp_active_h, 0.1]);
    }
}

// =============================================================================
// SECTION 11: ASSEMBLY
// =============================================================================

module assembly() {
    if (SHOW_TRAY) {
        color("SaddleBrown", 0.85)
            if (PRINT_MODE) case_tray_print();
            else             case_tray_finished();
    }

    if (SHOW_OVERLAY) {
        color("SaddleBrown", 0.75)
            if (PRINT_MODE) case_overlay_print();
            else  translate([0, 0, EXPLODE])
                      case_overlay_finished();
    }

    if (SHOW_RETAINER) {
        // Backing clamp sits at the bottom of the (tilted) display pocket.
        // display_retainer() extrudes upward from its local z=0, so placing
        // that origin at local z = −top_t (overlay underside) puts the
        // retainer flush with the overlay bottom, occupying the bottom
        // retainer_t of the pocket.
        rcx = p2c_x(disp_cx);
        rcy = p2c_y(disp_cy);
        translate([rcx, rcy, top_z(rcy)])
        rotate([tilt_angle, 0, 0])
        color("Goldenrod", 0.9)
        translate([-disp_pocket_w / 2,
                   -disp_pocket_h / 2,
                   -top_t + EXPLODE * 0.5])
            display_retainer();
    }

    if (SHOW_PLATE)
        translate([0, 0, EXPLODE * 0.5])
            plate_ghost();

    if (SHOW_DISPLAY)
        translate([0, 0, EXPLODE * 0.5])
            display_ghost();
}

// ─── RENDER ──────────────────────────────────────────────────────────────────
if (SHOW_SECTION) {
    difference() {
        assembly();
        // Section cut: remove front half for inspection
        translate([-1, -1, -1])
            cube([outer_w + 2, outer_d / 2, back_h + 2]);
    }
} else {
    assembly();
}

// ─── Dimension echo (for verification) ──────────────────────────────────────
echo("=== CASE DIMENSIONS ===");
echo(str("Overlay outer: ", outer_w, " x ", outer_d, " mm"));
echo(str("Tray outer: ", outer_w + 2*tray_ext, " x ", outer_d + 2*tray_ext, " mm (extended by ", tray_ext, " mm per side)"));
echo(str("Height: ", front_h, "-", back_h, " mm (front-back)"));
echo(str("Tray wall: ", tray_wall_t, " mm, Overlay wall: ", overlay_wall_t, " mm, Bottom: ", bottom_t, " mm, Top surface: ", top_t, " mm"));
echo(str("Tray magnet outer cheek: ", tray_wall_t/2 - magnet_d/2, " mm (was ", overlay_wall_t/2 - magnet_d/2, " mm)"));
echo(str("Tilt rise: ", tilt_rise, " mm over ", outer_d, " mm depth"));
echo(str("Display bottom pocket: ", disp_pocket_w, " x ", disp_pocket_h, " mm (r=", disp_pocket_r, ", depth=", disp_pocket_d, ")"));
echo(str("Display top window: ", disp_win_w, " x ", disp_win_h, " mm (corner r=", shelf_corner_r, "), shelf X=", shelf_frame_x, " Y=", shelf_frame_y, " t=", shelf_t, " mm"));
echo(str("USB cutout: ", usb_cut_w, " x ", usb_cut_h, " mm at back wall"));
echo(str("Internal depth below plate: ", depth_below, " mm"));
echo(str("Plate recess below tray wall top: ", plate_recess, " mm"));
