// =============================================================================
// mKey Keyboard Case - OpenSCAD Design
// =============================================================================
// Premium darkwood case for the mKey 65% mechanical keyboard.
// Design: wedge profile, gasket-mounted plate, integrated display window.
// Style: sleek defined lines, minimal bezel, premium feel.
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
disp_win_tol  = 0.15;   // retainer active-area window oversize (per side)
disp_cut_tol  = 0.15;   // overlay display through-cut oversize vs module (per side)
retainer_t    = 1.2;    // thickness of the separately-fabricated display retainer

// ─── Walls ───────────────────────────────────────────────────────────────────
wall_t    = 4.8;    // wall thickness = bezel (4.5mm visible + 0.3mm plate gap)
bottom_t  = 3.5;    // bottom plate thickness
top_t     = 3.0;    // top surface thickness (display cover area)

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
overlay_corner_r = 3.0;    // outer corner radius of overlay

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

// Overlay display through-cut dimensions (module outline + clearance)
disp_cut_w = disp_module_w + 2 * disp_cut_tol;
disp_cut_h = disp_module_h + 2 * disp_cut_tol;
disp_cut_r = disp_module_r + disp_cut_tol;

// Retainer active-area window dimensions
disp_win_w = disp_active_w + 2 * disp_win_tol;
disp_win_h = disp_active_h + 2 * disp_win_tol;

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

// Wall step: top top_t of all walls is removed (full width, no lip).
// The overlay sits flush on the shortened wall tops and overhangs on all sides.
// Plate and gaskets provide lateral registration.
rabbet_depth = wall_t;          // full wall width — no lip
rabbet_height = top_t;          // 3mm, matches overlay thickness

// ─── PIECE 1: TRAY (bottom + walls) ─────────────────────────────────────────
module case_tray() {
    difference() {
        // Outer shell — shortened by top_t so overlay sits flush on wall tops
        wedge_box(outer_w, outer_d, front_h - top_t, back_h - top_t);

        // Inner cavity (fully open top)
        translate([wall_t, wall_t, bottom_t])
            wedge_box(inner_w, inner_d,
                      front_h + 10,
                      back_h  + 10);

        // Gasket tab slots (cut into the walls)
        gasket_slots();

        // USB-C cutout through back wall
        usb_cutout();
    }
}

// ─── PIECE 2: TOP OVERLAY ───────────────────────────────────────────────────
// Overhangs the tray walls. Sits in the wall rabbet for registration.
// Has the key opening and display features.
module case_overlay() {
    difference() {
        // Full overlay with overhang and rounded corners
        translate([-overhang, -overhang, 0])
            rounded_wedge_box(overlay_w, overlay_d,
                              overlay_front_h, overlay_back_h,
                              overlay_corner_r);

        // Remove everything below top_t (keep only the top slab)
        translate([-overhang - 0.01, -overhang - 0.01, 0])
            rounded_wedge_box(overlay_w + 0.02, overlay_d + 0.02,
                              overlay_front_h - top_t,
                              overlay_back_h  - top_t,
                              overlay_corner_r + 0.01);

        // Note: the overlay and tray outer wall lip occupy the same Z range
        // where they meet. This is correct — they nest together physically
        // (overlay sits in the rabbet, lip provides lateral registration).
        // The CAD overlap is intentional and represents the assembly fit.

        // Key opening: union of per-keycap rectangles (plate hidden everywhere
        // except where a key needs to pass through)
        key_opening();

        // Display through-cut sized to the module outline. The active-area
        // window lip is provided by a separately-fabricated `display_retainer`
        // piece glued to the overlay underside (see SHOW_RETAINER and
        // display_retainer module below). This replaces the old blind pocket
        // + 1.2 mm hardwood lip, which could not be hand-cut and would crack.
        display_cutout();
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
module key_opening() {
    z0 = bottom_t - 0.01;
    zh = back_h + 2;
    t  = key_cap_clearance;

    for (r = key_rects) {
        x1 = r[0] - t;  y1 = r[1] - t;
        x2 = r[2] + t;  y2 = r[3] + t;
        translate([p2c_x(x1), p2c_y(y1), z0])
            cube([x2 - x1, y2 - y1, zh]);
    }
}


// ─── 7d. Display through-cut ────────────────────────────────────────────────
// Single rectangular through-cut in the overlay, sized to the display module
// outline + clearance. The module seats here from above, resting on the
// retainer lip from below; its 24-pin FPC ribbon drops down through the
// plate's display cutout to J2 on the PCB. The module body does NOT pass
// through the plate — only the ribbon. Four corner drill holes + four
// straight saw cuts are all that's needed to make this by hand. The
// active-area window and the 1.2 mm lip holding the display glass are
// provided by the separately-cut `display_retainer` piece (see below),
// glued to the overlay underside.
module display_cutout() {
    wcx = p2c_x(disp_cx);
    wcy = p2c_y(disp_cy);
    wx  = wcx - disp_cut_w / 2;
    wy  = wcy - disp_cut_h / 2;

    translate([wx, wy, bottom_t])
        linear_extrude(height = back_h + 1)
            offset(r=disp_cut_r) offset(delta=-disp_cut_r)
                square([disp_cut_w, disp_cut_h]);
}

// ─── 7e. Display retainer (separately fabricated) ───────────────────────────
// Flat piece (e.g. 1.2 mm plywood, brass, hardwood veneer, or 3D-printed)
// whose outer outline matches the overlay's display through-cut and whose
// inner window matches the display active area. Glued to the overlay
// underside around the display hole. The retainer provides:
//   • the visible active-area bezel ("picture frame")
//   • the mechanical lip the display glass rests against
//   • cross-grain strength if made from plywood/brass (wood overlay alone
//     would be short-grain at 1.2 mm and crack)
// Produced as a standalone piece when SHOW_RETAINER is true.
module display_retainer() {
    ow = disp_cut_w;          // matches overlay cutout
    oh = disp_cut_h;
    or_ = disp_cut_r;
    iw = disp_win_w;          // active-area + window tolerance
    ih = disp_win_h;
    ir = disp_active_r;

    difference() {
        linear_extrude(height = retainer_t)
            offset(r=or_) offset(delta=-or_)
                square([ow, oh]);
        translate([(ow - iw) / 2, (oh - ih) / 2, -0.1])
            linear_extrude(height = retainer_t + 0.2)
                offset(r=ir) offset(delta=-ir)
                    square([iw, ih]);
    }
}

// ─── 7f. USB-C cutout ───────────────────────────────────────────────────────
module usb_cutout() {
    // USB exits through the back wall
    ux = p2c_x(usb_plate_x) - usb_cut_w / 2;

    // USB-C mid-mount connector sits on the PCB surface.
    // PCB top = plate bottom - switch body depth
    // USB opening center ≈ PCB center line
    back_plate_z = plate_z(outer_d - wall_t) - plate_t;
    pcb_top_z = back_plate_z - switch_depth;
    usb_z_center = pcb_top_z - pcb_t / 2;  // center of PCB thickness

    translate([ux, outer_d - wall_t - 0.5, usb_z_center - usb_cut_h / 2])
        cube([usb_cut_w, wall_t + 1, usb_cut_h]);
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

// Compile-time check: slot_height must be large enough to reach above the
// tray wall top. If this fires, increase slot_open_margin or reduce
// plate_recess.
assert(slot_height >= 2 * plate_recess + plate_t + 2 * slot_open_margin,
       "slot_height too small: gasket slot does not reach the tray wall top — plate cannot drop in");

// Slot dimensions
slot_tol     = 0.3;    // clearance around tab (per side, length direction)
slot_depth   = 2.5;    // how deep the slot goes into the wall
slot_open_margin = 0.5;    // mm slot top extends ABOVE the tray wall top
                           // (turns each slot into an open-top channel so
                           //  the plate can drop straight in from above —
                           //  otherwise the slot would be sealed at the top
                           //  by wall material and the tab couldn't enter)
// slot_height is sized for BOTH constraints:
//   1. symmetric headroom around the plate tab + two gaskets (old need)
//        = plate_t + 2 * gasket_compressed + 0.5
//   2. drop-in access: slot top reaches wall_top + slot_open_margin, which
//      (since the slot is centered on plate midline at plate_z − plate_t/2)
//      requires slot_height ≥ 2·plate_recess + plate_t + 2·slot_open_margin
// With current values the drop-in constraint dominates:
//   2·2.0 + 1.6 + 2·0.5 = 6.6 mm  vs  1.6 + 3.0 + 0.5 = 5.1 mm
slot_height  = max(plate_t + 2 * gasket_compressed + 0.5,
                   2 * plate_recess + plate_t + 2 * slot_open_margin);

module gasket_slots() {
    // ── Back wall slots (3) ──────────────────────────────────────────────
    // Slots cut into the back wall from the inner face toward the outer face.
    // Tab enters from the cavity side.
    for (i = [0:2]) {
        cx = p2c_x(back_tab_cx[i]);
        wall_inner_y = outer_d - wall_t;  // inner face of back wall
        local_pz = plate_z(wall_inner_y);  // plate top Z at this Y
        slot_center_z = local_pz - plate_t / 2;  // center on plate mid-thickness

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
        slot_center_z = local_pz - plate_t / 2;

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
        slot_center_z_L = plate_z(p2c_y(left_tab_cy)) - plate_t / 2) {
        translate([wall_inner_x_L - slot_depth,
                   cy_L - (tab_len + slot_tol)/2,
                   slot_center_z_L - slot_height/2])
            cube([slot_depth + 0.01, tab_len + slot_tol, slot_height]);
    }

    // ── Right wall slot (1) ──────────────────────────────────────────────
    let(cy_R            = p2c_y(right_tab_cy),
        wall_inner_x_R  = outer_w - wall_t,
        local_pz_R      = plate_z(p2c_y(right_tab_cy)),
        slot_center_z_R = plate_z(p2c_y(right_tab_cy)) - plate_t / 2) {
        translate([wall_inner_x_R - 0.01,
                   cy_R - (tab_len + slot_tol)/2,
                   slot_center_z_R - slot_height/2])
            cube([slot_depth + 0.01, tab_len + slot_tol, slot_height]);
    }
}

// =============================================================================
// SECTION 10: EDGE CHAMFERS & BOTTOM FEATURES
// =============================================================================
// Premium chamfers on all top outer edges. These will be rounded/sanded
// after CNC or hand cutting for a refined hardwood finish.

chamfer = 1.0;   // 45° chamfer width (mm)

module edge_chamfers() {
    // Top edge chamfers: 45° cuts along all four top edges of the case
    // These follow the tilt on the back and sides

    // Front top edge (horizontal, at Y=0, Z=front_h)
    translate([-0.1, -0.1, front_h - chamfer])
        rotate([0, 0, 0])
            linear_extrude(height = chamfer + 0.1, scale = [1, 0])
                square([outer_w + 0.2, chamfer + 0.1]);

    // Simplified: cut prisms along each top edge
    // Front edge chamfer
    translate([-0.1, -0.1, front_h])
        rotate([45, 0, 0])
            cube([outer_w + 0.2, chamfer * 1.42, chamfer * 1.42]);

    // Back edge chamfer (at higher Z due to tilt)
    translate([-0.1, outer_d + 0.1, back_h])
        rotate([45, 0, 0])
            translate([0, -chamfer * 1.42, 0])
                cube([outer_w + 0.2, chamfer * 1.42, chamfer * 1.42]);

    // Left edge chamfer (follows tilt from front to back)
    translate([-0.1, 0, 0])
    hull() {
        translate([0, -0.1, front_h])
            rotate([0, -45, 0])
                cube([chamfer * 1.42, 0.01, chamfer * 1.42]);
        translate([0, outer_d + 0.1, back_h])
            rotate([0, -45, 0])
                cube([chamfer * 1.42, 0.01, chamfer * 1.42]);
    }

    // Right edge chamfer (follows tilt)
    translate([outer_w + 0.1, 0, 0])
    hull() {
        translate([0, -0.1, front_h])
            rotate([0, 45, 0])
                translate([-chamfer * 1.42, 0, 0])
                    cube([chamfer * 1.42, 0.01, chamfer * 1.42]);
        translate([0, outer_d + 0.1, back_h])
            rotate([0, 45, 0])
                translate([-chamfer * 1.42, 0, 0])
                    cube([chamfer * 1.42, 0.01, chamfer * 1.42]);
    }

    // Bottom edge chamfers (subtle, 0.5mm)
    bc = 0.5;
    // Front bottom
    translate([-0.1, -0.1, -0.1])
        rotate([-45, 0, 0])
            cube([outer_w + 0.2, bc * 1.42, bc * 1.42]);
    // Back bottom
    translate([-0.1, outer_d + 0.1, -0.1])
        rotate([-45, 0, 0])
            translate([0, -bc * 1.42, 0])
                cube([outer_w + 0.2, bc * 1.42, bc * 1.42]);
}

// Bottom rubber pad recesses (for anti-slip, no legs per spec)
pad_d   = 1.0;     // recess depth
pad_w   = 30.0;    // pad width
pad_h   = 10.0;    // pad length (front-back)
pad_inset = 15.0;  // inset from edges

module bottom_pad_recesses() {
    // Four pad recesses near the corners
    positions = [
        [pad_inset, pad_inset],                                    // front-left
        [outer_w - pad_inset - pad_w, pad_inset],                  // front-right
        [pad_inset, outer_d - pad_inset - pad_h],                  // back-left
        [outer_w - pad_inset - pad_w, outer_d - pad_inset - pad_h] // back-right
    ];

    for (pos = positions) {
        translate([pos[0], pos[1], -0.01])
            cube([pad_w, pad_h, pad_d + 0.01]);
    }
}

// =============================================================================
// SECTION 11: COMPLETE CASE WITH FINISHING
// =============================================================================

module case_tray_finished() {
    difference() {
        case_tray();
        // Bottom chamfers and pad recesses apply to tray only
        bc = 0.5;
        // Front bottom chamfer
        translate([-0.1, -0.1, -0.1])
            rotate([-45, 0, 0])
                cube([outer_w + 0.2, bc * 1.42, bc * 1.42]);
        // Back bottom chamfer
        translate([-0.1, outer_d + 0.1, -0.1])
            rotate([-45, 0, 0])
                translate([0, -bc * 1.42, 0])
                    cube([outer_w + 0.2, bc * 1.42, bc * 1.42]);
        bottom_pad_recesses();
    }
}

module case_overlay_finished() {
    // No CAD chamfers — the rounded corners make simple planar cuts
    // produce artifacts at the corners. Chamfers/edge breaks will be
    // applied during finishing (sanding/routing) on the physical piece.
    case_overlay();
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
    overlay_bottom_z = top_z(dcy) - top_t;

    // Module body sits inside the overlay through-cut, bottom face resting on
    // the retainer (which is glued to the overlay underside). Glass top ends
    // up (top_t - glass_t) below the overlay top surface — a slightly
    // recessed display. With top_t=3 and glass_t=1.56 that's ~1.44 mm recess.
    module_bottom_z = overlay_bottom_z;
    module_top_z = overlay_bottom_z + disp_glass_t;

    color("black", 0.4)
    translate([dcx - disp_module_w/2, dcy - disp_module_h/2, module_bottom_z])
        cube([disp_module_w, disp_module_h, disp_glass_t]);

    // Active area highlight (visible through overlay cutout from above)
    color("cyan", 0.3)
    translate([dcx - disp_active_w/2, dcy - disp_active_h/2,
               module_top_z + 0.01])
        cube([disp_active_w, disp_active_h, 0.1]);
}

// =============================================================================
// SECTION 11: ASSEMBLY
// =============================================================================

module assembly() {
    if (SHOW_TRAY) {
        color("SaddleBrown", 0.85)
            case_tray_finished();
    }

    if (SHOW_OVERLAY) {
        color("SaddleBrown", 0.75)
        translate([0, 0, EXPLODE])
            case_overlay_finished();
    }

    if (SHOW_RETAINER) {
        // Retainer sits against the overlay underside, around the display hole.
        rcx = p2c_x(disp_cx);
        rcy = p2c_y(disp_cy);
        color("Goldenrod", 0.9)
        translate([rcx - disp_cut_w / 2,
                   rcy - disp_cut_h / 2,
                   top_z(rcy) - top_t - retainer_t + EXPLODE * 0.5])
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
echo(str("Outer: ", outer_w, " x ", outer_d, " x ", front_h, "-", back_h, " mm"));
echo(str("Wall: ", wall_t, " mm, Bottom: ", bottom_t, " mm, Top surface: ", top_t, " mm"));
echo(str("Tilt rise: ", tilt_rise, " mm over ", outer_d, " mm depth"));
echo(str("Overlay display through-cut: ", disp_cut_w, " x ", disp_cut_h, " mm (r=", disp_cut_r, ")"));
echo(str("Retainer outer: ", disp_cut_w, " x ", disp_cut_h, " mm, window: ", disp_win_w, " x ", disp_win_h, " mm, t=", retainer_t, " mm"));
echo(str("USB cutout: ", usb_cut_w, " x ", usb_cut_h, " mm at back wall"));
echo(str("Internal depth below plate: ", depth_below, " mm"));
echo(str("Plate recess below tray wall top: ", plate_recess, " mm"));
