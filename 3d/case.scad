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
SHOW_PLATE      = false;   // ghost plate for fit check
SHOW_DISPLAY    = false;   // ghost display for fit check
SHOW_SECTION    = false;   // cross-section cut for inspection
EXPLODE         = 0;       // set >0 to separate overlay from tray (mm)

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
tab_len = 20.0;       // each tab is 20mm along its edge

// Back edge tabs (3) - at plate_local Y = plate_d (back of keyboard)
// KiCad Y = -107.641 side; tabs extend toward more negative Y (further back)
back_tab_cx = [   // center X positions
    92.890 - 0.588,    // = 92.302
    176.130 - 0.588,   // = 175.542
    264.380 - 0.588    // = 263.792
];
back_tab_ext = 1.780;  // perpendicular extension beyond plate edge

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
// SAFETY-CRITICAL: close fit required
//
// Dimensions CORRECTED by safety officer from two independent web suppliers:
//   - lcdtftdisplays.com (1.72" 356x400 QSPI module spec)
//   - toppoplcd.com (TT172LMN10A datasheet)
// Cross-validated: module width 31.500mm matches plate cutout width EXACTLY.
// Module height 37.220mm matches plate cutout 37.200mm within 0.02mm.

disp_module_w   = 31.500;   // module outline width (matches plate cutout)
disp_module_h   = 37.220;   // module outline height
disp_active_w   = 29.100;   // viewable/active area width
disp_active_h   = 32.700;   // viewable/active area height
disp_active_ox  = 1.200;    // active area offset from module left edge (centered)
disp_active_oy  = 2.260;    // active area offset from module top edge (centered)
disp_glass_t    = 1.560;    // module + glass stack thickness

// Display cutout in the plate (from plate.kicad_pcb, VERIFIED by safety officer)
// KiCad bbox: X 305.846..337.346, Y -95.946..-58.746 (corner radius 2.200mm)
// Plate-local:
disp_cut_x1 = 305.846 - 0.588;   // 305.258  (left edge)
disp_cut_x2 = 337.346 - 0.588;   // 336.758  (right edge)
disp_cut_y1 = 58.746 - 7.416;    // 51.330   (front/closer to user)
disp_cut_y2 = 95.946 - 7.416;    // 88.530   (back/further from user)

// Display center in plate-local coords
disp_cx = (disp_cut_x1 + disp_cut_x2) / 2;   // ~320.962
disp_cy = (disp_cut_y1 + disp_cut_y2) / 2;    // ~69.884

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
plate_gap     = 0.3;    // clearance between plate edge and inner wall (per side)
disp_win_tol  = 0.15;   // display window oversize (per side)
disp_pkt_tol  = 0.15;   // display pocket oversize (per side)

// ─── Walls ───────────────────────────────────────────────────────────────────
wall_t    = 4.8;    // wall thickness = bezel (4.5mm visible + 0.3mm plate gap)
bottom_t  = 3.5;    // bottom plate thickness
top_t     = 3.0;    // top surface thickness (display cover area)

// ─── Internal ────────────────────────────────────────────────────────────────
plate_recess     = 1.5;    // plate top sits this far below case top
switch_depth     = 5.0;    // MX switch body below plate
pin_depth        = 3.3;    // switch pins below body
pcb_t            = 1.6;    // main PCB thickness
component_h      = 3.0;    // tallest component below PCB
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
// Row 2 center: plate_local_y = 50.084, Row 3 center: plate_local_y = 31.084
// Key cutouts are ~14mm tall, so:
//   Row 2 front edge ≈ 50.084 - 7.0 = 43.084
//   Row 3 back  edge ≈ 31.084 + 7.0 = 38.084
// Split between them:
y_split = 41.0;   // plate-local Y for the L-shape horizontal step

// Rightmost mechanical feature in rows 0-2: backspace stabilizer at plate X ≈ 285.9
// plate_local_x = 285.312
// With clearance:
x_split = 290.0;  // plate-local X where narrow section right edge is

// =============================================================================
// SECTION 5: DERIVED DIMENSIONS
// =============================================================================

// Total depth below plate surface
depth_below = switch_depth + pin_depth + pcb_t + component_h + bottom_clearance;
// = 5.0 + 3.3 + 1.6 + 3.0 + 2.0 = 14.9 mm

// Total internal height from case floor to case top
internal_h = plate_recess + plate_t + depth_below;  // = 18.0 mm

// Inner cavity dimensions (plate + clearance gap on each side)
inner_w = plate_w + 2 * plate_gap;  // = 353.238
inner_d = plate_d + 2 * plate_gap;  // = 100.825

// Case outer dimensions (uniform wall_t around the cavity)
outer_w = inner_w + 2 * wall_t;   // = 353.238 + 9.6 = 362.838
outer_d = inner_d + 2 * wall_t;   // = 100.825 + 9.6 = 110.425

// Case heights
front_h  = internal_h + bottom_t;                     // = 21.5 mm
tilt_rise = outer_d * tan(tilt_angle);                 // ≈ 9.62 mm
back_h   = front_h + tilt_rise;                        // ≈ 31.12 mm

// Plate origin in case coords (front-left corner of plate inner boundary)
plate_ox = wall_t;              // = 4.8
plate_oy = wall_t;              // = 4.8

// Display window dimensions
disp_win_w = disp_active_w + 2 * disp_win_tol;   // = 27.804
disp_win_h = disp_active_h + 2 * disp_win_tol;   // = 31.116

// Display pocket dimensions
disp_pkt_w = disp_module_w + 2 * disp_pkt_tol;   // = 32.208
disp_pkt_h = disp_module_h + 2 * disp_pkt_tol;   // = 39.000

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

// Z height of case top surface at a given case Y coordinate
function top_z(cy) = front_h + (back_h - front_h) * cy / outer_d;

// Z height of plate top surface at a given case Y coordinate
function plate_z(cy) = top_z(cy) - plate_recess;

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

// Rabbet dimensions: step cut into the inner top of each wall for the overlay
// to sit flush. The overlay rests in this rabbet.
rabbet_depth = wall_t - 2.0;   // 2.8mm inward, leaving 2mm outer lip
rabbet_height = top_t;          // 3mm, matches overlay thickness

// ─── PIECE 1: TRAY (bottom + walls) ─────────────────────────────────────────
module case_tray() {
    difference() {
        // Outer shell
        wedge_box(outer_w, outer_d, front_h, back_h);

        // Inner cavity (fully open top - no top surface)
        translate([wall_t, wall_t, bottom_t])
            wedge_box(inner_w, inner_d,
                      front_h + 10,
                      back_h  + 10);

        // Rabbet: step around the inner top of the walls for the overlay
        // The overlay sits in this step, flush with the wall tops.
        // Cut inward from the inner wall face, at the top of each wall.
        // Front wall rabbet
        translate([wall_t - 0.01, wall_t - rabbet_depth, 0])
            wedge_box(inner_w + 0.02, rabbet_depth + 0.01,
                      front_h - rabbet_height,
                      front_h - rabbet_height + (back_h - front_h) * (wall_t / outer_d));
        // Back wall rabbet
        translate([wall_t - 0.01, outer_d - wall_t, 0])
            wedge_box(inner_w + 0.02, rabbet_depth + 0.01,
                      back_h - rabbet_height - (back_h - front_h) * (wall_t / outer_d),
                      back_h - rabbet_height);
        // Left wall rabbet
        translate([wall_t - rabbet_depth, wall_t, 0])
            wedge_box(rabbet_depth + 0.01, inner_d,
                      front_h - rabbet_height,
                      back_h  - rabbet_height);
        // Right wall rabbet
        translate([outer_w - wall_t, wall_t, 0])
            wedge_box(rabbet_depth + 0.01, inner_d,
                      front_h - rabbet_height,
                      back_h  - rabbet_height);

        // Gasket tab slots (cut into the walls below the rabbet)
        gasket_slots();

        // USB-C cutout through back wall
        usb_cutout();
    }
}

// ─── PIECE 2: TOP OVERLAY ───────────────────────────────────────────────────
// Sits in the wall rabbet. Has the key opening and display features.
module case_overlay() {
    difference() {
        // Full top panel following the tilt, sized to fit in the rabbet
        // Top face flush with wall tops, bottom face rests on rabbet ledge
        translate([wall_t - rabbet_depth, wall_t - rabbet_depth, 0])
            wedge_box(inner_w + 2 * rabbet_depth,
                      inner_d + 2 * rabbet_depth,
                      front_h,
                      back_h);

        // Remove everything below the overlay (keep only top_t of material)
        translate([wall_t - rabbet_depth - 0.01,
                   wall_t - rabbet_depth - 0.01, 0])
            wedge_box(inner_w + 2 * rabbet_depth + 0.02,
                      inner_d + 2 * rabbet_depth + 0.02,
                      front_h - top_t,
                      back_h  - top_t);

        // L-shaped key opening (through the overlay)
        key_opening();

        // Display viewing window
        display_window();

        // Display module pocket (recessed into overlay underside)
        display_pocket();
    }
}

// ─── COMBINED: both pieces together ─────────────────────────────────────────
module case_complete() {
    case_tray();
    case_overlay();
}

// ─── 7c. Key opening ────────────────────────────────────────────────────────
// L-shaped cut: front section full width, back section narrower on right
module key_opening() {
    // Boundaries in case coords
    x1 = wall_t;                          // left
    x2 = outer_w - wall_t;               // right (full width)
    y1 = wall_t;                          // front
    y2 = outer_d - wall_t;               // back
    ys = p2c_y(y_split);                  // Y where the step occurs
    xs = p2c_x(x_split);                 // X right edge of narrow section

    // Cut volumes start at cavity floor to avoid cutting the bottom plate
    z0 = bottom_t - 0.01;
    zh = back_h + 2;  // extend well above top

    // Front section: full width, from y1 to ys
    translate([x1, y1, z0])
        cube([x2 - x1, ys - y1, zh]);

    // Back section: left portion only, from ys to y2
    translate([x1, ys, z0])
        cube([xs - x1, y2 - ys, zh]);
}

// ─── 7d. Display viewing window ─────────────────────────────────────────────
// Precisely sized to the display active area + tight tolerance.
// Window is centered on the ACTIVE AREA center, not the plate cutout center,
// to handle any asymmetry in the display module (e.g. FPC side has wider bezel).
module display_window() {
    // Active area center relative to module center:
    // Module center is at plate cutout center (module fills cutout)
    // Active area center offset from module center:
    aa_offset_x = disp_active_ox + disp_active_w/2 - disp_module_w/2;  // 0 if centered
    aa_offset_y = disp_active_oy + disp_active_h/2 - disp_module_h/2;  // 0 if centered

    // Window center in plate-local coords
    // Note: plate-local Y increases toward BACK, but display oy is from module TOP
    // (which is the BACK side when installed). So aa_offset_y maps directly.
    win_cx = disp_cx + aa_offset_x;
    win_cy = disp_cy - aa_offset_y;  // subtract: oy from top → negative = toward front

    wx = p2c_x(win_cx) - disp_win_w / 2;
    wy = p2c_y(win_cy) - disp_win_h / 2;

    // Cut through the top surface only (not the bottom panel)
    translate([wx, wy, bottom_t])
        cube([disp_win_w, disp_win_h, back_h]);
}

// ─── 7e. Display module pocket ──────────────────────────────────────────────
// Recessed pocket in the underside of the top surface to seat the display module
module display_pocket() {
    px = p2c_x(disp_cx) - disp_pkt_w / 2;
    py = p2c_y(disp_cy) - disp_pkt_h / 2;

    // The pocket needs to accommodate the display glass thickness
    // below the top surface. It must follow the tilt approximately.
    // Use a generous cube that covers the tilt variation.
    // Tilt across pocket height: pocket_h/outer_d * tilt_rise ≈ 3.4mm
    // So we cut from well below to the top surface.

    // Z of top surface underside at pocket center
    z_top_under = top_z(p2c_y(disp_cy)) - top_t;

    // Cut deep enough to seat the module at any Y across the pocket
    pocket_depth = disp_glass_t + 1.0;  // extra clearance for tilt + tolerance
    translate([px, py, z_top_under - pocket_depth])
        cube([disp_pkt_w, disp_pkt_h, pocket_depth + 0.1]);

    // FPC ribbon routing slot (exits toward the PCB connector)
    // The FPC exits from the display module bottom edge (front side in our coords)
    // Route toward the FPC connector on the PCB
    fpc_w = 12.0;
    fpc_x = p2c_x(disp_cx) - fpc_w / 2;
    fpc_y = py - 5;  // extends 5mm past the pocket front edge
    translate([fpc_x, fpc_y, z_top_under - pocket_depth - 1])
        cube([fpc_w, disp_pkt_h / 2 + 5, pocket_depth + 2]);
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
// Slot cross-section:
//   ════════════  ← case wall outer surface
//   ║          ║
//   ║  ┌────┐  ║  ← gasket (top)
//   ║  │ TAB│  ║  ← plate tab slides in here
//   ║  └────┘  ║  ← gasket (bottom)
//   ║          ║
//   ════════════  ← case wall inner surface (cavity side)

// Slot dimensions
slot_tol     = 0.3;    // clearance around tab (per side, length direction)
slot_depth   = 2.5;    // how deep the slot goes into the wall
slot_height  = plate_t + 2 * gasket_compressed + 0.5;  // tab + gasket + clearance
// = 1.6 + 3.0 + 0.5 = 5.1mm

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
    {
        cy = p2c_y(left_tab_cy);
        wall_inner_x = wall_t;  // inner face of left wall
        local_pz = plate_z(cy);
        slot_center_z = local_pz - plate_t / 2;

        translate([wall_inner_x - slot_depth,
                   cy - (tab_len + slot_tol)/2,
                   slot_center_z - slot_height/2])
            cube([slot_depth + 0.01, tab_len + slot_tol, slot_height]);
    }

    // ── Right wall slot (1) ──────────────────────────────────────────────
    {
        cy = p2c_y(right_tab_cy);
        wall_inner_x = outer_w - wall_t;  // inner face of right wall
        local_pz = plate_z(cy);
        slot_center_z = local_pz - plate_t / 2;

        translate([wall_inner_x - 0.01,
                   cy - (tab_len + slot_tol)/2,
                   slot_center_z - slot_height/2])
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
    difference() {
        case_overlay();
        // Top edge chamfers apply to overlay
        // Front top edge
        translate([-0.1, -0.1, front_h])
            rotate([45, 0, 0])
                cube([outer_w + 0.2, chamfer * 1.42, chamfer * 1.42]);
        // Back top edge
        translate([-0.1, outer_d + 0.1, back_h])
            rotate([45, 0, 0])
                translate([0, -chamfer * 1.42, 0])
                    cube([outer_w + 0.2, chamfer * 1.42, chamfer * 1.42]);
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
    z_top_under = top_z(dcy) - top_t;

    // Module body
    color("black", 0.4)
    translate([dcx - disp_module_w/2, dcy - disp_module_h/2,
               z_top_under - disp_glass_t])
        cube([disp_module_w, disp_module_h, disp_glass_t]);

    // Active area highlight
    color("cyan", 0.3)
    translate([dcx - disp_active_w/2, dcy - disp_active_h/2,
               z_top_under + 0.01])
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

// ─── Dimension echo (uncomment for verification) ────────────────────────────
echo("=== CASE DIMENSIONS ===");
echo(str("Outer: ", outer_w, " x ", outer_d, " x ", front_h, "-", back_h, " mm"));
echo(str("Wall: ", wall_t, " mm, Bottom: ", bottom_t, " mm, Top surface: ", top_t, " mm"));
echo(str("Tilt rise: ", tilt_rise, " mm over ", outer_d, " mm depth"));
echo(str("Display window: ", disp_win_w, " x ", disp_win_h, " mm"));
echo(str("Display pocket: ", disp_pkt_w, " x ", disp_pkt_h, " mm"));
echo(str("USB cutout: ", usb_cut_w, " x ", usb_cut_h, " mm at back wall"));
echo(str("Internal depth below plate: ", depth_below, " mm"));
