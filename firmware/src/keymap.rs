/// Key definition: (modifier_bits, keycode)
/// Regular keys: (0, hid_keycode), Modifiers: (mod_bit, 0), Unused: (0, 0)
pub type KeyDef = (u8, u8);

const fn k(code: u8) -> KeyDef {
    (0, code)
}
const fn m(modifier: u8) -> KeyDef {
    (modifier, 0)
}
const __: KeyDef = (0, 0);

/// Keymap for 5×14 matrix — UK ISO layout
/// HID Usage IDs: https://usb.org/sites/default/files/hut1_4.pdf (Chapter 10)
#[rustfmt::skip]
pub const KEYMAP: [[KeyDef; NUM_COLS]; NUM_ROWS] = [
    // Row 0: Esc     1       2       3       4       5       6       7       8       9       0       -       =       Bksp
    [k(0x29), k(0x1E), k(0x1F), k(0x20), k(0x21), k(0x22), k(0x23), k(0x24), k(0x25), k(0x26), k(0x27), k(0x2D), k(0x2E), k(0x2A)],
    // Row 1: Tab     Q       W       E       R       T       Y       U       I       O       P       [{      ]}      Enter
    [k(0x2B), k(0x14), k(0x1A), k(0x08), k(0x15), k(0x17), k(0x1C), k(0x18), k(0x0C), k(0x12), k(0x13), k(0x2F), k(0x30), k(0x28)],
    // Row 2: Caps    A       S       D       F       G       H       J       K       L       ;:      '@      #~      (no switch)
    [k(0x39), k(0x04), k(0x16), k(0x07), k(0x09), k(0x0A), k(0x0B), k(0x0D), k(0x0E), k(0x0F), k(0x33), k(0x34), k(0x32), __     ],
    // Row 3: LShift  \|      Z       X       C       V       B       N       M       ,<      .>      /?      RShift  Up
    [m(0x02), k(0x64), k(0x1D), k(0x1B), k(0x06), k(0x19), k(0x05), k(0x11), k(0x10), k(0x36), k(0x37), k(0x38), m(0x20), k(0x52)],
    // Row 4: LCtrl   LGui    LAlt    --      Space   --      --      RAlt    Fn      RGui    RCtrl   Left    Down    Right
    [m(0x01), m(0x08), m(0x04), __,     k(0x2C), __,     __,     m(0x40), __,     m(0x80), m(0x10), k(0x50), k(0x51), k(0x4F)],
];

pub const NUM_ROWS: usize = 5;
pub const NUM_COLS: usize = 14;
pub const DEBOUNCE_SCANS: u8 = 5;
