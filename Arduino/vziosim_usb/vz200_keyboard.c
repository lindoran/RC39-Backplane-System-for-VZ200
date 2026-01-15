#include "vz200_keyboard.h"

// ---------------------------------------------------------------------------
// Matrix storage
// ---------------------------------------------------------------------------

uint8_t vz_matrix[8];

// ---------------------------------------------------------------------------
// PC scan code → VZ (row, bit) lookup table
// Unused entries are {255,255}
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t row;   // 0–7
    uint8_t bit;   // 0–5
} vz_key_t;

static vz_key_t pc_to_vz[256];

// ---------------------------------------------------------------------------
// Helper: assign mapping
// ---------------------------------------------------------------------------

static inline void map(uint8_t sc, uint8_t row, uint8_t bit) {
    pc_to_vz[sc].row = row;
    pc_to_vz[sc].bit = bit;
}

// ---------------------------------------------------------------------------
// Initialize matrix and mapping table
// ---------------------------------------------------------------------------

void vz_keyboard_init(void)
{
    // All keys released (active-high)
    for (int i = 0; i < 8; i++)
        vz_matrix[i] = 0b00111111;

    // Initialize lookup table to "unused"
    for (int i = 0; i < 256; i++) {
        pc_to_vz[i].row = 255;
        pc_to_vz[i].bit = 255;
    }

    // -----------------------------------------------------------------------
    // VZ-200 matrix mapping (row, bit)
    // -----------------------------------------------------------------------
    // Row 0xFE → index 0
    map(0x15, 0, 5);   // R
    map(0x14, 0, 4);   // Q
    map(0x08, 0, 3);   // E
    // NC at bit 2
    map(0x1A, 0, 1);   // W
    map(0x17, 0, 0);   // T

    // Row 0xFD → index 1
    map(0x09, 1, 5);   // F
    map(0x04, 1, 4);   // A
    map(0x07, 1, 3);   // D
    // CTRL at bit 2
    map(0x16, 1, 1);   // S
    map(0x0A, 1, 0);   // G

    // Row 0xFB → index 2
    map(0x19, 2, 5);   // V
    map(0x1D, 2, 4);   // Z
    map(0x06, 2, 3);   // C
    // SHIFT at bit 2
    map(0x1B, 2, 1);   // X
    map(0x05, 2, 0);   // B

    // Row 0xF7 → index 3
    map(0x21, 3, 5);   // 4
    map(0x1E, 3, 4);   // 1
    map(0x20, 3, 3);   // 3
    // NC at bit 2
    map(0x1F, 3, 1);   // 2
    map(0x22, 3, 0);   // 5

    // Row 0xEF → index 4
    map(0x10, 4, 5);   // M
    map(0x2C, 4, 4);   // SPACE
    map(0x36, 4, 3);   // ,
    // NC at bit 2
    map(0x37, 4, 1);   // .
    map(0x11, 4, 0);   // N

    // Row 0xDF → index 5
    map(0x24, 5, 5);   // 7
    map(0x27, 5, 4);   // 0
    map(0x25, 5, 3);   // 8
    map(0x2D, 5, 2);   // -
    map(0x26, 5, 1);   // 9
    map(0x23, 5, 0);   // 6

    // Row 0xBF → index 6
    map(0x18, 6, 5);   // U
    map(0x13, 6, 4);   // P
    map(0x0C, 6, 3);   // I
    map(0x28, 6, 2);   // RETURN
    map(0x12, 6, 1);   // O
    map(0x1C, 6, 0);   // Y

    // Row 0x7F → index 7
    map(0x0D, 7, 5);   // J
    map(0x33, 7, 4);   // ;
    map(0x0E, 7, 3);   // K
    map(0x34, 7, 2);   // : (apostrophe key)
    map(0x0F, 7, 1);   // L
    map(0x0B, 7, 0);   // H

    // -----------------------------------------------------------------------
    // Modifier keys (map to matrix keys)
    // Use HID modifier usage codes (0xE0 - 0xE7) to avoid collisions with
    // normal key usage codes that appear in the 6-key report array.
    // -----------------------------------------------------------------------
    map(0xE1, 2, 2);   // LSHIFT (HID 0xE1) → SHIFT
    map(0xE5, 2, 2);   // RSHIFT (HID 0xE5) → SHIFT
    map(0xE0, 1, 2);   // LCTRL  (HID 0xE0) → CTRL
    map(0xE4, 1, 2);   // RCTRL  (HID 0xE4) → CTRL
}

// ---------------------------------------------------------------------------
// Key press: set bit LOW
// ---------------------------------------------------------------------------

void vz_key_press(uint8_t scancode)
{
    vz_key_t k = pc_to_vz[scancode];
    if (k.row < 8 && k.bit < 6)
        vz_matrix[k.row] &= ~(1 << k.bit);
}

// ---------------------------------------------------------------------------
// Key release: set bit HIGH
// ---------------------------------------------------------------------------

void vz_key_release(uint8_t scancode)
{
    vz_key_t k = pc_to_vz[scancode];
    if (k.row < 8 && k.bit < 6)
        vz_matrix[k.row] |= (1 << k.bit);
}

// ---------------------------------------------------------------------------
// Helper: whether a given PC scancode is mapped to a VZ matrix position
// ---------------------------------------------------------------------------

bool vz_scancode_mapped(uint8_t scancode)
{
    vz_key_t k = pc_to_vz[scancode];
    return (k.row < 8 && k.bit < 6);
}

// ---------------------------------------------------------------------------
// Read row selected by VZ-200 (active-low address)
// ---------------------------------------------------------------------------

uint8_t vz_read_row(uint8_t row_address)
{
    uint8_t index = (~row_address) & 0x07;
    return vz_matrix[index];
}
