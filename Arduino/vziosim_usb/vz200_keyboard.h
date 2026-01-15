#ifndef VZ200_KEYBOARD_H
#define VZ200_KEYBOARD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// VZ-200 keyboard matrix
// 8 rows × 6 columns, active-low
// vz_matrix[row] bit 0–5 = key state (1=released, 0=pressed)
// bits 6–7 are unused and must remain 1
// ---------------------------------------------------------------------------

extern uint8_t vz_matrix[8];

// Initialize matrix (all keys released)
void vz_keyboard_init(void);

// Handle PC USB scan code press
void vz_key_press(uint8_t scancode);

// Handle PC USB scan code release
void vz_key_release(uint8_t scancode);

// Query whether a given PC scancode is mapped to the VZ matrix
bool vz_scancode_mapped(uint8_t scancode);

// Translate VZ row address (active-low) to matrix byte
uint8_t vz_read_row(uint8_t row_address);

#ifdef __cplusplus
}
#endif

#endif
