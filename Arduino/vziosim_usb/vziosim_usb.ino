/*
 * vziosim_usb.ino
 *
 * Arduino pico-core sketch that converts USB keyboard input to VZ-200
 * keyboard matrix output via PIO-based Z80 bus interface.
 *
 * Core 0: USB Host keyboard processing
 * Core 1: PIO-based Z80 bus timing (critical path)
 *
 * VZ200 Key Matrix
 * ----------------
 * The key matrix is minimally decoded and address bits 0-7 are used to decode
 * what row we are looking at.
 *
 * NC = no key is assigned
 * CTRL = CTRL
 * SHIFT = SHIFT
 * RETRN = RETURN
 *
 *                             5    4    3    2    1    0   (Bit Position)
 * ROW ADDRESS (LSB)   0xFE    R    Q    E    NC   W    T
 *                     0xFD    F    A    D   CTRL  S    G
 *                     0xFB    V    Z    C  SHIFT  X    B
 *                     0xF7    4    1    3    NC   2    5
 *                     0xEF    M   SPC   ,    NC   .    N
 *                     0xDF    7    0    8    -    9    6
 *                     0xBF    U    P    I  RETRN  O    Y
 *                     0x7F    J    ;    K    :    L    H
 */

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include "pico/stdlib.h"
#include "pio_vz_kbd.pio.h"
#include <SerialPIO.h>
#include <string.h>
#include <atomic>
#include "Adafruit_TinyUSB.h"
#include "vz200_keyboard.h"

// -----------------------------------------------------------------------------
// Compile-time configuration checks
// -----------------------------------------------------------------------------
#ifndef USE_TINYUSB_HOST
  #error This example requires usb stack configured as host in "Tools -> USB Stack -> Adafruit TinyUSB Host"
#endif

// -----------------------------------------------------------------------------
// Pin Definitions
// -----------------------------------------------------------------------------

/* Z80 Address Lines (Pins 0-7) */
#define Z80_A0          0
#define Z80_A1          1
#define Z80_A2          2
#define Z80_A3          3
#define Z80_A4          4
#define Z80_A5          5
#define Z80_A6          6
#define Z80_A7          7

/* Z80 Data Port (Pins 8-13) - D0-D5 only */
#define Z80_D0          8
#define Z80_D1          9
#define Z80_D2          10
#define Z80_D3          11
#define Z80_D4          12
#define Z80_D5          13

/* Z80 Control Bus */
#define Z80_VZRD        14

/* Serial Debug Pins */
#define TTL_TX          15
#define TTL_RX          26

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------
#define MAX_HID_IFACES 4

#define ROW0            0xFE
#define ROW1            0xFD
#define ROW2            0xFB
#define ROW3            0xF7
#define ROW4            0xEF
#define ROW5            0xDF
#define ROW6            0xBF
#define ROW7            0x7F

// -----------------------------------------------------------------------------
// Globals: Serial / USB Host
// -----------------------------------------------------------------------------
SerialPIO PIOSerial(TTL_TX, TTL_RX);
Adafruit_USBH_Host USBHost;

// -----------------------------------------------------------------------------
// Globals: Device & HID Interface Tracking
// -----------------------------------------------------------------------------
typedef struct {
  tusb_desc_device_t desc_device;
  uint16_t manufacturer[16];
  uint16_t product[24];
  uint16_t serial[12];
  bool mounted;
} dev_info_t;

dev_info_t dev_info[CFG_TUH_DEVICE_MAX] = { 0 };

struct HidIface {
  bool mounted;
  uint8_t dev_addr;
  uint8_t instance;
  bool pending;
  bool last_rearm_ok;
};

HidIface hid_ifaces[MAX_HID_IFACES];

// -----------------------------------------------------------------------------
// Globals: Keyboard State Tracking
// -----------------------------------------------------------------------------
uint8_t prev_report[8] = {0};
bool    prev_report_valid = false;

uint8_t prev_modifiers = 0;
bool    prev_mod_valid = false;

// Arrow key state tracking: stores which arrow key is currently pressed (0x50, 0x4F, 0x52, 0x51)
// 0 = no arrow key pressed
uint8_t current_arrow_key = 0;

// CTRL key state tracking for arrow keys and RUBOUT
// arrow_ctrl_active: CTRL was activated by arrow keys
// delete_key_active: DELETE/RUBOUT key is currently pressed
bool arrow_ctrl_active = false;
bool delete_key_active = false;

// -----------------------------------------------------------------------------
// Globals: Atomic Keyboard Matrix (double-buffered)
// -----------------------------------------------------------------------------
uint8_t keymatrix[256];
alignas(4) static uint8_t keymatrix_buf[2][256]; 
static std::atomic<uint8_t> active_buf{0};

// -----------------------------------------------------------------------------
// Globals: PIO
// -----------------------------------------------------------------------------
PIO pio = pio0;
uint sm = 0;
volatile bool pio_ready = false;
bool core1_disable_systick = true;

// -----------------------------------------------------------------------------
// Forward Declarations
// -----------------------------------------------------------------------------
void tuh_mount_cb(uint8_t daddr);
void tuh_umount_cb(uint8_t daddr);
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const* desc, uint16_t desc_len);
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance);
void tuh_hid_report_received_cb(tuh_xfer_t *xfer);
void print_device_descriptor(tuh_xfer_t *xfer);
void sync_vz_matrix_to_keymatrix();

// -----------------------------------------------------------------------------
// Atomic Keymatrix Helpers
// -----------------------------------------------------------------------------

// Core1: read helper (fast, lock-free)
static inline uint8_t keymatrix_read(uint8_t addr) {
  uint8_t idx = active_buf.load(std::memory_order_acquire);
  return keymatrix_buf[idx][addr] & 0x3F;
}

// Core0: update helper (atomic flip)
void update_keymatrix(const uint8_t new_map[256]) {
  uint8_t cur = active_buf.load(std::memory_order_relaxed);
  uint8_t inactive = cur ^ 1;
  
  memcpy(keymatrix_buf[inactive], new_map, 256);
  
  active_buf.store(inactive, std::memory_order_release);
}

// -----------------------------------------------------------------------------
// VZ Matrix Sync
// -----------------------------------------------------------------------------
void sync_vz_matrix_to_keymatrix() {
    // Map vz_matrix[0-7] to keymatrix[address]
    keymatrix[ROW0] = vz_matrix[0];
    keymatrix[ROW1] = vz_matrix[1];
    keymatrix[ROW2] = vz_matrix[2];
    keymatrix[ROW3] = vz_matrix[3];
    keymatrix[ROW4] = vz_matrix[4];
    keymatrix[ROW5] = vz_matrix[5];
    keymatrix[ROW6] = vz_matrix[6];
    keymatrix[ROW7] = vz_matrix[7];

    // Compute presence byte: AND all rows together
    // If ANY bit is low in ANY row, it will be low in result
    uint8_t presence = 0x3F;
    for (int i = 0; i < 8; i++) {
        presence &= vz_matrix[i];
    }
    keymatrix[0x00] = presence;  // Base address shows ANY key pressed

    // Atomic update for Core 1
    update_keymatrix(keymatrix);
}

// -----------------------------------------------------------------------------
// Core 0 Setup
// -----------------------------------------------------------------------------
unsigned long last_rearm_ms = 0;
const unsigned long REARM_INTERVAL_MS = 10;

void setup() {
    PIOSerial.begin(115200);
    PIOSerial.println("VZ-200 USB Keyboard Interface");

    // Initialize keymatrix to default (all keys up)
    for (int i = 0; i < 256; i++) {
        keymatrix[i] = 0x3F;  // All keys up
    }
    
    // Initialize VZ keyboard mapping
    init_hid_ifaces();
    vz_keyboard_init();
    
    // Sync initial state
    sync_vz_matrix_to_keymatrix();
    
    // Start USB host
    USBHost.begin(0);
}

// -----------------------------------------------------------------------------
// Core 0 Loop - USB Processing
// -----------------------------------------------------------------------------
void loop() {
  USBHost.task();

  unsigned long now = millis();
  if (now - last_rearm_ms >= REARM_INTERVAL_MS) {
    last_rearm_ms = now;

    for (int i = 0; i < MAX_HID_IFACES; i++) {
      if (hid_ifaces[i].mounted && !hid_ifaces[i].pending) {
        bool ok = tuh_hid_receive_report(
                    hid_ifaces[i].dev_addr,
                    hid_ifaces[i].instance);

        if (ok != hid_ifaces[i].last_rearm_ok) {
          hid_ifaces[i].last_rearm_ok = ok;
        }
        if (ok) hid_ifaces[i].pending = true;
        delay(1);
      }
    }
  }
  PIOSerial.flush();
}

// -----------------------------------------------------------------------------
// Core 1 Setup - PIO Initialization
// -----------------------------------------------------------------------------
void setup1() {
    // Configure /RD pin as normal GPIO input (used by PIO wait instruction)
    gpio_init(Z80_VZRD);
    gpio_set_dir(Z80_VZRD, GPIO_IN);
    gpio_disable_pulls(Z80_VZRD);

    // Load the compiled PIO program into the PIO instruction memory
    uint offset = pio_add_program(pio, &vz_kbd_program);

    // Configure state machine
    pio_sm_config c = vz_kbd_program_get_default_config(offset);

    // Set in/out pin bases
    sm_config_set_in_pins(&c, Z80_A0);
    sm_config_set_out_pins(&c, Z80_D0, 6);

    // Init address port as regular GPIO inputs
    for (int i = 0; i < 8; ++i) {
        gpio_init(Z80_A0 + i);
        gpio_set_dir(Z80_A0 + i, GPIO_IN);
        gpio_pull_up(Z80_A0 + i);
    }

    // Init and hand over data port pins to PIO
    for (int i = 0; i < 6; ++i) {
        pio_gpio_init(pio, Z80_D0 + i);
        gpio_disable_pulls(Z80_D0 + i);
    }

    // Shift config
    sm_config_set_in_shift(&c, false, true, 8);    // Shift left, autopush
    sm_config_set_out_shift(&c, true, false, 32);  // Shift right, manual

    // Clear any stale FIFO state
    pio_sm_clear_fifos(pio, sm);

    // Initialize state machine
    pio_sm_init(pio, sm, offset, &c);

    // ONLY set pin direction for OUTPUTS (dataport)
    pio_sm_set_consecutive_pindirs(pio, sm, Z80_D0, 6, true);
  
    // Final FIFO clear before enabling
    pio_sm_clear_fifos(pio, sm);

    // Pre-fill TX FIFO with default value (0x3F = all keys up)
    // See TODO in original vziosim.ino for why this is necessary
    pio_sm_put(pio, sm, (uint32_t)0x3F);

    // Mark setup as done
    pio_ready = true;
}

// -----------------------------------------------------------------------------
// Core 1 Loop - Timing-Critical Z80 Bus Service
// -----------------------------------------------------------------------------
void loop1() {
    // Wait for setup to complete
    while (!pio_ready) {
        tight_loop_contents();
    }
    
    // NOW enable the state machine
    pio_sm_set_enabled(pio, sm, true);
    
    // Service Z80 read requests
    while (1) {
        uint32_t raw = pio_sm_get_blocking(pio, sm);
        uint8_t addr = raw & 0xFF;
        uint8_t data6 = keymatrix_read(addr);
        pio_sm_put_blocking(pio, sm, (uint32_t)data6);
    }
}

// -----------------------------------------------------------------------------
// USB Device Descriptor Callbacks
// -----------------------------------------------------------------------------
void tuh_mount_cb(uint8_t daddr) {
  PIOSerial.printf("Device attached, address = %d\r\n", daddr);
  dev_info_t *dev = &dev_info[daddr - 1];
  dev->mounted = true;
  tuh_descriptor_get_device(daddr, &dev->desc_device,
                            18, print_device_descriptor, 0);
}

void tuh_umount_cb(uint8_t daddr) {
  PIOSerial.printf("Device removed, address = %d\r\n", daddr);
  dev_info_t *dev = &dev_info[daddr - 1];
  dev->mounted = false;
}

void print_device_descriptor(tuh_xfer_t *xfer) {
  if (XFER_RESULT_SUCCESS != xfer->result) {
    PIOSerial.println("Failed to get device descriptor");
    return;
  }
  tusb_desc_device_t *desc = &dev_info[xfer->daddr - 1].desc_device;
  PIOSerial.printf("Device %u: ID %04x:%04x\r\n",
                   xfer->daddr, desc->idVendor, desc->idProduct);
}

// -----------------------------------------------------------------------------
// HID Callbacks
// -----------------------------------------------------------------------------
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const* desc, uint16_t desc_len) {
  int slot = hid_slot_for(dev_addr, instance);
  if (slot < 0) {
    return;
  }

  hid_ifaces[slot].mounted = true;
  hid_ifaces[slot].dev_addr = dev_addr;
  hid_ifaces[slot].instance = instance;
  hid_ifaces[slot].pending = false;

  bool ok = tuh_hid_receive_report(dev_addr, instance);
  hid_ifaces[slot].last_rearm_ok = ok;
  if (ok) hid_ifaces[slot].pending = true;

  PIOSerial.println("Keyboard ready");
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (hid_ifaces[i].mounted &&
        hid_ifaces[i].dev_addr == dev_addr &&
        hid_ifaces[i].instance == instance) {
      hid_ifaces[i].mounted = false;
      hid_ifaces[i].pending = false;
      hid_ifaces[i].instance = 0xFF;
    }
  }
}

void tuh_hid_report_received_cb(tuh_xfer_t *xfer) {
  if (!xfer || !xfer->buffer) return;

  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (hid_ifaces[i].mounted &&
        hid_ifaces[i].dev_addr == xfer->daddr) {
      hid_ifaces[i].pending = false;
    }
  }

  if (xfer->result != XFER_RESULT_SUCCESS) return;
  process_hid_report((uint8_t const*)xfer->buffer, 8, xfer->daddr);
}

void tuh_hid_report_received_cb(uint8_t dev_addr,
                                uint8_t instance,
                                uint8_t const* report,
                                uint16_t len) {
  (void)instance;
  if (!report) return;

  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (hid_ifaces[i].mounted &&
        hid_ifaces[i].dev_addr == dev_addr) {
      hid_ifaces[i].pending = false;
    }
  }

  process_hid_report(report, len < 8 ? len : 8, dev_addr);
}

// -----------------------------------------------------------------------------
// HID Interface Helpers
// -----------------------------------------------------------------------------
void init_hid_ifaces() {
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    hid_ifaces[i].mounted = false;
    hid_ifaces[i].dev_addr = 0;
    hid_ifaces[i].instance = 0xFF;
    hid_ifaces[i].pending = false;
    hid_ifaces[i].last_rearm_ok = true;
  }
}

int hid_slot_for(uint8_t dev_addr, uint8_t instance) {
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (hid_ifaces[i].mounted &&
        hid_ifaces[i].dev_addr == dev_addr &&
        hid_ifaces[i].instance == instance) {
      return i;
    }
  }
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (!hid_ifaces[i].mounted) return i;
  }
  return -1;
}

// -----------------------------------------------------------------------------
// HID Report Processing
// -----------------------------------------------------------------------------
void emit_mod_event(bool pressed, uint8_t mask) {
  (void)pressed;
  uint8_t sc = 0;

  if      (mask == 0x01) sc = 0xE0;
  else if (mask == 0x02) sc = 0xE1;
  else if (mask == 0x10) sc = 0xE4;
  else if (mask == 0x20) sc = 0xE5;

  if (sc) {
    if (!vz_scancode_mapped(sc)) {
      PIOSerial.printf("WARN: modifier scancode 0x%02x not mapped\r\n", sc);
    }
    if (pressed) vz_key_press(sc);
    else         vz_key_release(sc);
  }
}

// Helper to press an arrow key, handling CTRL and transition between arrow keys
void handle_arrow_key_press(uint8_t arrow_code, uint8_t second_key) {
  if (current_arrow_key == 0) {
    // No arrow key was pressed before, press CTRL + second key
    vz_key_press(0xE0); // CTRL
    arrow_ctrl_active = true;
    vz_key_press(second_key);
  } else if (current_arrow_key != arrow_code) {
    // Different arrow key was pressed, release old second key and press new one
    // Keep CTRL pressed
    uint8_t old_second_key = 0;
    switch (current_arrow_key) {
      case 0x50: old_second_key = 0x10; break; // 'M'
      case 0x4F: old_second_key = 0x36; break; // ','
      case 0x52: old_second_key = 0x37; break; // '.'
      case 0x51: old_second_key = 0x2C; break; // space
    }
    if (old_second_key) vz_key_release(old_second_key);
    vz_key_press(second_key);
  }
  // else: same arrow key already pressed, do nothing
  
  current_arrow_key = arrow_code;
}

// Helper to release an arrow key, handling CTRL release when all arrow keys and delete are gone
void handle_arrow_key_release(uint8_t arrow_code, uint8_t second_key) {
  if (current_arrow_key == arrow_code) {
    vz_key_release(second_key);
    current_arrow_key = 0;
    
    // Only release CTRL if DELETE is not active
    if (arrow_ctrl_active && !delete_key_active) {
      vz_key_release(0xE0); // CTRL
      arrow_ctrl_active = false;
    }
  }
}

// Helper to press RUBOUT (DELETE), handling CTRL management
void handle_rubout_press() {
  if (!arrow_ctrl_active && !delete_key_active) {
    // Neither arrow keys nor DELETE was active, press CTRL + ';'
    vz_key_press(0xE0); // CTRL
    arrow_ctrl_active = true;
  }
  // If arrow keys OR delete already active, CTRL is already down
  vz_key_press(0x33); // ';' (RUBOUT key)
  delete_key_active = true;
}

// Helper to release RUBOUT (DELETE), handling CTRL release
void handle_rubout_release() {
  if (delete_key_active) {
    vz_key_release(0x33); // ';' (RUBOUT key)
    delete_key_active = false;
    
    // Only release CTRL if no arrow keys are active
    if (arrow_ctrl_active && current_arrow_key == 0) {
      vz_key_release(0xE0); // CTRL
      arrow_ctrl_active = false;
    }
  }
}

void process_hid_report(uint8_t const* buf_in,
                        unsigned report_len,
                        uint8_t daddr) {
  uint8_t buf[8] = {0};
  unsigned len = report_len < 8 ? report_len : 8;
  memcpy(buf, buf_in, len);

  uint8_t modifiers = buf[0];
  if (prev_mod_valid) {
    uint8_t newly_pressed  = modifiers & ~prev_modifiers;
    uint8_t newly_released = prev_modifiers & ~modifiers;

    for (uint8_t bit = 1; bit; bit <<= 1) {
      if (newly_pressed  & bit) emit_mod_event(true, bit);
      if (newly_released & bit) emit_mod_event(false, bit);
    }
  } else {
    prev_mod_valid = true;
  }

  for (unsigned i = 2; i < len; i++) {
    uint8_t usage = buf[i];
    if (usage == 0 || usage == 0x01) continue;

    bool seen_before = false;
    if (prev_report_valid) {
      for (unsigned j = 2; j < 8; j++) {
        if (prev_report[j] == usage) {
          seen_before = true;
          break;
        }
      }
    }
    // this is how we handle keys that don't belong to vz keyboard
    if (!seen_before) {
      // arrow keys remap
      switch(usage) {
        case 0x50: // left arrow CTRL + 'M'
          handle_arrow_key_press(0x50, 0x10);
          break;
        case 0x4F: // right arrow CTRL + ',' comma
          handle_arrow_key_press(0x4F, 0x36);
          break;
        case 0x52: // up arrow CTRL + '.' period
          handle_arrow_key_press(0x52, 0x37);
          break;
        case 0x51: // down arrow CTRL + 'spc' space
          handle_arrow_key_press(0x51, 0x2C);
          break;
        case 0x4C: // Delete (RUBOUT - CTRL + ';')
          handle_rubout_press();
          break;
        case 0x2A: // Backspace (same as left arrow - CTRL + 'M')
          handle_arrow_key_press(0x50, 0x10);
          break;

        default:
          vz_key_press(usage);
          break;
      }
    }
  }

  if (prev_report_valid) {
    for (unsigned i = 2; i < 8; i++) {
      uint8_t usage = prev_report[i];
      if (usage == 0 || usage == 0x01) continue;

      bool still_present = false;
      for (unsigned j = 2; j < len; j++) {
        if (buf[j] == usage) {
          still_present = true;
          break;
        }
      }
      if (!still_present) {
        switch(usage) {
          case 0x50: // left arrow CTRL + 'M'
            handle_arrow_key_release(0x50, 0x10);
            break;
          case 0x4F: // right arrow CTRL + ',' comma
            handle_arrow_key_release(0x4F, 0x36);
            break;
          case 0x52: // up arrow CTRL + '.' period
            handle_arrow_key_release(0x52, 0x37);
            break;
          case 0x51: // down arrow CTRL + 'spc' space
            handle_arrow_key_release(0x51, 0x2C);
            break;
          case 0x4C: // Delete (RUBOUT - CTRL + ';')
            handle_rubout_release();
            break;
          case 0x2A: // Backspace (same as left arrow)
            handle_arrow_key_release(0x50, 0x10);
            break;

          default:
            vz_key_release(usage);
            break;
        }
      }
    }
  }

  // Sync vz_matrix to keymatrix after any changes
  sync_vz_matrix_to_keymatrix();

  memcpy(prev_report, buf, 8);
  prev_report_valid = true;
  prev_modifiers = modifiers;
}