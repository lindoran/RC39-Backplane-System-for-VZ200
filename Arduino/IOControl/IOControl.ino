/* IOControl.ino
This is a Arduino pico-core sketch that uses the Adafruit TinyUSB Host stack 
to interface with a USB keyboard and translate key presses into a VZ-200
keyboard matrix representation.

VZ200 key matrix as read by the computer.

OK so the key matrix is minimally decoded and address bits 0-7 are used to 
decode what row we are looking at. below NC = no key is asigned

CTRL = CTRL
SHIFT = SHIFT
RETRN = RETURN

We don't actually care how the Shift key modifies the PC keyboard, we care 
about the symbol listed below

So:
                            5    4    3    2    1    0   (Bit of 6 Byte Nibble)
ROW ADDRESS (LSB)   0xFE    R    Q    E    NC   W    T
                    0xFD    F    A    D   CTRL  S    G
                    0xFB    V    Z    C  SHIFT  X    B
                    0xF7    4    1    3    NC   2    5
                    0xEF    M   SPC   ,    NC   .    N
                    0xDF    7    0    8    -    9    6
                    0xBF    U    P    I  RETRN  O    Y
                    0x7F    J    ;    K    :    L    H

The scan codes for a PC keyboard are as such, modifier keycodes are seprate 
from main keycodes and overlap, they have to be managed different. The modifier
codes are listed below. ';' and ':' are the same key on the PC keyboard
and so ':' is moved to the ( ' ) - apostrophe. key (next to enter).

                            5    4    3    2    1    0   (Bit of 6 Byte Nibble)
ROW ADDRESS (LSB)   0xFE  0x15 0x14 0x08   NC  0x1A 0x17
                    0xFD  0x09 0x04 0x07  CTRL 0x16 0x0A
                    0xFB  0x19 0x1D 0x06 SHIFT 0x1B 0x05
                    0xF7  0x21 0x1E 0x20   NC  0x1F 0x22
                    0xEF  0x10 0x2c 0x36   NC  0x37 0x11
                    0xDF  0x24 0x27 0x25 0x2D  0x26 0x23
                    0xBF  0x18 0x13 0x0C 0x28  0x12 0x1C
                    0x7F  0x0d 0x33 0x0E 0x34  0x0F 0x0b

These codes overlap and are managed independently using bit masks:                
LSHIFT = 02 and is mapped to SHIFT
RSHIFT = 20 and is mapped to SHIFT
LCTRL = 01 and is mapped to CTRL
RCTRL = 10 and is mapped to CTRL
*/

#include "Adafruit_TinyUSB.h"
#include "vz200_keyboard.h"

#ifndef USE_TINYUSB_HOST
  #error This example requires usb stack configured as host in "Tools -> USB Stack -> Adafruit TinyUSB Host"
#endif

#define MAX_HID_IFACES 4

// Device info storage
typedef struct {
  tusb_desc_device_t desc_device;
  // trimmed string buffers to save RAM; expand if you need longer strings
  uint16_t manufacturer[16];
  uint16_t product[24];
  uint16_t serial[12];
  bool mounted;
} dev_info_t;

dev_info_t dev_info[CFG_TUH_DEVICE_MAX] = { 0 };
Adafruit_USBH_Host USBHost;

// HID interface tracking
struct HidIface {
  bool mounted;
  uint8_t dev_addr;
  uint8_t instance;
  bool pending;
  // Track last re-arm result to avoid repeated identical logs
  bool last_rearm_ok;
} hid_ifaces[MAX_HID_IFACES];

void init_hid_ifaces() {
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    hid_ifaces[i].mounted = false;
    hid_ifaces[i].dev_addr = 0;
    hid_ifaces[i].instance = 0xFF;
    hid_ifaces[i].pending = false;
    hid_ifaces[i].last_rearm_ok = true; // assume OK until we observe otherwise
  }
}

int hid_slot_for(uint8_t dev_addr, uint8_t instance) {
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (hid_ifaces[i].mounted && hid_ifaces[i].dev_addr == dev_addr && hid_ifaces[i].instance == instance) return i;
  }
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (!hid_ifaces[i].mounted) return i;
  }
  return -1;
}

// previous report for keydown detection
uint8_t prev_report[8] = {0};
bool prev_report_valid = false;

// track modifier state (boot keyboard modifiers in report[0])
uint8_t prev_modifiers = 0;
bool prev_mod_valid = false;

// simple caps-lock tracking (toggled on CAPS usage press)
bool caps_lock_on = false;

// Snapshot of the VZ matrix used to detect changes. Initialized in setup().
uint8_t prev_vz_matrix[8] = {0};

// Emit the virtual keymap when any row changes; print a neatly aligned table
// matching the README/keymap layout and mark pressed keys with a leading '*'.
// Also clear the terminal between updates using ANSI escape codes.
void emit_vz_matrix_if_changed(void) {
  extern uint8_t vz_matrix[8];
  bool changed = false;
  for (int i = 0; i < 8; i++) {
    if (prev_vz_matrix[i] != vz_matrix[i]) { changed = true; break; }
  }
  if (!changed) return;
  // Update snapshot
  memcpy(prev_vz_matrix, vz_matrix, 8);

  // ANSI clear screen and move cursor to home
  Serial1.print("\x1b[2J\x1b[H");

  // Labels for bits 5..0 for each row index 0..7 (row 0 -> addr 0xFE)
  const char *labels[8][6] = {
    {"R","Q","E","NC","W","T"},
    {"F","A","D","CTRL","S","G"},
    {"V","Z","C","SHIFT","X","B"},
    {"4","1","3","NC","2","5"},
    {"M","SPC",",","NC",".","N"},
    {"7","0","8","-","9","6"},
    {"U","P","I","RETRN","O","Y"},
    {"J",";","K",":","L","H"}
  };

  const uint8_t row_addr[8] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F};

  // Print header aligned to the row prefix
  const char *row_prefix_example = "ROW ADDRESS (LSB)   0xFE";
  int prefix_len = strlen(row_prefix_example);
  for (int i = 0; i < prefix_len; i++) Serial1.print(' ');
  for (int b = 5; b >= 0; b--) Serial1.printf("%6d", b);
  Serial1.println("   (Bit of 6 Byte Nibble)");

  // Print each row with fixed-width columns (6 chars per column)
  for (int r = 0; r < 8; r++) {
    Serial1.printf("ROW ADDRESS (LSB)   0x%02x", row_addr[r]);
    for (int b = 5; b >= 0; b--) {
      bool pressed = (vz_matrix[r] & (1 << b)) == 0; // active-low
      char cell[16];
      if (pressed) snprintf(cell, sizeof(cell), "*%s", labels[r][5 - b]);
      else snprintf(cell, sizeof(cell), "%s", labels[r][5 - b]);
      Serial1.printf("%6s", cell);
    }
    Serial1.println();
  }
  Serial1.println();
}

// forward declarations
void tuh_mount_cb(uint8_t daddr);
void tuh_umount_cb(uint8_t daddr);
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc, uint16_t desc_len);
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance);
void tuh_hid_report_received_cb(tuh_xfer_t *xfer);

// Emit modifier press/release: map modifier mask to HID usage codes
void emit_mod_event(bool pressed, uint8_t mask) {
  (void)pressed;
  // Map modifier mask to HID modifier usage codes (0xE0..0xE7) to avoid
  // colliding with regular key usage codes reported in the key array.
  uint8_t sc = 0;
  if (mask == 0x01) sc = 0xE0; // LCTRL -> HID usage 0xE0
  else if (mask == 0x02) sc = 0xE1; // LSHIFT -> HID usage 0xE1
  else if (mask == 0x10) sc = 0xE4; // RCTRL -> HID usage 0xE4
  else if (mask == 0x20) sc = 0xE5; // RSHIFT -> HID usage 0xE5

  if (sc) {
    if (!vz_scancode_mapped(sc)) {
      Serial1.printf("WARN: modifier scancode 0x%02x not mapped to VZ matrix\r\n", sc);
    }
    if (pressed) vz_key_press(sc);
    else vz_key_release(sc);
  }
}

// Shared HID report processor used by both callback signatures
void process_hid_report(uint8_t const* buf_in, unsigned report_len, uint8_t daddr) {
  // Normalize to fixed 8-byte buffer
  uint8_t buf[8] = {0};
  unsigned len = report_len < 8 ? report_len : 8;
  memcpy(buf, buf_in, len);

  // Detect modifier changes and emit press/release for modifiers
  uint8_t modifiers = buf[0];
  if (prev_mod_valid) {
    uint8_t newly_pressed = modifiers & ~prev_modifiers;
    uint8_t newly_released = prev_modifiers & ~modifiers;
    for (uint8_t bit = 1; bit != 0; bit <<= 1) {
      if (newly_pressed & bit) emit_mod_event(true, bit);
      if (newly_released & bit) emit_mod_event(false, bit);
    }
  } else {
    prev_mod_valid = true;
  }

  // Key press events (present now, not in prev). Ignore 0x01 slots (error).
  for (unsigned i = 2; i < len; i++) {
    uint8_t usage = buf[i];
    if (usage == 0 || usage == 0x01) continue;
    bool seen_before = false;
    if (prev_report_valid) {
      for (unsigned j = 2; j < 8; j++) if (prev_report[j] == usage) { seen_before = true; break; }
    }
    if (!seen_before) {
      if (usage == 0x39) caps_lock_on = !caps_lock_on; // caps toggle

      // Update VZ-200 matrix via library (do not touch vz_matrix directly)
      vz_key_press(usage);
    }
  }

  // Key release events (present in prev, not in current). Ignore 0x01.
  if (prev_report_valid) {
    for (unsigned i = 2; i < 8; i++) {
      uint8_t usage = prev_report[i];
      if (usage == 0 || usage == 0x01) continue;
      bool still_present = false;
      for (unsigned j = 2; j < len; j++) if (buf[j] == usage) { still_present = true; break; }
      if (!still_present) {
        // Update VZ-200 matrix via library (do not touch vz_matrix directly)
        vz_key_release(usage);
      }
    }
  }

  // Save current report and modifier state
  // Emit VZ matrix only if any key state changed while processing this report
  emit_vz_matrix_if_changed();
  memcpy(prev_report, buf, 8);
  prev_report_valid = true;
  prev_modifiers = modifiers;
}


// ---------------- setup / loop ----------------
void setup() {
  Serial1.setRX(13);
  Serial1.setTX(12);
  Serial1.begin(115200);
  Serial1.println("TinyUSB Host: re-arm-all HID visible-keydown example");

  init_hid_ifaces();
  vz_keyboard_init();
  // capture initial VZ matrix snapshot (no output until changed)
  memcpy(prev_vz_matrix, vz_matrix, 8);
  USBHost.begin(0);
}

// Aggressive-but-controlled re-arm: try every mounted interface at modest rate
unsigned long last_rearm_ms = 0;
const unsigned long REARM_INTERVAL_MS = 10; // tune 5-50ms if needed

void loop() {
  USBHost.task();

  unsigned long now = millis();
  if (now - last_rearm_ms >= REARM_INTERVAL_MS) {
    last_rearm_ms = now;

    // Try to arm receive for every mounted interface (if not pending)
    for (int i = 0; i < MAX_HID_IFACES; i++) {
      if (hid_ifaces[i].mounted && !hid_ifaces[i].pending) {
        bool ok = tuh_hid_receive_report(hid_ifaces[i].dev_addr, hid_ifaces[i].instance);
        // Only log when the re-arm status changes to avoid flooding the serial output
        if (ok != hid_ifaces[i].last_rearm_ok) {
          Serial1.printf("DEBUG: re-arm dev=%u inst=%u -> %u (t=%lu)\r\n",
                         hid_ifaces[i].dev_addr, hid_ifaces[i].instance, ok ? 1 : 0, now);
          hid_ifaces[i].last_rearm_ok = ok;
        }
        if (ok) hid_ifaces[i].pending = true;
        // small gap so we don't hammer the stack too fast
        delay(1);
      }
    }
  }

  Serial1.flush();
}

// ---------------- device descriptor helpers (minimal) ----------------
void print_device_descriptor(tuh_xfer_t *xfer);

void tuh_mount_cb(uint8_t daddr) {
  Serial1.printf("Device attached, address = %d\r\n", daddr);
  dev_info_t *dev = &dev_info[daddr - 1];
  dev->mounted = true;
  tuh_descriptor_get_device(daddr, &dev->desc_device, 18, print_device_descriptor, 0);
}

void tuh_umount_cb(uint8_t daddr) {
  Serial1.printf("Device removed, address = %d\r\n", daddr);
  dev_info_t *dev = &dev_info[daddr - 1];
  dev->mounted = false;
}

void print_device_descriptor(tuh_xfer_t *xfer) {
  if (XFER_RESULT_SUCCESS != xfer->result) {
    Serial1.println("Failed to get device descriptor");
    return;
  }
  uint8_t daddr = xfer->daddr;
  dev_info_t *dev = &dev_info[daddr - 1];
  tusb_desc_device_t *desc = &dev->desc_device;
  Serial1.printf("Device %u: ID %04x:%04x\r\n", daddr, desc->idVendor, desc->idProduct);
}

// ---------------- HID callbacks ----------------
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc, uint16_t desc_len) {
  Serial1.printf("DEBUG: tuh_hid_mount_cb dev=%u instance=%u desc_len=%u\r\n", dev_addr, instance, desc_len);

  int slot = hid_slot_for(dev_addr, instance);
  if (slot < 0) {
    Serial1.println("DEBUG: no free HID slot");
    return;
  }

  hid_ifaces[slot].mounted = true;
  hid_ifaces[slot].dev_addr = dev_addr;
  hid_ifaces[slot].instance = instance;
  hid_ifaces[slot].pending = false;

  // Dump descriptor bytes for analysis
  Serial1.print("DEBUG: mount desc bytes:");
  for (uint16_t i = 0; i < desc_len; i++) Serial1.printf(" 0x%02x", desc[i]);
  Serial1.println();

  // Try to arm immediately (we also re-arm from loop)
  bool ok = tuh_hid_receive_report(dev_addr, instance);
  Serial1.printf("DEBUG: initial tuh_hid_receive_report dev=%u inst=%u returned=%u\r\n", dev_addr, instance, ok ? 1 : 0);
  // record initial re-arm status
  hid_ifaces[slot].last_rearm_ok = ok;
  if (ok) hid_ifaces[slot].pending = true;

  Serial1.println("DEBUG: press and release keys now");
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  Serial1.printf("DEBUG: tuh_hid_umount_cb dev=%u instance=%u\r\n", dev_addr, instance);
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (hid_ifaces[i].mounted && hid_ifaces[i].dev_addr == dev_addr && hid_ifaces[i].instance == instance) {
      hid_ifaces[i].mounted = false;
      hid_ifaces[i].dev_addr = 0;
      hid_ifaces[i].instance = 0xFF;
      hid_ifaces[i].pending = false;
      Serial1.printf("DEBUG: cleared slot %d for dev=%u inst=%u\r\n", i, dev_addr, instance);
    }
  }
}

void tuh_hid_report_received_cb(tuh_xfer_t *xfer) {
  if (!xfer) return;
  uint8_t daddr = xfer->daddr;

  // Clear pending for this device so loop() can re-arm
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (hid_ifaces[i].mounted && hid_ifaces[i].dev_addr == daddr) hid_ifaces[i].pending = false;
  }

  if (xfer->result != XFER_RESULT_SUCCESS) {
    Serial1.printf("DEBUG: transfer failed (%d)\r\n", xfer->result);
    return;
  }

  if (!xfer->buffer) {
    Serial1.println("DEBUG: empty buffer");
    return;
  }

  // Assume boot keyboard report length = 8 bytes (modifiers, reserved, 6 keycodes)
  const unsigned report_len = 8;
  uint8_t const* buf = (uint8_t const*) xfer->buffer;

  // Delegate to shared HID report processor
  process_hid_report(buf, report_len, daddr);

}

// Some versions of TinyUSB call the legacy callback signature instead of
// the tuh_xfer_t-based form above. Add a legacy-style handler so we
// definitely see incoming reports regardless of which signature the
// stack invokes. This duplicates minimal processing from the xfer-based
// handler so you can see raw bytes and visible keydowns in Serial1.
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
  // Clear pending for this device so loop() can re-arm (matches xfer-based handler)
  for (int i = 0; i < MAX_HID_IFACES; i++) {
    if (hid_ifaces[i].mounted && hid_ifaces[i].dev_addr == dev_addr) hid_ifaces[i].pending = false;
  }

  if (!report) return;

  // Minimal visible-keydown processing: assume boot keyboard-like layout
  const unsigned report_len = (len < 8) ? len : 8;
  uint8_t buf[8] = {0};
  memcpy(buf, report, report_len);

  // Delegate to shared processor
  process_hid_report(buf, report_len, dev_addr);
}