# RC39 Backplane for VZ200 Clone Workalike

This project is an **RC39-based backplane** designed for building home computer workalikes.  
The current focus is on creating a **VZ200 clone workalike**.

---

## Backplane Features
- **Power Input:** 5V supplied via a center-positive barrel jack  
- **Reset Circuit (Fast Edge):** Open-collector NPN transistor with strong pull-up  
- **Reset Circuit (Standard):** 555 timer-based reset design  
- **Mounting:** Lugs for an I/O shelf above the expansion header, positioned at the back of the board, with reset switch accessible from the front  
- **Expansion Connectors:**  
  - Five 39-pin RC39-compliant connectors for expansion cards  
  - I/O expansion header matching the VZ200 pinout  
  - RC39 is essentially the RC2014 bus with one pin removed, allowing connectors to fit on a 100mm × 100mm PCB  
- **Card Size:** All cards are ≤100mm × 100mm, with two-layer artwork or less, keeping manufacturing costs low for a complete set of boards  

---

## Current Cards
- **CPU / Clock Board**  
- **Serial Card:** 6850 UART with GAL-based selection logic  
- **RAM / ROM Card:** Includes GAL-based selection and banking logic  
- **Display Card:** MC6847 video chip with dual-port RAM and GAL selection logic  
- **I/O Interface Card (3V):**  
  - Beeper, tape, and sound support  
  - Built-in modern RP2040 I/O interface  
  - Front-facing design with classic VZ-style keyboard connector  
  - RP2040 can potentially control the computer keyboard via HID over USB or I²C header  

---

## License
This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**.  

You are free to use, modify, and distribute this project under the terms of the GPL.  
See the [LICENSE](https://www.gnu.org/licenses/gpl-3.0.en.html) file or the GNU website for full details.

