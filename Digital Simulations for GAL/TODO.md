# VZ200 I/O CONTROLLER — TODO

[1] Keyboard Interface Redesign
    - Remove legacy VZ-style keyboard connector entirely.
    - Commit to USB-only keyboard bridge via RP2040 USB host.
    - Verify that matrix-generation firmware matches new hardware.

[2] Address Bus Interface
    - Wire the 3.3 V buffer for the address bus as *always enabled*.
    - Confirm direction is permanently input-only toward the RP2040.
    - Re-check timing margins with the Z80 bus transceiver.

[3] Data Bus / Enable Logic
    - Rework wiring for the data-bus enable pin.
    - Rewire controller-side enable logic to match new scheme.
    - Ensure clean tri-state behavior when not actively driving.

[4] I²C Removal Cleanup
    - Remove I²C functionality from the design.
    - Pull the two I²C pull-up resistors (SDA/SCL).
    - Confirm those pins are now free for reassignment or left NC.

[5] Tape Input Pull-Up
    - Move the pull-up resistor for the tape input onto the I/O card.
    - Prevent the tape input pin from floating when the analog shelf
      is not connected.
    - Validate correct logic level during standalone operation.

[6] Tape Port Connector Update
    - Replace the existing tape connector with a 4‑pin JST.
    - Update PCB footprint and mechanical clearance.
    - Verify pinout mapping for MIC / EAR / GND / +5 V (if used).
    - Ensure compatibility with existing cables or document new wiring.


# ANALOG BOARD — TODO

[1] Tape Jack Orientation
    - Correct the tape jack orientation (currently backwards).
    - Replace with matching 4‑pin JST to align with I/O controller.
    - Confirm mechanical alignment and cable strain relief.

[2] Video Port Orientation
    - Flip the video port (currently reversed).
    - Ensure the cable no longer needs to be flipped at the far end.
    - Update silkscreen and mechanical notes to reflect orientation.


# SERIAL BOARD — TODO

[1] Crystal Power Pin Update
    - Update power pins for the crystal footprint.
    - Schematic is correct; footprint requires correction.
    - Verify pad numbering and power routing match the crystal spec.
    - Regenerate footprint library entry if needed for consistency.


# VIDEO BOARD — TODO

[1] Address Bus Tri‑State Fixes
    - Move the existing bodges that handle address‑bus tri‑stating
      during port conflicts into the proper PCB design.
    - Ensure the tri‑state control logic is clean, deterministic,
      and synchronized with the I/O controller’s bus arbitration.
    - Validate that no unintended drive occurs during shared‑bus
      cycles (VDG vs. CPU vs. controller).
    - Update routing and silkscreen to reflect the corrected logic.


# RAM BOARD — TODO

[1] Bank Register Address Interface
    - Confirm all address lines feeding the bank‑select register are
      correctly routed (A0–A15 already present).
    - Verify the bank register latches the intended address bits and
      no unintended mirroring occurs.
    - Check timing and setup/hold margins relative to the Z80 bus.
    - Ensure the bank‑select output drives the RAM decode cleanly.


# FIRMWARE / ROM INVESTIGATION — TODO

[1] Cartridge ROM Space Exploration
    - Investigate whether firmware can open up the ROM region for
      external cartridges.
    - Determine if the ROM area is hard‑coded, mirrored, or gated.

[2] Boot‑Time ROM Checks
    - Examine the disassembly to see whether the ROM performs any
      calls, checks, or integrity tests on the cartridge ROM area
      during boot.
    - Identify any vectors or jump tables that might redirect into
      cartridge space.

[3] Continue Disassembly Work
    - Push further on the ROM disassembly to map:
        * startup vectors
        * memory‑map assumptions
        * any hidden or undocumented ROM hooks
    - Document all findings for future firmware patches.

