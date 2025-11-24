This is my RC39 based backplane for building home computer workalikes.  The focus right now is to buld a VZ200 Clone work alike.

Backplane / features: 
    - 5V input provided by a center positive barel jack 
    - Reset is open collector provided by a NPN transistor with a strong pull up for a very fast edge
    - Reset is a standard 555 timer reset circut.
    - mounting lugs for an IO shelft above the IO expansion header, all which are positioned in the back
      of the board. with accsess to the reset switch from the front.
    - 5 39 pin RC39 compliant connectors for expansion cards and a IO expansion header that matches the
      same pinout as the VZ200.  RC39 is RC2014 bus with 1 pin cut off so that the connecctor will fit on
      a 100mm x 100mm PCB.
    - None of the cards are more than 100mmx100mm, with all artwork being 2 sided or less, which should 
      keep cost low for ordering a full set of boards.

Current cards: 
   - CPU / CLOCK Board
   - SERIAL 6850 Card with GAL Selection logic
   - RAM / ROM Card with GAL Selection and banking logic
   - 6847 display, Dual port ram card, with GAL selection logic
   - 3v IO interface with beeper, tape and sound support. built in modern RP2040 IO interface.
     Card is front facing, with a clasic VZ style keyboard interface. RP2040 can be potentially
     used to control the computer keyboard via HID connection through its USB or I2C header.
