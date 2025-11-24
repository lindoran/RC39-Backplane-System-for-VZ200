These are the H. Neeman Digital Simulations for the GAL's for the VZ200 Clone.
All of the logic was built in digital a digital logic simulation tool by H.Neeman
his repository is here:

https://github.com/hneemann/Digital

Some of the builds require a patched version:

https://github.com/lindoran/Digital

This version allows the 'Q' input to be used as a regular chip. Most modernly
avalible GAL22V10, allow for pin 1 to be a general purpose IO pin.  As
its still testing I haven't done a formal pull request, also it would require
more work to turn the feature on and off... This is not the case at the moment.

All the simulations will work in a standard copy of digital, but will not
export to JED or CPL without the patched version particularly the decode GAL for
the RAM ROM expansion.

The Ram / ROM card requires a bodge to bring the A14/A15 lines through the gal for
banking.  This will be updated before the manufacturing files release.
