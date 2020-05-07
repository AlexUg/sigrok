# Kingst LA1010 support for Sigrok

To enable Kingst LA-1010 support in libsigrok there need perform these steps:<br>
<ol>
  <li>Get libsigrok sources (see docs for libsigrok).</li>
  <li>Copy directory 'kingst-la1010' to subdirectory 'src/hardware' in libsigrok source directory.</li>
  <li>Apply patches 'Makefile.am.patch' and 'configure.ac.patch' to corresponding files in libsigrok source directory.</li>
  <li>Build libsigrok (see docs for libsigrok).</li>
  <li>Use Python script 'fwextractor.py' for extracting firmwares from KingsVIS (Python v3.x). Firmwares will be extracted to current directory.</li>
  <li>Copy firmwares to directory '~/.local/share/sigrok-firmware/'. For Kingst-LA1010 the firmware 'fw01A2.hex' must be converted from Intel-HEX to binary with name 'LA1010.fw' and the firmware 'LA1010A_0' must be renamed to 'LA1010.bitstream' without any convertation. Convertation from Intel-HEX to binary will be performed if You answer 'y' when execution script 'fwextractor.py', but renaming is still required.</li>
</ol>
Extracting firmwares is avalible from KingsVIS executable which are provided with new version of Kingst software for Linux platform (tested on version 3.4.1).
