# Kingst LA1010 support for Sigrok

To enable Kingst LA-1010 support in libsigrok there need perform these steps:<br>
<ol>
  <li>Get libsigrok sources (see docs for libsigrok).</li>
  <li>Copy directory 'kingst-la1010' to subdirectory 'src/hardware' in libsigrok source directory.</li>
  <li>Apply patches 'Makefile.am.patch' and 'configure.ac.patch' to corresponding files in libsigrok source directory.</li>
  <li>Build libsigrok (see docs for libsigrok).</li>
  <li>Use Python script 'fwextractor.py' for extracting firmwares from KingsVIS (Python v3.x). Firmwares will be extracted to current directory.</li>
  <li>Copy firmwares to directory '~/.local/share/sigrok-firmware/'. Kingst-LA1010 uses the firmware 'fw01A2.hex' or 'fw01A2.fw' and the firmware 'LA1010A_0.bitstream' (after extracting may not have extension '.bitstream' -- must be renamed). If You answer 'y' when executing the script 'fwextractor.py' all operations (rename, format convertation etc.) will be performed.</li>
</ol>
Extracting firmwares is avalible from KingsVIS executable which are provided with version 3.4.x of Kingst software for Linux platform (tested on version 3.4.1).
