# Kingst LA1010 support for Sigrok

Although this project contains up-to-date sources, it is mainly used for bug reports.

For build 'sigrok' with Kingst LA-1010 support use sources from https://github.com/AlexUg/libsigrok (this is fork of original repo).
The support of Kingst LA-2016 is included in original repo. Implementation of LA-2016 is conflicting with LA-1010, so the support of LA-2016 is removed in fork.

For the device to work, you need firmwares that can be extracted from the KingstVIS version 3.5.x binary using a script 'fwextractor.py'.
This script extracts the firmwares for all devices supported by KingstVIS software.
By default, the firmwares are copied to the 'kingst' subdirectory of the '~/.local/share/sigrok-firmware/' directory (where driver LA-1010 will search firmwares).
You can specify another directory, a subdirectory 'kingst' will be created in it and the firmware files will be placed in it.
