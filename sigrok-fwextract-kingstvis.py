#!/usr/bin/python3
#!/usr/bin/python3
##
## This file is part of the sigrok-util project.
##
## Copyright (C) 2017 Alexandr Ugnenko <ugnenko@mail.ru>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##


import sys
import os
import hashlib
import struct


'''
LA_MODEL_NAME => {    
                    SHA256 => {
                                FW => ( start, size ),
                                BIN => ( start, size )
                                }
                }
'''
fw_offsets_dict = {
    "LA1010" : { "e46c7a334b81769535bef396515fe2f1a5b2888f7a9963a5f34dfba8902b920f" :
                    { "FW" : ( 0x323F8, 0x1948 ), "BIN" : ( 0x13A58, 0x1E9A0 ) }
                }
    }

def createFileName(laname, ext, dirname, basename):
    result = laname + '.' + ext
    if result == basename:
        result = basename + '.' + ext
    return (os.path.join(dirname, result), result);

def main():
    if len(sys.argv) != 2:
        print('Usage: {} <path/to/LAxxxx.dll>'.format(os.path.basename(sys.argv[0])))
        return -1
    filename = sys.argv[1]
    dirname = os.path.dirname(filename)
    base = os.path.basename(filename)
    laname = os.path.splitext(base)[0].upper()
    (hexfilepath, hexfilename) = createFileName(laname, 'hex', dirname, base)
    (fwfilepath, fwfilename) = createFileName(laname, 'fw', dirname, base)
    (binfilepath, binfilename) = createFileName(laname, 'bitstream', dirname, base)
    try:
        fw_offsets = fw_offsets_dict[laname];
    except KeyError:
        print('LA \'{}\' isn\'t supported'.format(laname))
        return -1
    with open(filename, 'rb') as f:
        data = f.read()
    sha256 = hashlib.sha256(data).hexdigest().lower()
    try:
        fw_offsets = fw_offsets[sha256];
    except KeyError:
        print('This version of \'{}\' library isn\'t supported. Wrong SHA256'.format(base))
        return -1
    out_bytes = data[fw_offsets["BIN"][0] : fw_offsets["BIN"][0] + fw_offsets["BIN"][1]]
    with open(binfilepath, 'wb') as f:
         f.write(out_bytes)
    print('Spartan bitstream saved to {}'.format(binfilepath))
    out_bytes = data[fw_offsets["FW"][0] : fw_offsets["FW"][0] + fw_offsets["FW"][1]]
    idx = 0
    fw_out_data = bytearray()
    for i in range(0x4000):
        fw_out_data.append(0)
    with open(hexfilepath, 'w') as f:
        while idx < fw_offsets["FW"][1]:
            check_sum = (out_bytes[idx] + out_bytes[idx + 1] + out_bytes[idx + 2]) & 0xFF
            (addr, size) = struct.unpack("<hb", out_bytes[idx : idx + 3])
            idx += 3
            fw_data = out_bytes[idx : idx + size]
            hex_record = ':{0:02X}{1:04X}00'.format(size, addr)
            i = 0
            while i < size:
                hex_record += '{0:02X}'.format(fw_data[i])
                fw_out_data[addr + i] = fw_data[i]
                check_sum = (check_sum + fw_data[i]) & 0xFF;
                i += 1
            check_sum = (-check_sum) & 0xFF;
            hex_record += '{0:02X}'.format(check_sum) + '\n'
            f.write(hex_record)
            idx += size
        f.write(':00000001FF')
    print('Cypress firmware in Intel HEX format saved to {}'.format(hexfilepath))
    with open(fwfilepath, 'wb') as f:
        f.write(fw_out_data)
    print('Cypress firmware in binary format saved to {}'.format(fwfilepath))

main()
