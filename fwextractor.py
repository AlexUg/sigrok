'''
Created on 7 мая 2020 г.

@author: alexandr
'''

import sys
import os
import hashlib
import struct


from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection
import zlib
from shutil import copyfile
from builtins import int


'''

QT resources file system in ELF:

TreeEntry:

14 byte structs.
Nodes are at node_id * 14.
Root node is has node_id 0.

If flags has directory bit:
| offset | size | description                                                                                        |
|--------+------+----------------------------------------------------------------------------------------------------|
|      0 |    4 | offset of entry in `names` (big endian)                                                            |
|      4 |    2 | flags (1 = compressed, 2 = directory)                                                              |
|      6 |    4 | count of children                                                                                  |
|     10 |    4 | node id of first child (rest of children are sequential from this number, ordered by hash of name) |

Otherwise:
| offset | size | description                             |
|--------+------+-----------------------------------------|
|      0 |    4 | offset of entry in `names` (big endian) |
|      4 |    2 | flags (1 = compressed, 2 = directory)   |
|      6 |    2 | country                                 |
|      8 |    2 | language                                |
|     10 |    4 | offset of entry in payload              |



NameEntry:

| offset |    size | description                   |
|--------+---------+-------------------------------|
|      0 |       2 | length of name                |
|      2 |       4 | hash of name (for comparison) |
|      6 | len * 2 | name in UTF-16                |



PayloadEntry:

| offset | size   | description |
|--------+--------+-------------|
|      0 | 4      | length      |
|      4 | length | data        |

'''


debug = False

fwusb_str = 'fwusb'
fwfpga_str = 'fwfpga'

rsrc_symbols = {'qt_resource_struct' : None,
                'qt_resource_name' : None,
                'qt_resource_data' : None
                }


def getRsrcSymbolsOffsets(elffile):
    section = elffile.get_section_by_name('.symtab')
    if not section:
        print('No \'.symtab\' section found. Wrong ELF-file?')
        return False
    if isinstance(section, SymbolTableSection):
        for s in section.iter_symbols():
            if len(s.name) > 0:
                for n in rsrc_symbols:
                    if s.name.endswith(n):
                        if rsrc_symbols[n] is None:
                            rsrc_symbols[n] = s
                            break
                        else:
                            print('Duplicate symbol \'%s\' in table' % n)
                            return False
    return True



def readEntryName(name_bdata, name_offset):
    (name_len, name_hash) = struct.unpack(">HI", name_bdata[name_offset : name_offset + 6])
    name_offset += 6
    return name_bdata[name_offset : name_offset + name_len * 2].decode('utf-16be')


def decompressResource(resource_data):
    (data_size,) = struct.unpack(">I", resource_data[0 : 4])
    resource_data = resource_data[4 : ]
    return zlib.decompress(resource_data, zlib.MAX_WBITS)


def readEntry(symbols_bdata, entry_idx, processed_chldren):
    if entry_idx in processed_chldren:
        return None
    
    if debug:
        print('Process entry: %d' % (entry_idx))
    struct_bdata = symbols_bdata['qt_resource_struct']
    processed_chldren.append(entry_idx)
    struct_offset = entry_idx * 14
    (name_offset, flags) = struct.unpack(">IH", struct_bdata[struct_offset : struct_offset + 6])
    struct_offset += 6
    entry_name = readEntryName(symbols_bdata['qt_resource_name'], name_offset)
    if flags == 2:
        child_entries = {}
        (children_cnt, first_child_id) = struct.unpack(">II", struct_bdata[struct_offset : struct_offset + 8])
        children_last_idx = first_child_id + children_cnt
        while first_child_id < children_last_idx:
            child_entry = readEntry(symbols_bdata, first_child_id, processed_chldren)
            if not child_entry is None:
                child_entries[child_entry[0]] = child_entry[1]
            first_child_id += 1
        return (entry_name, child_entries)
    else:
        (country, lang, data_offset) = struct.unpack(">HHI", struct_bdata[struct_offset : struct_offset + 8])
        (data_size,) = struct.unpack(">I", symbols_bdata['qt_resource_data'][data_offset : data_offset + 4])
        data_offset += 4;
        entry_data = symbols_bdata['qt_resource_data'][data_offset : data_offset + data_size]
        if flags == 1:
            if debug:
                print('Entry %s is compressed' % (entry_name))
            entry_data = decompressResource(entry_data)
            
        return (entry_name, entry_data)
    
    return None


'''
    struct_entry:
        'name' => dir_entry | data_entry
    
    dir_entry:
        ( struct_entry )+
        
    data_entry:
        binary_data
'''
def getResources(symbols_bdata):
    result = {}
    processed_chldren = []
    size = len(symbols_bdata['qt_resource_struct'])
    entry_idx = 0
    while entry_idx < (size / 14):
        entry = readEntry(symbols_bdata, entry_idx, processed_chldren)
        if not entry is None:
            result[entry[0]] = entry[1]
        entry_idx += 1
    return result


def registerFirmware(rsrc_name, firmwares, fw_str, hr_str):
    if ('/' + fw_str + '/') in rsrc_name:
        base = os.path.basename(rsrc_name)
        if fw_str in firmwares:
            firmwares[fw_str].append(rsrc_name)
        else:
            firmwares[fw_str] = [rsrc_name]
        print("May be a firmware for %s : %s" % (hr_str, base))
    

def saveResources(resources, rsrc_name, firmwares):
    if isinstance(resources, dict):
        for k in resources:
            entry = resources[k]
            saveResources(entry, os.path.join(rsrc_name, k), firmwares)
    else:
        dirname = os.path.dirname(rsrc_name)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
        with open(rsrc_name, 'wb') as f:
            f.write(resources)
        print("Resource exported to: %s" % (rsrc_name))
        registerFirmware(rsrc_name, firmwares, fwusb_str, 'Cypress')
        registerFirmware(rsrc_name, firmwares, fwfpga_str, 'Spartan')


def hexToFw(src_file_name, dst_file_name):
    bin_data = bytearray()
    for i in range(0x4000):
        bin_data.append(0)
    with open(src_file_name, 'r') as src_file:
        for l in src_file:
            size = int(l[1:3], 16)
            addr = int(l[3:7], 16)
            rectype = int(l[7:9], 16)
            if rectype == 0:
                check_sum = (size + (addr & 0xFF) + ((addr >> 8) & 0xFF) + rectype) & 0xFF
                idx = 0
                while idx < size:
                    db = int(l[9 + idx * 2 : 11 + idx * 2 ], 16)
                    check_sum = (check_sum + db) & 0xFF
                    bin_data[addr + idx] = db
                    idx += 1
                db = (int(l[9 + idx * 2 : 11 + idx * 2 ], 16) + check_sum) & 0xFF
                if db != 0:
                    print('Wrong checksum \'%d\', expected \'%d\'' % (check_sum, db))
                 
    with open(dst_file_name, 'wb') as dst_file:
        dst_file.write(bin_data)
    

def saveFirmwares(firmwares, sigrok_fw_dir):
    sigrok_fw_dir = os.path.join(sigrok_fw_dir, 'kingst')
    if not os.path.isdir(sigrok_fw_dir):
        os.makedirs(sigrok_fw_dir)
    if fwusb_str in firmwares:
        cypress = firmwares[fwusb_str]
        for fw in cypress:
            base = os.path.basename(fw)
            with open(fw, 'rb') as file:
                data = file.read(3)
                if data == b':10':
                    fw_path = os.path.join(sigrok_fw_dir, base + '.hex')
                    fw2_path = os.path.join(sigrok_fw_dir, base + '.fw')
                    hexToFw(fw, fw2_path)
                    print("Cypress firmware \'%s\' was copied to \'%s\'" % (base, fw2_path))
                else:
                    fw_path = os.path.join(sigrok_fw_dir, base + '.fw')
                copyfile(fw, fw_path)
                print("Cypress firmware \'%s\' was copied to \'%s\'" % (base, fw_path))
    if fwfpga_str in firmwares:
        spartan = firmwares[fwfpga_str]
        for fw in spartan:
            base = os.path.basename(fw)
            fw_path = os.path.join(sigrok_fw_dir, base + '.bitstream')
            copyfile(fw, fw_path)
            print("Spartan firmware \'%s\' was copied to \'%s\'" % (base, fw_path))
    

def main():
    if len(sys.argv) != 2:
        print('Usage: {} <path/to/KingstVIS>'.format(os.path.basename(sys.argv[0])))
        return -1
    filename = sys.argv[1]
    
    sigrok_fw_dir = '~/.local/share/sigrok-firmware/'
    
    with open(filename, 'rb') as file:
        elffile = ELFFile(file)
        if getRsrcSymbolsOffsets(elffile):
            section = elffile.get_section_by_name('.rodata')
            if not section:
                print('No .rodata section found. Wrong ELF-file?')
                return
            if debug:
                print('Section name: \'%s\', type: %s' %( section.name, section['sh_type']))
            file.close()
        else:
            return
            
    
    symbols_bdata = {}
    with open(filename, 'rb') as file:
        for sname in rsrc_symbols:
            s = rsrc_symbols[sname]
            if debug:
                print('Symbol name: %s, section: %s, offset: %x, size: %x'
                              %( s.name, s['st_name'], s['st_value'], s['st_size']))
            foffset = ( s['st_value'] - section['sh_addr'] ) + section['sh_offset']
            file.seek(foffset)
            symbols_bdata[sname] = file.read(s['st_size'])
        file.close()
        
    resources = getResources(symbols_bdata)
    firmwares = {}
    saveResources(resources, '.', firmwares)
    
    if len(firmwares) > 0:
        print("Firmwares will be copied to \'%s\' dir or enter new full path" % (sigrok_fw_dir))
        ans = input("WARNING! This rewrite existing files in target dir!: ")
        if len(ans) > 0:
            sigrok_fw_dir = ans
        if sigrok_fw_dir[0:2] == '~/':
            home_dir = os.path.expanduser('~')
            sigrok_fw_dir = os.path.join(home_dir, sigrok_fw_dir[2:])
        saveFirmwares(firmwares, sigrok_fw_dir)
    
    print('Done')


if __name__ == '__main__':
    main()

 
    
