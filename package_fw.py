# Takes the firmware binary in build/ESFloppy.ino.bin and the header defined in fwVersion.h
# Writes the result to build/esfloppy_fw.bin and then deletes the original ESFloppy.ino.bin file

import os
import zlib

# To build the header, we need to first read the version number string from fwVersion.h from the #define FIRMWARE_VERSION line
with open("ESFloppy/fwVersion.h", "r") as fw_header:
    fw_version = None
    for line in fw_header:
        # Look for the line in the header file that defines the firmware version and split it into its component words
        # Then take the last word (the version number in quotes) and strip the quotes to just get the string
        if line.startswith("#define FIRMWARE_VERSION"):
            fw_version = line.split()[-1].strip('"')
            break

# Exit if the firmware version string wasn't found in the header file
if fw_version is None:
    raise RuntimeError("Error: Couldn't find firmware version string in fwVersion.h!")

# Also make sure it's at most 7 characters long since the header only has 8 bytes of space for it (7 plus the terminator)
assert len(fw_version) <= 7, "Error: Firmware version string is too long; must be at most 7 characters!"

# Now read the binary file into memory
fw_binary = open("build/ESFloppy.ino.bin", "rb").read()

# Compute the CRC32 of the file since we need to include that in the header as well
crc = zlib.crc32(fw_binary) & 0xFFFFFFFF

# Now build the header, which is in the following format:
# 8 bytes: "ESFloppy" magic string
# 8 bytes: Firmware version string
# 4 bytes: Firmware size in bytes (little-endian)
# 4 bytes: CRC32 of the firmware binary (also little-endian)

header = b"ESFloppy" \
        + fw_version.encode("ascii").ljust(8, b"\0") \
        + len(fw_binary).to_bytes(4, "little") \
        + crc.to_bytes(4, "little")

print(f"Packaging firmware version {fw_version} with size {len(fw_binary)} bytes and CRC32 {crc:08X}...")

# Now write the header and then the firmware to the output file
with open("build/esfloppy_fw.bin", "wb") as fw_output:
    fw_output.write(header)
    fw_output.write(fw_binary)

# And finally, delete the original firmware binary since we don't need it anymore
os.remove("build/ESFloppy.ino.bin")