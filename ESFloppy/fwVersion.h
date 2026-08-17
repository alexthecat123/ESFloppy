// ESFloppy's current firmware version and a struct representing a header for ESFloppy firmware update files
#define FIRMWARE_VERSION "1.0"

// Firmware Changelog:
// 1.0 - Initial release

struct FirmwareUpdateHeader {
    char magicString[8]; // A magic string "ESFloppy" that identifies this as an ESFloppy firmware update file
    char versionString[8]; // The firmware version string for this update file
    uint32_t firmwareSize; // The size of the firmware binary in bytes
    uint32_t firmwareCRC; // A 32-bit CRC of the firmware binary
};