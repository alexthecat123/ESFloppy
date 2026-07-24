// This file contains all the type definitions used throughout the ESFloppy codebase
#pragma once

// Pin definitions for the LisaFPGA onboard ESFloppy
#define LED 1
#define OLED_SCL 2
#define RDA 4
#define WRD 5
#define SNS 6
#define WRQ 7
#define HDS 8
#define PH3 9
#define PH2 10
#define PH1 11
#define PH0 12
#define SD_SCK 41 // MTDI 41
#define SD_MOSI 40 // MTDO 40
#define SD_MISO 13 // 13
#define SD_CS 16 // 32K_N 16
#define MT1 17
#define MT0 18
#define DR1 21
#define DR0 33
#define FPGA_0 34
#define FPGA_1 35
#define FPGA_2 36
#define FPGA_3 37
#define FPGA_4 38
#define FPGA_5 39
#define FPGA_6 40
#define FPGA_7 41
#define PWM 42
#define OLED_SDA 46

// The sizes of the data and tags on 400K and 800K disks
#define DATA_SIZE_400K 409600
#define DATA_SIZE_800K 819200
#define TAG_SIZE_400K 9600
#define TAG_SIZE_800K 19200

// Addresses for all the RMT-related registers
#define RMT_BASE 0x60016000
#define SYSTEM_BASE 0x600C0000
#define GPIO_BASE 0x60004000

#define RMT_CH0_FIFO RMT_BASE
#define RMT_CH0CONF0_REG (RMT_BASE + 0x20)
#define RMT_CH0STATUS_REG (RMT_BASE + 0x50)
#define RMT_SYS_CONF_REG (RMT_BASE + 0xC0)
#define RMT_INT_RAW_REG (RMT_BASE + 0x70)
#define RMT_INT_CLR_REG (RMT_BASE + 0x7C)
#define RMT_CH0_TX_LIM_REG (RMT_BASE + 0xA0)

#define SYSTEM_PERIP_CLK_EN0_REG (SYSTEM_BASE + 0x18)
#define SYSTEM_PERIP_RST_EN0_REG (SYSTEM_BASE + 0x20)

#define GPIO_FUNC4_OUT_SEL_CFG_REG (GPIO_BASE + 0x564)

// Lookup tables for the number of sectors per track and tachometer pulses per track
extern uint32_t sectorsPerTrack[80];
extern uint32_t tachPulsesPerTrack[80];

// A decoded sector is super simple: just the track, sector, side, format, and 524 bytes of data
// The only one of these that even needs explanation is the format byte, which is as follows:
// Bits [4:0] - Interleave factor: 2 = 2:1 interleave, 4 = 4:1 interleave
// Bit 5 - Set if double-sided disk, clear if not
// Bits [7:6] - Always 0 because GCR only uses 6 bits and this byte gets transferred as-is in the sector header
struct DecodedSector {
    uint8_t track;
    uint8_t sector;
    uint8_t side;
    uint8_t format;
    uint8_t data[524];
};

// A GCR-encoded sector is a lot more complex, since it has all the sync bytes, prologues, epilogues, checksums, and GCR-encoded data
struct GcrSector {
    // First there's a header that comes before the data itself; it contains info about the sector
    // Before the header though, there are 6 sync bytes that the floppy state machine looks for to get in sync
    uint8_t headerSync[6] = {0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF};
    // And then a prologue to mark the start of the header
    uint8_t headerPrologue[3] = {0xD5, 0xAA, 0x96};
    // Then the actual header fields; first we have the low 6 bits of the track number
    // Remember, everything here must fit in 6 bits since this is GCR-encoded data
    uint8_t loTrack;
    // Then we have the sector number
    uint8_t sector;
    // Then a byte that contains the high bit of the track number as well as the side number
    // The side number (0 = side 0, 1 = side 1) is in bit 5, and the high track bit is in bit 0, all other bits are 0
    uint8_t hiTrackSide;
    // Then we have the format byte, which is transferred as-is from the decoded sector
    uint8_t format;
    // The header checksum is next, and it's just a simple XOR of the previous four header bytes
    uint8_t headerChecksum;
    // Finally, we have an epilogue to mark the end of the header
    uint8_t headerEpilogue[3] = {0xDE, 0xAA, 0xFF}; // Epilogue is only 2 bytes, the third is the don't care "heads off" byte
    // Then another set of 6 sync bytes to mark the start of the data field
    uint8_t dataSync[6] = {0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF};
    // A data prologue
    uint8_t dataPrologue[3] = {0xD5, 0xAA, 0xAD};
    // And then the sector number again for some reason (not sure why they put it here and in the header)
    uint8_t sector_again;
    // And finally the data itself, which is 699 bytes of GCR-encoded data (corresponding to 524 bytes of decoded data)
    uint8_t data[699];
    // Then a 4-byte checksum which is just a simple sum of all the data bytes
    uint8_t dataChecksum[4];
    // And last but not least, a data epilogue
    uint8_t dataEpilogue[3] = {0xDE, 0xAA, 0xFF}; // Epilogue is only 2 bytes, the third is the don't care "heads off" byte
};

// Each RMT data item is 32 bits:
// Bit 31 - Level of first part of the pulse
// Bits [30:16] - Duration of first part of the pulse in RMT clock ticks
// Bit 15 - Level of second part of the pulse
// Bits [14:0] - Duration of second part of the pulse in RMT clock
struct RMTDataItem {
    uint16_t duration0 : 15;
    uint16_t level0 : 1;
    uint16_t duration1 : 15;
    uint16_t level1 : 1;
};

// A sector in RMT format
struct RMTSector {
    // Each GCR sector it 733 bytes long total, and each byte becomes 8 RMT data items (1 per bit)
    RMTDataItem data[733 * 8];
};

// An enum for whether the drive is a 400K or 800K drive
enum DriveType {
    Drive400 = 0,
    Drive800 = 1
};

// One for the image format, DC42 or raw
enum ImageType {
    DC42,
    RAW
};

// And one for the head step direction, in (towards spindle) or out (away from spindle)
enum StepDirection {
    IN = false,
    OUT = true
};

// The structure of a DC42 disk image header
struct DC42Header {
    uint8_t nameLength;
    char volumeName[63];
    uint32_t dataSize;
    uint32_t tagSize;
    uint32_t dataChecksum;
    uint32_t tagChecksum;
    uint8_t diskEncoding;
    uint8_t diskFormat;
    uint16_t dc42Magic;
};

// A struct that contains all the current disk image's metadata
// So the dc42 header, but also the image type, drive type, whether tags are present, and whether a disk is inserted
struct DiskImageMetadata {
    DC42Header header;
    ImageType imageType;
    DriveType driveType;
    bool tagsPresent;
    bool diskInserted = false;
};
