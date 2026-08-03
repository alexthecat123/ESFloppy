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
#define SD_SCK 41
#define SD_MOSI 40
#define SD_MISO 13
#define SD_CS 16
#define MT1 17
#define MT0 18
#define DR1 21
#define DR0 33
#define LEFT 34
#define SEL 35
#define RIGHT 36
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

// Same for Twiggy disks
#define DATA_SIZE_TWIGGY 871424
#define TAG_SIZE_TWIGGY 20424

// A dc42 image is a 400K image if the encoding byte is 0
#define DC42_400K_ENCODING 0x00
// It's 800K if the encoding byte is 1
#define DC42_800K_ENCODING 0x01
// And it's a Twiggy image if the encoding byte is 0x54
#define DC42_TWIGGY_ENCODING 0x54

// The size of each GCR-encoded sector in bits
#define BITS_PER_SECTOR (sizeof(GcrSector) * 8)

// The header and data prologues for a GCR sector
#define HEADER_PROLOGUE 0xD5AA96
#define DATA_PROLOGUE 0xD5AAAD

// The timings (in CPU cycles) for the various bit reception cells during a write operation
// These are used to determine what bit pattern was sent by the Lisa based on the time between edges on WRD
#define BIT_TIME_1 717 // 2us with some tolerance (1.5 bit cells)
#define BIT_TIME_01 1195 // 4us with some tolerance (2.5 bit cells)
#define BIT_TIME_001 1673 // 6us with some tolerance (3.5 bit cells)

// LUTs describing the drive geometry and tachometer pulse counts for the various different drive types that we support
// Go check geometry.cpp for the actual definitions and meanings of these arrays
extern uint32_t sectorsPerTrackSony[80];
extern uint32_t sectorsPerTrackTwiggy[46];
extern uint32_t tachPulsesPerTrackMac[80];
extern uint32_t tachDividerPerTrackMac[80];
extern uint32_t tachPulsesPerTrackLisa[80];
extern uint32_t tachDividerPerTrackLisa[80];

// This enum defines the different commands that we can send to the SD card task that runs on the other core
enum SdTaskCommand {
    READ_TRACK, // Just read a track and encode it to GCR
    WRITE_READ_TRACK, // Write out a track and then read in another
    CLOSE_IMAGE // Close the disk image file with closeImage()
};

// This struct is used to pass data to the SD card task
struct SdTaskInterface {
    uint8_t writeTrack; // What track to write
    uint8_t readTrack; // What track to read
    SdTaskCommand command; // What command to execute
    bool start; // We set this high to tell the task to start processing our requect
    bool finished; // The task sets this high when it's finished processing our request
    bool initDone; // This goes high when the task is done initializing itelf
};

// A decoded sector is super simple: just the track, sector, side, format, and 524 bytes of data
// The only one of these that even needs explanation is the format byte, which is as follows for a Sony disk:
// Bits [4:0] - Interleave factor: 2 = 2:1 interleave, 4 = 4:1 interleave
// Bit 5 - Set if double-sided disk, clear if not
// Bits [7:6] - Always 0 because GCR only uses 6 bits and this byte gets transferred as-is in the sector header
// The interpretation for a Twiggy disk is significantly simpler; it can only be 0, 1, or 2, representing which computer the disk was formatted for
// 0 means Apple II or ///, 1 means Lisa, and 2 means Mac, so realistically all we'll ever see is 1
struct DecodedSector {
    uint8_t track;
    uint8_t sector;
    uint8_t side;
    uint8_t format;
    uint8_t data[524];
};

// A GCR-encoded sector is a lot more complex, since it has all the sync bytes, prologues, epilogues, checksums, and GCR-encoded data
// Conveniently enough, the GCR sector encoding is identical between Sony and Twiggy disks
// The only difference is the interpretation of a few of the header fields
struct GcrSector {
    // First there's a header that comes before the data itself; it contains info about the sector
    // Before the header though, there are 6 sync bytes that the floppy state machine looks for to get in sync
    // But we need an inter-sector gap to make sure that the FDC is able to keep up at higher interleaves, so do 50 bytes instead
    uint8_t headerSync[75] = {0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC};
    // And then a prologue to mark the start of the header
    uint8_t headerPrologue[3] = {0xD5, 0xAA, 0x96};
    // Then the actual header fields; first we have the low 6 bits of the Sony track number (which is the whole track number for a Twiggy)
    // Remember, everything here must fit in 6 bits since this is GCR-encoded data
    uint8_t loTrack;
    // Then we have the sector number
    uint8_t sector;
    // Then a byte that contains the high bit of the track number as well as the side number for Sony, and just the side number for Twiggy
    // For Sony, the side number (0 = side 0, 1 = side 1) is in bit 5, and the high track bit is in bit 0, all other bits are 0
    // For Twiggy, the side number is in bit 0, all other bits are 0
    uint8_t hiTrackSide;
    // Then we have the format byte, which has different meanings between Sony and Twiggy, as described with DecodedSector above
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

// A struct that contains some parameters related to a track
// The only reason we really need this is because we need to pass these to a couple different functions
// And it would be a pain to pass them all individually
struct TrackParams {
    bool motorOn; // Whether the motor is on or not
    uint32_t pendingTrackToWrite; // The track number we want to write to (if any)
    SdTaskCommand pendingCommand; // The command we want to send to the SD card task (if any)
    bool pendingDispatch; // A flag that indicates whether there's a pending command to dispatch to the SD card task
    int32_t currentTrack; // The track number we're currently on; signed so that we can reach Twiggy track -1
    bool trackChanged; // A flag that indicates whether the track has changed since the last time we checked
    uint32_t stashCount; // How many sectors are currently in the write stash
    bool dirty; // A flag that indicates whether the current track has been modified and needs to be written out
};

// An enum for whether the drive is a 400K Sony, 800K Sony, or a Twiggy
enum DriveType {
    Drive400 = 0,
    Drive800 = 1,
    DriveTwiggy = 2
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

// And yet another for what state of a write operation we're in
enum WriteState {
    PROLOGUE, // We're still searching for the prologue
    HEADER, // It was a header prologue, so we're now reading the header
    DATA // It was a data prologue, so we're now reading the data
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
// So the dc42 header, but also the image type, drive type, whether tags are present, whether a disk is inserted, and drive index
struct DiskImageMetadata {
    DC42Header header;
    ImageType imageType;
    DriveType driveType;
    bool tagsPresent;
    bool diskInserted = false;
    uint32_t startAddress; // The starting address of the disk image in the SD card file system
    uint32_t endAddress; // And the end address
    uint32_t driveIndex; // The index of the drive that this disk image is currently inserted into (0 or 1); needed for Twiggy support
};

// We can't pass multiple parameters to a task, so we need to make a struct to hold all the parameters that we want to pass to the SD task
struct SdTaskParams {
    volatile SdTaskInterface* sdTaskInterface; // A pointer to the SdTaskInterface struct is the first param
    GcrSector (&trackBufferGCR)[2][22]; // We also need to pass in references to the two track buffers
    DecodedSector (&trackBufferDecoded)[2][22];
    DiskImageMetadata* diskMetadata; // And finally, a pointer to the disk metadata struct
};