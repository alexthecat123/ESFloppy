#include <Arduino.h>
#include "SdFat.h"
#include "diskLib.h"
#include "GCRLib.h"
#include "GPIO.h"
#include "RMT.h"
#include "types.h"

// Lookup table for number of sectors per track for each of the 80 tracks on a standard 400K/800K floppy
uint32_t sectorsPerTrack[80] = {
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, // Tracks 0-15
    11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, // Tracks 16-31
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, // Tracks 32-47
    9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9, // Tracks 48-63
    8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8, // Tracks 64-79
};

// Another LUT for the tachometer pulse frequency that's needed for each track
// Are these right? One source (the 800K drive spec) says this, another (the 400K spec) is slightly different...
// And I can't find the final source, but I got something from somewhere else that breaks it down quite differently:
/*
    Track/RPM pairings
    0...9 363
    10...25 393
    26...40 429
    41...55 472
    56...71 524
    72...79 590
*/
uint32_t tachPulsesPerTrack[80] = {
    394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, // Tracks 0-15
    429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, // Tracks 16-31
    472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, // Tracks 32-47
    525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, // Tracks 48-63
    578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, // Tracks 64-79
};

// Watchdog timer write enable register and value
#define TIMG1_WDT_WE 0x050D83AA1
#define TIMG1_WDT_WE_REG 0x3FF60064

// Watchdog timer configuration register and enable bit
#define TIMG1_WDT_CONF_REG 0x3FF60048
#define TIMG1_WDT_EN 1 << 31

SPIClass SD_SPI(HSPI); // These two lines make sure that we use hardware SPI at 20MHz for the SD card
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(20), &SD_SPI)

SdFat32 SDCard; // The SD card object
File32 disk; // The disk image that ESFloppy is using
FatFile rootDir; // The root directory of the SD card

static DecodedSector trackBufferDecoded[2][12]; // Buffer for decoded sectors for each side of the disk (2 sides, max 12 sectors per track)
static GcrSector trackBufferGCR[2][12]; // Buffer for GCR-encoded sectors for each side of the disk


uint32_t currentSector = 2; // Start with sector 2 since we preloaded sectors 0 and 1
uint32_t inSectorIndex = 0;
uint32_t rmtBufferIndex = 0;
uint8_t currentTrack = 0;
bool loadZero = false;
bool loadOne = false;

//static RMTSector trackBufferRMT[2][2]; // Double-sided double buffer for RMT sectors // [2][2]

DiskImageMetadata diskMetadata;

// This function preloads sectors 0 and 1 of both sides into the RMT buffers
// We call it whenever we open a new disk image or step to a new track to ensure that the RMT has data to send right away
void preloadSectors() {
    // First, encode those two sectors on each side into GCR format and stick them in the RMT buffer
    //convertGCRToRMT(&trackBufferGCR[0][0], &trackBufferRMT[0][0]); // Preload the first sector on side 0
    //convertGCRToRMT(&trackBufferGCR[0][1], &trackBufferRMT[0][1]); // And the second
    //convertGCRToRMT(&trackBufferGCR[1][0], &trackBufferRMT[1][0]); // Preload the first sector on side 1
    //convertGCRToRMT(&trackBufferGCR[1][1], &trackBufferRMT[1][1]); // And the second
    // Now clear out the state of the RMT so it starts sending from the beginning of sector 0 on side 0
    currentSector = 2; // Reset the current sector to 2 since we preloaded sectors 0 and 1
    inSectorIndex = 0; // And the in-sector index to 0
    rmtBufferIndex = 0; // Also reset the RMT buffer index to 0
}

// This function will get called on loop whenever the Lisa is accessing the floppy's read data register
// It checks if the RMT needs more data, and if so, fills it from our double buffer of RMTSectors
// It also handles loading the next sector into the other buffer as needed
void transmitTrack(bool side) {
    // Check if the RMT needs more data
    if (rmtNeedsData()) {
        // If so, we need to clear the interrupt flag that was set to tell us it was empty
        clearRMTInt();
        // Now copy the next 96 RMTDataItems from the current sector buffer to the RMT memory FIFO
        for(uint32_t i = 0; i < 96; i++) {
            // If the inSectorIndex has reached the end of the sector, we need to move to the next one
            if (inSectorIndex >= 5864) {
                // So move to buffer location 1 if we're in 0
                if (rmtBufferIndex == 0) {
                    // And set a flag to indicate we need to load the next sector into buffer 0
                    loadZero = true;
                    loadOne = false;
                    rmtBufferIndex = 1;
                }
                // And to buffer location 0 if we're in 1
                else {
                    // And set a flag to indicate we need to load the next sector into buffer 1
                    loadOne = true;
                    loadZero = false;
                    rmtBufferIndex = 0;
                }
                // Then increment the current sector, wrapping back to 0 if we reach the end of the track
                currentSector++;
                if (currentSector >= sectorsPerTrack[currentTrack]) {
                    currentSector = 0;
                }
                // And reset the in-sector index since we just started a new one
                inSectorIndex = 0;
            }
            // Actually copy the data now, making sure to select the correct side's buffer
            //REG_WRITE(RMT_CH0_FIFO, *((uint32_t*)&trackBufferRMT[side][rmtBufferIndex].data[inSectorIndex++]));
        }
        // Now that we've copied the data, we can check if we need to load the next sector into the other buffer
        // It's the same for buffers zero or one, just with different trackBufferRMT indices
        // And don't forget to load both sides, not just one
        // If it's not actually a double-sided disk, then who cares, we'll get garbage RMT data for side 1 which will simulate reading the wrong side of a 400K disk pretty well
        if (loadZero) {
            loadZero = false;
            //convertGCRToRMT(&trackBufferGCR[0][currentSector], &trackBufferRMT[0][0]);
            //convertGCRToRMT(&trackBufferGCR[1][currentSector], &trackBufferRMT[1][0]);
        }
        else if (loadOne) {
            loadOne = false;
            //convertGCRToRMT(&trackBufferGCR[0][currentSector], &trackBufferRMT[0][1]);
            //convertGCRToRMT(&trackBufferGCR[1][currentSector], &trackBufferRMT[1][1]);
        }
    }
}

void setup() {
    init(); // Init the ESP32 Arduino core
    // Just like ESProFile, we need to disable the interrupt watchdog timer
    // For the sake of speed, we have to disable interrupts throughout our entire program, and the watchdog would trigger and break everything
    REG_WRITE(TIMG1_WDT_WE_REG, TIMG1_WDT_WE); // Enable writing to the watchdog timer registers
    REG_CLR_BIT(TIMG1_WDT_CONF_REG, TIMG1_WDT_EN); // And clear the timer's enable bit to disable it
    //noInterrupts(); // Now that it's safe to do so, disable interrupts for the rest of the program
    Serial.begin(115200); // Start serial comms for debugging
    Serial.println("Starting ESFloppy...");
    initRMT(); // Initialize the RMT peripheral for floppy data transmission
    GPIOControl(); // Give the GPIO (not RMT) control over the RDA pin initially
    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); // Start comms with the SD card using our hardware SPI instance
    //initPins(); // Set all of ESFloppy's pins to the correct direction and state
    if (!SDCard.begin(SD_CONFIG)) { // Initialize the SD card with our hardware SPI instance
        Serial.println("SD card initialization failed! Halting..."); // And print an error/go into an infinite loop on failure
        while(1);
    }
    rootDir.open("/"); // Then open the card's root directory
    if (!openImage("800k_mw2install.dc42", &disk, &diskMetadata)) { // And try opening a disk image file
        Serial.println("Failed to open disk image! Halting..."); // Give another error/infinite loop on failure
        while(1); // If we fail to open the image, just hang here
    }
    // Now do a test read and write of track 0
    readTrack(0, &disk, trackBufferDecoded, &diskMetadata); // Read track 0 into the decoded buffer
    // Now print out the contents of every sector (on both sides of the disk) for debugging, and divide it up by side and sector
    /*for (int side = 0; side < 2; side++) {
        for (int sector = 0; sector < sectorsPerTrack[0]; sector++) {
            Serial.print("Side ");; Serial.print(side); Serial.print(", Sector "); Serial.print(sector); Serial.println(":");
            for (int i = 0; i < 524; i++) {
                if (i % 16 == 0) {
                    Serial.println(); // New line every 16 bytes
                }
                uint8_t byteToPrint = trackBufferDecoded[side][sector].data[i];
                if (byteToPrint < 0x10) {
                    Serial.print("0"); // Leading zero for single-digit values
                }
                Serial.print(byteToPrint, HEX); Serial.print(" ");
            }
            Serial.println(); Serial.println(); // Two new lines after each sector
        }
    }*/
    encodeTrackToGCR(0, trackBufferDecoded, trackBufferGCR, &diskMetadata);
    preloadSectors(); // Preload sectors 0 and 1 of both sides into the RMT buffers
    // For testing purposes, let's modify sector 0's data a bit
    // Invert all 524 bytes of all the sectors on both sides of track 0
    /*for (int side = 0; side < 2; side++) {
        for (int sector = 0; sector < sectorsPerTrack[0]; sector++) {
            for (int i = 0; i < 524; i++) {
                trackBufferDecoded[side][sector].data[i] = ~trackBufferDecoded[side][sector].data[i];
            }
        }
    }*/
    //writeTrack(0, &disk, trackBufferDecoded, &diskMetadata); // Write the modified track back to the disk image
    closeImage(&disk, &diskMetadata); // Close the image
    Serial.println("ESFloppy is ready!"); // If all this succeeds, print a ready message
}


bool prevLSTRB = 0;
bool currLSTRB = 0;

StepDirection stepDirection = OUT;
bool stepComplete = true;
bool motorOn = false;

bool ejectPending = false;
uint16_t ejectStartTime;
uint32_t tachFreq = 0;

bool dirty = false;

void loop() {
    // Don't do anything unless the drive is enabled (DR1 low)
        // When it's enabled, we check for commands to the drive
        // The read/write register comes in on CA2-CA0 (PH2-PH0) and the low bit (SEL) is on HDS
        // As long as the drive is enabled and LSTRB (PH3) is low, we put the contents of that register on the bus
        // But when LSTRB goes high, we write whatever's on the bus into a register
        // Now the data to write is on CA2 (PH2) and the register is determined by CA1-CA0 (PH1-PH0) and HDS
        // Let's edge-trigger on LSTRB going high instead of level-trigger
        // Reading EJECT always returns 0, to eject a disk you have to write a 1 to EJECT but LSTRB has to be high for at least 500ms
        // MOTORON shouldn't do anything unless a disk is inserted
        // The host sets STEP to 0 to step the heads, but the drive must set it back to 1 again within 12ms. I'm guessing the host polls for this, and since we're planning on writing to the SD card during steps, we need to make sure to only set it back to 1 after the SD card write is finished
    if (readDR1() == 0) { // If the drive is enabled, then we need to check for commands
        currLSTRB = readPH3(); // Read the current state of LSTRB (PH3)
        if (currLSTRB == 0) { // If LSTRB is low, then we need to put the selected register on the bus
            // Figure out which register the host wants to read from
            uint8_t regNum = (readPH2() << 3) | (readPH1() << 2) | (readPH0() << 1) | (readHDS() << 0);
            // Now we need to figure out what data to send for that register
            if (regNum != 7) { // If we're not reading the TACH register, make sure to stop any ongoing tach pulse generation
                ledcDetach(RDA);
            }
            if (regNum != 8 && regNum != 9) { // If we're not reading RDDATA for either head, make sure to give control of RDA back to GPIO (not the RMT)
                GPIOControl();
            }
            switch (regNum) {
                case 0: // /DIRTN register (head step direction)
                    writeRDA(stepDirection);
                    break;
                case 1: // /CSTIN register (false if disk inserted, else true)
                    writeRDA(diskMetadata.diskInserted ? 0 : 1);
                    break;
                case 2: // /STEP register (host sets low to step heads, drive sets high when step is complete)
                    writeRDA(stepComplete);
                    break;
                case 3: // /WRPROT register (low if disk is write-protected, else high)
                    writeRDA(1); // We never have write protection enabled, so always return high
                    break;
                case 4: // /MOTORON register (low if motor is on, else high)
                    writeRDA(motorOn ? 0 : 1);
                    break;
                case 5: // /TK0 register (low if heads are on track 0, else high)
                    writeRDA((currentTrack == 0) ? 0 : 1);
                    break;
                case 6: // /EJECT register (write-only, always returns 0)
                    writeRDA(0);
                    break;
                case 7: // /TACH register (produces 60 pulses per revolution when motor is on)
                    // Use our LUT to figure out how many tach pulses per second we need to generate for the current track
                    tachFreq = tachPulsesPerTrack[currentTrack];
                    // We'll use the LEDC peripheral to generate these pulses
                    // But only output TACH pulses if the motor is on (DUH)
                    if (motorOn == true) {
                        ledcAttach(RDA, tachFreq, 1); // Attach to RDA pin, with the proper frequency and 1-bit resolution
                        ledcWrite(RDA, 1); // And set the duty cycle to 50% for a square wave
                    }
                    // If the motor is off, just output a constant low
                    else {
                        writeRDA(0);
                    }
                    break;
                case 8: // RDDATA register for head 0
                    RMTControl(); // First hand over control of RDA to the RMT so it can send data to the Lisa
                    // Assuming the motor is running, the drive should already be sending data
                    // The calls to transmitTrack just ensure that the RMT's buffer always stays full
                    // The 0 means side 0
                    transmitTrack(0);
                    break;
                case 9: // RDDATA register for head 1; only valid for 800k drives
                    // Do the same thing we did for RDDATA for side 0, except now it's side 1
                    RMTControl();
                    transmitTrack(1);
                    break;
                case 12:
                case 13: // SIDES register (duplicated on both addresses 12 and 13); returns 0 for 400K drives, 1 for 800K drives
                    writeRDA(diskMetadata.driveType == Drive800 ? 1 : 0);
                    break;
                case 14:
                case 15: // /DRVIN register (duplicated on both addresses 14 and 15); hard-coded to 0 as a way for the host to detect a drive connected
                    writeRDA(0);
                    break;
                default: // If an invalid register is selected, just return 0
                    writeRDA(0);
                    break;
            }
        }
        // Otherwise, if LSTRB (PH3) is on a rising edge, or LSTRB is just high period while an eject is pending, then we need to write to a register
        else if ((prevLSTRB == 0 && currLSTRB == 1) || (currLSTRB == 1 && ejectPending == true)) {
            // So figure out which register the host wants to write to
            uint8_t regNum = (readPH1() << 2) | (readPH0() << 1) | (readHDS() << 0);
            bool regData = readPH2(); // The data to write is on CA2 (PH2)
            // Now write the data to the selected register
            switch (regNum) {
                case 0: // /DIRTN register (head step direction, low for IN, high for OUT)
                    stepDirection = regData ? OUT : IN;
                    break;
                case 2: // /STEP register (host sets low to step heads, drive sets high when step is complete)
                    if (regData == 0 && stepComplete == true) { // If the host is trying to step the heads and we're not already in the middle of a step
                        stepComplete = false; // Mark that a step is in progress
                        // Before we step, check the dirty bit to see if the current track was modified
                        if (dirty == true) {
                            // If so, write the current track back to the disk image before we seek away
                            decodeTrackFromGCR(currentTrack, trackBufferGCR, trackBufferDecoded, &diskMetadata);
                            writeTrack(currentTrack, &disk, trackBufferDecoded, &diskMetadata);
                            dirty = false; // And clear the dirty bit
                        }
                        // Now we can actually perform the step
                        if (stepDirection == IN) { // If stepping IN, decrement the track
                            if (currentTrack > 0) {
                                currentTrack--;
                            }
                        }
                        else { // If stepping OUT, increment the track
                            if (currentTrack < 79) {
                                currentTrack++;
                            }
                        }
                        // Now that we've updated the track number, we need to read in all the sectors for this new track from the disk image
                        readTrack(currentTrack, &disk, trackBufferDecoded, &diskMetadata);
                        encodeTrackToGCR(currentTrack, trackBufferDecoded, trackBufferGCR, &diskMetadata);
                        // And preload a couple sectors into our RMT buffer to get ready for reading
                        preloadSectors();
                        stepComplete = true; // Once they're read in, mark that the step is complete
                    }
                    break;
                case 4: // /MOTORON register (low to turn motor on, high to turn motor off)
                    motorOn = (regData == 0);
                    // We also only want the "read head" to send out a bitstream when the motor is on
                    // So we implement that by starting and stopping the RMT
                    if (motorOn) {
                        startRMT(); // Start the RMT if the motor is turned on
                    }
                    else {
                        stopRMT(); // And stop it if the motor is turned off
                    }
                    break;
                case 6: // EJECT register (write-only, write a 1 to eject the disk)
                    // The catch here is that the 1 must be held for at least 500ms to actually eject the disk
                    // So we need to start a timer when we see a 1 written, and if it stays 1 for 500ms, we eject the disk
                    // Start by checking if the written data is a 1 and if we're not already pending an eject
                    if (regData == 1 && ejectPending == false) {
                        ejectPending = true; // If so, mark that an eject is pending
                        ejectStartTime = millis(); // And record the start time
                    }
                    else if (regData == 0) { // If the host writes a 0, we need to cancel any pending eject
                        ejectPending = false;
                    }
                    else if (ejectPending == true) { // Otherwise, if an eject is pending, check if 500ms has passed
                        if (millis() - ejectStartTime >= 500) {
                            // If so, eject the disk
                            diskMetadata.diskInserted = false;
                            motorOn = false; // Turn off the motor too
                            ejectPending = false;
                            Serial.println("Disk ejected.");
                        }
                    }
                    break;
                default:
                    // Do nothing for writes to other (invalid) registers
                    break;
            }
        }
        prevLSTRB = currLSTRB; // Update prevLSTRB for the next iteration
    }
    else {
        // If the drive is disabled, make sure the step direction is OUT (this is the idle state)
        stepDirection = OUT;
        // And sit here until it's enabled again
    }
}

// What do do in terms of reading from a disk:
// When the user inserts a disk or we seek to a new track, check what track we're on and figure out how many sectors are on that track
// Then go ahead and read all of those sectors from the disk image into a RAM buffer of DecodedSector structs
// If the disk is double-sided, the we need 2 buffers, one for each side
// If the disk has tags, then they're the first 12 bytes of each sector, if not, then make the first 12 bytes zero
// Now go ahead and encode all our DecodedSector structs into GcrSector structs in another RAM buffer
// Then make another RAM buffer of RMTDataItem structs to hold the RMT data for the entire track
// And convert each GcrSector into its corresponding RMTDataItem sequence in that buffer
// Now we finally clear that bit to say "seek complete"
// When the host starts reading data, we just start the RMT transmitting from our RMTDataItem buffer
// Keep track of where we are in the buffer (a sector number and an in-sector index, or maybe just a global index) so we know where to refill the RMT from when it runs out
// The index will also be useful later when we add write support so we know where to write incoming data to
// When the RMT signals that it needs more data, we just copy the next chunk of RMTDataItems from our buffer into the RMT memory FIFO
// When we reach the end of the track buffer, just loop back to the beginning to simulate a spinning disk
// When the host stops spinning the motor or stops reading the readData register, just stop the RMT and freeze the indices in place; we'll reload the RMT from those indices next time we want to read
// If it's a DS disk and the host switches sides, just switch to the other buffer and continue from the same track/sector/index; don't bother clearing the RMT, just let it run out its current side data and then start sending from the other side's buffer

// Well, it turns out that we only have enough room in RAM to hold two RMTSectors at a time; not an entire track's worth
// We can still hold the entire track's DecodedSectors and GcrSectors in RAM though
// So we'll have to convert each sector to RMT format on-the-fly as we need it
// Since we have two RMTSectors worth of RAM, one of the sectors can be feeding the RMT while the other is being prepared
// So let's do that now

// Procedure for writes/formats:
// Writing doesn't care what register is being accessed; as long as the drive is enabled and WRQ is low, we write whatever's on WRD
// So in our main loop, inside, the "if drive enabled" block, we just need to check if WRQ is low
// If it is, call a function that reads data from a receiving RMT channel hooked to WRD
// So of course we'll need to set up an RMT channel for receiving data on WRD somewhere during initialization
// In this function, we'll do nothing unless the RMT's receive buffer is full, in which case we read all the RMTDataItems from it
// Then we need to scan through that data to find the GCR sync marks that indicate the start of a sector
// Once we find them, check if it's a header or data field
// If it's a header, then the host is trying to do a format op, so figure out which sector it's for
// So figure out which sector it's for and save the Format byte to that sector's header in the GCRSector struct
// If it's a data field, then the host is writing sector data (either as part of a format or a normal write op)
// So figure out which sector it's for (by looking at the last header we passed over) according to currentSector - 2 or something
// And decode the data and stick it in that sector's data array in the GCRSector struct
// If the host was doing a format, make sure to update currentSector to whatever sector the host tried to format so that it writes data to that sector next
// When WRQ goes high again, detect that edge and call the write function one more time; we need to read any residual data in the RMT receive buffer
// Process that data (it's probably the end of a sector) the same way as before
// And then set the dirty bit before returning to the main loop