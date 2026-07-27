#include <Arduino.h>
#include <Adafruit_SH110X.h>
#include "SdFat.h"
#include <esp_cpu.h>
#include "diskLib.h"
#include "GCRLib.h"
#include "GPIO.h"
#include "LEDC.h"
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
uint32_t tachPulsesPerTrackMac[80] = {
    394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, // Tracks 0-15
    429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, // Tracks 16-31
    472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, // Tracks 32-47
    525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, // Tracks 48-63
    578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, // Tracks 64-79
};

// For the sake of speed, we can't be converting these TACH RPM values into LEDC divider values on the fly
// So precompute the dividers and store them in another LUT
uint32_t tachDividerPerTrackMac[80] = {
    203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, // Tracks 0-15
    186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, // Tracks 16-31
    169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, // Tracks 32-47
    152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, // Tracks 48-63
    138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, // Tracks 64-79
};

// The Lisa uses a slightly different set of TACH pulse frequencies, so here's a separate set of LUTs for those
uint32_t tachPulsesPerTrackLisa[80] = {
    407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, // Tracks 0-15
    443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, // Tracks 16-31
    489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, // Tracks 32-47
    545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, // Tracks 48-63
    613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, // Tracks 64-79
};

uint32_t tachDividerPerTrackLisa[80] = {
    196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, // Tracks 0-15
    180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, // Tracks 16-31
    163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, // Tracks 32-47
    146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, // Tracks 48-63
    130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, // Tracks 64-79
};

// Watchdog timer write enable register and value
#define TIMG1_WDT_WE 0x050D83AA1
#define TIMG1_WDT_WE_REG 0x60020064

// Watchdog timer configuration register and enable bit
#define TIMG1_WDT_CONF_REG 0x60020048
#define TIMG1_WDT_EN 1 << 31

SPIClass SD_SPI(HSPI); // These two lines make sure that we use hardware SPI at 20MHz for the SD card
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(20), &SD_SPI)

// Create the OLED display object; we want a 128x64 display
Adafruit_SH1106G OLED = Adafruit_SH1106G(128, 64, &Wire, -1);

SdFat32 SDCard; // The SD card object
SdCard* card; // A pointer to the card object so we can use it in other files
File32 disk; // The disk image that ESFloppy is using
FatFile rootDir; // The root directory of the SD card

static DecodedSector trackBufferDecoded[2][12]; // Buffer for decoded sectors for each side of the disk (2 sides, max 12 sectors per track)
static GcrSector trackBufferGCR[2][12]; // Buffer for GCR-encoded sectors for each side of the disk


uint32_t currentSector = 0;
uint32_t inSectorIndex = 0;
uint32_t currentTrack = 0;
bool trackChanged = false;
uint32_t bitTime = 0;
uint32_t prevBitTime = 0;

DiskImageMetadata diskMetadata;

bool prevLSTRB = 0;
bool currLSTRB = 0;

bool ledcAttached = false;
StepDirection stepDirection = OUT;
bool stepComplete = true;
bool motorOn = false;

bool ejectPending = false;
uint32_t ejectStartTime;
uint32_t tachFreq = 0;

bool dirty = false;

// This enum defines the different commands that we can send to the SD card task
enum SdTaskCommand {
    READ_TRACK, // Just read a track and encode it to GCR
    WRITE_READ_TRACK, // Write out a track and then read in another
    CLOSE_IMAGE // Close the disk image file with closeImage()
};

// This struct is used to pass data to the SD card access task that runs on the other core
struct SdTaskInterface {
    uint8_t writeTrack; // What track to write
    uint8_t readTrack; // What track to read
    SdTaskCommand command; // What command to execute
    bool start; // We set this high to tell the task to start processing our requect
    bool finished; // The task sets this high when it's finished processing our request
    bool initDone; // This goes high when the task is done initializing itelf
};

// Make an SdTaskInterface struct; make sure it's static and volatile since it's shared between two cores
static volatile SdTaskInterface sdTaskInterface = {0, 0, READ_TRACK, false, true, false};

// This is set if we have a full sector or header in the pending write buffer that we need to write out once the SD task is done reading in the track
bool writeBufferPending = false;


void updateOLED () {
    static uint32_t prevTrack = 0xFFFFFFFF;
    static uint32_t prevMotorOn = 0xFFFFFFFF;
    static uint32_t prevImageType = 0xFFFFFFFF;
    static uint32_t prevDriveType = 0xFFFFFFFF;
    static uint32_t prevDiskInserted = 0xFFFFFFFF;
    static uint32_t prevEjectPending = 0xFFFFFFFF;
    if (prevTrack == currentTrack && prevMotorOn == motorOn && prevImageType == diskMetadata.imageType && prevDriveType == diskMetadata.driveType && prevDiskInserted == diskMetadata.diskInserted && prevEjectPending == ejectPending) {
        return; // If nothing has changed, don't update the display
    }
    OLED.clearDisplay();
    //OLED.setTextSize(1);
    //OLED.setTextColor(SH110X_WHITE);
    OLED.setCursor(0, 0);
    OLED.print("Track: "); OLED.println(currentTrack);
    OLED.print("Motor: "); OLED.println(motorOn ? "ON" : "OFF");
    OLED.print("Image Type: "); OLED.println(diskMetadata.imageType == DC42 ? "DC42" : "RAW");
    OLED.print("Drive Type: "); OLED.println(diskMetadata.driveType == Drive400 ? "400K" : "800K");
    OLED.print("Disk in Place: "); OLED.println(diskMetadata.diskInserted ? "YES" : "NO");
    OLED.print("Eject Pending: "); OLED.println(ejectPending ? "YES" : "NO");
    OLED.display();

    prevTrack = currentTrack;
    prevMotorOn = motorOn;
    prevImageType = diskMetadata.imageType;
    prevDriveType = diskMetadata.driveType;
    prevDiskInserted = diskMetadata.diskInserted;
    prevEjectPending = ejectPending;
}

struct DbgMsg { uint8_t code; int32_t value; };
static volatile DbgMsg dbgRing[16];
static volatile uint8_t dbgHead = 0, dbgTail = 0;

// interface core — never blocks, drops on overflow
static inline void dbg(uint8_t code, int32_t value) {
    uint8_t next = (dbgHead + 1) & 15;
    if (next == dbgTail) return;
    dbgRing[dbgHead].code = code;
    dbgRing[dbgHead].value = value;
    __sync_synchronize();
    dbgHead = next;
}

// This is the SD card access task that runs on the other core
// It handles all the SD card ops so that we don't block the timing-sensitive stuff on the main core
void sdCardTask(void* parameter) {
    Serial.begin(115200); // Serial is handled from this task, so start it here
    Serial.println("Starting ESFloppy...");
    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); // Start comms with the SD card using our hardware SPI instance
    // Now initialize the OLED
    Wire.begin(OLED_SDA, OLED_SCL); // Start the I2C bus for the OLED
    OLED.begin(0x3C, true); // And init it
    // Clear the display since there might be garbage on it after reset
    OLED.clearDisplay();
    OLED.display();
    // And now move onto the SD card
    if (!SDCard.begin(SD_CONFIG)) { // Initialize the SD card with our hardware SPI instance
        Serial.println("SD card initialization failed! Halting..."); // And print an error/go into an infinite loop on failure
        OLED.clearDisplay();
        OLED.setTextSize(2);
        OLED.setTextColor(SH110X_WHITE);
        OLED.setCursor(0, 0);
        OLED.print("SD Init Failed!");
        OLED.display();
        while(1);
    }
    card = SDCard.card(); // Now that the card is initialized, store a pointer to its object
    rootDir.open("/"); // Then open the card's root directory
    //My Lisa Stuff/MWP/MW_1.018_Install.img
    //test_image.dc42
    //My Lisa Stuff/LOS 3 Debozoed/LisaCalc.dc42
    //My Lisa Stuff/MacWorks Plus II Install.image
    //LisaTest 3.0 1.image
    if (!openImage("copy_image_800k.dc42", &disk, &diskMetadata)) { // And try opening a disk image file
        Serial.println("Failed to open disk image! Halting..."); // Give another error/infinite loop on failure
        OLED.clearDisplay();
        OLED.setTextSize(2);
        OLED.setTextColor(SH110X_WHITE);
        OLED.setCursor(0, 0);
        OLED.print("Can't Open Disk!");
        OLED.display();
        while(1); // If we fail to open the image, just hang here
    }
    // Read and encode track 0 so that we can start sending it out when the Lisa requests it
    readTrack(0, &disk, trackBufferDecoded, &diskMetadata);
    encodeTrackToGCR(0, trackBufferDecoded, trackBufferGCR, &diskMetadata);
    OLED.setTextSize(1); // Set the OLED text size to 1 and color to white
    OLED.setTextColor(SH110X_WHITE);
    for(int i = 0; i < 80; i++) {
        setFreq(tachPulsesPerTrackLisa[i], 8);
    }
    Serial.println("ESFloppy is ready!"); // And if all this succeeds, print a ready message
    // Make sure that everything above here is truly done before we continue
    __sync_synchronize();
    sdTaskInterface.initDone = true; // Set initDone to tell the main task that we're done initializing now
    // We don't ever want this task to exit, so infinite-loop in here
    while (1) {
        if (!sdTaskInterface.start) {
            // If we haven't been told to start processing a request, then just wait a little while and check again
            while (dbgTail != dbgHead) {
                Serial.printf("%u %ld\n", dbgRing[dbgTail].code, (long)dbgRing[dbgTail].value);
                dbgTail = (dbgTail + 1) & 15;
            }
            //updateOLED(); // Update the OLED display with the current status; this only actually writes the display if something on the screen changed
            vTaskDelay(1);
            continue;
        }
        // Otherwise, we need to get to work processing the request
        // First, call __sync_synchronize to make sure that the compiler doesn't optimize/rearrange any of our accesses to the shared sdTaskInterface struct
        // Basically, it ensures that everything above __sync_synchronize is done before everything below it
        // If we put a corresponding call at the site where we set start = true, then we can be sure that optimization doesn't cause the two tasks to write the struct at the same time
        __sync_synchronize();
        // Things are pretty easy from here on; first check if we need to write a track, and if so, do it
        if (sdTaskInterface.command == WRITE_READ_TRACK) {
            decodeTrackFromGCR(sdTaskInterface.writeTrack, trackBufferGCR, trackBufferDecoded, &diskMetadata);
            writeTrack(sdTaskInterface.writeTrack, &disk, trackBufferDecoded, &diskMetadata);
        }
        if (sdTaskInterface.command == READ_TRACK || sdTaskInterface.command == WRITE_READ_TRACK) {
            // If the command is either a read OR write, then we now need to read the requested track and encode it to GCR
            readTrack(sdTaskInterface.readTrack, &disk, trackBufferDecoded, &diskMetadata);
            encodeTrackToGCR(sdTaskInterface.readTrack, trackBufferDecoded, trackBufferGCR, &diskMetadata);
        }
        else if (sdTaskInterface.command == CLOSE_IMAGE) {
            // If the command is to close the disk image, then obey
            closeImage(&disk, &diskMetadata);
        }
        // Now that we're done, synchronize again to make sure that everything above here is truly done before we say we're done
        __sync_synchronize();
        // And finally, set finished high and start low to tell the main core that we're done
        sdTaskInterface.start = false;
        sdTaskInterface.finished = true;
    }
}

// This function will get called whenever the Lisa is accessing the floppy's read data register
// It checks if it's time to send out a new flux transition, and if so, it bit-bangs it out on the RDA pin
// For the sake of speed, we don't return from here until the Lisa stops reading from the read data register
// We need speed here, so make sure to stick it in IRAM and optimize it as much as possible
__attribute__((optimize("Ofast"))) IRAM_ATTR void transmitTrack() {
    // Each bit time is 2us long; for a 1 bit, we send a falling edge at the start, followed by a rising edge 1us later
    // For a 0 bit, we just keep it high the whole time

    // There's a good chance that RDA will be low coming into here if we were reading a different register before this
    // So go ahead and set it high to start with so that we're all ready to send out a falling edge for a 1 bit if we need to
    // Otherwise, if the first bit we send is a 1 bit, it'll be missed because we can't send a falling edge if RDA is already low
    writeRDA(true);

    // There's a chance that we just arrived here after switching from one track class to another (like from 12 sectors per track to 11)
    // And in that case, there's a possibility that the current sector is now out of bounds for the new track class
    // So check for that and reset it to 0 if it's out of bounds
    if (currentSector >= sectorsPerTrack[currentTrack]) {
        currentSector = 0;
    }

    // On a real drive, the disk doesn't stop turning just because we exited transmitTrack
    // So if the Lisa exits transmitTrack to check status and reenters, we need to catch up to the current time to simulate the spinning disk
    // You'd think that we'd be able to just pick up where we left off, which we can most of the time, but there's a problem with that
    // The Lisa periodically polls /DRVIN once per sector, and tries to do it in phase with the sector sync field
    // This way, the only data missed is some of the sync data, and we can recover with the remaining sync data when we return
    // But sometimes the /DRVIN check gets a little out of phase and happens in the middle of data instead of sync
    // Now we have a problem because the Lisa can't recover from that since there's no sync
    // And retrying won't help because the /DRVIN will still be out of phase and hit the data again
    // So the simple fix is to simulate the disk spinning while we're not in transmitTrack so that the phase constantly drifts instead of getting stuck out of sync
    // This will lead to occasional errors, but they should be recoverable in one retry and won't be frequent
    uint32_t currentTime = esp_cpu_get_cycle_count(); // To do this, start by getting the current time
    uint32_t timeAway = currentTime - prevBitTime; // And then get the number of CPU cycles since the last bit time (the time that we were away)
    if (timeAway > 480 && trackChanged == false) {
        // If we were away for more than 480 cycles (one full bit time), then we have catch-up work to do
        inSectorIndex += timeAway / 480; // Increment the in-sector index by the number of full bit times that have passed
        // But if this pushes it past the end of the sector, then we have even more work to do
        if (inSectorIndex >= BITS_PER_SECTOR) {
            // Increment the current sector by the number of full sectors that have passed, wrapping around to sector 0 if necessary
            currentSector = (currentSector + (inSectorIndex / BITS_PER_SECTOR)) % sectorsPerTrack[currentTrack];
            inSectorIndex %= BITS_PER_SECTOR; // And then set the in-sector index to the appropriate spot within the new sector
        }
        prevBitTime = currentTime; // And finally, update the previous time to now
    } else if (trackChanged == true) {
        // If the track has changed, then we need to reset the in-sector index and current sector to 0 and not do a catch-up
        // This isn't necessary, but it speeds things up since the FDC normally starts reading new tracks from sector 0, so might as well start there
        currentSector = 0; // So make those updates
        inSectorIndex = 0;
        prevBitTime = currentTime; // Set the previous time to now
        trackChanged = false; // And mark that we've handled the track change
    }

    while (1) {
        // Before we do anything, we need to check to see if the Lisa is still reading from the read data register to begin with
        // For the sake of speed, do a raw REG_READ here instead of using any helper functions
        uint32_t gpioIn = REG_READ(GPIO_IN_REG); // Read the GPIO input register
        // Now check for the proper pattern; the read registers are registers 8 and 9
        // We don't care about the low side select bit; we just need to make sure the high 3 bits {PH2, PH1, PH0} are 100
        // And we also need to be sure that LSTRB (PH3) isn't high; if it is, then we might miss a write to regs 0 or 1 which look like regs 8 and 9 in write mode
        if (!(gpioIn & 1 << PH2 && !(gpioIn & 1 << PH1) && !(gpioIn & 1 << PH0)) || (gpioIn & 1 << PH3)) {
            // If not, then return
            return;
        }
        // We also need to check to see if WRQ went low, meaning that the Lisa is trying to write to the drive
        if (!(gpioIn & 1 << WRQ)) {
            // If so, then get out of here and let the write handler (receiveSector) take over
            return;
        }

        // Otherwise, grab the side number from the low HDS bit and continue
        uint32_t side = (gpioIn & 1 << HDS) ? 1 : 0;

        // When we arrive here, we'll be on the first half of a bit, so just wait until it's time to send out that first half
        prevBitTime += 240;
        while ((int32_t)(esp_cpu_get_cycle_count() - prevBitTime) < 0); // Get the number of CPU cycles between now and the last bit time; esp_cpu_get_cycle_count() is faster than ESP.getCycleCount()
        // Once that while loop finishes (240 cycles at 240MHz is 1us), it's time to send out the first half of our bit
        // Now we need to extract the next bit from trackBufferGCR[side][currentSector]
        // But there's a catch: trackBufferGCR may not be ready yet if the SD task on the other core is still running
        // So we need to wait until it's finished before we read the track buffer and just send out the FF sync pattern until then
        // The Lisa won't care; it'll just think that the disk hasn't spun around to the next sector yet and will patiently wait for us
        static const uint8_t syncPattern[5] = {0xFF, 0x3F, 0xCF, 0xF3, 0xFC};
        uint8_t currentByte;
        if (sdTaskInterface.finished) {
            // If the SD task is finished, then we can sedn out the next bit from the track buffer
            currentByte = ((uint8_t*)&trackBufferGCR[side][currentSector])[inSectorIndex >> 3]; // Get the byte that contains the bit we want
        } else {
            // If the SD task is not finished, then we need to send out the sync pattern
            currentByte = syncPattern[(inSectorIndex >> 3) % 5];
        }
        bool bit = (currentByte >> (7 - (inSectorIndex & 0x07))) & 0x01; // Extract the bit we want from the byte
        // And now we can send out the bit on RDA
        if (bit) {
            writeRDA(false); // Send a falling edge on RDA to indicate a 1 bit
        } else {
            writeRDA(true); // Or keep RDA high to indicate a 0 bit
        }
        
        // Now we need to wait for the second half of the bit time, which is another 1us
        prevBitTime += 240;
        while ((int32_t)(esp_cpu_get_cycle_count() - prevBitTime) < 0);
        // Time to send the second half of the bit; if it was a 1, we need to send a rising edge; if it was a 0, we just keep it high
        // We don't need to retrieve the bit again and check its value because regardless of whether it was a 0 or a 1, we need to set it high!
        writeRDA(true); // Nice and easy!
        
        // We now need to increment to the next bit in the sector
        inSectorIndex++;
        if (inSectorIndex >= BITS_PER_SECTOR) {
            // If we're about to go past the end of the sector, we need to move to the next one
            currentSector++;
            if (currentSector >= sectorsPerTrack[currentTrack]) {
                currentSector = 0; // Or wrap back to sector 0 if we're at the end of the track
            }
            inSectorIndex = 0; // Don't forget to reset the in-sector index since we're starting a new sector
        }
    }
}

// This helper function gets called by receiveSector to interpret the raw data coming from the Lisa, extract prologues, and stick everything into the data buffer
// It returns true once the full header or sector data has been received, and false if we need to keep receiving more data
// It also modifies the writeState variable to indicate to the caller whether we're still in the prologue, reading the header, or reading the data
__attribute__((optimize("Ofast"))) IRAM_ATTR bool processRawWriteData(uint32_t rawInputData, uint8_t* dataBuffer, WriteState& writeState, bool newReception) {
    static uint32_t bitCounter = 0; // This keeps track of how many bits we've received so far for the current byte
    static uint32_t byteCounter = 0; // And how many bytes we've received so far in total

    if (newReception) {
        // If this is the start of a new reception, then we need to reset the bit and byte counters back to 0
        // We have to do this because they're static and would persist otherwise
        // And we can't rely on just resetting them in the "return true" case below because there's a chance that we only get a partial/corrupt sector and never get there
        bitCounter = 0;
        byteCounter = 0;
    }

    if (writeState == PROLOGUE) {
        // If we're in the prologue state, then we need to check if the data matches the header or data prologue
        if ((rawInputData & 0xFFFFFF) == DATA_PROLOGUE) {
            writeState = DATA; // Update the writeState accordingly
        } else if ((rawInputData & 0xFFFFFF) == HEADER_PROLOGUE) {
            writeState = HEADER;
        }
    } else if (writeState == DATA || writeState == HEADER) {
        // If we're in the data or header state, then we need to check two things
        // First, we need to check if we've received enough data to form a full byte to copy into the data buffer
        // And second, we need to check if we've received enough data to have the full header or sector data, and exit if so
        bitCounter++; // Increment the bit counter since we just received a bit
        // Start with the first check; if we've received 8 bits, then it's time to copy a byte into the buffer
        if (bitCounter == 8) {
            dataBuffer[byteCounter] = rawInputData & 0xFF; // Copy the LSB of rawInputData into the data buffer
            bitCounter = 0; // Reset the bit counter back to the start of the next byte
            byteCounter++; // And the byte counter since we just received a full byte
            // Now check to see if we've received enough bytes to have the full header or sector data
            if ((writeState == DATA && byteCounter >= 704) || (writeState == HEADER && byteCounter >= 5)) {
                // If so, then we can return true to indicate that we're done receiving data
                return true;
            }
        }
    }
    return false; // Return false if we haven't filled up the header/data buffer yet
}

// This function gets called whenever the Lisa pulls WRQ low and starts writing data to the disk
// Note that it's called receiveSector instead of receiveTrack because the Lisa only writes one sector at a time
// This makes our life a LOT easier
__attribute__((optimize("Ofast"))) IRAM_ATTR void receiveSector() {
    // The write process, despite sounding scary at first, is actually not too bad and looks like this:
        // 1. The Lisa lowers WRQ and starts sending valid data on WRD; each edge represents a 1, and the absence of an edge for 2us represents a 0
        // 2. The start of the data is the classic FF FF 3F CF F3 FC FF FF self-sync pattern, which we can safely ignore; we'll be fine even if we don't start sampling until midway through it
        // 3. After self-sync, the Lisa sends the D5 AA AD data prologue, which is our queue to start sampling
        // 4. Then we get sector_again, which we can use to determine which sector we're writing to
        // 5. Now we get our 699 bytes of GCR-encoded data, the 4-byte checksum, and the DE AA FF data epilogue, which gets cut off midway through the FF and we can ignore
        // 6. That can all just go straight into a buffer, get copied into a DecodedSector struct, and then get written to the image file
        // Notice that the header wasn't touched here at all; standard write ops completely ignore it
        // Format ops, on the other hand, overwrite BOTH the header and the data
        // A format works like this:
        // 1. The Lisa lowers WRQ and starts WRD data
        // 2. We get the self-sync pattern
        // 3. The Lisa sends the D5 AA 96 header prologue, which is our queue to start sampling the header
        // 4. We read in the header and its checksum, followed by the DE AA header epilogue, which we can ignore
        // 5. We can ignore most of the header, just paying attention to the sector number, side, and format to figure out where it needs to go and update our disk image
        // 6. WRQ goes high for a bit
        // 7. WRQ goes low again, and we get a standard data write as described above to clear out the data portion of the sector
        // So we can really just handle the header and data portions of things separately, and don't care that the data is tied to a format at all

    // Anyway, time to get started on all of that; first, just put RDA in a defined 0 state so that we're not sending garbage during a write
    writeRDA(false);

    // This holds what state of the write op we're in; we start in PROLOGUE since the first thing we do is hunt for the prologue
    // We never modify this directly; we pass it to processRawWriteData, which updates it as needed
    WriteState writeState = PROLOGUE;
    // We also need this second version that holds the write state from whatever header/sector is in dataBufferPending
    // It needs to be static so that it persists between calls in the event that we stick a sector in dataBufferPending and then come back later to write it out
    static WriteState writeStatePending = PROLOGUE;

    // Get the current and previous times in CPU cycles so that we can measure the time between edges on WRD
    uint32_t currTime = 0;
    uint32_t prevTime = esp_cpu_get_cycle_count();

    // We also need a firstTime variable to know when we see the first edge on WRD so that we can start measuring time between edges
    bool firstTime = true;
    
    // This will hold the current state of the GPIO input register so that we can check the WRD and WRQ lines
    uint32_t gpioData = 0;
    // We need a gpioDataPending for the same reason we need a writeStatePending; the HDS bit from it is the side number for dataBufferPending in certain circumstances
    static uint32_t gpioDataPending = 0;
    bool prevWRD = REG_READ(GPIO_IN_REG) & (1 << WRD); // This will hold the previous state of WRD so that we can detect edges
    bool currWRD = prevWRD; // And the current state

    // This buffer will hold the raw WRD data that we're currently receiving from the Lisa, and we'll use it like a shift register
    // It's 4 bytes long so that we can hold a full D5 AA AD or D5 AA 96 prologue that we use to detect the start of data or header
    uint32_t rawInputData = 0;

    // This is the big buffer that we copy the data into once we've seen the prologue and know that we're in a data or header write
    // It's 704 bytes long so that it can hold sector_again, the 699 bytes of data, and the 4-byte checksum
    uint8_t dataBuffer[704];
    
    // Here's another data buffer that we can stick the write data into if we've accumulated a full header/sector, but the SD card task hasn't finished loading the track yet
    // We can just stick the data in here until the SD card task is finished, and then copy it into the track buffer when it's ready
    static uint8_t dataBufferPending[704];

    // This flag is set if the last thing we received (on the last call to receiveSector) was a header; it's used to determine where to get the side number and interleave from
    static bool lastWriteWasHeader = false;
    static uint32_t prevSideNum = 0; // This is the side number from the last header
    static uint32_t prevFormat = 0; // And the format byte

    // Skip all of the read logic here if we already have a full header/sector in the pending buffer that we need to write out
    if (!writeBufferPending) {
        // Now we need to sync with the prologue and then do the actual data/header read
        while (1) {
            // As mentioned earlier, a 1 is represented by an edge on WRD, and a 0 is represented by the absence of an edge
            // The easiest way to detect edges is to start a timer when we see the first edge, and then constantly poll for edges on WRD
            // When we see the next edge, we check the time since the last edge and figure out what multiple of 2us it is (with some tolerance)
            // If it's just 2us, then we shift in a 1; if it's 4us, then we shift in a 01; if it's 6us, then we shift in a 001
            // The problem with this strategy is that 0 bits will only be committed when we see the next edge, so if the data ends with a 0, we won't see it
            // But this is fine because the end of the data is always the epilogue anyway, so we can just stop once we get through the data itself
            gpioData = REG_READ(GPIO_IN_REG); // Read the GPIO input register to get the current state of WRD and WRQ
            if ((gpioData & (1 << WRQ))) {
                // If WRQ goes high, then the Lisa has stopped writing and we can just return
                // This either means that the Lisa is being stupid and never sent all the data, or that we missed it
                return;
            }
            prevWRD = currWRD; // Update the previous WRD state to the current outdated one
            currWRD = gpioData & (1 << WRD); // And then update the current state of WRD to what it actually is
            if (currWRD == prevWRD) {
                continue; // If there was no change in WRD, then just continue to the next iteration of the loop and skip everything below here
            }
            // Otherwise, we saw an edge on WRD, so we need to check the time since the last edge to see what bit combination to shift in
            if (firstTime) {
                // If it's the first time through the loop, then we don't care about the time since the last edge
                // So just shift in a 1 bit and mark that we've seen the first edge
                rawInputData = (rawInputData << 1) | 1;
                prevTime = esp_cpu_get_cycle_count(); // Set prevTime to now so that we can measure the time to the next edge
                // Go process that data based on the current writeState to see if we've received the prologue or the full header/data yet
                // We pass true for the newReception parameter so that processRawWriteData knows to reset its bit and byte counters
                processRawWriteData(rawInputData, dataBuffer, writeState, true); 
                firstTime = false; // And mark that we've seen the first edge
            } else {
                // Otherwise, we need to check the time since the last edge
                currTime = esp_cpu_get_cycle_count(); // Get the current time in CPU cycles
                if (currTime - prevTime < BIT_TIME_1) {
                    // If the time since the last edge is less than our 2us with some tolerance, then shift in a 1 bit
                    rawInputData = (rawInputData << 1) | 1;
                    prevTime = currTime; // And update prevTime to now so that we can measure the time to the next edge
                    // Then go interpret the data based on the current writeState
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break; // If processRawWriteData returns true, then we've received the full header/data and can go deal with it
                    }          
                } else if (currTime - prevTime < BIT_TIME_01) {
                    // Same for 4us; shift in a 01
                    // We have to do this in 2 steps (shift in the 0, then the 1) so that we can call processRawWriteData after each bit
                    rawInputData = (rawInputData << 1);
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    rawInputData = (rawInputData << 1) | 1;
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    prevTime = currTime;
                } else if (currTime - prevTime < BIT_TIME_001) {
                    // And 6us; shift in a 001
                    // As before, we have to do this in steps; 3 steps this time
                    rawInputData = (rawInputData << 1);
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    rawInputData = (rawInputData << 1);
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    rawInputData = (rawInputData << 1) | 1;
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    prevTime = currTime;
                } else {
                    // Something weird happened if we end up here; there should never be a gap of more than 6us between edges
                    // This points to data corruption, so there's no point in continuing to receive data
                    // Just return and let the Lisa retry the write if it feels like it
                    lastWriteWasHeader = false; // Make sure to reset this so that we don't try to use a header that's stale
                    interrupts();
                    Serial.println("MORE THAN 6US");
                    noInterrupts();
                    return;
                }
            }
        }
    } else {
        // A reminder: That entire while loop gets skipped if we already have a full pending buffer that needs to get written out
        // We end up here in that case, where we need to double-check that it's safe to write to trackBufferGCR
        if (!sdTaskInterface.finished) {
            // If the task is still busy, then we just need to return again
            return;
        } else {
            // Otherwise, it's safe to proceed and write the pending buffer to trackBufferGCR
            memcpy(dataBuffer, dataBufferPending, 704); // So copy it back into the main buffer again
            writeBufferPending = false; // And mark that we no longer have a pending buffer
            // And also restore writeState and gpioData
            writeState = writeStatePending;
            gpioData = gpioDataPending;
        }
    }

    // Next, we need to check to see if the SD card task is busy reading in a track; if so, then we can't touch trackBufferGCR
    if (!sdTaskInterface.finished) {
        // If the SD card task is busy, then we need to copy the data into dataBufferPending and set writeBufferPending to true
        memcpy(dataBufferPending, dataBuffer, 704);
        writeBufferPending = true;
        // Don't forget to also save the current writeState and gpioData
        writeStatePending = writeState;
        gpioDataPending = gpioData;
        return; // And then return so that we don't touch trackBufferGCR whatsoever; the loop can call us again once the task is done
    }

    // Now that we have all of our header data or data data, we need write it to the proper GcrSector in trackBufferGCR
    // Note that we DO NOT write it to the disk image file yet because it would take too long
    // That's handled when the Lisa seeks to another track, turns off the motor, or ejects the disk
    if (writeState == DATA) {
        // If this is a data write, then we need to copy sector_again, the data, and the checksum
        // This is as easy as doing a memcpy into the proper GcrSector in trackBufferGCR, starting at the sector_again field
        uint32_t sectorNum = gcr_8to6[dataBuffer[0]]; // The first byte of the data buffer is the sector number (sector_again)
        // The side number is a little trickier; if the last write was a header, then we can read it from that header
        // And this is required during formats because HDS won't reflect the new side number during a format op
        // But if the last write was a data write, then we can just read it from HDS like normal
        uint32_t sideNum = (lastWriteWasHeader) ? (prevSideNum) : ((gpioData & (1 << HDS)) ? 1 : 0);
        // One more issue: we can't do a memcpy directly into trackBufferGCR[sideNum][sectorNum] because of the order in which trackbufferGCR stores sectors
        // It stores them in physical order, NOT logical order, so trackBufferGCR[sideNum][1] with 2:1 interleave on a 12-sector track would actually be sector 6
        // And we'd overwrite sector 6 with sector 1's data
        // So we actually need to run our sectorNum through the interleave table to figure out which physical slot to write it into
        uint32_t slot = 0xFFFFFFFF; // Initialize to an invalid value so that we can check later if we found a valid slot
        // Iterate through each sector in the current track
        for (uint32_t i = 0; i < sectorsPerTrack[currentTrack]; i++) {
            // Check what interleave factor this sector has and use the appropriate interleave table
            uint32_t currentFormat = 0;
            // If we're in the middle of a format, then use the interleave from the last header that was formatted
            if (lastWriteWasHeader) {
                currentFormat = prevFormat;
            // But if we're not in the middle of a format, then just use the interleave from literally any sector on the track since they're all the same 
            } else {
                currentFormat = gcr_8to6[trackBufferGCR[0][0].format];
            }
            // Now use the appropriate interleave table to find the physical slot for this sector
            if ((currentFormat & 0x1F) == 0x04) {
                // 4:1 interleave; use the interleave4to1 table
                if (interleave4to1[sectorsPerTrack[currentTrack]][i] == sectorNum) {
                    slot = i;
                    break;
                }
            } else if ((currentFormat & 0x1F) == 0x01) {
                // 1:1 interleave; just map the sector straight through
                if (i == sectorNum) {
                    slot = i;
                    break;
                }
            } else {
                // 2:1 interleave in all other case (this is the default); use the interleave2to1 table
                if (interleave2to1[sectorsPerTrack[currentTrack]][i] == sectorNum) {
                    slot = i;
                    break;
                }
            }
        }
        // Now that we've used it, reset lastWriteWasHeader to false so that we don't accidentally use a stale header for the next data write if we end up aborting right now
        lastWriteWasHeader = false;
        if (slot == 0xFFFFFFFF) {
            // If we didn't find a valid slot, then something went wrong (perhaps a corrupted header), so just abort
            interrupts();
            Serial.println("NO VALID DATASLOT");
            noInterrupts();
            return;
        }
        // Now copy the 704 bytes of data into the proper GcrSector, using the slot we just found as the physical sector number
        memcpy(&trackBufferGCR[sideNum][slot].sector_again, &dataBuffer[0], 704);
        dirty = true; // Don't forget to set dirty to true so that we know to write the track back to the disk image later
    } else if (writeState == HEADER) {
        // If this is a header write, then we need to copy the 4 bytes of header data and the 1-byte header checksum into the proper GcrSector
        // And we'll actually update the format byte and checksum in ALL of the sectors, not just the one we're writing to, to make sure it's always consistent across the whole track
        // Since the format byte controls interleave, we don't want a situation where part of a track uses one interleave and part uses another mid-format, so might as well just update them all
        // Before we do any of that though, make sure that all the bytes in the header are valid GCR bytes; if not, then abort
        uint8_t validGCR = 0;
        validGCR = gcr_8to6[dataBuffer[0]] | gcr_8to6[dataBuffer[1]] | gcr_8to6[dataBuffer[2]] | gcr_8to6[dataBuffer[3]] | gcr_8to6[dataBuffer[4]];
        if (validGCR & 0xC0) {
            // If any of the bytes have a 1 in the top 2 bits, then it's not valid GCR and we need to abort
            interrupts();
            Serial.println("BAD HEADER GCR");
            noInterrupts();
            return;
        }
        // Now make sure that the new track number in the header matches currentTrack
        // If it doesn't, then either the Lisa is going crazy or we missed a seek, but either way we need to abort
        if ((gcr_8to6[dataBuffer[0]] | ((gcr_8to6[dataBuffer[2]] & 1) << 6)) != currentTrack) {
            interrupts();
            Serial.println("HEADER TRACK MISMATCH");
            noInterrupts();
            return;
        }
        uint32_t sectorNum = gcr_8to6[dataBuffer[1]]; // The second byte of the data buffer is the sector number
        uint32_t sideNum = (gcr_8to6[dataBuffer[2]] & (1 << 5)) ? 1 : 0; // We can read the side number from hiTrackSide in the new header instead of HDS this time
        prevSideNum = sideNum; // Store the side number for the next data write in this format op
        prevFormat = gcr_8to6[dataBuffer[3]]; // And store the format for the next data write in this format op
        // As we did for the data write, we need to map the logical sector number to a physical slot using the appropriate interleave table
        // The nice part here is that we ALWAYS have a valid format byte to use (stored in prevFormat right above), so no need to worry about where to get the format byte from
        // Aside from that, it's literally the exact same logic as for the data write, so nothing else to say here
        uint32_t slot = 0xFFFFFFFF;
        for (uint32_t i = 0; i < sectorsPerTrack[currentTrack]; i++) {
            if ((prevFormat & 0x1F) == 0x04) {
                if (interleave4to1[sectorsPerTrack[currentTrack]][i] == sectorNum) {
                    slot = i;
                    break;
                }
            } else if ((prevFormat & 0x1F) == 0x01) {
                if (i == sectorNum) {
                    slot = i;
                    break;
                }
            } else {
                if (interleave2to1[sectorsPerTrack[currentTrack]][i] == sectorNum) {
                    slot = i;
                    break;
                }
            }
        }
        if (slot == 0xFFFFFFFF) {
            // Abort on an invalid slot as before
            interrupts();
            Serial.println("NO VALID HEADERSLOT");
            noInterrupts();
            return;
        }
        // Now write our data to the sector headers using the physical slot number we just found
        for (uint32_t i = 0; i < sectorsPerTrack[currentTrack]; i++) {
            trackBufferGCR[sideNum][i].format = dataBuffer[3]; // Update the format byte for all sectors on this side of the track
            // And then update the header checksums of all those sectors to reflect the new format byte
            trackBufferGCR[sideNum][i].headerChecksum = gcr_6to8[gcr_8to6[trackBufferGCR[sideNum][i].loTrack] ^ gcr_8to6[trackBufferGCR[sideNum][i].sector] ^ gcr_8to6[trackBufferGCR[sideNum][i].hiTrackSide] ^ gcr_8to6[trackBufferGCR[sideNum][i].format]];
        }
        // And then memcpy over the full sector header for the particular sector that we just wrote to
        memcpy(&trackBufferGCR[sideNum][slot].loTrack, &dataBuffer[0], 5);
        lastWriteWasHeader = true; // Mark that the last write was a header
        dirty = true; // Once again, make sure to indicate that this track needs to be written back
    }
}


void setup() {
    init(); // Init the ESP32 Arduino core
    initLEDC(RDA); // Initialize the LEDC peripheral on the RDA pin for sending TACH pulses
    setDuty(128); // Set the LEDC duty cycle to 50%
    enableLEDCOutput(true); // And enable its output (note that this doesn't actually connect it to the pin though)
    initPins(); // Set all of ESFloppy's pins to the correct direction and state

    // Now we need to create a task to handle SD card ops, the OLED, and serial comms on the other core
    // This way, the main core can focus solely on the time-critical task of sending data to the Lisa
    // Now that I think about it, we're basically doing what Floppy Emu does, with our main core being the CPLD and our SD core being the AVR
    xTaskCreatePinnedToCore(
        sdCardTask, // The function to run on the other core
        "sdCardTask", // A name for the task, it doesn't really matter
        8192, // The stack size for the task in bytes
        NULL, // No parameters to pass to it
        1, // Give it a priority of 1, same as the main core loop task
        NULL, // Don't return a handle to the task since we don't need it
        xPortGetCoreID() == 0 ? 1 : 0 // Put the task on whichever core we're not currently running on
    );

    while(!sdTaskInterface.initDone); // Wait for the SD card task to finish initializing before we continue

    // Just like ESProFile, we need to disable the interrupt watchdog timer
    // For the sake of speed, we have to disable interrupts throughout our entire program, and the watchdog would trigger and break everything
    // Note that this ONLY disables things on this core; the other core is still free to use interrupts all it wants
    REG_WRITE(TIMG1_WDT_WE_REG, TIMG1_WDT_WE); // Enable writing to the watchdog timer registers
    REG_CLR_BIT(TIMG1_WDT_CONF_REG, TIMG1_WDT_EN); // And clear the timer's enable bit to disable it
    delay(100); // Wait a little bit to let the Serial.println finish before we actually kill interrupts
    noInterrupts(); // Now that it's safe to do so, disable interrupts for the rest of the program
}

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
        
    // First up, read in the states of all the I/O pins that we care about
    uint32_t gpioIn = REG_READ(GPIO_IN_REG); // Read the GPIO input register    

    if ((gpioIn & (1 << DR1)) == 0) { // If the drive is enabled, then we need to check for commands

        static bool prevWRQ = true;
        static uint32_t lostSectorCount = 0;
        bool currWRQ = (gpioIn & (1 << WRQ)) != 0;
        if (!currWRQ && prevWRQ && writeBufferPending) {
            lostSectorCount++;
            dbg(2, lostSectorCount);
        }
        prevWRQ = currWRQ; // must run before the early returns below

        // If WRQ is low, then the Lisa is trying to write to the drive, so go and handle that
        if ((gpioIn & (1 << WRQ)) == 0) {
            receiveSector();
            return; // This makes sure that we refresh gpioIn with an updated read after we get back from receiveSector
        }

        // If we have a pending write buffer and the SD card task is finished, then call receiveSector again to write the pending buffer to trackBufferGCR
        if (writeBufferPending && sdTaskInterface.finished) {
            receiveSector();
            return;
        }
            
        currLSTRB = (gpioIn & (1 << PH3)) ? 1 : 0; // Read the current state of LSTRB (PH3)
        if (currLSTRB == 0) { // If LSTRB is low, then we need to put the selected register on the bus
            // Figure out which register the host wants to read from
            uint8_t regNum = ((gpioIn & (1 << PH2)) ? 8 : 0) | ((gpioIn & (1 << PH1)) ? 4 : 0) | ((gpioIn & (1 << PH0)) ? 2 : 0) | ((gpioIn & (1 << HDS)) ? 1 : 0);
            // Now we need to figure out what data to send for that register
            if (regNum != 7 && ledcAttached) { // If we're not reading the TACH register, make sure to stop any ongoing tach pulse generation
                // Only do this if it's not already detached since repeatedly detaching it wastes tons of time
                ledcAttached = false;
                GPIOControl(RDA); // Give the GPIO registers control of the RDA pin back so we can bit-bang data to it
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
                    // Use our LUT to figure out what the LEDC divider value should be for the current track's TACH frequency
                    tachFreq = tachDividerPerTrackLisa[currentTrack];
                    // Only output TACH pulses if the motor is on (DUH), and only start the LEDC if it's not already running
                    if (motorOn && !ledcAttached) {
                        setFreqRaw(tachFreq, 8); // Set the LEDC divider value to our TACH frequency with 8-bit duty resolution
                        LEDCControl(RDA); // And give the LEDC control of the RDA pin so it can output the TACH pulses
                        ledcAttached = true; // Mark that the LEDC is attached so we don't attach/detach it unnecessarily
                    }
                    // If the motor is off, just output a constant low
                    else if (!motorOn) {
                        writeRDA(0);
                    }
                    break;
                case 8: // RDDATA register for head 0
                    // If the drive's motor is running, then transmit data for side 0; otherwise, do nothing
                    if (motorOn) {
                        transmitTrack();
                    }
                    break;
                case 9: // RDDATA register for head 1; only valid for 800k drives
                    // If the drive's motor is running, then transmit data for side 1; otherwise, do nothing
                    if (motorOn) {
                        transmitTrack();
                    }
                    break;
                case 12:
                case 13: // SIDES register (duplicated on both addresses 12 and 13); returns 0 for 400K drives, 1 for 800K drives
                    // Always return 1; I previously based it on the image size, but then discovered that certain programs cache the value
                    // So if you switch images, the program will still assume it's the same drive type as before and things break
                    writeRDA(true);
                    //writeRDA(diskMetadata.driveType == Drive800 ? 1 : 0);
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
            uint8_t regNum = ((gpioIn & (1 << PH1)) ? 4 : 0) | ((gpioIn & (1 << PH0)) ? 2 : 0) | ((gpioIn & (1 << HDS)) ? 1 : 0);
            bool regData = (gpioIn & (1 << PH2)) ? 1 : 0; // The data to write is on CA2 (PH2)
            // Now write the data to the selected register
            switch (regNum) {
                case 0: // /DIRTN register (head step direction, low for IN, high for OUT)
                    stepDirection = regData ? OUT : IN;
                    break;
                case 2: // /STEP register (host sets low to step heads, drive sets high when step is complete)
                    if (regData == 0 && stepComplete == true) { // If the host is trying to step the heads and we're not already in the middle of a step
                        stepComplete = false; // Mark that a step is in progress
                        writeRDA(stepComplete); // And set the STEP register low to indicate that we're busy stepping
                        uint32_t waitStart = esp_cpu_get_cycle_count();
                        while(!sdTaskInterface.finished); // Wait for the SD card task to finish its current command before we dispatch it again
                        dbg(3, (esp_cpu_get_cycle_count() - waitStart) / 240);   // µs
                        trackChanged = true; // Mark that the track has changed so we can reset the sector back to 0 in transmitTrack
                        // We need to dispatch the SD card task on the other core to save the current track (if necessary) and load the next one
                        sdTaskInterface.writeTrack = currentTrack; // We want it to write out the current (pre-step) track
                        sdTaskInterface.command = dirty ? WRITE_READ_TRACK : READ_TRACK; // If the track is dirty, then we need to write it out first, otherwise we can just read in the new track

                        // Now we can actually perform the step
                        if (stepDirection == IN) { // If stepping IN, increment the track
                            if (currentTrack < 79) {
                                currentTrack++;
                            }
                        }
                        else { // If stepping OUT, decrement the track
                            if (currentTrack > 0) {
                                currentTrack--;
                            }
                        }

                        // currentTrack is now the new track that we want to read in, so set readTrack accordingly
                        sdTaskInterface.readTrack = currentTrack;
                        dirty = false; // Clear the dirty bit since the task already knows about it now
                        sdTaskInterface.finished = false; // And mark that the task isn't finished yet
                        // Now we need to synchronize to make sure that everything above here is truly done before we proceed
                        // We can't afford to have the compiler reorder anything because then us and the SD task might try to access the same memory at the same time
                        __sync_synchronize();
                        // Now that we're synced, start the task
                        sdTaskInterface.start = true;
                        dbg(0, currentTrack);
                        stepComplete = true; // Now that we've dispatched the other core's task, mark the step as complete
                    }
                    break;
                case 4: // /MOTORON register (low to turn motor on, high to turn motor off)
                    motorOn = (regData == 0);
                    // Turn on the activity LED whenever the motor is on too
                    if (motorOn) {
                        REG_WRITE(GPIO_OUT_W1TS_REG, 1 << LED);
                    }
                    else {
                        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << LED);
                        // If the motor just turned off, this is a prime opportunity to write the current track to the image if necessary
                        // This works basically the same as in the step case above, except that we don't need to change the track number since we're not stepping
                        if (dirty == true && sdTaskInterface.finished) {
                            sdTaskInterface.writeTrack = currentTrack;
                            sdTaskInterface.command = WRITE_READ_TRACK;
                            sdTaskInterface.readTrack = currentTrack;
                            dirty = false;
                            sdTaskInterface.finished = false;
                            __sync_synchronize();
                            sdTaskInterface.start = true;
                        }
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
                            // First make sure that any pending writes are flushed to the image; dispatch the SD task the same way as usual
                            if (dirty == true) {
                                // Wait until the SD card task is finished with its current command before we dispatch it again
                                // It's okay to block like this in the eject handler because we're ejecting the disk anyway
                                while (sdTaskInterface.finished == false);
                                sdTaskInterface.writeTrack = currentTrack;
                                sdTaskInterface.command = WRITE_READ_TRACK;
                                sdTaskInterface.readTrack = currentTrack;
                                dirty = false;
                                sdTaskInterface.finished = false;
                                __sync_synchronize();
                                sdTaskInterface.start = true;
                            }
                            // Wait until the SD card task is done with its current command
                            while (sdTaskInterface.finished == false);
                            sdTaskInterface.command = CLOSE_IMAGE; // Now tell it to close the image
                            sdTaskInterface.finished = false;
                            __sync_synchronize();
                            sdTaskInterface.start = true; // And start the task
                            motorOn = false; // Turn off the motor too
                            ejectPending = false;
                        }
                    }
                    break;
                default:
                    // Do nothing for writes to other (invalid) registers
                    break;
            }
        }
        prevLSTRB = currLSTRB; // Update prevLSTRB for the next iteration
    } else {
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
// Make sure to clear the receive buffer on the falling edge of WRQ though so that we start fresh for each write operation
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