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
        // Otherwise, grab the side number from the low HDS bit and continue
        uint32_t side = (gpioIn & 1 << HDS) ? 1 : 0;

        // When we arrive here, we'll be on the first half of a bit, so just wait until it's time to send out that first half
        prevBitTime += 240;
        while ((int32_t)(esp_cpu_get_cycle_count() - prevBitTime) < 0); // Get the number of CPU cycles between now and the last bit time; esp_cpu_get_cycle_count() is faster than ESP.getCycleCount()
        // Once that while loop finishes (240 cycles at 240MHz is 1us), it's time to send out the first half of our bit
        // Now we need to extract the next bit from trackBufferGCR[side][currentSector]
        GcrSector* gcrSector = &trackBufferGCR[side][currentSector]; // Get a pointer to the current GCR sector
        uint8_t* gcrPtr = (uint8_t*)gcrSector; // And then get a uint8_t pointer to the sector data
        // And finally extract the bit; this line does several things in one
        // First it extracts the byte we need from the sector by dividing the in-sector index by 8
        // And then it shifts that byte to the right by 7 minus the remainder of the in-sector index divided by 8, which gives us the bit we want in the LSB
        bool bit = (gcrPtr[inSectorIndex >> 3] >> (7 - (inSectorIndex & 7))) & 1;
        // Now we can send out the bit on RDA
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

void updateOLED () {
    OLED.clearDisplay();
    //OLED.setTextSize(1);
    //OLED.setTextColor(SH110X_WHITE);
    OLED.setCursor(0, 0);
    OLED.print("Track: "); OLED.println(currentTrack);
    //OLED.print("Motor: "); OLED.println(motorOn ? "ON" : "OFF");
    //OLED.print("Image Type: "); OLED.println(diskMetadata.imageType == DC42 ? "DC42" : "RAW");
    //OLED.print("Drive Type: "); OLED.println(diskMetadata.driveType == Drive400 ? "400K" : "800K");
    //OLED.print("Disk in Place: "); OLED.println(diskMetadata.diskInserted ? "YES" : "NO");
    //OLED.print("Eject Pending: "); OLED.println(ejectPending ? "YES" : "NO");
    OLED.display();
}


void setup() {
    init(); // Init the ESP32 Arduino core
    Serial.begin(115200); // Start serial comms for debugging
    Serial.println("Starting ESFloppy...");
    initLEDC(RDA); // Initialize the LEDC peripheral on the RDA pin for sending TACH pulses
    setDuty(128); // Set the LEDC duty cycle to 50%
    enableLEDCOutput(true); // And enable its output (note that this doesn't actually connect it to the pin though)
    initPins(); // Set all of ESFloppy's pins to the correct direction and state
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
    if (!openImage("test_image.dc42", &disk, &diskMetadata)) { // And try opening a disk image file
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
    //closeImage(&disk, &diskMetadata); // Close the image
    OLED.setTextSize(1);
    OLED.setTextColor(SH110X_WHITE);
    for(int i = 0; i < 80; i++) {
        setFreq(tachPulsesPerTrackLisa[i], 8);
    }
    //updateOLED(); // Update the OLED with the current status
    Serial.println("ESFloppy is ready!"); // If all this succeeds, print a ready message
    // Just like ESProFile, we need to disable the interrupt watchdog timer
    // For the sake of speed, we have to disable interrupts throughout our entire program, and the watchdog would trigger and break everything
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
                        trackChanged = true; // Mark that the track has changed so we can reset the sector back to 0 in transmitTrack
                        // Before we step, check the dirty bit to see if the current track was modified
                        if (dirty == true) {
                            // If so, write the current track back to the disk image before we seek away
                            decodeTrackFromGCR(currentTrack, trackBufferGCR, trackBufferDecoded, &diskMetadata);
                            writeTrack(currentTrack, &disk, trackBufferDecoded, &diskMetadata);
                            dirty = false; // And clear the dirty bit
                        }
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
                        // Now that we've updated the track number, we need to read in all the sectors for this new track from the disk image
                        //interrupts();
                        readTrack(currentTrack, &disk, trackBufferDecoded, &diskMetadata);
                        //noInterrupts();
                        encodeTrackToGCR(currentTrack, trackBufferDecoded, trackBufferGCR, &diskMetadata);
                        //updateOLED(); // Update the OLED with the current status
                        interrupts();
                        Serial.println(currentTrack);
                        noInterrupts();
                        stepComplete = true; // Once they're read in, mark that the step is complete
                    }
                    break;
                case 4: // /MOTORON register (low to turn motor on, high to turn motor off)
                    motorOn = (regData == 0);
                    //updateOLED(); // Update the OLED with the current status
                    // Turn on the activity LED whenever the motor is on too
                    if (motorOn) {
                        REG_WRITE(GPIO_OUT_W1TS_REG, 1 << LED);
                    }
                    else {
                        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << LED);
                    }
                    break;
                case 6: // EJECT register (write-only, write a 1 to eject the disk)
                    // The catch here is that the 1 must be held for at least 500ms to actually eject the disk
                    // So we need to start a timer when we see a 1 written, and if it stays 1 for 500ms, we eject the disk
                    // Start by checking if the written data is a 1 and if we're not already pending an eject
                    if (regData == 1 && ejectPending == false) {
                        ejectPending = true; // If so, mark that an eject is pending
                        ejectStartTime = millis(); // And record the start time
                        //updateOLED(); // Update the OLED with the current status
                    }
                    else if (regData == 0) { // If the host writes a 0, we need to cancel any pending eject
                        ejectPending = false;
                        //updateOLED(); // Update the OLED with the current status
                    }
                    else if (ejectPending == true) { // Otherwise, if an eject is pending, check if 500ms has passed
                        if (millis() - ejectStartTime >= 500) {
                            // If so, eject the disk
                            closeImage(&disk, &diskMetadata); // Close the disk image file
                            motorOn = false; // Turn off the motor too
                            ejectPending = false;
                            //updateOLED(); // Update the OLED with the current status
                            //Serial.println("Disk ejected.");
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