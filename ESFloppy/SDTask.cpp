#include <Arduino.h>
#include <Adafruit_SH110X.h>
#include <SdFat.h>
#include "diskLib.h"
#include "GCRLib.h"
#include "SDTask.h"
#include "types.h"

// The SD card, OLED, and serial task that runs on the other core
// It handles reading/writing tracks to the SD card, updating the OLED, and handling serial comms
// This way, the main core can focus on the timing-sensitive protocol stuff without having to worry about all this overhead

SPIClass SD_SPI(HSPI); // These two lines make sure that we use hardware SPI at 20MHz for the SD card
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(20), &SD_SPI)

// Create the OLED display object; we want a 128x64 display
Adafruit_SH1106G OLED = Adafruit_SH1106G(128, 64, &Wire, -1);

SdFat32 SDCard; // The SD card object
SdCard* card; // A pointer to the card object so we can use it in other files
File32 disk; // The disk image that ESFloppy is using
FatFile rootDir; // The root directory of the SD card

//uint32_t startTime = 0;

static char debugString[MAX_DEBUG_STRING_LENGTH]; // A string buffer for sending debug messages to the SD card task for printing over serial

// This function tries to dispatch any pending SD card operations (if there are any) to the SD card task if it's finished with its previous operation
__attribute__((optimize("Ofast"))) IRAM_ATTR bool tryToStartSD(volatile SdTaskInterface* sdTaskInterface, TrackParams* trackParams) {
    if (!trackParams->pendingDispatch || !sdTaskInterface->finished) {
        return false; // If there's nothing to dispatch or the SD card task isn't finished, then return false
    }
    if (!disk.isOpen()) {
        // If the disk image isn't open anymore (disk ejected), then we can't dispatch anything
        // So just return true to make the caller think that everything's okay
        // And of course set our flags to indicate that there's nothing pending anymore
        __sync_synchronize();
        sdTaskInterface->start = false;
        sdTaskInterface->finished = true;
        return true;
    }
    //snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Took %dus to dispatch SD task.\n", micros() - startTime);
    //debugPrint(debugString, strlen(debugString));
    // Otherwise, start up the SD task with the pending operation
    sdTaskInterface->writeTrack = trackParams->pendingTrackToWrite; // The track that we need to write back to the disk image
    sdTaskInterface->readTrack = trackParams->currentTrack; // Whatever track we ended up on and need to read
    sdTaskInterface->command = trackParams->pendingCommand; // Whether there's a track to write in the first place
    trackParams->pendingTrackToWrite = trackParams->currentTrack; // Update pendingWriteTrack to wherever we are now for next time
    sdTaskInterface->finished = false; // We're obviously not finished anymore since we're dispatching a new operation
    __sync_synchronize(); // Call synchronize to make sure that everything above here is done before we move on
    sdTaskInterface->start = true; // Start the SD task
    return true; // And return true to indicate that we dispatched a new operation
}

volatile char debugRingBuf[16][MAX_DEBUG_STRING_LENGTH]; // The ring buffer that holds debug messages to be printed over serial
volatile uint32_t debugHead = 0; // Head and tail pointers for the ring buffer
volatile uint32_t debugTail = 0;

// This function is used to send debug messages (character strings) from the main core to the SD card task core
// They're stored in a ring buffer and printed over serial whenever the SD core gets a chance
void debugPrint(char* inString, uint32_t length) {
    if (length >= MAX_DEBUG_STRING_LENGTH) {
        return; // If the string is too long, then we can't print it
    }
    uint32_t nextSlot = (debugHead + 1) & 15; // Find the next open slot in the ring buffer that we can write this string to
    if (nextSlot == debugTail) {
        return; // If the ring buffer is full, then just give up; debug messages aren't THAT important
    }
    // Otherwise, copy the string into the buffer
    strncpy((char*)debugRingBuf[debugHead], inString, length); // Copy the string into the ring buffer at the head pointer
    debugRingBuf[debugHead][length] = '\0'; // Null-terminate the string too
    __sync_synchronize(); // Make sure that all the writes to the ring buffer are done before we update the head pointer
    // And now update the head pointer to indicate that there's a new message in the buffer
    debugHead = nextSlot;
}

// This is the actual task itself that runs on the other core
void sdCardTask(void* params) {
    SdTaskParams* sdTaskParams = (SdTaskParams*)params; // First, cast the params pointer from void to the correct type (SdTaskParams)
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
        while(1) {
            // If we fail to init the SD card, then spin forever
            // Make sure to call vTaskDelay instead of just a plain while(1) loop so that the watchdog doesn't reset the ESP32
            vTaskDelay(1);
        }
    }
    card = SDCard.card(); // Now that the card is initialized, store a pointer to its object
    rootDir.open("/"); // Then open the card's root directory
    //My Lisa Stuff/MWP/MW_1.018_Install.img
    //test_image.dc42
    //My Lisa Stuff/LOS 3 Debozoed/LisaCalc.dc42
    //My Lisa Stuff/MacWorks Plus II Install.image
    //LisaTest 3.0 1.image
    //copy_image_800k.dc42
    if (!openImage("Twiggy/LisaGraph 1.0.dc42", &disk, sdTaskParams->diskMetadata)) { // And try opening a disk image file
        Serial.println("Failed to open disk image! Halting..."); // Give another error/infinite loop on failure
        OLED.clearDisplay();
        OLED.setTextSize(2);
        OLED.setTextColor(SH110X_WHITE);
        OLED.setCursor(0, 0);
        OLED.print("Can't Open Disk!");
        OLED.display();
        while(1) {
            // If we fail to open the disk image, then spin forever
            vTaskDelay(1);
        }
    }
    // Read and encode track 0 so that we can start sending it out when the Lisa requests it
    readTrack(0, &disk, sdTaskParams->trackBufferDecoded, sdTaskParams->diskMetadata);
    encodeTrackToGCR(0, sdTaskParams->trackBufferDecoded, sdTaskParams->trackBufferGCR, sdTaskParams->diskMetadata);
    OLED.setTextSize(1); // Set the OLED text size to 1 and color to white
    OLED.setTextColor(SH110X_WHITE);
    Serial.println("ESFloppy is ready!"); // And if all this succeeds, print a ready message
    // Make sure that everything above here is truly done before we continue
    __sync_synchronize();
    sdTaskParams->sdTaskInterface->initDone = true; // Set initDone to tell the main task that we're done initializing now
    // We don't ever want this task to exit, so infinite-loop in here
    while (1) {
        if (!sdTaskParams->sdTaskInterface->start) {
            // If we haven't been told to start processing a request, then just wait a little while and check again
            // While we're at it, might as well print out any debug messages that have been sent to us from the main core
            while ((debugTail != debugHead) && !sdTaskParams->sdTaskInterface->start) {
                Serial.print((const char*)debugRingBuf[debugTail]); // Print the next debug message in the ring buffer
                debugTail = (debugTail + 1) & 15; // And move the tail pointer to the next message in the ring buffer
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
        if (sdTaskParams->sdTaskInterface->command == WRITE_READ_TRACK) {
            decodeTrackFromGCR(sdTaskParams->sdTaskInterface->writeTrack, sdTaskParams->trackBufferGCR, sdTaskParams->trackBufferDecoded, sdTaskParams->diskMetadata);
            uint32_t startTime = micros();
            writeTrack(sdTaskParams->sdTaskInterface->writeTrack, &disk, sdTaskParams->trackBufferDecoded, sdTaskParams->diskMetadata);
            snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Took %dus to write track.\n", micros() - startTime);
            debugPrint(debugString, strlen(debugString));
        }
        if (sdTaskParams->sdTaskInterface->command == READ_TRACK || sdTaskParams->sdTaskInterface->command == WRITE_READ_TRACK) {
            // If the command is either a read OR write, then we now need to read the requested track and encode it to GCR
            readTrack(sdTaskParams->sdTaskInterface->readTrack, &disk, sdTaskParams->trackBufferDecoded, sdTaskParams->diskMetadata);
            encodeTrackToGCR(sdTaskParams->sdTaskInterface->readTrack, sdTaskParams->trackBufferDecoded, sdTaskParams->trackBufferGCR, sdTaskParams->diskMetadata);
        }
        else if (sdTaskParams->sdTaskInterface->command == CLOSE_IMAGE) {
            // If the command is to close the disk image, then obey
            closeImage(&disk, sdTaskParams->diskMetadata);
        }
        // Now that we're done, synchronize again to make sure that everything above here is truly done before we say we're done
        __sync_synchronize();
        // And finally, set finished high and start low to tell the main core that we're done
        sdTaskParams->sdTaskInterface->start = false;
        sdTaskParams->sdTaskInterface->finished = true;
    }
}