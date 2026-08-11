#include <Arduino.h>
#include <U8g2lib.h>
#include <SdFat.h>
#include <Wire.h>
#include "diskLib.h"
#include "GCRLib.h"
#include "profont6.h"
#include "SDTask.h"
#include "types.h"
#include "ui.h"

// The SD card, OLED, and serial task that runs on the other core
// It handles reading/writing tracks to the SD card, updating the OLED, and handling serial comms
// This way, the main core can focus on the timing-sensitive protocol stuff without having to worry about all this overhead

SPIClass SD_SPI(HSPI); // These two lines make sure that we use hardware SPI at 20MHz for the SD card
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(20), &SD_SPI)

// Create the OLED display object; we want a SH1106-based 128x64 I2C display
U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, U8X8_PIN_NONE);


SdFat32 SDCard; // The SD card object
SdCard* card; // A pointer to the card object so we can use it in other files
File32 disk[2]; // The disk images for the lower and upper floppy drives; 0 is upper Twiggy and 1 is lower Twiggy/Sony
FatFile rootDir; // The root directory of the SD card

uint32_t diskInsertDelay = 0; // When we started the SD card task; used to insert the lower disk after a delay
bool diskLoadingComplete = false; // A flag to indicate whether we've finished loading the lower disk yet

// Whether the buffer was dirty before the SD task executed its command; used by the UI to determine whether the last op was a read or write
bool bufferWasDirty[2] = {false, false};

static char debugString[MAX_DEBUG_STRING_LENGTH]; // A string buffer for sending debug messages to the SD card task for printing over serial

// This function tries to dispatch any pending SD card operations (if there are any) to the SD card task if it's finished with its previous operation
__attribute__((optimize("Ofast"))) IRAM_ATTR bool tryToStartSD(volatile SdTaskInterface* sdTaskInterface, TrackParams* trackParams, BufferStatus* bufferStatus) {
    if (!trackParams->pendingDispatch || !sdTaskInterface->finished || bufferStatus->stashCount != 0) {
        return false; // If there's nothing to dispatch, the SD card task isn't finished, or we haven't emptied the stash yet, then return false
    }
    if (!disk[trackParams->drive].isOpen()) {
        // If the disk image for the drive we're reading from isn't open, then we can't dispatch anything to the SD card task
        // So just return true to make the caller think that everything's okay
        // This should be okay because the Twiggy controller shouldn't ever interact with an ejected disk anyway
        // And of course set our flags to indicate that there's nothing pending anymore
        __sync_synchronize();
        sdTaskInterface->start = false;
        sdTaskInterface->finished = true;
        return true;
    }

    // Otherwise, start up the SD task with the pending operation
    // We need to set the command for the task to execute; if the track is dirty do a write, if it's not do a read
    // But downgrade a write to a read if the drive we're trying to write to is ejected
    if ((bufferStatus->bufferDirty && !disk[bufferStatus->bufferOwnerDrive].isOpen()) || !(bufferStatus->bufferDirty)) {
        sdTaskInterface->command = READ_TRACK;
    } else {
        sdTaskInterface->command = WRITE_READ_TRACK;
    }
    sdTaskInterface->writeTrack = bufferStatus->bufferOwnerTrack; // The track that we need to write back to the disk image
    sdTaskInterface->readTrack = trackParams->currentTrack; // Whatever track we ended up on and need to read
    sdTaskInterface->writeDrive = bufferStatus->bufferOwnerDrive; // We want to write writeTrack to the drive that currently owns the track buffer
    bufferStatus->bufferOwnerDrive = trackParams->drive; // Update the buffer owner drive and track to the ones we're currently working with
    bufferStatus->bufferOwnerTrack = trackParams->currentTrack;
    bufferStatus->bufferDirty = false; // Mark that the buffer is no longer dirty if it was before
    sdTaskInterface->readDrive = trackParams->drive; // And read readTrack from the drive that we're currently working with
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
    Serial.setTxTimeoutMs(0); // Make sure that serial doesn't block if nothing is reading from the other end
    Serial.println("Starting ESFloppy...");
    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); // Start comms with the SD card using our hardware SPI instance
    // Now initialize the OLED
    Wire.begin(OLED_SDA, OLED_SCL); // Start the I2C bus for the OLED
    OLED.begin(); // And init it
    // Clear the display since there might be garbage on it after reset
    OLED.clearBuffer();
    OLED.sendBuffer();
    // Set the default font and text settings for the OLED
    //OLED.setFont(u8g2_font_Untitled16PixelSansSerifBitmap_tr);
    OLED.setFont(u8g2_font_profont6_tr); // u8g2_font_profont11_tf
    OLED.setFontRefHeightExtendedText(); // Not sure what this one does; I copy-pasted it from one of the u8g2 examples
    OLED.setDrawColor(1);
    OLED.setFontPosTop(); // Same goes for this one
    OLED.setFontDirection(0);
    // And now move onto the SD card
    if (!SDCard.begin(SD_CONFIG)) { // Initialize the SD card with our hardware SPI instance
        Serial.println("SD card initialization failed! Halting..."); // And print an error/go into an infinite loop on failure
        OLED.clearBuffer();
        OLED.drawStr( 0, 0, "SD init failed!");
        OLED.sendBuffer();
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
    //Twiggy/LisaDraw 1.0.dc42
    sdTaskParams->diskMetadata[0]->driveIndex = 0; // Set the drive index for the upper drive
    if (!openImage("Twiggy/LisaGraph 1.0.dc42", &disk[0], sdTaskParams->diskMetadata[0])) { // And try opening a disk image file for the upper drive
        Serial.println("Failed to open disk image for upper drive! Halting..."); // Give another error/infinite loop on failure
        OLED.clearBuffer();
        OLED.drawStr(0, 0, "Failed to open image!");
        OLED.sendBuffer();
        while(1) {
            // If we fail to open the disk image, then spin forever
            vTaskDelay(1);
        }
    }
    sdTaskParams->diskMetadata[1]->driveIndex = 1; // Set the drive index for the lower/Sony drive
    if (!openImage("Twiggy/LisaDraw 1.0.dc42", &disk[1], sdTaskParams->diskMetadata[1])) { // And the lower drive
        Serial.println("Failed to open disk image for lower/Sony drive! Halting..."); // Give another error/infinite loop on failure
        OLED.clearBuffer();
        OLED.drawStr(0, 0, "Failed to open image!");
        OLED.sendBuffer();
        while(1) {
            // If we fail to open the disk image, then spin forever
            vTaskDelay(1);
        }
    }
    // Mark the upper disk as inserted, but the lower one as not inserted; we'll insert it later on a couple-second delay in the main SD task
    sdTaskParams->diskMetadata[0]->diskInserted = true;
    sdTaskParams->diskMetadata[1]->diskInserted = false;
    //sdTaskParams->diskMetadata[1]->diskInserted = true;
    diskLoadingComplete = false; // Mark that we haven't finished loading the lower disk yet
    // Read and encode track 0 so that we can start sending it out when the Lisa requests it
    readTrack(0, &disk[sdTaskParams->sdTaskInterface->readDrive], sdTaskParams->trackBufferDecoded, sdTaskParams->diskMetadata[sdTaskParams->sdTaskInterface->readDrive]);
    encodeTrackToGCR(0, sdTaskParams->trackBufferDecoded, sdTaskParams->trackBufferGCR, sdTaskParams->diskMetadata[sdTaskParams->sdTaskInterface->readDrive]);
    Serial.println("ESFloppy is ready!"); // And if all this succeeds, print a ready message
    // Make sure that everything above here is truly done before we continue
    __sync_synchronize();
    // Now that all of the initialization is done, there's just one last thing to do before we tell the main core that we're ready
    // And that's to get the UI to display the welcome screen and allow the user to configure any settings that they may want to change
    // The uiUpdate function will return true when this process is done and false otherwise, so just keep calling it until it returns true
    while (!uiUpdate()) {
        vTaskDelay(1);
    }
    sdTaskParams->sdTaskInterface->initDone = true; // The UI is happy, so set initDone to tell the main task that we're done initializing now
    diskInsertDelay = esp_cpu_get_cycle_count(); // Get the current time so that we know when to insert the lower disk after a couple seconds
    // We don't ever want this task to exit, so infinite-loop in here
    while (1) {
        if (!sdTaskParams->sdTaskInterface->start) {
            // If we haven't been told to start processing a request, then just wait a little while and check again
            // While we're at it, might as well print out any debug messages that have been sent to us from the main core
            while ((debugTail != debugHead) && !sdTaskParams->sdTaskInterface->start) {
                Serial.print((const char*)debugRingBuf[debugTail]); // Print the next debug message in the ring buffer
                debugTail = (debugTail + 1) & 15; // And move the tail pointer to the next message in the ring buffer
            }
            // If it's been 3 seconds since we started, then insert the lower Twiggy if it hasn't been inserted yet (and we're emulating Twiggies)
            if ((esp_cpu_get_cycle_count() - diskInsertDelay) > 720000000 && !diskLoadingComplete && !sdTaskParams->diskMetadata[1]->diskInserted && sdTaskParams->diskMetadata[1]->driveType == DriveTwiggy) {
                sdTaskParams->diskMetadata[1]->diskInserted = true; // Mark the lower disk as inserted
                diskLoadingComplete = true; // And mark that we've finished loading the lower disk
            }
            // Also call the uiUpdate function to update the OLED if necessary
            uiUpdate();
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
            bufferWasDirty[sdTaskParams->sdTaskInterface->writeDrive] = true; // Mark that the buffer was dirty before we executed this command
            decodeTrackFromGCR(sdTaskParams->sdTaskInterface->writeTrack, sdTaskParams->trackBufferGCR, sdTaskParams->trackBufferDecoded, sdTaskParams->diskMetadata[sdTaskParams->sdTaskInterface->writeDrive]);
            uint32_t startTime = micros();
            // If a -1 arrives here (the Twiggy timing track), then it goes through as a 255 and writeTrack rejects it, just like we want
            writeTrack(sdTaskParams->sdTaskInterface->writeTrack, &disk[sdTaskParams->sdTaskInterface->writeDrive], sdTaskParams->trackBufferDecoded, sdTaskParams->diskMetadata[sdTaskParams->sdTaskInterface->writeDrive]);
            /*snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "wrote drive %d track %d in %dus\n",
                sdTaskParams->sdTaskInterface->writeDrive, sdTaskParams->sdTaskInterface->writeTrack,
                micros() - startTime);
            debugPrint(debugString, strlen(debugString));*/
        }
        if (sdTaskParams->sdTaskInterface->command == READ_TRACK || sdTaskParams->sdTaskInterface->command == WRITE_READ_TRACK) {
            // If the command is either a read OR write, then we now need to read the requested track and encode it to GCR
            // Same deal here with the -1 for the Twiggy timing track; readTrack will reject it and not read anything
            /*snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "read drive %d track %d\n",
                    sdTaskParams->sdTaskInterface->readDrive, sdTaskParams->sdTaskInterface->readTrack);
            debugPrint(debugString, strlen(debugString));*/
            readTrack(sdTaskParams->sdTaskInterface->readTrack, &disk[sdTaskParams->sdTaskInterface->readDrive], sdTaskParams->trackBufferDecoded, sdTaskParams->diskMetadata[sdTaskParams->sdTaskInterface->readDrive]);
            encodeTrackToGCR(sdTaskParams->sdTaskInterface->readTrack, sdTaskParams->trackBufferDecoded, sdTaskParams->trackBufferGCR, sdTaskParams->diskMetadata[sdTaskParams->sdTaskInterface->readDrive]);
        }
        else if (sdTaskParams->sdTaskInterface->command == CLOSE_IMAGE) {
            // If the command is to close the disk image for the currently-selected drive, then obey
            closeImage(&disk[sdTaskParams->sdTaskInterface->readDrive], sdTaskParams->diskMetadata[sdTaskParams->sdTaskInterface->readDrive]);
        }
        // Now that we're done, synchronize again to make sure that everything above here is truly done before we say we're done
        __sync_synchronize();
        // And finally, set finished high and start low to tell the main core that we're done
        sdTaskParams->sdTaskInterface->start = false;
        sdTaskParams->sdTaskInterface->finished = true;
    }
}