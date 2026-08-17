#include <Arduino.h>
#include "esp_ota_ops.h"
#include "esp_rom_crc.h"
#include <U8g2lib.h>
#include <SdFat.h>
#include <Wire.h>
#include "diskLib.h"
#include "fwVersion.h"
#include "GCRLib.h"
#include "profont6.h"
#include "SDTask.h"
#include "types.h"
#include "ui.h"
#include "uiHelpers.h"

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

// This function checks to see if a firmware update file is present on the SD card and if so, upgrades/downgrades the firmware
void attemptFirmwareUpdate() {
    // First up, try to open the firmware update file on the SD card
    File32 updateFile;
    if (!updateFile.open("/esfloppy_fw.bin", O_RDONLY)) {
        // If we can't open the file, then it probably doesn't exist and there's no update to do, so just return
        return;
    }

    // If we succeeded in opening it, then we need to take a look at its header to make sure that it's valid
    FirmwareUpdateHeader header;
    if (updateFile.read(&header, sizeof(FirmwareUpdateHeader)) != sizeof(FirmwareUpdateHeader)) {
        // If we couldn't read the header, then the file is probably corrupted, so just close it and return without doing anything
        updateFile.close();
        return;
    }

    header.versionString[7] = '\0'; // Make sure the version string is null-terminated

    // But if we did read the header, then we need to make sure that it's an actual header and not just garbage data
    if (strncmp(header.magicString, "ESFloppy", 8) != 0) {
        // Start by making sure that the magic string is in fact "ESFloppy" and return if not
        updateFile.close();
        return;
    }

    // Now we know it's a real firmware file; check its version string to see if it's the same as the current firmware version
    // If so, then there's no need to update, so just return
    // Add an extra check too so that the user can override the version requirement if they hold LEFT and RIGHT at the same time
    // If they do, this, we ignore the version check and allow them to "upgrade" to the same version if they want to
    if ((strncmp(header.versionString, FIRMWARE_VERSION, 8) == 0) && !((digitalRead(LEFT) == LOW) && (digitalRead(RIGHT) == LOW))) {
        // If so, then there's no need to update, so return once again
        updateFile.close();
        return;
    }

    // If we get here, then we actually have an update to do, so see if we have a place to store it
    // In its default partition scheme, the ESP32 should have two OTA partitions that we can use to store firmware
    // We're using one of them for the current firmware, so find the other one and use it for the new one
    const esp_partition_t* updatePartition = esp_ota_get_next_update_partition(NULL);
    if ((updatePartition == NULL) || (header.firmwareSize > updatePartition->size)) {
        // If we couldn't find a partition to write to (the ESP is programmed with the wrong scheme) or the firmware is too big for the partition, then print an error
        updateFile.close();
        drawFwUpdateError("No space for new FW!");
        return;
    }

    // If we made it here, then we should be in good shape to actually start the update
    esp_ota_handle_t updateHandle;
    if (esp_ota_begin(updatePartition, header.firmwareSize, &updateHandle) != ESP_OK) {
        // If we failed to start the update, then print another error and return
        updateFile.close();
        drawFwUpdateError("Can't start update!");
        return;
    }

    // Now that we've started the update, we need to read the firmware data from the file and write it to the updatePartition
    static uint8_t fwBuffer[4096]; // A buffer to hold the firmware data as we read it from the file
    uint32_t runningCRC = 0; // A running CRC that we compute as we write the firmware data to the partition
    uint32_t bytesWritten = 0; // How many bytes we've written so far
    while (bytesWritten < header.firmwareSize) {
        // Keep going until we've written the entire firmware file to the partition
        // Read data in chunks of 4KB (or less if we're at the end of the file and there's less than that left)
        uint32_t bytesToRead = min((uint32_t)sizeof(fwBuffer), header.firmwareSize - bytesWritten);
        uint32_t bytesRead = updateFile.read(fwBuffer, bytesToRead);
        if (bytesRead != bytesToRead) {
            // If we didn't read the expected number of bytes, then something is wrong with the file, so abort, print an error, and return
            esp_ota_abort(updateHandle);
            updateFile.close();
            drawFwUpdateError("Can't read FW file!");
            return;
        }
        
        // Now try and write the data to the update partition
        if (esp_ota_write(updateHandle, fwBuffer, bytesRead) != ESP_OK) {
            // If we failed to write the data, then abort, print an error, and return just like before
            esp_ota_abort(updateHandle);
            updateFile.close();
            drawFwUpdateError("Failed to write FW!");
            return;
        }

        // Now update our running CRC with the data we just wrote
        runningCRC = esp_rom_crc32_le(runningCRC, fwBuffer, bytesRead);
        // Update the bytesWritten counter
        bytesWritten += bytesRead;
        // And update the progress bar on the OLED
        drawFwUpdateProgress(bytesWritten, header.firmwareSize, header.versionString);
    }

    // Once we're out of that loop, the firmware should be fully written, and we just need to finalize the update
    // Start by checking the CRC of the firmware we just wrote to make sure that it matches the expected CRC in the header
    if (runningCRC != header.firmwareCRC) {
        // If not, then abort with an error
        esp_ota_abort(updateHandle);
        updateFile.close();
        drawFwUpdateError("FW CRC mismatch!");
        return;
    }

    // If the CRC matches, then finalize the update
    if (esp_ota_end(updateHandle) != ESP_OK) {
        // If we failed to finalize the update, then abort with an error
        updateFile.close();
        drawFwUpdateError("Bad FW image!");
        return;
    }

    // And the very last thing: if all of that succeeded, then we need to switch the boot partition to the new firmware's partition
    // The beauty of this method is that if anything fails up to this point, the ESP will just keep booting from the old partition and nothing breaks
    // We only switch to the new partition here at the very end
    if (esp_ota_set_boot_partition(updatePartition) != ESP_OK) {
        // Abort with yet another error if this failed too
        updateFile.close();
        drawFwUpdateError("Boot switch failed!");
        return;
    }

    // If all of that succeeded, then we actually succeeded with the firmware update, so close the file and display a success message
    updateFile.close();
    drawFwUpdateSuccess(header.versionString);
    // The success message blocks until the user presses SEL, so when we get here, they'll have acknowledged the update and we can reboot into the new firmware
    ESP.restart();
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
        // Display an error message and the "bad SD card" icon on the OLED as well
        OLED.clearBuffer();
        OLED.drawStr(((OLED.getDisplayWidth() - OLED.getStrWidth("SD card init failed!")) / 2), 0, "SD card init failed!");
        OLED.drawStr(((OLED.getDisplayWidth() - OLED.getStrWidth("Is a card inserted?")) / 2), MENU_ITEM_HEIGHT * 1, "Is a card inserted?");
        drawSDErrorIcon(((128 - SD_ERROR_ICON_WIDTH) / 2), ((MENU_ITEM_HEIGHT * 2) + (OLED.getDisplayHeight() - (MENU_ITEM_HEIGHT * 2) - SD_ERROR_ICON_HEIGHT) / 2));
        OLED.sendBuffer();
        while(1) {
            // Make sure to call vTaskDelay instead of just a plain while(1) loop so that the watchdog doesn't reset the ESP32
            vTaskDelay(1);
        }
    }
    card = SDCard.card(); // Now that the card is initialized, store a pointer to its object
    // Now try to open the SD card's root directory as an experiment to make sure that we can actually read from it
    if (!rootDir.open("/")) {
        // If we fail, then the SD card is bad or maybe has an unsupported filesystem, so print an error just like we did earlier and halt
        Serial.println("Failed to open SD card root directory! Halting...");
        OLED.clearBuffer();
        OLED.drawStr(((OLED.getDisplayWidth() - OLED.getStrWidth("Can't open SD root!")) / 2), 0, "Can't open SD root!");
        OLED.drawStr(((OLED.getDisplayWidth() - OLED.getStrWidth("Make sure it's FAT32.")) / 2), MENU_ITEM_HEIGHT * 1, "Make sure it's FAT32.");
        drawSDErrorIcon(((128 - SD_ERROR_ICON_WIDTH) / 2), ((MENU_ITEM_HEIGHT * 2) + (OLED.getDisplayHeight() - (MENU_ITEM_HEIGHT * 2) - SD_ERROR_ICON_HEIGHT) / 2));
        OLED.sendBuffer();
        while(1) {
            vTaskDelay(1);
        }
    }
    // Now that we know we can talk to the SD card, check for a firmware update file and if it's present, update the firmware
    attemptFirmwareUpdate();
    // If we end up here, then there was no firmware update to do (the ESP will reboot if there was one)
    sdTaskParams->diskMetadata[0]->driveIndex = 0; // Set the drive index for the upper drive
    sdTaskParams->diskMetadata[1]->driveIndex = 1; // And also for the lower/Sony drive
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
            /*if ((esp_cpu_get_cycle_count() - diskInsertDelay) > 720000000 && !diskLoadingComplete && !sdTaskParams->diskMetadata[1]->diskInserted && sdTaskParams->diskMetadata[1]->driveType == DriveTwiggy) {
                sdTaskParams->diskMetadata[1]->diskInserted = true; // Mark the lower disk as inserted
                diskLoadingComplete = true; // And mark that we've finished loading the lower disk
            }*/
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