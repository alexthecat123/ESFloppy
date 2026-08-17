#include "types.h"

// Header file for the SD card, OLED, and serial task that runs on the other core
// It handles reading/writing tracks to the SD card, updating the OLED, and handling serial comms
// This way, the main core can focus on the timing-sensitive protocol stuff without having to worry about all this overhead

#define MAX_DEBUG_STRING_LENGTH 255 // The max length of a debug string that can be sent to the SD card task for printing over serial

// This function tries to dispatch any pending SD card operations (if there are any) to the SD card task if it's finished with its previous operation
__attribute__((optimize("Ofast"))) IRAM_ATTR bool tryToStartSD(volatile SdTaskInterface* sdTaskInterface, TrackParams* trackParams, BufferStatus* bufferStatus);

// This function is used to send debug messages (character strings) from the main core to the SD card task core
// They're stored in a ring buffer and printed over serial whenever the SD core gets a chance
void debugPrint(char* inString, uint32_t length);

// This function checks to see if a firmware update file is present on the SD card and if so, upgrades/downgrades the firmware
void attemptFirmwareUpdate();

// This is the actual task itself that runs on the other core
void sdCardTask(void* params);