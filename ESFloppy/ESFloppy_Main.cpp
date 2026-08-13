#include <Arduino.h>
#include "GPIO.h"
#include "LEDC.h"
#include "SDTask.h"
#include "sonyInterface.h"
#include "twiggyInterface.h"
#include "types.h"

// Watchdog timer write enable register and value
#define TIMG1_WDT_WE_REG 0x60020064
#define TIMG1_WDT_WE 0x050D83AA1

// Watchdog timer configuration register and enable bit
#define TIMG1_WDT_CONF_REG 0x60020048
#define TIMG1_WDT_EN 1 << 31

static DecodedSector trackBufferDecoded[2][22]; // Buffer for decoded sectors for each side of the disk (2 sides, max 22 sectors per track)
static GcrSector trackBufferGCR[2][22]; // Buffer for GCR-encoded sectors for each side of the disk

DiskImageMetadata diskMetadata[2]; // Metadata for the disk images, one for each drive
// And pointers to the metadata
DiskImageMetadata* diskMetadataPointers[2] = {&diskMetadata[0], &diskMetadata[1]};
// Make an SdTaskInterface struct; make sure it's static and volatile since it's shared between two cores
static volatile SdTaskInterface sdTaskInterface = {0, 0, 1, 1, READ_TRACK, false, true, false};

// Make an instance of the SdTaskParams struct and initialize it with the appropriate pointers
SdTaskParams sdTaskParams = {&sdTaskInterface, trackBufferGCR, trackBufferDecoded, {diskMetadataPointers[0], diskMetadataPointers[1]}};

// This is the struct that holds all of the system settings that are saved to NVS and loaded on boot
extern ConfigSettings configSettings;

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
        &sdTaskParams, // Pass a pointer to the SdTaskParams struct
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

__attribute__((optimize("Ofast"))) IRAM_ATTR void loop() {
    // In the main loop here, we just need to run the appropriate drive interface loop based on the emulation mode set in configSettings
    if (configSettings.emulMode == ModeSonyLisa || configSettings.emulMode == ModeSonyMac) {
        // If it's a 400K or 800K disk, then run the Sony loop
        sonyLoop(&sdTaskInterface, trackBufferGCR, diskMetadataPointers);
    }
    else if (configSettings.emulMode == ModeTwiggy) {
        // If it's a Twiggy disk, then run the Twiggy loop (duh)
        twiggyLoop(&sdTaskInterface, trackBufferGCR, diskMetadataPointers);
    }
    // Otherwise, just don't do anything (although we do have to yield our task)
    //vTaskDelay(0);
}