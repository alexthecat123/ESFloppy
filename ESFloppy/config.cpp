#include <Arduino.h>
#include <Preferences.h>
#include "types.h"

// This file contains all of ESFloppy's routines for handling configuration settings
// All of the settings are stored in the ESP32's non-volatile storage so that they persist across power cycles

Preferences preferences; // The Preferences object that we use to read/write settings to the ESP32's non-volatile storage

// Initializes the Preferences object to prep it for reading/writing settings
void initConfig() {
    preferences.begin("ESFloppy", false); // Open the "ESFloppy" namespace in the Preferences object for reading/writing
}

// Reads the configuration settings from non-volatile storage and returns them in a ConfigSettings struct
// If the storage appears to be uninitialized, then it'll return a ConfigSettings struct with default values instead
ConfigSettings readConfig() {
    ConfigSettings settings;
    if (!preferences.isKey("configStruct")) {
        // If the configStruct key doesn't exist, then the storage is uninitialized, so return default values
        // No need to write them back to NVS here since the user will do that if they care about changing them from the defaults
        settings.emulMode = ModeSonyLisa; // Default to a Lisa Sony drive
        settings.dimDisplay = true; // Dim the OLED after inactivity
        settings.brightness = 255; // And set it to max brightness
        return settings;
    }
    // If it does exist, then read the configStruct key from NVS into the settings struct
    uint32_t retrievedBytes =preferences.getBytes("configStruct", &settings, sizeof(settings));
    if (retrievedBytes != sizeof(settings) || (settings.emulMode != ModeSonyLisa && settings.emulMode != ModeSonyMac && settings.emulMode != ModeTwiggy)) {
        // If the number of bytes read from NVS doesn't match the size of the settings struct, then something is wrong
        // Also assume a problem if the emulMode isn't one of the valid values because this will cause the loop() to not execute any of the drive loops
        // Just return default values in this case as well
        settings.emulMode = ModeSonyLisa; // Default to a Lisa Sony drive
        settings.dimDisplay = true;
        settings.brightness = 255;
    }
    return settings;
}

// Writes the given configuration settings to non-volatile storage
void writeConfig(const ConfigSettings& settings) {
    // This is really simple; just write the entire settings struct to non-volatile storage
    preferences.putBytes("configStruct", &settings, sizeof(settings));
}

// Closes the Preferences object once we're done with it
void closeConfig() {
    preferences.end();
}