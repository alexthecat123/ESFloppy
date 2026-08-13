#include <Arduino.h>
#include <SdFat.h>
#include <U8g2lib.h>
#include "config.h"
#include "diskLib.h"
#include "types.h"
#include "ui.h"
#include "uiErrorScreen.h"
#include "uiFilePicker.h"
#include "uiHelpers.h"
#include "uiSettingsMenu.h"
#include "uiState.h"

// All of the source code for ESFloppy's OLED-based UI

// The maximum number of screens that we can have on the stack at once
// Note that this is NOT the max directory depth, which is something completely different and is handled within the file browser screen itself
// This is just the total number of "high-level" screens that we can have on the stack, like status, file browser, and settings
#define SCREEN_STACK_SIZE 8

#define WRITE_INDICATOR_DURATION 500 // How long (in ms) the write indicator stays on after a write occurs

#define FADE_DELAY 30000 // How long (in ms) of inactivity before the OLED dims itself

// We need a stack to hold multiple different screens so that we can push and pop them as we navigate through the UI
// This makes going back to a previous screen easy since we can just pop it off the stack
Screen* screenStack[SCREEN_STACK_SIZE] = {nullptr}; // Initialize the screen stack to all null pointers
uint32_t screenStackIndex = 0; // How many screens are currently on the stack

Screen *currentScreen = nullptr; // A pointer to the current screen that we're on in the UI

// Flag that gets set whenever something on screen has changed and the framebuffer needs to be resent to the OLED
static bool redrawScreen = true;

// The disk image metadata and the image files that are currently inserted (or not) in the drives
// These are the same ones that are set up in SDTask and used throughout the emulator, but we need them here too
extern DiskImageMetadata* diskMetadataPointers[2];
extern File32 disk[2];

// This is the struct that holds all of the system settings that are saved to NVS and loaded on boot
ConfigSettings configSettings;

// The real-time status of the drives, straight from sonyInterface and twiggyInterface, so we can show it on the status screen
// These are literally just the TrackParams and BufferStatus structs used internally by the drive interfaces, so they're always fully up to date
extern TrackParams* sonyUiTrackParams;
extern BufferStatus* sonyUiBufferStatus;
extern TrackParams* twiggyUiTrackParams[2];
extern BufferStatus* twiggyUiBufferStatus;

// The welcome screen is the first thing that the user sees when they turn on ESFloppy
// It shows a welcome message, tells them what drive type they're emulating, and gives them the option to go to the settings screen
Screen welcomeScreen = {
    welcomeEnter, // The function that gets called when we enter the welcome screen
    welcomeTick, // The function that gets called periodically while we're on the welcome screen
    welcomeButtonPress, // The function that gets called when a button is pressed while we're on the welcome screen
    welcomeDrawScreen // The function that gets called to draw the welcome screen
};

// The settings screen can be accessed from the welcome screen and allows the user to configure their ESFloppy
// You can't exit this screen and go to any other screen; your only option is to reboot
Screen settingsScreen = {
    settingsEnter,
    settingsTick,
    settingsButtonPress,
    settingsDrawScreen
};

// The status screen is the screen that we show when the UI is idle; it shows the current state of the drives/disks
Screen statusScreen = {
    statusEnter,
    statusTick,
    statusButtonPress,
    statusDrawScreen
};

// The file picker screen is probably pretty self-explanatory; it's what lets the user choose what image they want to insert into a drive
Screen filePickerScreen = {
    filePickerEnter,
    filePickerTick,
    filePickerButtonPress,
    filePickerDrawScreen
};

// The error screen that we show when something goes wrong, complete with a custom message and X icon
Screen errorScreen = {
    errorEnter,
    errorTick,
    errorButtonPress,
    errorDrawScreen
};

// Marks the whole screen as dirty and needing to be redrawn/sent to the OLED
void redrawWholeScreen() {
    redrawScreen = true;
    markAllPagesDirty();
}

// Marks the screen as needing to be redrawn, but only sends the region covering rows yStart through yEnd to the OLED
// This is much more efficient than redrawing the whole screen, and avoids blocking the SD task for the time of a full frame
void redrawRegion(uint32_t yStart, uint32_t yEnd) {
    redrawScreen = true;
    markRegionDirty(yStart, yEnd);
}

// This function pushes a new screen onto the screen stack and makes it the current screen
void pushScreen(Screen* newScreen) {
    // If the current screen is not null, then we need to push it onto the stack
    // Make sure that the stack isn't full too
    if (currentScreen != nullptr && screenStackIndex < SCREEN_STACK_SIZE) {
        screenStack[screenStackIndex] = currentScreen; // Push the current screen onto the top of the stack
        screenStackIndex++; // And increment to the next slot in the stack for the next push
    }
    currentScreen = newScreen; // Make the new screen the current screen
    currentScreen->enter(); // And call the enter function for the new screen
    redrawWholeScreen(); // The whole screen just changed, so everything has to be redrawn
}

// This function pops the current screen off the screen stack and makes the previous screen the current screen
void popScreen() {
    // Make sure that the stack isn't empty before we pop it
    if (screenStackIndex > 0) {
        screenStackIndex--;
        currentScreen = screenStack[screenStackIndex]; // Otherwise, make the previous screen the current screen
        screenStack[screenStackIndex] = nullptr; // And clear out the slot that we just popped
    } else {
        // If the stack is in fact empty, then go back to the status screen
        currentScreen = &statusScreen;
    }
    // Now that the screen has changed, call its enter function and redraw the whole screen just like for pushScreen
    currentScreen->enter();
    redrawWholeScreen();
}

// Returns true if we're emulating Twiggies rather than a Sony; I'm getting really tired of typing metadata->driveType == DriveTwiggy all over the place
static bool twiggyMode() {
    return configSettings.emulMode == ModeTwiggy;
}

// Returns how many drives the current emulation mode actually has; Twiggy has two, Sony has one
static uint32_t getDriveCount() {
    return twiggyMode() ? 2 : 1;
}

// This function returns the drive index that corresponds to a given display slot on the status screen
// In Twiggy mode, drive 0 goes to slot 0 (the top slot) and drive 1 goes to slot 1 (the bottom slot)
// But in Sony mode, the single drive (drive 1) always goes into slot 0 and the bottom slot is unused
static uint32_t driveForScreenSlot(uint32_t slot) {
    return twiggyMode() ? slot : 1;
}

// Returns the proper trackParams struct for a given drive, depending on whether we're in Twiggy or Sony mode
static TrackParams* getTrackParams(uint32_t drive) {
    // In Twiggy mode, return the appropriate trackParams for the given drive, but in Sony mode, just return the one single trackParams struct that we have
    return twiggyMode() ? twiggyUiTrackParams[drive] : sonyUiTrackParams;
}

// This is the same thing as getTrackParams, but for the bufferStatus struct instead of trackParams
static BufferStatus* getBufferStatus() {
    return twiggyMode() ? twiggyUiBufferStatus : sonyUiBufferStatus;
}

uint32_t welcomeStartTime = 0; // The time at which we entered the welcome screen, so we can tell how long we've been on it
uint32_t lastInteractionTime = 0; // The last time there was any user or disk interaction; used to determine when to dim the display

// This function gets called when we first enter the welcome screen
void welcomeEnter() {
    // When we enter the welcome screen, we need to retrieve the system settings saved to NVS
    initConfig();
    configSettings = readConfig();
    closeConfig();
    // Set up a few things based on these settings
    OLED.setContrast(configSettings.brightness); // Set the OLED brightness to the saved value
    // And set the system type for both image metadata structs to the proper system type based on the emulMode
    if (configSettings.emulMode == ModeSonyLisa || configSettings.emulMode == ModeTwiggy) {
        // If we're in either of the Lisa modes, then the system type is Lisa
        diskMetadataPointers[0]->systemType = SystemLisa;
        diskMetadataPointers[1]->systemType = SystemLisa;
    } else {
        // But if we're in the Mac mode, then the system type is Mac (obviously)
        diskMetadataPointers[0]->systemType = SystemMac;
        diskMetadataPointers[1]->systemType = SystemMac;
    }
    // Note that we don't do anything with driveType here; it gets auto-loaded from the disk image metadata whenever a disk is inserted
    // So the entire point of our driveType setting is to prevent the UI from allowing the user to load a disk that doesn't match the desired type
    welcomeStartTime = millis(); // Record the time we entered the welcome screen
    // And then mark the whole screen as dirty since it's about to get redrawn with the welcome screen
    redrawWholeScreen();
}

// This function gets called periodically while we're on the welcome screen
void welcomeTick(uint32_t currentTime) {
    // The only thing we need to do here is figure out how long we've been on the welcome screen and leave if it's been long enough
    if (currentTime - welcomeStartTime >= 5000) {
        // If it's been more than 5 seconds, then we need to get out of here and go to the status screen
        // If the user wanted to go to the settings screen, they would've pressed SEL and welcomeButtonPress would've handled it
        // Clear out the screen stack so that we don't accidentally go back to the welcome screen once in operational mode
        screenStackIndex = 0;
        screenStack[0] = nullptr;
        currentScreen = &statusScreen;
        currentScreen->enter();
    }

}

// This function gets called when a button is pressed while we're on the welcome screen
void welcomeButtonPress(bool buttonStates[3]) {
    // The only button that matters on the welcome screen is SEL; if it's pressed, then we need to head to the settings screen
    if (buttonStates[1]) {
        pushScreen(&settingsScreen);
    }
}

// This function gets called to draw the welcome screen on the OLED
// The screen is in the following format:
/*
     Welcome to ESFloppy!
Emulation Mode: Lisa 400K/800K
    |---------------|
    |     Icon      |
    |_______________|
   SEL: Change settings...
*/
void welcomeDrawScreen() {
    char buffer[40]; // Use this temp buffer to build some of the lines of text that we need to draw
    OLED.clearBuffer(); // Clear the OLED's framebuffer before we draw anything
    // Start by drawing the welcome message at the top of the screen
    snprintf(buffer, sizeof(buffer), "Welcome to ESFloppy!");
    OLED.drawStr(((128 - OLED.getStrWidth(buffer)) / 2), 0, buffer); // Make sure to center it horizontally too
    // Next, build the emulation mode string based on the emulMode setting
    // For 400K/800K we want "Lisa 400K/800K" or "Mac 400K/800K", but for Twiggy just say "Twiggy" regardless of system type
    if (configSettings.emulMode == ModeTwiggy) {
        snprintf(buffer, sizeof(buffer), "Mode: Twiggy");
    } else {
        snprintf(buffer, sizeof(buffer), "Mode: %s 400K/800K", (configSettings.emulMode == ModeSonyLisa) ? "Lisa" : "Mac");
    }
    OLED.drawStr(((128 - OLED.getStrWidth(buffer)) / 2), (2 + (MENU_ITEM_HEIGHT * 1)), buffer); // And draw it once it's built, making sure to center it too
    // Now draw the appropriate system icon in the middle of the screen based on the current emulation mode
    if (configSettings.emulMode == ModeTwiggy) {
        // For Twiggy, draw a Lisa 1 icon
        drawLisa1Icon(((128 - LISA1_ICON_WIDTH) / 2), (((MENU_ITEM_HEIGHT * 7) - (2 + (MENU_ITEM_HEIGHT * 2)) - LISA1_ICON_HEIGHT) / 2) + (2 + (MENU_ITEM_HEIGHT * 2)));
    } else if (configSettings.emulMode == ModeSonyLisa) {
        // For Lisa 400K/800K, draw a Lisa 2 icon
        drawLisa2Icon(((128 - LISA2_ICON_WIDTH) / 2), (((MENU_ITEM_HEIGHT * 7) - (2 + (MENU_ITEM_HEIGHT * 2)) - LISA2_ICON_HEIGHT) / 2) + (2 + (MENU_ITEM_HEIGHT * 2)));
    } else {
        // For Mac 400K/800K, draw a Happy Mac icon
        drawHappyMacIcon(((128 - HAPPYMAC_ICON_WIDTH) / 2), (((MENU_ITEM_HEIGHT * 7) - (2 + (MENU_ITEM_HEIGHT * 2)) - HAPPYMAC_ICON_HEIGHT) / 2) + (2 + (MENU_ITEM_HEIGHT * 2)));
    }
    // And finally, put the settings prompt below all of that, ensuring it's horizontally centered as well
    snprintf(buffer, sizeof(buffer), "SEL: Open Settings...");
    OLED.drawStr(((128 - OLED.getStrWidth(buffer)) / 2), (MENU_ITEM_HEIGHT * 7), buffer);
}


// These are all of the things that the status screen remembers in between ticks so that ti can tell when something has changed and needs to be redrawn
extern bool bufferWasDirty[2]; // Whether the buffer was dirty before the SD task executed its command; this is written by the SD task itself
static char imageFilename[2][256]; // The filename of each disk image
static bool diskInserted[2]; // Whether or not each disk was inserted last time we checked
static int32_t lastTrack[2]; // The track number that each drive was on
static bool lastSide[2]; // The side of the disk that each drive was on
static bool lastMotorState[2]; // Each drive's motor state
static uint32_t writeIndicatorStopTime[2]; // The time at which the write indicator for each drive should stop being displayed
static bool lastWriteIndicator[2]; // The last state of the write indicator for each drive, so we can tell when it changes and needs to be redrawn
static uint32_t scrollOffset[2]; // How far in characters each drive's filename has scrolled so far
static uint32_t scrollTime[2]; // When the last scroll step happened for each drive's filename
uint32_t selectedDrive = 1; // Which drive is currently selected for the user to interact with; not relevant for Sony mode
static uint32_t selHoldTime = 0; // How long SEL has been held down for; used to check for long presses
static bool selPressed = false; // Whether the SEL button is currently held down to begin with
static bool selActedUpon = false; // Whether or not we've already acted upon a SEL button press yet
static uint32_t selTargetDrive = 0; // The drive that's being targeted by the current SEL press

// This function refreshes the filename of the disk image that's inserted into the given drive
// This actually accesses the filesystem, so it can be slow and should only be called when the image has changed
static void refreshImageFilename(uint32_t drive) {
    imageFilename[drive][0] = '\0';
    // Only proceed if the disk is actually inserted and the file is open
    if (diskMetadataPointers[drive]->diskInserted && disk[drive].isOpen()) {
        // If the disk is inserted and the file is open, then copy its name into the appropriate imageFilename buffer
        if (disk[drive].getName(imageFilename[drive], sizeof(imageFilename[drive])) == 0) {
            // If the copy fails for some reason, then put a placeholder name in there so that we don't end up with a blank string
            strncpy(imageFilename[drive], "ERROR: Failed to get image name!", sizeof(imageFilename[drive]) - 1);
            imageFilename[drive][sizeof(imageFilename[drive]) - 1] = '\0'; // Don't forget to null-terminate!
        }
    }
    scrollOffset[drive] = 0; // We just stuck a new filename into the buffer, so reset the scroll offset back to the start
    scrollTime[drive] = millis(); // And same goes for the scroll timer
}

// Builds the disk type string for a given drive into the provided buffer of the given length
static void buildTypeString(uint32_t drive, char* outString, uint32_t length) {
    DiskImageMetadata* metadata = diskMetadataPointers[drive]; // Start by getting the metadata for this drive
    // Now build the string based on the drive type, image type, and whether tags are present
    const char* geometryString = ""; // First we need to figure out the geometry string based on the drive type
    // This is 400K, 800K, or Twiggy
    if (metadata->driveType == Drive400) {
        geometryString = "400K";
    } else if (metadata->driveType == Drive800) {
        geometryString = "800K";
    } else if (metadata->driveType == DriveTwiggy) {
        geometryString = "Twiggy";
    }
    // Then use an snprintf to combine this with the image type and whether tags are present into the final string
    // As an example, a 400K DC42 with tags would display as "400K DC42 Image + Tags"
    snprintf(outString, length, "%s %s%s", geometryString,
             (metadata->imageType == DC42) ? "DC42" : "Raw",
             metadata->tagsPresent ? "+Tags" : "");
}

// Draws a Twiggy drive's status using half the screen (32 pixels)
// y is the coordinate of the top of the block
// The format of the screen is:
/*
|------| 0 LOS 1.0.dc42
| Icon | Twiggy DC42+Tags
|______| Trk 00 S0 Idle
*/
static void drawDriveCompact(uint32_t drive, uint32_t y, uint32_t currentTime) {
    char buffer[40]; // Use this temp buffer to build some of the lines of text that we need to draw
    DiskImageMetadata* metadata = diskMetadataPointers[drive]; // Get the metadata for this drive's image
    TrackParams* trackParams = getTrackParams(drive); // And its trackParams

    // Start by drawing the Twiggy drive icon itself
    // Draw it in inverse video if the drive is selected for user interaction, otherwise draw it normally
    if (selectedDrive == drive) {
        drawTwiggyIcon(1, y + 4, true);
    } else {
        drawTwiggyIcon(1, y + 4, false);
    }

    // Now draw the drive number badge in the top-left corner next to the filename; it's drawn in inverse video for readability
    snprintf(buffer, sizeof(buffer), "%d", (int)drive);
    uint32_t badgeWidth = drawInvertedString(28, y, buffer, false); // Draw the inverse badge and get its width
    uint32_t nameStartX = 28 + badgeWidth + 4; // And then figure out where the filename should start based on that width

    // Everything else changes depending on whether or not a disk is inserted
    if (metadata->diskInserted) {
        // If a disk is inserted, then start by drawing the filename of the image
        // Remember that we want to scroll it if it's too long to fit, so use drawClippedText for that
        drawClippedText(nameStartX, y, 128 - nameStartX, imageFilename[drive], scrollOffset[drive]);
        // Next build and draw the image type string (the "Twiggy DC42+Tags" line)
        buildTypeString(drive, buffer, sizeof(buffer));
        OLED.drawStr(28, y + 2 + (MENU_ITEM_HEIGHT * 1), buffer);
        // A Twiggy's two heads are on opposite sides of the spindle, so one carriage position means two different
        // track numbers: side 0 is at 45 - T and side 1 is at T. Showing just one of them would not match what the Lisa thinks
        // Now draw the track/side/status line, which is a bit more complex than what we'd do for Sony
        // On Twiggy, the track number differs depending on which side we're on; it's just currentTrack for side 1, but 45-currentTrack for side 0
        // And there's also a currentTrack = -1 case for the calibration track, which we also need to handle
        // Start by building the track portion of the string
        char trackString[8];
        if (trackParams->currentTrack < 0) {
            // If we're on the calibration track, then just display "Trk CAL" instead of a number, regardless of which side we're on
            snprintf(trackString, sizeof(trackString), "Trk CAL");
        } else {
            // Otherwise, display the track number based on the side
            snprintf(trackString, sizeof(trackString), "Trk %02d ", ((trackParams->side == 0) ? (int)(45 - trackParams->currentTrack) : (int)trackParams->currentTrack));
        }
        // Now do the side portion of the string, which is just "S0" or "S1"
        char sideString[3];
        snprintf(sideString, sizeof(sideString), "S%d", (int)trackParams->side);
        // And finally, the status portion of the string, which is "Idle", "Read", or "Write" depending on the motor state and whether the buffer was dirty before the SD task executed its most recent command
        char statusString[6];
        if (!trackParams->motorOn) {
            // If the motor is off, then the drive is idle
            snprintf(statusString, sizeof(statusString), "Idle ");
        } else if (currentTime < writeIndicatorStopTime[drive]) {
            // If the motor is on and the write indicator set by the status tick function is still active, then the drive is writing
            snprintf(statusString, sizeof(statusString), "Write");
        } else {
            // If the motor is on but the write indicator is inactive, then the drive is reading
            snprintf(statusString, sizeof(statusString), "Read ");
        }

        // Now combine all of those into the final line and draw it
        snprintf(buffer, sizeof(buffer), "%s %s %s", trackString, sideString, statusString);
        OLED.drawStr(28, y + 4 + (MENU_ITEM_HEIGHT * 2), buffer);
    } else {
        // If a disk is NOT inserted, then just display a message telling the user to insert one
        OLED.drawStr(nameStartX, y, "No disk!");
        snprintf(buffer, sizeof(buffer), "Pick drive %d and", (int)drive);
        OLED.drawStr(28, y + 2 + (MENU_ITEM_HEIGHT * 1), buffer);
        OLED.drawStr(28, y + 4 + (MENU_ITEM_HEIGHT * 2), "hit SEL to load...");
    }
}

// Draws a Sony drive's status using the whole screen
// y is the coordinate of the top of the block, which should pretty much always be 0 here
// The format of the screen is:
/*
LOS 3.0 Disk 1.image
Non-Macintosh Disk
400K DC42 + Tags
|-----| Idle
|Icon | Track 00 Side 0
|_____| Buffer dirty!
SEL: Force Eject
*/
static void drawDriveExpanded(uint32_t drive, uint32_t y, uint32_t currentTime) {
    char lineBuffer[63]; // This is the temp buffer that we'll use to build the various lines of text that we need to draw
    DiskImageMetadata* metadata = diskMetadataPointers[drive]; // Get the metadata for this drive so we can figure out what to draw
    TrackParams* trackParams = getTrackParams(drive); // And also get its trackParams

    // First up, draw the Sony drive icon in the bottom half of the screen on the left side
    // Center it vertically between the third and 7th lines of text
    drawSonyIcon(3, y + 4 + (MENU_ITEM_HEIGHT * 3), false);

    // If a disk isn't inserted, then just fill the top two lines of the screen with a message telling the user to insert one
    if (!metadata->diskInserted) {
        //OLED.drawFrame(3, y + 6, 22, 24);
        OLED.drawStr(0, y, "No disk inserted!");
        OLED.drawStr(0, y + 1 + (MENU_ITEM_HEIGHT * 1), "Hit SEL to load one...");
        return; // And that's about it
    }

    // But if a disk is inserted, then we have some more fun stuff to print
    // Start by printing the filename of the disk image, which scrolls if it's too long to fit on the screen
    drawClippedText(0, y, 128, imageFilename[drive], scrollOffset[drive]);

    // Next, print the image's volume name if it's a DC42, or just a blank line if not
    // The volume name is a Pascal string, not a C string, so copy it to our buffer and null-terminate it before we mess with it
    // If the name is "-not a Macintosh disk-", then print "Non-Macintosh Disk" instead, since that's a bit more user-friendly
    if (metadata->imageType == DC42) {
        // Start by copying the volume name into our string buffer
        uint32_t nameLength = metadata->header.nameLength;
        if (nameLength > sizeof(lineBuffer) - 1) {
            // Make sure we don't overflow our buffer by capping the name length to the buffer length
            nameLength = sizeof(lineBuffer) - 1;
        }
        for(uint32_t i = 0; i < nameLength; i++) {
            lineBuffer[i] = metadata->header.volumeName[i];
        }
        lineBuffer[nameLength] = '\0'; // Don't forget the null terminator

        // then do the "not a macintosh disk" check
        if (strcmp(lineBuffer, "-not a Macintosh disk-") == 0) {
            OLED.drawStr(0, y + 1 + (MENU_ITEM_HEIGHT * 1), "Non-Macintosh Disk");
        } else {
            OLED.drawStr(0, y + 1 + (MENU_ITEM_HEIGHT * 1), lineBuffer); // Otherwise, just draw the volume name as-is
        }
    } else {
        OLED.drawStr(0, y + 1 + (MENU_ITEM_HEIGHT * 1), ""); // Just draw a blank line if it's not a DC42
    }

    // Next up is the disk type string, which is built from the drive type, image type, and whether tags are present
    buildTypeString(drive, lineBuffer, sizeof(lineBuffer));
    OLED.drawStr(0, y + 2 + (MENU_ITEM_HEIGHT * 2), lineBuffer);

    // Now onto the bottom half of the screen; we need to print the main drive status, which is "Idle", "Read", or "Write" depending on what the drive is doing
    // This is based on the state of the motor and the bufferStatus struct
    if (!trackParams->motorOn) {
        // If the motor is off, then the drive is idle
        OLED.drawStr(32, y + 3 + (MENU_ITEM_HEIGHT * 3), "Idle");
    } else if (currentTime < writeIndicatorStopTime[drive]) {
        // If the motor is on and the write indicator set by the status tick function is still active, then the drive is writing
        OLED.drawStr(32, y + 3 + (MENU_ITEM_HEIGHT * 3), "Write");
    } else {
        // If the motor is on but the write indicator is inactive, then the drive is reading
        OLED.drawStr(32, y + 3 + (MENU_ITEM_HEIGHT * 3), "Read");
    }

    // Next up we need to print the current track and side numbers that we're accessing
    // Force the side number to 0 if the motor is off since HDS is meaningless (just a register address) when the motor is off
    snprintf(lineBuffer, sizeof(lineBuffer), "Track %02d  Side %d", (int)trackParams->currentTrack, ((trackParams->motorOn && diskMetadataPointers[drive]->driveType != Drive400) ? (int)trackParams->side : 0));
    OLED.drawStr(32, y + 4 + (MENU_ITEM_HEIGHT * 4), lineBuffer);

    // And then we need to print the buffer dirty indicator if the buffer is currently dirty, as a warning to the user that they shouldn't turn off the power
    if (getBufferStatus()->bufferDirty) {
        OLED.drawStr(32, y + 5 + (MENU_ITEM_HEIGHT * 5), "Buffer dirty!");
    } else {
        // If the buffer isn't dirty, then just leave the line blank
        OLED.drawStr(32, y + 5 + (MENU_ITEM_HEIGHT * 5), "");
    }

    // And last but not least, we need to print "SEL: Force Eject"
    OLED.drawStr(2, y + 6 + (MENU_ITEM_HEIGHT * 6), "SEL: Force Eject");
}

// This is the function that gets called when we first enter the status screen
void statusEnter() {
    // Re-read all of the info about the images/drives so that we can show the latest stuff on the screen
    for (uint32_t drive = 0; drive < 2; drive++) {
        diskInserted[drive] = diskMetadataPointers[drive]->diskInserted;
        refreshImageFilename(drive);
        lastTrack[drive] = getTrackParams(drive)->currentTrack;
        lastSide[drive] = getTrackParams(drive)->side;
        lastMotorState[drive] = getTrackParams(drive)->motorOn;
    }
    // Reset the SEL hold-detection state variables too so that we don't accidentally mis-detect a hold when entering this screen
    selHoldTime = 0;
    selPressed = false;
    selActedUpon = false;
    redrawWholeScreen(); // Mark the whole screen as dirty so that it gets fully redrawn next time the UI updates
}

// This is the function that gets called periodically while we're on the status screen
void statusTick(uint32_t currentTime) {
    uint32_t slots = getDriveCount(); // Start by figuring out how many drives we actually have

    // And then iterate through each of these slots
    for (uint32_t slot = 0; slot < slots; slot++) {
        // Figure out which drive goes into this slot on the screen
        // In Twiggy mode, drive 0 goes to slot 0 and drive 1 goes to slot 1
        // But in Sony mode, the single drive (drive 1) always goes into slot 0
        uint32_t drive = driveForScreenSlot(slot);
        // Also figure out where the top of this drive's block is on the screen so we can draw it in the right place
        uint32_t blockTop = twiggyMode() ? (slot * 33) : 0;

        // Now check to see if the diskInserted state has changed since the last time we were here
        if (diskMetadataPointers[drive]->diskInserted != diskInserted[drive]) {
            // If so, then update the diskInserted state
            diskInserted[drive] = diskMetadataPointers[drive]->diskInserted;
            refreshImageFilename(drive); // And get the new filename for the new image that's now inserted (or clear it if the disk was removed)
            redrawWholeScreen(); // A bunch of stuff in the status block changes when we swap disks, so mark the whole thing as dirty
        }

        // Next do most of the other stuff that can change while the disk is still inserted, like track number, motor state, and so on
        TrackParams* trackParams = getTrackParams(drive); // Start by getting the trackParams for this drive
        if (trackParams->currentTrack != lastTrack[drive] || trackParams->motorOn != lastMotorState[drive] || trackParams->side != lastSide[drive]) {
            // If any of the things that we use to synthesize the info on the screen have changed, then update our latched values to the current ones
            lastTrack[drive] = trackParams->currentTrack;
            lastSide[drive] = trackParams->side;
            lastMotorState[drive] = trackParams->motorOn;
            // And mark the region of the screen that contains the stuff that just changed as dirty
            // It's a different region depending on Sony vs Twiggy mode given that Sony takes up the full screen and Twiggy takes up half
            if (twiggyMode()) {
                redrawRegion(blockTop + 4 + (MENU_ITEM_HEIGHT * 2), blockTop + 4 + (MENU_ITEM_HEIGHT * 3));
            } else {
                redrawRegion(blockTop + 3 + (MENU_ITEM_HEIGHT * 3), blockTop + 6 + (MENU_ITEM_HEIGHT * 6));
            }
        }

        // Now handle the case where the image file has been written to, which is indicated by the buffer being dirty
        if (bufferWasDirty[drive]) {
            // If the buffer was dirty before the SD task executed its most recent command, then mark it as no longer dirty
            bufferWasDirty[drive] = false;
            // And then set the write indicator for this drive to be displayed for a short time so the user knows a write occurred
            writeIndicatorStopTime[drive] = currentTime + WRITE_INDICATOR_DURATION;
        }

        // Now determine when we need to redraw the write indicator 
        // First, figureout whether the write indicator should be on or off right now
        bool writeIndicatorOn = (currentTime < writeIndicatorStopTime[drive]);
        if (writeIndicatorOn != lastWriteIndicator[drive]) {
            // If the write indicator state has changed since the last time we were here, then update our latched value to the current one
            lastWriteIndicator[drive] = writeIndicatorOn;
            // And mark the region of the screen that contains the write indicator as dirty so it gets redrawn
            // The row differs between Twiggy and Sony
            if (twiggyMode()) {
                redrawRegion(blockTop + 4 + (MENU_ITEM_HEIGHT * 2), blockTop + 4 + (MENU_ITEM_HEIGHT * 3));
            } else {
                redrawRegion(blockTop + 3 + (MENU_ITEM_HEIGHT * 3), blockTop + 3 + (MENU_ITEM_HEIGHT * 4));
            }
        }

        // Finally, implement the scrolling of the filename if it's too long to fit on the screen
        if (diskInserted[drive]) {
            // Start by figuring out the x-coordinate where the filename starts
            // It's the start of the screen on Sony
            // But it's the Twiggy icon plus the drive number on Twiggy
            uint32_t nameStartX = twiggyMode() ? (28 + OLED.getStrWidth("0") + 6) : 0;
            // Now determine how wide a character is in our font
            uint32_t charWidth = OLED.getMaxCharWidth(); // The font is fixed-width, so this is the same for every character
            // Use that data to figure out how many characters can fit in the space alotted for the filename
            // The charWidth check probably isn't really necessary, but we don't want to accidentally divide by 0
            uint32_t charsThatFit = (charWidth > 0) ? ((128 - nameStartX) / charWidth) : 1;
            uint32_t nameLength = strlen(imageFilename[drive]); // And finally, get the actual length of the filename
            if (nameLength > charsThatFit) {
                // If the name is too long to fit in our space, then we've got a problem and need to scroll it
                // First, get the offset of the last character that DOES fit in the space we have
                uint32_t lastCharOffset = nameLength - charsThatFit;
                bool atEnd = (scrollOffset[drive] >= lastCharOffset); // And check if we're already at the end of the filename
                // Now figure out how long we should wait before we take the next scroll step
                // If we're at the beginning or end of the filename, then we want to pause for SCROLL_PAUSE ms
                // But if we're in the middle of scrolling it, then we want to step every SCROLL_STEP ms
                uint32_t interval = (scrollOffset[drive] == 0 || atEnd) ? SCROLL_PAUSE : SCROLL_STEP;
                if ((currentTime - scrollTime[drive]) >= interval) {
                    // If it's time to take a scroll step, then reset the scroll timer
                    scrollTime[drive] = currentTime;
                    // And increment the scroll offset, wrapping back to 0 if we reach the end of the filename
                    scrollOffset[drive] = atEnd ? 0 : (scrollOffset[drive] + 1);
                    // Now mark the region of the screen that contains the filename as dirty so it gets redrawn
                    redrawRegion(blockTop, blockTop + MENU_ITEM_HEIGHT);
                }
            }
        }
    }
}

// This is the function that handles button presses on the status screen
void statusButtonPress(bool buttonStates[3]) {
    // Handling button presses differs between Sony and Twiggy modes
    if (!twiggyMode()) {
        // If we're in Sony mode, then the only button that does anything is the SEL button
        // If a disk is inserted, then pressing SEL will force an eject of that disk
        selectedDrive = 1; // The only drive in Sony mode is drive 1, so make sure it's always the one selected for interaction
        if (buttonStates[1] && diskMetadataPointers[1]->diskInserted) {
            TrackParams* params = getTrackParams(1); // Get the trackParams for drive 1, which is the only drive in Sony mode
            params->ejectRequested = EjectForce; // Set the ejectRequested flag to EjectForce to force an eject
        }
        // If no disk is inserted, then pressing SEL will bring up the file picker so the user can select a disk to insert
        else if (buttonStates[1] && !diskMetadataPointers[1]->diskInserted) {
            // Reset the file picker to the root directory before we push it
            filePickerReset();
            pushScreen(&filePickerScreen);
        }
    } else {
        // In Twiggy mode, the user can select which drive they want to interact with using the left and right buttons
        if (buttonStates[0]) {
            // If the left button is pressed, then select the previous drive (wrapping around to the last one if we go off the end)
            selectedDrive = (selectedDrive == 0) ? (getDriveCount() - 1) : (selectedDrive - 1);
            redrawWholeScreen(); // The whole screen needs to be redrawn since the selected drive changed
        } else if (buttonStates[2]) {
            // If the right button is pressed, then select the next drive, also wrapping around if needed
            selectedDrive = (selectedDrive + 1) % getDriveCount();
            redrawWholeScreen(); // Force a full redraw here too
        }
        // The user can press SEL here too, in which case we should eject the disk if there's one in place
        // There are two types of ejects that can happen here: a normal eject if the user presses SEL once, or a force eject if they hold it for LONG_PRESS_DURATION ms
        if (buttonStates[1]) {
            // If the user presses SEL, then record the time that it went down and mark that SEL is now held down if there's a disk inserted
            if (diskMetadataPointers[selectedDrive]->diskInserted) {
                selHoldTime = millis();
                selPressed = true;
                selActedUpon = false; // And that we haven't taken any action on it yet
                // Also set the current drive as the target drive for the SEL press so that it doesn't change if the user presses LEFT/RIGHT while holding SEL
                selTargetDrive = selectedDrive;
            } else {
                // If there's no disk inserted, then just bring up the file picker so the user can select a disk to insert
                // Reset the file picker to the root directory before we push it
                filePickerReset();
                pushScreen(&filePickerScreen);
            }
        } else if (selPressed && getButtonHeld(1)) {
            // If selHeld says that SEL is held down and the debounced button state agrees, then check to see how long it's been down for
            if (!selActedUpon && (millis() - selHoldTime >= LONG_PRESS_DURATION)) {
                // If it's been held down for long enough and we haven't acted upon it yet, then force an eject of the disk in selTargetDrive
                TrackParams* params = getTrackParams(selTargetDrive);
                params->ejectRequested = EjectForce; // Set the ejectRequested flag to EjectForce to force an eject
                selActedUpon = true; // And mark that we've acted upon the press so we don't do it again
            }
        } else if (selPressed) {
            // Otherwise, if selPressed says that it's held, but it's not actually held down anymore, then we need to perform the short-press action
            // But only if we haven't already done the long-press action
            selPressed = false;
            if (!selActedUpon) {
                selActedUpon = true;
                // If the user just briefly pressed SEL, then do a normal eject instead of a force eject
                TrackParams* params = getTrackParams(selTargetDrive);
                params->ejectRequested = EjectNormal; // Set the ejectRequested flag to EjectNormal to request a normal eject
            }
        }
    }
}

// And this is the function that actually draws the status screen on the OLED
void statusDrawScreen() {
    OLED.clearBuffer(); // Start by clearing the framebuffer so we can draw a fresh new screen
    uint32_t currentTime = millis(); // Get the current time so we can pass it to the draw functions for the drives
    if (twiggyMode()) {
        // If we're in Twiggy mode, then we have two drives that each get a 32-pixel tall slot on the screen
        drawDriveCompact(0, 0, currentTime); // So draw the top one (drive 0, the upper Twiggy) first
        OLED.drawHLine(0, 31, 128); // Then a dividing line
        drawDriveCompact(1, 33, currentTime); // And then the bottom one (drive 1, the lower Twiggy) offset 32 pixels down from the top of the OLED
    } else {
        // If we're in Sony mode, then the single drive (drive 1) gets the privilege of using the entire screen
        drawDriveExpanded(1, 0, currentTime);
    }
}

bool displayDimmed = false; // Whether the display is currently dimmed or not

// This function gets called periodically to update the UI; it basically manages the screens and calls their appropriate functions
// It returns false if the UI is in the startup process (welcome or settings screen) and true otherwise
bool uiUpdate() {
    // If the current screen is null, then we need to set it to the welcome screen and call its enter function so that it can initialize itself
    if (currentScreen == nullptr) {
        currentScreen = &welcomeScreen;
        currentScreen->enter();
    }

    // Now get the current time
    uint32_t currentTime = millis();

    // And then get the current states of the buttons (accounting for auto-repeat) and store them in an array
    bool buttonStates[3] = {false, false, false};
    getButtonStates(buttonStates);

    // If any button is pressed, then record the time so we can determine when to dim the display
    // Also reset the lastInteractionTime if a drive motor comes on so that the display is bright when there's actual disk activity
    if (buttonStates[0] || buttonStates[1] || buttonStates[2] || getTrackParams(0)->motorOn || getTrackParams(1)->motorOn) {
        lastInteractionTime = currentTime;
    }

    // Now figure out whether or not we should dim the display
    // If it's been a while since the last button press and the user has enabled dimming, then we need to dim it
    bool shouldDim = configSettings.dimDisplay && (currentTime - lastInteractionTime >= FADE_DELAY);
    if (shouldDim != displayDimmed) {
        // If the current dimmed state is different than the desired dimmed state, then we need to change it
        // So either dim to 1/8 of the full brightness value or restore to the full brightness value
        // We do it this way so that we only run setContrast once; it does a whole I2C transaction on each call which is really inefficient
        OLED.setContrast(shouldDim ? (configSettings.brightness / 8) : configSettings.brightness);
        displayDimmed = shouldDim; // And then update our record of the current dimmed state
    }

    // Call the button press function for the current screen
    currentScreen->buttonPress(buttonStates);
    // As well as the tick function
    currentScreen->tick(currentTime);

    // Only redraw the framebuffer if something actually changed; no point in wasting precious clock cycles if there's nothing to update
    // Keep in mind that this is only updating the framebuffer in RAM, not sending anything out over I2C, but still it's better not to waste time
    if (redrawScreen) {
        currentScreen->drawScreen();
        redrawScreen = false;
    }

    // And finally, actually send the framebuffer out to the OLED over I2C if there's anything to actually send
    // This function only sends one 8-line page per call, so it only blocks for about 3ms instead of more like 24ms
    // If we sent the whole frame at once, the SD card task wouldn't be able to respond to the Lisa fast enough and everything would break
    sendOneOLEDPage();

    // This function needs to return true if the UI is in its normal operating state and false if it's still in the startup process
    // So return false if the currentScreen is the welcomeScreen or settingsScreen, and true otherwise
    if (currentScreen == &welcomeScreen || currentScreen == &settingsScreen) {
        return false;
    } else {
        return true;
    }
}
