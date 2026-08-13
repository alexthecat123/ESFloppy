#pragma once

// Header file for the ESFloppy OLED user interface code

#include <Arduino.h>
#include <SdFat.h>
#include <U8g2lib.h>
#include "types.h"
#include "ui.h"
#include "uiHelpers.h"
#include "uiState.h"

// All of the source code for ESFloppy's OLED-based UI

// Marks the whole screen as dirty and needing to be redrawn/sent to the OLED
void redrawWholeScreen();

// Marks the screen as needing to be redrawn, but only sends the region covering rows yStart through yEnd to the OLED
// This is much more efficient than redrawing the whole screen, and avoids blocking the SD task for the time of a full frame
void redrawRegion(uint32_t yStart, uint32_t yEnd);

// The error screen that can be pushed onto the screen stack to show an error message to the user
// Anything that wants to use it just fills in the errorMessageLines array with the text to show and then calls pushScreen
extern Screen errorScreen;

// This function pushes a new screen onto the screen stack and makes it the current screen
void pushScreen(Screen* newScreen);

// This function pops the current screen off the screen stack and makes the previous screen the current screen
void popScreen();

// Returns true if we're emulating Twiggies rather than a Sony; I'm getting really tired of typing metadata->driveType == DriveTwiggy all over the place
static bool twiggyMode();

// Returns how many drives the current emulation mode actually has; Twiggy has two, Sony has one
static uint32_t getDriveCount();

// This function returns the drive index that corresponds to a given display slot on the status screen
// In Twiggy mode, drive 0 goes to slot 0 (the top slot) and drive 1 goes to slot 1 (the bottom slot)
// But in Sony mode, the single drive (drive 1) always goes into slot 0 and the bottom slot is unused
static uint32_t driveForScreenSlot(uint32_t slot);

// Returns the proper trackParams struct for a given drive, depending on whether we're in Twiggy or Sony mode
static TrackParams* getTrackParams(uint32_t drive);

// This is the same thing as getTrackParams, but for the bufferStatus struct instead of trackParams
static BufferStatus* getBufferStatus();

// This is the function that gets called when we first enter the welcome screen
void welcomeEnter();

// This is the function that gets called periodically while we're on the welcome screen
void welcomeTick(uint32_t currentTime);

// This is the function that handles button presses on the welcome screen
void welcomeButtonPress(bool buttonStates[3]);

// And this is the function that actually draws the welcome screen on the OLED
void welcomeDrawScreen();

// This is the function that gets called when we first enter the status screen
void statusEnter();

// This is the function that gets called periodically while we're on the status screen
void statusTick(uint32_t currentTime);

// This is the function that handles button presses on the status screen
void statusButtonPress(bool buttonStates[3]);

// And this is the function that actually draws the status screen on the OLED
void statusDrawScreen();

// This function refreshes the filename of the disk image that's inserted into the given drive
// This actually accesses the filesystem, so it can be slow and should only be called when the image has changed
static void refreshImageFilename(uint32_t drive);

// Builds the little "800K DC42+tags" description line for a drive
static void buildTypeString(uint32_t drive, char* out, uint32_t length);

// Draws one drive's status in a compact 32-pixel-tall block; this is what Twiggy mode uses for each of its two drives
static void drawDriveCompact(uint32_t drive, uint32_t y, uint32_t currentTime);

// Draws one drive's status using the whole screen; this is what Sony mode uses, since it only has the one drive
static void drawDriveExpanded(uint32_t drive, uint32_t y, uint32_t currentTime);

// This is the function that gets called when we first enter the status screen
void statusEnter();

// This is the function that gets called periodically while we're on the status screen
void statusTick(uint32_t currentTime);

// This is the function that handles button presses on the status screen
void statusButtonPress(bool buttonStates[3]);

// And this is the function that actually draws the status screen on the OLED
void statusDrawScreen();

// This function gets called periodically to update the UI; it basically manages the screens and calls their appropriate functions
// It returns false if the UI is in the startup process (welcome or settings screen) and true otherwise
bool uiUpdate();