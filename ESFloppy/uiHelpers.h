#pragma once

// Header file for all of our UI helper functions

#include <Arduino.h>
#include <U8g2lib.h>

// This is the OLED object for our display; it's actually defined in SDTask, but we need to reference it here so that we can use it in all our UI stuff
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED;

// These are the heights and widths of the various icons that we draw on the OLED
#define SONY_ICON_WIDTH 22
#define SONY_ICON_HEIGHT 24
#define TWIGGY_ICON_WIDTH 24
#define TWIGGY_ICON_HEIGHT 24
#define LISA1_ICON_WIDTH 40
#define LISA1_ICON_HEIGHT 24
#define LISA2_ICON_WIDTH 40
#define LISA2_ICON_HEIGHT 24
#define HAPPYMAC_ICON_WIDTH 25
#define HAPPYMAC_ICON_HEIGHT 32

// Reads the current state of the buttons and returns them in the buttonStates array
// This also accounts for auto-repeat so that if a button is held down, it will repeatedly return true for that button after a certain period
void getButtonStates(bool buttonStates[3]);

// This function draws the Sony floppy icon on the OLED at the specified x/y coordinates and optionally in inverse video
void drawSonyIcon(uint32_t x, uint32_t y, bool inverseVideo);

// And this one draws the Twiggy icon at the specified coordinates and optionally in inverse video
void drawTwiggyIcon(uint32_t x, uint32_t y, bool inverseVideo);

// Another one to draw a Lisa 1 icon, the same one that the Lisa itself uses as the Preferences icon
void drawLisa1Icon(uint32_t x, uint32_t y);

// As well as a Lisa 2 icon, also the Lisa Preferences one
void drawLisa2Icon(uint32_t x, uint32_t y);

// And finally, a Happy Mac icon
void drawHappyMacIcon(uint32_t x, uint32_t y);

// This function is really simple; it just marks every single page as dirty to force the entire display to be resent
void markAllPagesDirty();

// This function marks just the pages from rows yStart through yEnd (inclusive) as dirty
// Each page only takes about 3ms to draw versus about 24ms for the whole display, so this saves an absurd amount of time if you're just redrawing a page or two
void markRegionDirty(uint32_t yStart, uint32_t yEnd);

// Sends a page of data to the OLED over I2C if there are any pages marked as dirty; returns true if it actually sent something and false if not
// Note that this only sends one page per call even if multiple are dirty so that it doesn't block the SD card task for more than about 3ms per call
bool sendOneOLEDPage();

// Draws a text string at (x, y) clipped to a window of windowWidth pixels
// The draw will start charOffset characters into the string, allowing for easy horizontal scrolling effects
// Returns the full (unclipped) width of the string so the caller can tell whether it needs to scroll to fit it within the window
uint32_t drawClippedText(uint32_t x, uint32_t y, uint32_t windowWidth, const char* text, uint32_t charOffset);

// Draws a string in inverse video at (x, y) and returns the total width of the box
// If fullWidth is true, then the box will be the full width of the display, regardless of the string width
uint32_t drawInvertedString(uint32_t x, uint32_t y, const char* text, bool fullWidth);

// Draws a menu row at the specified y coordinate, in inverse video if it's selected
// Just like drawClippedText, this also supports a charOffset for horizontal scrolling of the text and returns the full string width
uint32_t drawMenuRow(uint32_t y, const char* text, uint32_t charOffset, bool selected);