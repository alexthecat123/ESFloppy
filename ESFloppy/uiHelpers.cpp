#include <Arduino.h>
#include "types.h"
#include "uiHelpers.h"
#include "uiState.h"

// A bunch of helper functions used in our ESFloppy UI code

#define BUTTON_REPEAT_DELAY 250 // The delay (in ms) before a button starts auto-repeating when held down
#define BUTTON_REPEAT_INTERVAL 50 // The interval (in ms) between auto-repeats once it's started repeating
#define BUTTON_DEBOUNCE 20 // The debounce time (in ms) for the buttons

// This array is the icon for a 3.5" Sony floppy disk
// I suck at drawing, so I had Claude generate a preliminary version that I then refined myself
static const uint8_t sonyIconBits[] = {
    0xFF, 0xFF, 0x0F,
    0xE1, 0xFF, 0x11,
    0xE1, 0x8F, 0x21,
    0xE1, 0x8F, 0x21,
    0xE1, 0x8F, 0x21,
    0xE1, 0x8F, 0x21,
    0xE1, 0x8F, 0x21,
    0xE1, 0x8F, 0x21,
    0xE1, 0xFF, 0x21,
    0x01, 0x00, 0x20,
    0x01, 0x00, 0x20,
    0x01, 0x00, 0x20,
    0xFD, 0xFF, 0x2F,
    0x05, 0x00, 0x28,
    0x05, 0x00, 0x28,
    0xF5, 0xFF, 0x2B,
    0x05, 0x00, 0x28,
    0x05, 0x00, 0x28,
    0xF5, 0xFF, 0x2B,
    0x05, 0x00, 0x28,
    0x05, 0x00, 0x28,
    0xF5, 0xFF, 0x2B,
    0x05, 0x00, 0x28,
    0xFF, 0xFF, 0x3F,
};

// Here's the Sony icon again, but in inverse video
static const uint8_t sonyIconBitsInvid[] = {
    0x00, 0x00, 0x00,
    0x1E, 0x00, 0x0E,
    0x1E, 0x70, 0x1E,
    0x1E, 0x70, 0x1E,
    0x1E, 0x70, 0x1E,
    0x1E, 0x70, 0x1E,
    0x1E, 0x70, 0x1E,
    0x1E, 0x70, 0x1E,
    0x1E, 0x00, 0x1E,
    0xFE, 0xFF, 0x1F,
    0xFE, 0xFF, 0x1F,
    0xFE, 0xFF, 0x1F,
    0x02, 0x00, 0x10,
    0xFA, 0xFF, 0x17,
    0xFA, 0xFF, 0x17,
    0x0A, 0x00, 0x14,
    0xFA, 0xFF, 0x17,
    0xFA, 0xFF, 0x17,
    0x0A, 0x00, 0x14,
    0xFA, 0xFF, 0x17,
    0xFA, 0xFF, 0x17,
    0x0A, 0x00, 0x14,
    0xFA, 0xFF, 0x17,
    0x00, 0x00, 0x00,
};

// And here's the icon for a Twiggy disk
static const uint8_t twiggyIconBits[] = {
    0xFF, 0xFF, 0x3F,
    0x09, 0x00, 0x20,
    0x01, 0x08, 0xE6,
    0x01, 0x14, 0x80,
    0x01, 0x14, 0x80,
    0x01, 0x14, 0x80,
    0x01, 0x08, 0xBE,
    0x01, 0x00, 0xA2,
    0x01, 0x00, 0xA2,
    0x01, 0x1C, 0xBE,
    0x01, 0x22, 0xA2,
    0x01, 0x49, 0xA2,
    0x01, 0x5D, 0xBE,
    0x01, 0x49, 0xA2,
    0x01, 0x22, 0xA2,
    0x01, 0x1C, 0xBE,
    0x01, 0x00, 0xA2,
    0x01, 0x08, 0xA2,
    0x01, 0x14, 0xBE,
    0x01, 0x14, 0xA2,
    0x01, 0x14, 0xA2,
    0x01, 0x08, 0xBE,
    0x01, 0x00, 0xA2,
    0xFF, 0xFF, 0xFF,
};

// Plus Twiggy in inverse video
static const uint8_t twiggyIconBitsInvid[] = {
    0xF7, 0xFF, 0x3F,
    0xF7, 0xFF, 0x3F,
    0xFF, 0xF7, 0xF9,
    0xFF, 0xEB, 0xFF,
    0xFF, 0xEB, 0xFF,
    0xFF, 0xEB, 0xFF,
    0xFF, 0xF7, 0xC1,
    0xFF, 0xFF, 0xC1,
    0xFF, 0xFF, 0xC1,
    0xFF, 0xE3, 0xDD,
    0xFF, 0xDD, 0xC1,
    0xFF, 0xB6, 0xC1,
    0xFF, 0xA2, 0xDD,
    0xFF, 0xB6, 0xC1,
    0xFF, 0xDD, 0xC1,
    0xFF, 0xE3, 0xDD,
    0xFF, 0xFF, 0xC1,
    0xFF, 0xF7, 0xC1,
    0xFF, 0xEB, 0xDD,
    0xFF, 0xEB, 0xC1,
    0xFF, 0xEB, 0xC1,
    0xFF, 0xF7, 0xDD,
    0xFF, 0xFF, 0xC1,
    0xFF, 0xFF, 0xFF,
};

// Here's the Lisa 1 icon, ripped straight from LOS
static const uint8_t lisa1IconBits[] = {
    0xFC, 0xFF, 0xFF, 0xFF, 0x3F,
    0x02, 0x00, 0x00, 0x00, 0x40,
    0xF1, 0xFF, 0xFF, 0x00, 0x80,
    0x09, 0x00, 0x00, 0x01, 0x9E,
    0x09, 0x00, 0x00, 0xF9, 0x91,
    0x09, 0x00, 0x00, 0x01, 0x9E,
    0x09, 0x00, 0x00, 0x01, 0x80,
    0x09, 0x00, 0x00, 0x01, 0x80,
    0x09, 0x00, 0x00, 0x01, 0x80,
    0x09, 0x00, 0x00, 0x01, 0x9E,
    0x09, 0x00, 0x00, 0xF9, 0x91,
    0x09, 0x00, 0x00, 0x01, 0x9E,
    0xF1, 0xFF, 0xFF, 0x00, 0x80,
    0x02, 0x00, 0x00, 0x00, 0x40,
    0xFC, 0xFF, 0xFF, 0xFF, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0xFC, 0xFF, 0xFF, 0xFF, 0x7F,
    0x02, 0x00, 0x00, 0x00, 0x80,
    0x52, 0x55, 0x55, 0x95, 0x8A,
    0xA2, 0xAA, 0xAA, 0x0A, 0x85,
    0x02, 0x55, 0x55, 0x81, 0x8A,
    0x02, 0x00, 0x00, 0x00, 0x80,
    0x02, 0x00, 0x00, 0x00, 0x80,
    0xFC, 0xFF, 0xFF, 0xFF, 0x7F,
};

// And the Lisa 2, also from LOS
static const uint8_t lisa2IconBits[] = {
    0xFC, 0xFF, 0xFF, 0xFF, 0x3F,
    0x02, 0x00, 0x00, 0x00, 0x40,
    0xF1, 0xFF, 0xFF, 0xFC, 0x9F,
    0x09, 0x00, 0x00, 0x01, 0x80,
    0x09, 0x00, 0x00, 0xFD, 0x9F,
    0x09, 0x00, 0x00, 0x01, 0x80,
    0x09, 0x00, 0x00, 0xFD, 0x9F,
    0x09, 0x00, 0x00, 0x01, 0x80,
    0x09, 0x00, 0x00, 0x01, 0x80,
    0x09, 0x00, 0x00, 0x01, 0x9C,
    0x09, 0x00, 0x00, 0xE1, 0x93,
    0x09, 0x00, 0x00, 0x01, 0x9C,
    0xF1, 0xFF, 0xFF, 0x00, 0x80,
    0x02, 0x00, 0x00, 0x00, 0x40,
    0xFC, 0xFF, 0xFF, 0xFF, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0xFC, 0xFF, 0xFF, 0xFF, 0x7F,
    0x02, 0x00, 0x00, 0x00, 0x80,
    0x52, 0x55, 0x55, 0x95, 0x8A,
    0xA2, 0xAA, 0xAA, 0x0A, 0x85,
    0x02, 0x55, 0x55, 0x81, 0x8A,
    0x02, 0x00, 0x00, 0x00, 0x80,
    0x02, 0x00, 0x00, 0x00, 0x80,
    0xFC, 0xFF, 0xFF, 0xFF, 0x7F,
};

// And finally the Happy Mac
static const uint8_t happyMacIconBits[] = {
    0xFC, 0xFF, 0x7F, 0x00,
    0x02, 0x00, 0x80, 0x00,
    0x01, 0x00, 0x00, 0x01,
    0xF1, 0xFF, 0x1F, 0x01,
    0x09, 0x00, 0x20, 0x01,
    0x09, 0x00, 0x20, 0x01,
    0x09, 0x00, 0x20, 0x01,
    0x09, 0x11, 0x21, 0x01,
    0x09, 0x11, 0x21, 0x01,
    0x09, 0x10, 0x20, 0x01,
    0x09, 0x10, 0x20, 0x01,
    0x09, 0x18, 0x20, 0x01,
    0x09, 0x00, 0x20, 0x01,
    0x09, 0x42, 0x20, 0x01,
    0x09, 0x3C, 0x20, 0x01,
    0x09, 0x00, 0x20, 0x01,
    0x09, 0x00, 0x20, 0x01,
    0xF1, 0xFF, 0x1F, 0x01,
    0x01, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01,
    0x19, 0x80, 0x1F, 0x01,
    0x01, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01,
    0xFE, 0xFF, 0xFF, 0x00,
    0x02, 0x00, 0x80, 0x00,
    0x02, 0x00, 0x80, 0x00,
    0x02, 0x00, 0x80, 0x00,
    0xFE, 0xFF, 0xFF, 0x00,
};

// Reads the current state of the buttons and returns them in the buttonStates array
// This also accounts for auto-repeat so that if a button is held down, it will repeatedly return true for that button after a certain period
void getButtonStates(bool buttonStates[3]) {
    static uint32_t lastEventTime[3] = {0, 0, 0}; // The last time (in ms) that each button fired a press event
    static uint32_t lastChangeTime[3] = {0, 0, 0}; // The last time (in ms) that each button's raw state changed; used for debouncing
    static bool buttonHeld[3] = {false, false, false}; // Whether each button is currently being held down, after accounting for debounce
    static bool rawPreviousStates[3] = {false, false, false}; // The raw (pre-debounce) state from the previous call
    static bool repeating[3] = {false, false, false}; // Whether each button has already started auto-repeating
    static bool repeatAllowed[3] = {true, false, true}; // Whether we want to even allow auto-repeat for each button to begin with; yet for all but SEL
    bool currentStates[3]; // The current states of the buttons

    // First, get the current state of each button; do this with a single REG_READ to save time
    uint32_t gpio1Data = REG_READ(GPIO_IN1_REG);
    // Keep in mind that all buttons are active-low
    currentStates[0] = !(gpio1Data & (1 << (LEFT - 32)));
    currentStates[1] = !(gpio1Data & (1 << (SEL - 32)));
    currentStates[2] = !(gpio1Data & (1 << (RIGHT - 32)));

    uint32_t currentTime = millis();

    // Now that we have the current states, loop through each button and determine whether it should be considered pressed or not
    for (int i = 0; i < 3; i++) {
        buttonStates[i] = false; // Assume they're all unpressed until we determine otherwise

        // Start with debouncing all of the buttons; if the raw state has changed, then it's not stable yet and we shouldn't trust it
        if (currentStates[i] != rawPreviousStates[i]) {
            rawPreviousStates[i] = currentStates[i]; // So update the previous state to the current one
            lastChangeTime[i] = currentTime; // And start the debounce timer for this button
            continue;
        }
        // If the button state hasn't just changed, but also hasn't been stable for long enough, then also don't trust it yet
        if ((currentTime - lastChangeTime[i]) < BUTTON_DEBOUNCE) {
            continue; // So just move onto the next button and keep this one in the default unpressed state
        }

        if (currentStates[i]) {
            // We end up here if the button is currently pressed and actually stable/valid
            // So now we need to check to see if it's a new press or if it's being held down for auto-repeat
            if (!buttonHeld[i]) {
                // If it wasn't already held, then this is a new press, so we should return true for the button and mark it as held
                buttonStates[i] = true;
                buttonHeld[i] = true;
                repeating[i] = false; // It's a brand-new press, so it hasn't started repeating yet
                lastEventTime[i] = currentTime; // Also record the current time for auto-repeat purposes
            } else if (repeatAllowed[i]) {
                // Otherwise, the button is already held down, so we need to check if it's time to auto-repeat it
                // But only if we want to allow it to repeat, of course
                // The first time around, we want to wait BUTTON_REPEAT_DELAY before repating
                // But then after that, we want to repeat every BUTTON_REPEAT_INTERVAL until the button gets released
                uint32_t repeatTime = repeating[i] ? BUTTON_REPEAT_INTERVAL : BUTTON_REPEAT_DELAY;
                if ((currentTime - lastEventTime[i]) >= repeatTime) {
                    // If we've passed whichever threshold is appropriate, then return true for this button
                    buttonStates[i] = true;
                    lastEventTime[i] = currentTime; // Also reset the event time so that we can measure the next repeat interval
                    repeating[i] = true; // And if we weren't already repeating, now we are, so the next repeat will be at BUTTON_REPEAT_INTERVAL not BUTTON_REPEAT_DELAY
                }
            }
        } else {
            // If the button isn't being pressed, then clear the held and repeating flags
            buttonHeld[i] = false;
            repeating[i] = false;
        }
    }
}

// This function draws the Sony floppy icon on the OLED at the specified x/y coordinates, optionally in inverse video
void drawSonyIcon(uint32_t x, uint32_t y, bool inverseVideo) {
    if (inverseVideo) {
        OLED.drawXBM(x, y, SONY_ICON_WIDTH, SONY_ICON_HEIGHT, sonyIconBitsInvid);
    } else {
        OLED.drawXBM(x, y, SONY_ICON_WIDTH, SONY_ICON_HEIGHT, sonyIconBits);
    }
}

// And this one draws the Twiggy icon at the specified coordinates, optionally in inverse video
void drawTwiggyIcon(uint32_t x, uint32_t y, bool inverseVideo) {
    if (inverseVideo) {
        OLED.drawXBM(x, y, TWIGGY_ICON_WIDTH, TWIGGY_ICON_HEIGHT, twiggyIconBitsInvid);
    } else {
        OLED.drawXBM(x, y, TWIGGY_ICON_WIDTH, TWIGGY_ICON_HEIGHT, twiggyIconBits);
    }
}

// Another one to draw a Lisa 1 icon, the same one that the Lisa itself uses as the Preferences icon
void drawLisa1Icon(uint32_t x, uint32_t y) {
    OLED.drawXBM(x, y, LISA1_ICON_WIDTH, LISA1_ICON_HEIGHT, lisa1IconBits);
}

// As well as a Lisa 2 icon, also the Lisa Preferences one
void drawLisa2Icon(uint32_t x, uint32_t y) {
    OLED.drawXBM(x, y, LISA2_ICON_WIDTH, LISA2_ICON_HEIGHT, lisa2IconBits);
}

// And finally, a Happy Mac icon
void drawHappyMacIcon(uint32_t x, uint32_t y) {
    OLED.drawXBM(x, y, HAPPYMAC_ICON_WIDTH, HAPPYMAC_ICON_HEIGHT, happyMacIconBits);
}

// This is a bitmask representing which pages of the OLED are dirty and need to be resent to the display
// We need this because it's too slow to send the whole display out over I2C in one go, so we just send a single page at a time
// Each page is 8 pixels tall, so there are 8 pages in total on our 64-pixel display
static uint8_t dirtyPages = 0;

// This function is really simple; it just marks every single page as dirty to force the entire display to be resent
void markAllPagesDirty() {
    dirtyPages = 0xFF;
}

// This function marks just the pages from rows yStart through yEnd (inclusive) as dirty
// Each page only takes about 3ms to draw versus about 24ms for the whole display, so this saves an absurd amount of time if you're just redrawing a page or two
void markRegionDirty(uint32_t yStart, uint32_t yEnd) {
    // First, make sure that yStart and yEnd are both within the bounds of the display
    if (yStart > 63) {
        yStart = 63;
    }
    if (yEnd > 63) {
        yEnd = 63;
    }
    // Once they are, then iterate through the 8-pixel pages that cover the range and mark each one as dirty in the dirtyPages bitmask
    for (uint32_t page = (yStart >> 3); page <= (yEnd >> 3); page++) {
        dirtyPages |= (1 << page);
    }
}

// Sends a page of data to the OLED over I2C if there are any pages marked as dirty; returns true if it actually sent something and false if not
// Note that this only sends one page per call even if multiple are dirty so that it doesn't block the SD card task for more than about 3ms per call
bool sendOneOLEDPage() {
    if (dirtyPages == 0) {
        return false; // If there are no dirty pages, there's nothing to do, so just return false
    }
    for (uint8_t page = 0; page < 8; page++) {
        // Otherwise, iterate through all 8 pages that fit on the display and see if any of them are dirty
        if (dirtyPages & (1 << page)) {
            // If a page is dirty, then call updateDisplayArea to send it out to the OLED
            // updateDisplayArea works in 8x8 tiles, so we're telling it to send 16 8x8 tiles horizontally and 1 tile vertically to redraw the page
            OLED.updateDisplayArea(0, page, 16, 1);
            dirtyPages &= ~(1 << page); // Once the page has been sent, clear its dirty bit
            return true; // And return true since we actually sent something
        }
    }
    return false; // We should never get here, but if we somehow do, then return false
}

// Draws a text string at (x, y) clipped to a window of windowWidth pixels
// The draw will start charOffset characters into the string, allowing for easy horizontal scrolling effects
// Returns the full (unclipped) width of the string so the caller can tell whether it needs to scroll to fit it within the window
uint32_t drawClippedText(uint32_t x, uint32_t y, uint32_t windowWidth, const char* text, uint32_t charOffset) {
    // Start by getting the full unclipped width of the string and its length in characters
    uint32_t fullWidth = OLED.getStrWidth(text);
    uint32_t length = strlen(text);
    if (fullWidth <= windowWidth || charOffset > length) {
        // If the string fits within the window or the charOffset is past the end of the string, then just draw it normally without an offset
        charOffset = 0;
    }
    // Now set the coordinates of the clipping window to our (x, y) on one corner and (x + windowWidth, y + 16) on the other corner
    // The y + 16 is overkill since the fone is more like 8-12 pixels tall depending on the character, but better safe than sorry; only the x matters
    OLED.setClipWindow(x, y, x + windowWidth, y + 16);
    OLED.drawStr(x, y, text + charOffset); // And draw the string starting at the charOffset into it
    OLED.setMaxClipWindow(); // Once we're done, reset the clipping window back to the full display so it doesn't affect anything else
    return fullWidth; // And finally, return the full string width
}

// Draws a string in inverse video at (x, y) and returns the total width of the box
// If fullWidth is true, then the box will be the full width of the display, regardless of the string width
uint32_t drawInvertedString(uint32_t x, uint32_t y, const char* text, bool fullWidth) {
    // If fullWidth is false, then get the width of the text and add pixels for padding on each side to get the total width of the box
    uint32_t boxWidth;
    if (!fullWidth) {
        uint32_t textWidth = OLED.getStrWidth(text);
        boxWidth = textWidth + 2;
    } else {
        // Otherwise, the box width is the full width of the display
        boxWidth = OLED.getDisplayWidth();
    }
    uint32_t boxHeight = OLED.getMaxCharHeight() + 1; // Add a pixel of padding on the bottom of the box too
    OLED.drawBox(x, y, boxWidth, boxHeight); // Once we have the box dimensions, draw it in full white
    OLED.setDrawColor(0); // Now set the draw color to black
    OLED.drawStr(x + 1, y, text); // Now draw the black text inside the white box with 1 pixel of padding on the left side
    OLED.setDrawColor(1); // Don't forget to set the draw color back to white so that the rest of the UI draws correctly
    return boxWidth; // And finally, return the total box width
}

// Draws a menu row at the specified y coordinate, in inverse video if it's selected
// Just like drawClippedText, this also supports a charOffset for horizontal scrolling of the text and returns the full string width
uint32_t drawMenuRow(uint32_t y, const char* text, uint32_t charOffset, bool selected) {
    if (selected) {
        // If the item is selected, then draw it in inverse video
        OLED.drawBox(0, y, OLED.getDisplayWidth(), MENU_ITEM_HEIGHT); // Start by drawing a the white box
        OLED.setDrawColor(0); // Set the draw color to black
        uint32_t fullWidth = drawClippedText(0, y, OLED.getDisplayWidth(), text, charOffset); // Then draw the black text inside it
        OLED.setDrawColor(1); // And finally set the draw color back to white for the rest of the UI
        return fullWidth; // Return the full string width so the caller can determine if it needs to scroll
    } else {
        // If the item isn't selected, then just draw it normally
        // And return the string width too
        return drawClippedText(0, y, OLED.getDisplayWidth(), text, charOffset);
    }
}
