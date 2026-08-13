#include <Arduino.h>
#include "ui.h"
#include "uiErrorScreen.h"
#include "uiState.h"

// Source file for the error screen that gets displayed by the ESFloppy UI

char errorMessageLines[3][40]; // The 3 lines of error message text that we can display on the error screen
uint32_t errorMessageLineCount; // The number of lines of text in the error message that we're actually using

// This function gets called when we first enter the error screen
void errorEnter() {
   
}

// This function gets called periodically while we're on the error screen
void errorTick(uint32_t currentTime) {
    
}

// This function gets called when a button is pressed while we're on the error screen
void errorButtonPress(bool buttonStates[3]) {
    // This is incredibly simple; if the user presses any button, then we just pop this screen and go back to wherever we were before
    if (buttonStates[0] || buttonStates[1] || buttonStates[2]) {
        popScreen();
    }
}

// And this function gets called to actually draw the error screen on the OLED
void errorDrawScreen() {
    // All we need to do here is to display the lines of text that make up the error message
    // These vary based on the error, so just use the errorMessageLines array to get the text for each line
    OLED.clearBuffer(); // Start by clearing the framebuffer so we can draw a fresh new screen
    // When we iterate through the messages, make sure to cap the number of lines to 3 so that we don't index out of our array
    for (uint32_t i = 0; i < (errorMessageLineCount > 3 ? 3 : errorMessageLineCount); i++) {
        // Center each line on the OLED horizontally and draw it at the appropriate vertical position based on its index in the array
        OLED.drawStr(((128 - OLED.getStrWidth(errorMessageLines[i])) / 2), (i * MENU_ITEM_HEIGHT), errorMessageLines[i]);
    }
    // Now draw the error icon below the text, centered horizontally
    // Sane deak as with the loop; cap errorMessageLineCount to 3
    drawErrorIcon(((128 - ERROR_ICON_WIDTH) / 2), ((MENU_ITEM_HEIGHT * (errorMessageLineCount > 3 ? 3 : errorMessageLineCount)) + ((MENU_ITEM_HEIGHT * 7) - (MENU_ITEM_HEIGHT * errorMessageLineCount) - ERROR_ICON_HEIGHT) / 2));
    // And finally, draw the "Press SEL..." message at the bottom of the screen, centered horizontally
    char buffer[40];
    snprintf(buffer, sizeof(buffer), "Press SEL...");
    OLED.drawStr(((128 - OLED.getStrWidth(buffer)) / 2), (MENU_ITEM_HEIGHT * 7), buffer);
}