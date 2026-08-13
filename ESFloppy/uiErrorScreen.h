#pragma once

// Header file for ESFloppy UI's error screen

extern char errorMessageLines[3][40]; // The 3 lines of error message text that we can display on the error screen
extern uint32_t errorMessageLineCount; // The number of lines of text in the error message that we're actually using

// This function gets called when we first enter the error screen
void errorEnter();

// This function gets called periodically while we're on the error screen
void errorTick(uint32_t currentTime);

// This function gets called when a button is pressed while we're on the error screen
void errorButtonPress(bool buttonStates[3]);

// And this function gets called to actually draw the error screen on the OLED
void errorDrawScreen();