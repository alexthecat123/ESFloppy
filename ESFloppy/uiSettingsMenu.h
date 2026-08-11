// Header file for the ESFloppy UI's settings menu

// Pops a menu off the stack and makes it the current menu
static void popMenu();

// This is the function that gets called when we first enter the settings screen
void settingsEnter();

// This is the function that gets called periodically while we're on the settings screen
void settingsTick(uint32_t currentTime);

// This is the function that handles button presses on the settings screen
void settingsButtonPress(bool buttonStates[3]);

// This function formats the text for a menu item based on its type and value
void formatItemText(const MenuItem* item, char* buffer, uint32_t bufferSize);

// And this is the function that actually draws the settings screen on the OLED
void settingsDrawScreen();