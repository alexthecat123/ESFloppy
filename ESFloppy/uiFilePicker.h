// Header file for ESFloppy's file picker UI

// A struct that represents a single file or directory in the file picker; it contains the filename and the other relevant metadata we need
struct FileEntry;

// This function populates the file picker menu items based on the current directory contents
void populateMenuItems(MenuItem* menuItems, FileEntry* fileEntries, uint32_t count);

// This function gets called when we first enter the file picker screen
void filePickerEnter();

// This function gets called periodically while we're on the file picker screen
void filePickerTick(uint32_t currentTime);

// This function gets called when a button is pressed while we're on the file picker screen
void filePickerButtonPress(bool buttonStates[3]);

// And this function gets called to actually draw the file picker screen on the OLED
void filePickerDrawScreen();