// Header file for ESFloppy's file picker UI

// A struct that represents a single file or directory in the file picker; it contains the filename and the other relevant metadata we need
struct FileEntry;

// This function populates the file picker menu items based on the current directory contents
void populateMenuItems(MenuItem* menuItems, FileEntry* fileEntries, uint32_t count);

// Call this whenever we want to re-enter the file picker at the root directory with everything reset to a fresh state
// If we don't call this, then the picker will resume where it left off in the last directory that was open, with the last-selected item highlighted
void filePickerReset();

// This function gets called when we first enter the file picker screen
// Note that it doesn't reset the current directory or selected item; just the scroll offset and SEL state
// To reset those things, call filePickerReset first
void filePickerEnter();

// This function gets called periodically while we're on the file picker screen
void filePickerTick(uint32_t currentTime);

// This function gets called when a button is pressed while we're on the file picker screen
void filePickerButtonPress(bool buttonStates[3]);

// And this function gets called to actually draw the file picker screen on the OLED
void filePickerDrawScreen();