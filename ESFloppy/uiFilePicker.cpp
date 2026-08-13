#include <Arduino.h>
#include <SdFat.h>
#include <U8g2lib.h>
#include "config.h"
#include "diskLib.h"
#include "types.h"
#include "ui.h"
#include "uiErrorScreen.h"
#include "uiState.h"

// This file contains all of the source code related to the ESFloppy file picker UI

// A struct that represents a single file or directory in the file picker; it contains the filename and the other relevant metadata we need
struct FileEntry {
    char filename[256]; // The file/directory name
    bool isDirectory; // Whether this entry is a directory or a file
    bool upOneLevel; // Whether this entry is the special ".." item that allows the user to go up one directory level
    DriveType driveType; // The drive type that this file is compatible with (400K, 800K, or Twiggy)
};

// An array of FileEntries that represents the current directory's contents
FileEntry directoryContents[256]; // Assume a max of 256 entries in a single directory

// An array containing all of the acceptable file extensions that we allow; anything not in this list is rejected
const char* allowedExtensions[] {
    ".dc42", // DC42
    ".image", // Sometimes DC42, simetimes raw
    ".img", // Raw
    ".dsk" // Raw
};

// How many pixels a row with an icon on it (a folder or the ".." item) needs to keep free on its right-hand end to fit the icon
#define ROW_ICON_PADDING (FOLDER_ICON_WIDTH + 2)

// An array of forbidden file/directory prefixes; any file or directory that starts with one of these prefixes is rejected
const char* forbiddenPrefixes[] {
    ".", // Hidden files/directories
};

// The current directory that we're in, as a full path
char currentDirectory[1024] = "/"; // Start in the root directory

// The emulator's current settings stored in non-volatile storage
extern ConfigSettings configSettings;

uint32_t numMenuEntries = 0; // The number of menu entries in the current directory

static uint32_t currentItemIndex = 0; // The index of the currently-selected item in the menu
static uint32_t frameStartIndex = 0; // The index of the first item that's visible on the screen in the current menu; used for scrolling

// The disk image metadata and the image files that are currently inserted (or not) in the drives
// These are the same ones that are set up in SDTask and used throughout the emulator, but we need them here too
extern DiskImageMetadata* diskMetadataPointers[2];
extern File32 disk[2];

extern uint32_t selectedDrive; // Which drive is currently selected for the user to interact with

static uint32_t scrollOffset = 0; // How far in characters the currently-selected filename has scrolled so far
static uint32_t scrollTime = 0; // When the last scroll step happened for the filename

static uint32_t holdStartTime = 0; // The time at which the user first pressed and held the SEL button; used to detect long presses
static bool selHeld = false; // Whether the SEL button is currently being held down
static bool selActedUpon = false; // Whether or not we've already acted upon a SEL button press yet

char driveFilenames[2][1024]; // The filenames of the disk images that are currently inserted into the drives

// This function scans the current directory and populates directoryContents with all of the files/directories that are present
// The files are filtered based on the allowedExtensions list, and are sorted alphabetically with directories first
// It returns the number of entries found in the directory
uint32_t assembleDirectoryContents() {
    uint32_t startTime = millis();
    uint32_t entryCount = 0; // How many entries we've found so far
    // Start by opening the current directory
    FatFile dir;
    if (!dir.open(currentDirectory)) {
        return 0; // If we can't open the directory, then just return and leave directoryContents empty
    }
    // Now iterate through all of the entries in the directory
    File32 entry;
    while ((entry.openNext(&dir, O_RDONLY))) {
        if (entryCount >= sizeof(directoryContents) / sizeof(directoryContents[0])) {
            entry.close();
            break; // If we've hit the max number of entries, then we're done
        }
        char nameBuffer[256];
        entry.getName(nameBuffer, sizeof(nameBuffer));
        // First, just check to see if the name is empty, and if it is, then skip this entry
        if (nameBuffer[0] == '\0') {
            entry.close();
            continue;
        }
        // For each entry, check if it's a directory or a file
        if (entry.isDirectory()) {
            // If it's a directory, then check to see if it starts with any of the forbidden prefixes
            bool forbidden = false;
            for (uint32_t i = 0; i < sizeof(forbiddenPrefixes) / sizeof(forbiddenPrefixes[0]); i++) {
                if (strncmp(nameBuffer, forbiddenPrefixes[i], strlen(forbiddenPrefixes[i])) == 0) {
                    forbidden = true; // If it does, then mark it as forbidden and break out of the loop
                    break;
                }
            }
            if (!forbidden) {
                // If it's not forbidden, then add it to the directoryContents array
                entry.getName(directoryContents[entryCount].filename, sizeof(directoryContents[entryCount].filename));
                directoryContents[entryCount].isDirectory = true; // And flag it as a directory
                directoryContents[entryCount].upOneLevel = false; // And it's not the ".." item
                directoryContents[entryCount].driveType = Drive400; // Drive type doesn't matter, but give it a known value
                entryCount++; // Increment the entry count since we just added something to directoryContents
            }
        } else {
            // If it's a file, then check if its extension is in the allowedExtensions list
            // There could be multiple "." characters in the filename, so we need to find the last one to get the extension
            // I discovered this cool function called strrchr that does exactly that; it finds the last occurence of a character in a string
            // It returns a pointer to that character, so it basically just isolates the extension for us
            const char* extension = strrchr(nameBuffer, '.');
            // Now that we have the extension, we can check if it's in our list of allowed extensions
            for (uint32_t i = 0; i < sizeof(allowedExtensions) / sizeof(allowedExtensions[0]); i++) {
                if (extension && strcasecmp(extension, allowedExtensions[i]) == 0) {
                    // If the extension matches one of the allowed extensions, then check to see if the filename starts with any of the forbidden prefixes
                    bool forbidden = false;
                    DriveType driveType;
                    for (uint32_t j = 0; j < sizeof(forbiddenPrefixes) / sizeof(forbiddenPrefixes[0]); j++) {
                        if (strncmp(nameBuffer, forbiddenPrefixes[j], strlen(forbiddenPrefixes[j])) == 0) {
                            forbidden = true; // If it does, then mark it as forbidden and break out of the loop
                            break;
                        }
                    }
                   // After that, check its drive type compatibility based on the file size
                    uint8_t cardReadBuffer[512]; // Buffer for reading the first sector
                    DC42Header header;

                    // Read the first sector of the image into the buffer
                    int32_t bytesRead = entry.read(cardReadBuffer, sizeof(cardReadBuffer));
                    if (bytesRead < (int32_t)sizeof(cardReadBuffer)) {
                        continue; // If we couldn't read it, then skip this extension and move on to the next one, hoping that we'll succeed that time
                    }
                    memcpy(&header, cardReadBuffer, sizeof(DC42Header)); // And copy the potential DC42 header into our header struct

                    // The fields in the header are big-endian, but the ESP32 is little-endian
                    // So we need to swap the byte order of the multi-byte fields
                    // Only swap the fields we care about; no need to do the others
                    header.dataSize = __builtin_bswap32(header.dataSize);
                    header.tagSize = __builtin_bswap32(header.tagSize);
                    header.dc42Magic = __builtin_bswap16(header.dc42Magic);

                    if(header.dc42Magic == 0x0100) { // If the magic number in the header is 0x0100, then it's a DC42 image
                        // First, see if the file is smaller than the DC42 header claims it is
                        if (entry.fileSize() < sizeof(DC42Header) + header.dataSize + header.tagSize) {
                            forbidden = true; // If it is, then mark it as forbidden
                        }
                        // Now see if it's a 400K, 800K, or Twiggy image based on the disk encoding and data size
                        if (header.diskEncoding == DC42_400K_ENCODING && header.dataSize == DATA_SIZE_400K && configSettings.emulMode != ModeTwiggy) {
                            driveType = Drive400; // If it's 400K and we're not in Twiggy mode, then set the drive type to 400K
                            // But if the tag size is not 0 or the expected size, then mark it as forbidden
                            if (header.tagSize != TAG_SIZE_400K && header.tagSize != 0) {
                                forbidden = true;
                            }
                        }
                        // Now repeat that same process for 800K images
                        else if (header.diskEncoding == DC42_800K_ENCODING && header.dataSize == DATA_SIZE_800K && configSettings.emulMode != ModeTwiggy) {
                            driveType = Drive800;
                            if (header.tagSize != TAG_SIZE_800K && header.tagSize != 0) {
                                forbidden = true;
                            }
                        }
                        // And finally one more time for Twiggy images, this time ensuring we're in Twiggy mode
                        else if (header.diskEncoding == DC42_TWIGGY_ENCODING && header.dataSize == DATA_SIZE_TWIGGY && configSettings.emulMode == ModeTwiggy) {
                            driveType = DriveTwiggy;
                            if (header.tagSize != TAG_SIZE_TWIGGY && header.tagSize != 0) {
                                forbidden = true;
                            }
                        }
                        else {
                            // If the DC42 doesn't match any of those specs, then it's either bad or incompatible with our EmulMode, so reject it
                            forbidden = true;
                        }
                    } else {
                        // If it's not a DC42, then use the file size to determine the drive type
                        if (entry.fileSize() == DATA_SIZE_400K && configSettings.emulMode != ModeTwiggy) {
                            driveType = Drive400;
                        } else if (entry.fileSize() == DATA_SIZE_800K && configSettings.emulMode != ModeTwiggy) {
                            driveType = Drive800;
                        } else if (entry.fileSize() == DATA_SIZE_TWIGGY && configSettings.emulMode == ModeTwiggy) {
                            driveType = DriveTwiggy;
                        } else {
                            forbidden = true; // If it's none of those, then mark it as forbidden  
                        }
                    }
                    if (!forbidden) {
                        // If the image isn't forbidden after all of that, then add it to the directoryContents array
                        entry.getName(directoryContents[entryCount].filename, sizeof(directoryContents[entryCount].filename));
                        directoryContents[entryCount].isDirectory = false; // Flag it as a file
                        directoryContents[entryCount].upOneLevel = false; // Flag it as not the special ".." item
                        directoryContents[entryCount].driveType = driveType; // And set its drive type
                        entryCount++; // Increment the entry count again
                        break; // And break out of the loop since we found a match
                    }
                }
            }
        }
        entry.close();
    }
    dir.close();
    // At this point, directoryContents should be populated with all of the files/directories in the current directory that match our requirements
    // So now we just need to sort them alphabetically, with directories first
    // I'm just going to use bubble sort here since it's simple and we only have 256 entries max
    FileEntry temp; // A temporary variable for swapping entries
    uint32_t limit = entryCount; // The limit of the unsorted portion of the array
    // Keep going until the unsorted portion of the array is just 1 entry
    while (limit > 1) {
        uint32_t newLimit = 0;
        // Iterate through the whole unsorted portion of the array
        for (uint32_t i = 0; i + 1 < limit; i++) {
            // Now we need to compare the current entry with the next one to see if they need to be swapped
            if (directoryContents[i].isDirectory && !directoryContents[i + 1].isDirectory) {
                // If the current entry is a directory and the next one is a file, then we don't need to swap them since directories come first
                continue;
            } else if (!directoryContents[i].isDirectory && directoryContents[i + 1].isDirectory) {
                // If the current entry is a file and the next one is a directory, then we need to swap them to give the directory priority
                temp = directoryContents[i];
                directoryContents[i] = directoryContents[i + 1];
                directoryContents[i + 1] = temp;
                newLimit = i + 1; // Update the new limit to the last swap position
            } else if (strcasecmp(directoryContents[i].filename, directoryContents[i + 1].filename) > 0) {
                // If both entries are the same type, then we need to compare their filenames alphabetically
                // If the current entry's filename is greater than the next entry's filename, then we need to swap them
                temp = directoryContents[i];
                directoryContents[i] = directoryContents[i + 1];
                directoryContents[i + 1] = temp;
                newLimit = i + 1; // Update the new limit to the last swap position
            }
        }
        limit = newLimit; // Update the limit for the next pass
    }
    // And one final thing: we need to add the special ".." item to the start of directoryContents if we're not in the root directory
    if (strcmp(currentDirectory, "/") != 0) {
        // To do that, shift all of the entries in directoryContents down by one to make room
        // If we already have 256 items in the array, then this shift will take the last item off the end and out of bounds
        // So decrement entryCount by 1 if it's already at the max to avoid going out of bounds
        // We'll lose that item, but that's fine
        if (entryCount >= sizeof(directoryContents) / sizeof(directoryContents[0])) {
            entryCount--;
        }
        for (uint32_t i = entryCount; i > 0; i--) {
            directoryContents[i] = directoryContents[i - 1];
        }
        // Now add the ".." to the start
        strcpy(directoryContents[0].filename, "..");
        directoryContents[0].isDirectory = true;
        directoryContents[0].upOneLevel = true; // Flag it as the special ".." item
        directoryContents[0].driveType = Drive400; // Drive type doesn't matter just like for directories
        entryCount++; // Increment the entry count again
    }
    return entryCount; // Return the number of entries found in the directory
}

// This function gets called when a file or directory is selected from the file picker menu
void handleFileSelection() {
    // First, check to see if the current item is a file or a directory
    if (directoryContents[currentItemIndex].isDirectory) {
        // If it's a directory, then we need to change the current directory to that directory and reassemble the directory contents there
        // So just append the directory name to the currentDirectory string and then call assembleDirectoryContents() again
        // Only append a trailing "/" if the pathname doesn't already end in one
        // And also make sure that currentDirectory has enough space to hold the new name and abort if not
        if (strlen(currentDirectory) + strlen(directoryContents[currentItemIndex].filename) + 2 > sizeof(currentDirectory)) {
            return;
        }
        if (currentDirectory[strlen(currentDirectory) - 1] != '/') {
            strcat(currentDirectory, "/");
        }
        strcat(currentDirectory, directoryContents[currentItemIndex].filename);
        numMenuEntries = assembleDirectoryContents();
        // Also reset currentItemIndex and frameStartIndex to 0 so that we're at the top of the directory
        currentItemIndex = 0;
        frameStartIndex = 0;
    } else {
        // If it's instead a disk image file, then we need to load that image into the selected drive
        // We already know the image is compatible since we filtered it in assembleDirectoryContents()
        char fullPath[1024];
        snprintf(fullPath, sizeof(fullPath), "%s/%s", currentDirectory, directoryContents[currentItemIndex].filename);
        // But there's one check we need to do before we load it: if the other drive has the same image loaded, then we need to abort
        // We obviously can't open the same image in both drives at once
        if (diskMetadataPointers[1 - selectedDrive]->diskInserted && strcmp(fullPath, driveFilenames[1 - selectedDrive]) == 0) {
            // If this ends up being the case, then show the user an error message and return without loading the image
            snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: Image");
            snprintf(errorMessageLines[1], sizeof(errorMessageLines[1]), "already loaded in");
            snprintf(errorMessageLines[2], sizeof(errorMessageLines[2]), "other drive!");
            errorMessageLineCount = 3;
            pushScreen(&errorScreen);
            return; // Once they've dismissed the error, return without loading the image
        }
        OpenResult openResult = openImage(fullPath, &disk[selectedDrive], diskMetadataPointers[selectedDrive]);
        if (openResult == ResultSuccess) {
            // If we succeeded in opening the image, then mark the disk as inserted
            diskMetadataPointers[selectedDrive]->diskInserted = true;
            // Store the filename of the image in the driveFilenames array so that we can check for duplicates later
            strncpy(driveFilenames[selectedDrive], fullPath, sizeof(driveFilenames[selectedDrive]));
            // And then pop the file picker screen off the stack to return to the main UI
            popScreen();
        } else {
            // If we failed to open the image, then show the user an error message explaining why
            switch (openResult) {
                case ResultFailedOpen:
                    snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: Failed to open");
                    snprintf(errorMessageLines[1], sizeof(errorMessageLines[1]), "image file!");
                    errorMessageLineCount = 2;
                    break;
                case ResultNotContiguous:
                    snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: Image file is");
                    snprintf(errorMessageLines[1], sizeof(errorMessageLines[1]), "not contiguous!");
                    errorMessageLineCount = 2;
                    break;
                case ResultInvalidTagSize:
                    snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: Invalid");
                    snprintf(errorMessageLines[1], sizeof(errorMessageLines[1]), "tag size in");
                    snprintf(errorMessageLines[2], sizeof(errorMessageLines[2]), "DC42 header!");
                    errorMessageLineCount = 3;
                    break;
                case ResultInvalidDiskEncoding:
                    snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: Invalid");
                    snprintf(errorMessageLines[1], sizeof(errorMessageLines[1]), "disk encoding in");
                    snprintf(errorMessageLines[2], sizeof(errorMessageLines[2]), "DC42 header!");
                    errorMessageLineCount = 3;
                    break;
                case ResultInvalidDiskFormat:
                    snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: Invalid");
                    snprintf(errorMessageLines[1], sizeof(errorMessageLines[1]), "disk format in");
                    snprintf(errorMessageLines[2], sizeof(errorMessageLines[2]), "DC42 header!");
                    errorMessageLineCount = 3;
                    break;
                case ResultDARTNotSupported:
                    snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: DART images");
                    snprintf(errorMessageLines[1], sizeof(errorMessageLines[1]), "are not supported!");
                    errorMessageLineCount = 2;
                    break;
                case ResultInvalidImageSize:
                    snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: Invalid");
                    snprintf(errorMessageLines[1], sizeof(errorMessageLines[1]), "image size!");
                    errorMessageLineCount = 2;
                    break;
                default:
                    snprintf(errorMessageLines[0], sizeof(errorMessageLines[0]), "Error: Unknown error!");
                    errorMessageLineCount = 1;
                    break;
            }
            // Once the message is formed, push the error screen onto the stack to show it to the user
            pushScreen(&errorScreen);
        }
    }
}

// This function gets called when the user selects the ".." item in the file picker menu to go up one directory level
void upOneDirectory() {
    // All we need to do here is remove the last directory from the currentDirectory string and then reassemble the directory contents
    // This is as simple as finding the last "/" character in the string and replacing it with a null terminator
    char* lastSlash = strrchr(currentDirectory, '/');
    // If lastSlash is the first character of the string, then we were already at the root directory, so don't do anything else
    // This should never happen since we don't show the ".." item in the root directory, but might as well be safe
    // The lastSlash check will be equal to currentDirectory in two cases: if we're in the root directory ("/") or if we're in a 1-depth subdirectory ("/dir")
    // So make sure that we don't mis-detect the subdirectory case as "already at root directory" and not go up one level
    if (lastSlash == currentDirectory) {
        currentDirectory[1] = '\0';
    } else if (lastSlash) {
        *lastSlash = '\0';
    }
    // Then reassemble the menus
    numMenuEntries = assembleDirectoryContents();
    // And don't forget to reset the currentItemIndex and frameStartIndex to 0 so that we start at the top of the new directory
    currentItemIndex = 0;
    frameStartIndex = 0;
}

// This function builds the label string for a menu item in the file picker, given its currentItemIndex
void buildItemLabel(uint32_t index, char* buffer, uint32_t size) {
    // First, check to see if the item is a file or directory
    if (directoryContents[index].isDirectory) {
        // If it's a directory, then just print the directory name
        snprintf(buffer, size, "%s", directoryContents[index].filename);
    } else {
        // Files have the drive type after them, like "filename (400K)", "filename (800K)", or "filename (Twiggy)"
        snprintf(buffer, size, "%s %s", directoryContents[index].filename, directoryContents[index].driveType == Drive400 ? "(400K)" : (directoryContents[index].driveType == Drive800 ? "(800K)" : "(Twiggy)"));
    }
}

// Call this whenever we want to re-enter the file picker at the root directory with everything reset to a fresh state
// If we don't call this, then the picker will resume where it left off in the last directory that was open, with the last-selected item highlighted
void filePickerReset() {
    strcpy(currentDirectory, "/"); // Start in the root directory
    currentItemIndex = 0; // Start with the first item selected
    frameStartIndex = 0; // And visible
    // Assemble the directory of images for the current directory
    numMenuEntries = assembleDirectoryContents();
}

// This function gets called when we first enter the file picker screen
// Note that it doesn't reset the current directory or selected item; just the scroll offset and SEL state
// To reset those things, call filePickerReset first
void filePickerEnter() {
    // Reset the scroll offset, scroll time, and SEL button state so that everything is completely fresh
    scrollOffset = 0;
    scrollTime = 0;
    selHeld = false;
    selActedUpon = false;
}

// This function gets called periodically while we're on the file picker screen
void filePickerTick(uint32_t currentTime) {
    // On each tick, build the label string for the currently-selected item
    // Make sure the directory isn't empty first though; just return immediately if it is
    if (numMenuEntries == 0) {
        return;
    }
    char labelBuffer[280];
    buildItemLabel(currentItemIndex, labelBuffer, sizeof(labelBuffer));
    // Now handle scrolling of the filename if it's too long to fit in the window
    // Directories have an icon at the end of their rows (a folder for normal ones and an arrow for the ".." item), so we need to reserve some space for that icon when calculating the text width
    uint32_t textWidth = OLED.getDisplayWidth() - (directoryContents[currentItemIndex].isDirectory ? ROW_ICON_PADDING : 0);
    if (OLED.getStrWidth(labelBuffer) <= textWidth) {
        // If the string is narrower than the window, then we don't need to scroll it, so no need to execute any of the following code
        return;
    }
    // Otherwise, we need to scroll; start by figuring out the offset into the string of the last character that fits in the window
    // The font is monospaced, so the character count is just the window width divided by the character width
    uint32_t lastCharOffset = strlen(labelBuffer) - (textWidth / OLED.getMaxCharWidth());
    bool atEnd = (scrollOffset >= lastCharOffset); // Now check if we're already at the end of the filename
    // If we're at the start or end of the filename, then pause scrolling for a bit
    // Otherwise, set the scrollInterval to the normal scroll step time
    uint32_t scrollInterval = (scrollOffset == 0 || atEnd) ? SCROLL_PAUSE : SCROLL_STEP;
    // And finally, if enough time has passed since the last scroll step, then update the scroll offset to scroll the filename
    if (currentTime - scrollTime >= scrollInterval) {
        scrollTime = currentTime; // Update the last scroll time to the current time
        if (atEnd) {
            scrollOffset = 0; // If we're at the end of the name, then reset to the start
        } else {
            scrollOffset++; // Otherwise, just scroll forward by one character
        }
        // Now we need to redraw the screen to show the updated scroll position; just redraw the current menu item
        // This is just the redraw request; the actual redraw happens in filePickerDrawScreen
        uint32_t yPosition = (currentItemIndex - frameStartIndex + 1) * MENU_ITEM_HEIGHT;
        redrawRegion(yPosition, yPosition + MENU_ITEM_HEIGHT);
    }
}

// This function gets called when a button is pressed while we're on the file picker screen
void filePickerButtonPress(bool buttonStates[3]) {
    // If the LEFT button is pressed, then we need to move the selection up one item in the menu
    if (buttonStates[0]) {
        if (currentItemIndex > 0) {
            currentItemIndex--;
        }
        scrollOffset = 0; // Reset the scroll offset to 0 so that the filename starts at the beginning
        scrollTime = 0; // And the scroll timer too
        // And command a redraw of the whole screen
        redrawWholeScreen();
    }
    // SEL is a little more complicated; we need to detect whether it's a short press or a long press and act accordingly
    // A short press should select the currently-highlighted file or directory (mount the image or go into the directory)
    // Whereas a long press should pop the file picker screen off the stack and return to the main UI
    if (buttonStates[1]) {
        // When the SEL button is first pressed, record the time that it went down
        holdStartTime = millis();
        selHeld = true; // Mark that SEL is now held
        selActedUpon = false; // And that we haven't taken any action on it yet
    } else if (selHeld && getButtonHeld(1)) {
        // If selHeld says that SEL is held down and the debounced button state agrees, then check to see how long it's been down for
        if (!selActedUpon && (millis() - holdStartTime >= LONG_PRESS_DURATION)) {
            // If it's been held down for long enough and we haven't acted upon it yet, then pop the file picker off the stack and return to the status screen
            popScreen();
            selActedUpon = true; // And mark that we've acted upon this button press so that we don't do it again, although we should never get here again
        }
    } else if (selHeld) {
        // Otherwise, if selHeld says that it's held, but it's not actually held down anymore, then we need to perform the short-press action
        // Of course, only do this if we haven't already acted upon it with the long-press action
        selHeld = false;
        if (!selActedUpon) {
            selActedUpon = true;
            // Only perform the action if there are actually menu entries to select; otherwise, just ignore it
            if (numMenuEntries > 0) {
                if (directoryContents[currentItemIndex].upOneLevel) {
                    // If the currently-selected item is the special ".." item, then go up a directory
                    upOneDirectory();
                } else {
                    // Otherwise, handle the file or directory selection as normal
                    handleFileSelection();
                }
            }
            scrollOffset = 0;
            scrollTime = 0;
            redrawWholeScreen();
        }
    }
    if (buttonStates[2]) {
        // If the RIGHT button is pressed, then we need to move the selection down one item in the menu
        if (currentItemIndex + 1 < numMenuEntries) {
            currentItemIndex++;
        }
        scrollOffset = 0;
        scrollTime = 0;
        redrawWholeScreen();
    }
    // There's a chance that the currentItemIndex has moved out of the visible frame after all this, so we need to check on that
    if (currentItemIndex < frameStartIndex) {
        // If it's above the top of the screen, then just set frameStartIndex to the current item index
        frameStartIndex = currentItemIndex;
    } else if (currentItemIndex >= (frameStartIndex + ((OLED.getDisplayHeight() - MENU_ITEM_HEIGHT) / MENU_ITEM_HEIGHT))) {
        // If it's below the bottom of the screen, then set frameStartIndex so that the current item is at the bottom of the screen
        frameStartIndex = currentItemIndex - ((OLED.getDisplayHeight() - MENU_ITEM_HEIGHT) / MENU_ITEM_HEIGHT) + 1;
    }
}

// And this function gets called to actually draw the file picker screen on the OLED
void filePickerDrawScreen() {
    // Drawing the file picker screen is pretty simple; start with the title bar
    // Center it horizontally to make it look good
    // The string differs depending on the disk emulation mode and drive configuration
    char titleBuffer[32];
    snprintf(titleBuffer, sizeof(titleBuffer), "Select Image%s", configSettings.emulMode == ModeTwiggy ? (selectedDrive == 0 ? " - Upper" : " - Lower") : "");
    OLED.clearBuffer();
    OLED.drawStr(((128 - OLED.getStrWidth(titleBuffer)) / 2), (0 * MENU_ITEM_HEIGHT), titleBuffer);
    // And draw a horizontal line under it to separate it from the menu items
    OLED.drawHLine(0, MENU_ITEM_HEIGHT, OLED.getDisplayWidth());
    // Now we need to draw all of the actual menu items that are visible on the screen
    // Don't start at index 0; start at frameStartIndex since the menu may be scrolled down a bit
    // And end whenever we run out of space on the screen or we run out of menu items, whichever comes first
    for (uint32_t i = frameStartIndex; i < numMenuEntries; i++) {
        // Start by computing each item's y position on the screen based on its index and the height of each menu item
        uint32_t yPosition = (i - frameStartIndex + 1) * MENU_ITEM_HEIGHT;
        if (yPosition + MENU_ITEM_HEIGHT > OLED.getDisplayHeight()) {
            // If the item is off the bottom of the screen, then it's time to break
            break;
        }
        // Otherwise, we need to draw it
        // Start by using our buildItemLabel function to get the properly-formatted label string for this item
        char labelBuffer[280];
        buildItemLabel(i, labelBuffer, sizeof(labelBuffer));
        // Remember that we display an icon for directories and the ".." item, so reserve space from it just like we did in the tick function
        uint32_t reservedWidth = directoryContents[i].isDirectory ? ROW_ICON_PADDING : 0;
        // Now draw the menu row with the appropriate scroll offset and selection state
        // Only scroll the currently-selected item; all other items are drawn with a scroll offset of 0
        // And only draw the selected item in inverse video too
        drawMenuRow(yPosition, labelBuffer, (i == currentItemIndex) ? scrollOffset : 0, (i == currentItemIndex), reservedWidth);
        // Now draw the appropriate icon for the menu entry if it's a directory or the ".." item
        if (reservedWidth > 0) {
            if (i == currentItemIndex) {
                // If it's the currently-selected item, then set the draw color to black so that the icon is drawn in inverse video
                OLED.setDrawColor(0);
            }
            if (directoryContents[i].upOneLevel) {
                // The ".." item gets an arrow icon
                drawUpOneLevelIcon(OLED.getDisplayWidth() - FOLDER_ICON_WIDTH, yPosition);
            } else {
                // And regular directories get a folder icon
                drawFolderIcon(OLED.getDisplayWidth() - FOLDER_ICON_WIDTH, yPosition);
            }
            if (i == currentItemIndex) {
                // Don't forget to set the draw color back to white if we drew the icon in inverse video
                OLED.setDrawColor(1);
            }
        }
    }
    // If there happen to be zero menu entries on the screen, then just draw a message saying that no valid images were found
    // There are two cases in which this can happen
    // Either we're in the root and literally nothing shows up at all
    // Or we're in a subdirectory and so we get the ".." item, but nothing else shows up
    // So check for both cases and print the "no images" message either way
    if (numMenuEntries == 0 || (numMenuEntries == 1 && directoryContents[0].upOneLevel)) {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "No valid images!");
        // We want the message to be centered between the last menu item (either the title bar or "..") and the bottom of the screen
        uint32_t menuEnd = numMenuEntries == 0 ? MENU_ITEM_HEIGHT : (2 * MENU_ITEM_HEIGHT);
        OLED.drawStr(((128 - OLED.getStrWidth(buffer)) / 2), ((OLED.getDisplayHeight() - menuEnd) / 2), buffer);
    }
}