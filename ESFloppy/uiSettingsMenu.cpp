#include <Arduino.h>
#include "config.h"
#include "fwVersion.h"
#include "types.h"
#include "ui.h"
#include "uiHelpers.h"
#include "uiState.h"

// The settings menu for the ESFloppy UI

extern ConfigSettings configSettings; // The global configuration settings struct that we use throughout the UI

// Let's start by actually creating the menus that we want to show in the settings screen using the structs from uiState.h
// The top-level menu looks like this:
/*
ESFloppy Settings
Emulation Mode...
Contrast: 255
Screen Dim: On
Save and Reboot
Discard and Reboot
About...
*/
// The "Emulation Mode..." item opens a Menu_Enum submenu that looks like this:
/*
Emulation Mode
Back...
Lisa 400K/800K
Mac 400K/800K
Twiggy
*/
// Contrast is a Menu_Numeric that lets you set the OLED contrast from 0 to 255 in increments of 1
// And screen dim is a Menu_Toggle that just lets you turn the dimming feature on or off

static void popMenu(); // Forward declaration of the popMenu function so that we can use it in the menu items below

static void drawAboutScreen(); // Forward declaration of drawAboutScreen for the menu items below

// Start by making all of the MenuItems for the emulation mode submenu
static const MenuItem emulationModeItems[] = {
    // Our first menu is a Menu_Action that just pops the current menu off the stack and returns to the previous one
    {"Back...", Menu_Action, nullptr, 0, 0, 0, 0, nullptr, nullptr, []() { popMenu(); }},
    // Each of these is a Menu_Enum that sets the emulMode setting to the appropriate value
    {"Lisa 400K/800K", Menu_Enum, &configSettings.emulMode, 0, 0, 0, ModeSonyLisa, nullptr, nullptr},
    {"Mac 400K/800K", Menu_Enum, &configSettings.emulMode, 0, 0, 0, ModeSonyMac, nullptr, nullptr},
    {"Twiggy", Menu_Enum, &configSettings.emulMode, 0, 0, 0, ModeTwiggy, nullptr, nullptr}
};

// Now make the submenu for emulation mode selection
static const Menu emulationModeMenu = {
    "Emulation Mode", // The menu title
    emulationModeItems, // The array of items in the menu
    4 // The number of items in the menu
};

// Now do all of the MenuItems for the main menu
static const MenuItem mainMenuItems[] = {
    // The Menu_Submenu for selecting the emulation mode; all we need to fill in is the submenu pointer
    {"Emulation Mode", Menu_Submenu, nullptr, 0, 0, 0, 0, &emulationModeMenu, nullptr, nullptr},
    // The Menu_Numeric for contrast; fill in the value pointer, min/max values (8/255), and step amount (8)
    // And also make the onChange function set the OLED contrast to the new value whenever it changes
    {"Contrast", Menu_Numeric, &configSettings.brightness, 8, 255, 8, 0, nullptr, []() { OLED.setContrast(configSettings.brightness); }, nullptr},
    // The Menu_Toggle for screen dimming; just fill in the value pointer
    {"Screen Dim", Menu_Toggle, &configSettings.dimDisplay, 0, 0, 0, 0, nullptr, nullptr, nullptr},
    // The Menu_Action for saving and rebooting; all we need is the function, which literally just saves the settings and then reboots
    {"Save and Reboot", Menu_Action, nullptr, 0, 0, 0, 0, nullptr, nullptr, []() { initConfig(); writeConfig(configSettings); closeConfig(); ESP.restart(); }},
    // A Menu_Action for exiting without saving; it just reboots without doing anything else
    {"Discard and Reboot", Menu_Action, nullptr, 0, 0, 0, 0, nullptr, nullptr, []() { ESP.restart(); }},
    // And finally an About... item that shows the firmware version
    {"About...", Menu_Action, nullptr, 0, 0, 0, 0, nullptr, nullptr, []() { drawAboutScreen(); }}
};

// And finally, the main menu itself
static const Menu mainMenu = {
    "ESFloppy Settings",
    mainMenuItems,
    6
};

static const Menu* currentMenu; // The currently-selected menu
static uint32_t currentItemIndex = 0; // The index of the currently-selected item in the current menu
static uint32_t frameStartIndex = 0; // The index of the first item that's visible on the screen in the current menu; used for scrolling
static bool editingItem = false; // Whether or not we're editing the setting that's configured by the selected item

// A nice and easy way to be able to move in and out of submenus is with a menu stack
// So let's make one
static const Menu* menuStack[8]; // 8 deep is plenty for our tiny settings menu; we really only need 2
static uint32_t itemIndexStack[8]; // We also need to keep track of which item was selected in each menu so we can return to it when we pop back up
static uint32_t frameStartIndexStack[8]; // And we also need to keep track of the frame start index for each menu so that we can return to it when we pop back up
static uint32_t menuStackPointer = 0; // The pointer to the current top of the menu stack

// Pops a menu off the stack and makes it the current menu
static void popMenu() {
    // If the stack pointer is 0, then abort
    if (menuStackPointer > 0) {
        // Otherwise, grab the top menu and item index off the stack and make them the current ones
        menuStackPointer--;
        currentMenu = menuStack[menuStackPointer];
        currentItemIndex = itemIndexStack[menuStackPointer];
        frameStartIndex = frameStartIndexStack[menuStackPointer];
    }
}

// Draws the About screen, which shows some general information about ESFloppy like the firmware version
static void drawAboutScreen() {
    // For now, the only thing I can think of to put here is the firmware version
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "FW Version: %s", FIRMWARE_VERSION);
    OLED.clearBuffer();
    // Draw a title and horizontal separator underneath it
    OLED.drawStr(((128 - OLED.getStrWidth("About ESFloppy")) / 2), 0, "About ESFloppy");
    OLED.drawHLine(0, MENU_ITEM_HEIGHT, OLED.getDisplayWidth());
    // Then draw the version string below that
    OLED.drawStr(((128 - OLED.getStrWidth(buffer)) / 2), MENU_ITEM_HEIGHT * 2, buffer);
    // And finally, draw a "press SEL to continue" prompt at the bottom of the screen
    OLED.drawStr(((128 - OLED.getStrWidth("Press SEL...")) / 2), MENU_ITEM_HEIGHT * 7, "Press SEL...");
    OLED.sendBuffer();
    while (digitalRead(SEL) == LOW) {
        vTaskDelay(1); // Wait for the user to release SEL if it's being held
    }
    while (digitalRead(SEL) == HIGH) {
        vTaskDelay(1); // And then wait for them to press it again before continuing
    }
}

// This function gets called when we first enter the settings screen
void settingsEnter() {
    // Start by getting our state in order; the main menu is the current menu
    currentMenu = &mainMenu;
    currentItemIndex = 0; // The first item in the menu is selected by default
    frameStartIndex = 0; // The first item in the menu is visible by default
    editingItem = false; // And we're not editing anything yet
    menuStackPointer = 0; // Reset the stack pointer just to be safe, although it should already be 0
    // And finally, mark the whole screen as dirty so that it gets redrawn with the settings menu
    redrawWholeScreen();
}

// This function gets called periodically while we're on the settings screen
void settingsTick(uint32_t currentTime) {

}   

// This function gets called when a button is pressed while we're on the settings screen
void settingsButtonPress(bool buttonStates[3]) {
    // First, let's see if the SEL button is being pressed
    if (buttonStates[1]) {
        // If so, then there are two potential cases that we can be in
        if (editingItem) {
            // If we're currently editing a setting, then we need to stop editing it and return to the menu
            editingItem = false;
        } else {
            // Otherwise, we're not editing a setting, so we need to execute the action for the currently-selected item
            const MenuItem* selectedItem = &currentMenu->items[currentItemIndex];
            // Figure out what to do based on the type of the selected item
            // Once we're done with the action, we need to execute the item's onChange function if it exists
            switch (selectedItem->type) {
                case Menu_Submenu:
                    // If it's a submenu, then we need to push the current menu onto the stack and switch to the submenu
                    if (menuStackPointer < 8) { // Make sure we don't overflow the stack
                        menuStack[menuStackPointer] = currentMenu;
                        itemIndexStack[menuStackPointer] = currentItemIndex;
                        frameStartIndexStack[menuStackPointer] = frameStartIndex;
                        menuStackPointer++;
                        currentMenu = selectedItem->submenu;
                        if (currentMenu == &emulationModeMenu) {
                            // If we're entering the emulation mode submenu, then set currentItemIndex to the one that corresponds to the current emulMode setting
                            // This isn't technically necessary, but it's convenient for the user, so might as well do it
                            for (uint32_t i = 0; i < currentMenu->itemCount; i++) {
                                if ((currentMenu->items[i].type == Menu_Enum) && (currentMenu->items[i].enumValue == configSettings.emulMode)) {
                                    currentItemIndex = i;
                                    break;
                                }
                            }
                        } else {
                            currentItemIndex = 0; // Otherwise start with the first item selected
                        }
                        if (selectedItem->onChange) {
                            // And then call onChange if it exists
                            selectedItem->onChange();
                        }
                        frameStartIndex = 0; // The first item in the menu is visible by default
                    }
                    break;
                case Menu_Toggle:
                    // If it's a toggle, then we just need to flip the boolean value
                    *(bool*)selectedItem->value = !(*(bool*)selectedItem->value);
                    if (selectedItem->onChange) {
                        selectedItem->onChange();
                    }
                    break;
                case Menu_Numeric:
                    // If it's a numeric, then we need to start editing it
                    editingItem = true;
                    // Don't call onChange here; the value hasn't actually changed yet
                    break;
                case Menu_Enum:
                    // If it's an enum, then we need to set the value to this item's enumValue
                    *(uint32_t*)selectedItem->value = selectedItem->enumValue;
                    if (selectedItem->onChange) {
                        selectedItem->onChange();
                    }
                    break;
                case Menu_Action:
                    // And if it's an action, then we just need to call the action function, if it exists
                    if (selectedItem->action) {
                        selectedItem->action();
                    }
                    break;
            }            
        }
        // Now mark the screen as dirty and in need of a redraw
        // We don't need to do just a particular region because timing isn't critical here; the emulator isn't running yet
        redrawWholeScreen();
    }
    // Next up, let's see if the LEFT (AKA the UP) button is being pressed
    if (buttonStates[0]) {
        // If so, then we once again need to do one of two different things
        if (editingItem) {
            // If we're currently editing a setting, then we need to decrement the value of the setting
            const MenuItem* selectedItem = &currentMenu->items[currentItemIndex];
            if (selectedItem->type == Menu_Numeric) {
                // If it's a numeric (the only thing that's editable), then we need to decrement the value by the step amount
                uint32_t* valuePtr = (uint32_t*)selectedItem->value;
                // Just make sure it doesn't go below the minimum value
                // This can also look like integer underflow, so check if the new value is greater than the max too
                if (((*valuePtr - selectedItem->stepAmount) > selectedItem->minValue) && !((*valuePtr - selectedItem->stepAmount) > selectedItem->maxValue)) {
                    *valuePtr -= selectedItem->stepAmount;
                } else {
                    *valuePtr = selectedItem->minValue;
                }
                // Call the item's onChange function if it exists
                if (selectedItem->onChange) {
                    selectedItem->onChange();
                }
            }
        } else {
            // Otherwise, we're not editing a setting, so we need to move the selection up in the menu
            if (currentItemIndex > 0) {
                currentItemIndex--;
            }
        }
        redrawWholeScreen();
    }
    // The final case is when the RIGHT (AKA the DOWN) button is being pressed
    if (buttonStates[2]) {
        // This is basically the same as the LEFT button case, but in reverse
        if (editingItem) {
            // Increment the numeric value of the currently-selected item if we're editing it
            const MenuItem* selectedItem = &currentMenu->items[currentItemIndex];
            if (selectedItem->type == Menu_Numeric) {
                uint32_t* valuePtr = (uint32_t*)selectedItem->value;
                if (((*valuePtr + selectedItem->stepAmount) < selectedItem->maxValue) && !((*valuePtr + selectedItem->stepAmount) < selectedItem->minValue)) {
                    *valuePtr += selectedItem->stepAmount;
                } else {
                    *valuePtr = selectedItem->maxValue;
                }
                // Call the item's onChange function if it exists
                if (selectedItem->onChange) {
                    selectedItem->onChange();
                }
            }
        } else {
            // Otherwise, move the selection down to the next menu item
            if (currentItemIndex + 1 < currentMenu->itemCount) {
                currentItemIndex++;
            }
        }
        redrawWholeScreen();
    }
    // One other thing here: if a button press just moved the selection to a different item that's off-screen, then we need to scroll
    // Technically we don't need to handle scrolling for the settings menu, but it'll be good practice for the file picker
    // So what we need to do is start drawing items starting at frameStartIndex and stop when we run out of space on the screen
    // But there's also the chance that the currently-selected item isn't visible on the screen, and we need to make sure it is
    // So if it's less than frameStartIndex or off the bottom of the screen, then adjust frameStartIndex so that it is visible
    // Start by doing the check for whether it's off the screen
    if (currentItemIndex < frameStartIndex) {
        // If it's above the top of the screen, then just set frameStartIndex to the current item index
        frameStartIndex = currentItemIndex;
    } else if (currentItemIndex >= (frameStartIndex + ((OLED.getDisplayHeight() - MENU_ITEM_HEIGHT) / MENU_ITEM_HEIGHT))) {
        // If it's below the bottom of the screen, then set frameStartIndex so that the current item is at the bottom of the screen
        frameStartIndex = currentItemIndex - ((OLED.getDisplayHeight() - MENU_ITEM_HEIGHT) / MENU_ITEM_HEIGHT) + 1;
    }
}

// This function formats the text for a menu item based on its type and value
void formatItemText(const MenuItem* item, char* buffer, uint32_t bufferSize) {
    switch (item->type) {
        case Menu_Submenu:
            // A submenu is just the label followed by "..."
            snprintf(buffer, bufferSize, "%s...", item->label);
            break;
        case Menu_Toggle:
            // A toggle is the label followed by ": On" or ": Off" depending on the boolean value
            snprintf(buffer, bufferSize, "%s: %s", item->label, (*(bool*)item->value) ? "On" : "Off");
            break;
        case Menu_Numeric:
            // A numeric is the label followed by ": " and the numeric value
            // The value is in brackets if it's being edited and just the plain value otherwise
            if (editingItem && item == &currentMenu->items[currentItemIndex]) {
                snprintf(buffer, bufferSize, "%s: <%u>", item->label, *(uint32_t*)item->value);
            } else {
                snprintf(buffer, bufferSize, "%s: %u", item->label, *(uint32_t*)item->value);
            }
            break;
        case Menu_Enum:
            // An enum is either an empty space or a dot followed by the label depending on whether the enum value matches this item's enumValue
            snprintf(buffer, bufferSize, "%s %s", (*(uint32_t*)item->value == item->enumValue) ? "*" : " ", item->label);
            break;
        case Menu_Action:
            // An action is perhaps the most boring one; it's just the label
            snprintf(buffer, bufferSize, "%s", item->label);
            break;
    }
}

// This function gets called to draw the settings screen on the OLED
void settingsDrawScreen() {
    // Drawing a menu might sound complex, but it really isn't too bad
    // Start by clearing the framebuffer so we can draw a fresh new screen
    OLED.clearBuffer();
    // And then draw the title of the current menu at the top of the screen
    // Make sure to center it horizontally too
    OLED.drawStr(((128 - OLED.getStrWidth(currentMenu->title)) / 2), (0 * MENU_ITEM_HEIGHT), currentMenu->title);
    // To denote that it's the title, draw a horizontal line underneath it
    OLED.drawHLine(0, MENU_ITEM_HEIGHT, 128);
    // Now we need to draw each of the items in the menu
    char buffer[40]; // Use this temp buffer to build the text for each item
    // Iterate through the items starting at frameStartIndex and draw them until we run out of space on the screen or run out of items
    for (uint32_t i = frameStartIndex; i < currentMenu->itemCount; i++) {
        // For each item, we need to figure out where it goes on the screen
        uint32_t yPosition = (i - frameStartIndex + 1) * MENU_ITEM_HEIGHT;
        if (yPosition + MENU_ITEM_HEIGHT > OLED.getDisplayHeight()) {
            // If the item would go off the bottom of the screen, then we need to stop drawing items
            break;
        }
        // Now we need to get the text for the item; the format of the text varies depending on the item type, so call formatItemText to get it
        formatItemText(&currentMenu->items[i], buffer, sizeof(buffer));
        // Now draw the item in either normal or inverse video depending on whether it's the currently-selected item
        if (i == currentItemIndex) {
            // Make the inverse video box the full width of the display so that it looks like a proper selection highlight
            drawMenuRow(yPosition, buffer, 0, true);
        } else {
            drawMenuRow(yPosition, buffer, 0, false);
        }
    }
}