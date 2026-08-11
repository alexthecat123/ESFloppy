#pragma once

#include <Arduino.h>

// Header file containing all of the types related to the UI state

#define MENU_ITEM_HEIGHT 8 // The height of a single menu item in pixels

// This struct represents a screen in the UI
struct Screen {
    void (*enter)(void); // Pointer to the function that gets called when we enter this screen
    void (*tick)(uint32_t currentTime); // Pointer to the "periodic update" function that gets called while we're on this screen
    void (*buttonPress)(bool buttonStates[3]); // Pointer to the function that gets called when a button is pressed while we're on this screen
    void (*drawScreen)(void); // Pointer to the function that gets called to draw this screen
};

// This enum represents the different types of menu items that can appear in scrollable menus
enum MenuItemType {
    Menu_Submenu, // A menu item that opens a submenu
    Menu_Toggle, // A menu item that toggles a boolean setting
    Menu_Numeric, // A menu item that allows the user to select a numeric value
    Menu_Enum, // A menu item that allows the user to select from a list of enumerated values
    Menu_Action // A menu item that performs an action (executes a function) when selected
};

struct Menu;

// This struct represents a single item in a scrollable menu
struct MenuItem {
    const char* label; // The text label for this item
    MenuItemType type; // The type of this item
    void* value; // The boolean value, numeric value, or enum value that this item actually sets for Menu_Toggle, Menu_Numeric, and Menu_Enum
    uint32_t minValue; // The minimum value for Menu_Numeric items
    uint32_t maxValue; // The maximum value for Menu_Numeric items
    uint32_t stepAmount; // The amount to increment/decrement for Menu_Numeric items
    uint8_t enumValue; // The value that this item represents for Menu_Enum items
    const Menu* submenu; // The submenu that this item opens for Menu_Submenu items
    void (*onChange)(void); // The function that gets called when this item is changed for anything that's not a Menu_Action
    void (*action)(void); // The function that gets called when this item is selected for Menu_Action items
};

// This struct represents a full menu
struct Menu {
    const char* title; // The title of the menu
    const MenuItem* items; // The array of items in the menu
    uint32_t itemCount; // The number of items in the menu
};