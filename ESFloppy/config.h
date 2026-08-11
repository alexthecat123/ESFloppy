#include "types.h"

// Header file for the ESFloppy configuration setting routines

// Initializes the Preferences object to prep it for reading/writing settings
void initConfig();

// Reads the configuration settings from non-volatile storage and returns them in a ConfigSettings struct
// If the storage appears to be uninitialized, then it'll return a ConfigSettings struct with default values instead
ConfigSettings readConfig();

// Writes the given configuration settings to non-volatile storage
void writeConfig(const ConfigSettings& settings);

// Closes the Preferences object once we're done with it
void closeConfig();