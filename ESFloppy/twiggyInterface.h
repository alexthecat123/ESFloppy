#include "types.h"

// Header file for the routines that emulate the Twiggy drive's register-level interface with the Lisa's floppy controller

// The main register-acces loop for the Twiggy drive interface
__attribute__((optimize("Ofast"))) IRAM_ATTR void twiggyLoop(volatile SdTaskInterface* sdTaskInterface, GcrSector trackBufferGCR[2][22], DiskImageMetadata* metadata);