#include "types.h"

// Header file for the routines that emulate the Twiggy drive's register-level interface with the Lisa's floppy controller

// Ejects the disk in the specified Twiggy drive
__attribute__((optimize("Ofast"))) IRAM_ATTR void ejectDisk(uint32_t drive, volatile SdTaskInterface* sdTaskInterface, DiskImageMetadata* metadata);

// The main register-acces loop for the Twiggy drive interface
__attribute__((optimize("Ofast"))) IRAM_ATTR void twiggyLoop(volatile SdTaskInterface* sdTaskInterface, GcrSector trackBufferGCR[2][22], DiskImageMetadata* metadata[2]);