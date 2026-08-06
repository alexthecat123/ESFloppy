#include "types.h"

// Header file for the routines that emulate the Sony drive's register-level interface with the Lisa's floppy controller

// The main register-acces loop for the Sony drive interface
__attribute__((optimize("Ofast"))) IRAM_ATTR void sonyLoop(volatile SdTaskInterface* sdTaskInterface, GcrSector trackBufferGCR[2][22], DiskImageMetadata* metadata[2]);