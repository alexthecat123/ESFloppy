#include "types.h"

// Header file for the routines for reading and writing flux data to and from the Lisa's floppy drive interface

// This function gets called whenever the Lisa tries to read data from the floppy drive
// It checks if it's time to send out a new flux transition, and if so, it bit-bangs it out on the RDA pin
// For the sake of speed, we don't return from here until the Lisa stops reading from the drive
// We need speed here, so make sure to stick it in IRAM and optimize it as much as possible
__attribute__((optimize("Ofast"))) IRAM_ATTR void transmitTrack(GcrSector trackBufferGCR[2][22], volatile SdTaskInterface *sdTaskInterface, TrackParams* trackParams, BufferStatus* bufferStatus, DiskImageMetadata* metadata);

// This helper function gets called by receiveSector to interpret the raw data coming from the Lisa, extract prologues, and stick everything into the data buffer
// It returns true once the full header or sector data has been received, and false if we need to keep receiving more data
// It also modifies writeState to indicate to the caller whether we're still in the prologue, reading the header, or reading the data
__attribute__((optimize("Ofast"))) IRAM_ATTR bool processRawWriteData(uint32_t rawInputData, uint8_t* dataBuffer, WriteState& writeState, bool newReception);

// This function gets called whenever the Lisa pulls WRQ low and starts writing data to the disk
// Note that it's called receiveSector instead of receiveTrack because the Lisa only writes one sector at a time
// This makes our life a LOT easier
__attribute__((optimize("Ofast"))) IRAM_ATTR void receiveSector(GcrSector trackBufferGCR[2][22], volatile SdTaskInterface *sdTaskInterface, TrackParams* trackParams, BufferStatus* bufferStatus, DiskImageMetadata* metadata);
