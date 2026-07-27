// Header file for GCR encoding/decoding functions
#include "types.h"

// Lookup table for converting 6-bit nibbles to 8-bit GCR bytes; we need it here in the header so that the main program can use it
extern const uint8_t gcr_6to8[64];
// And the same thing for converting 8-bit GCR bytes back to 6-bit nibbles
extern const uint8_t gcr_8to6[256];

// We also need to make the interleave tables visible to the main program
extern const uint8_t interleave2to1[13][12];
extern const uint8_t interleave4to1[13][12];

// This function takes a decoded sector and encodes it into GCR format
void encodeSector(DecodedSector* decoded, GcrSector* gcr);

// And this one takes an encoded GCR sector and decodes it back into decoded format
// It returns true if the decoding was successful, or false if there was a checksum error or if any of the GCR bytes were invalid
bool decodeSector(GcrSector* gcr, DecodedSector* decoded);

// This function encodes an entire decoded track (all sectors) into GCR format
void encodeTrackToGCR(uint8_t track, DecodedSector decodedSectors[2][12], GcrSector gcrSectors[2][12], DiskImageMetadata* metadata);

// This function decodes an entire GCR track (all sectors) into decoded format
void decodeTrackFromGCR(uint8_t track, GcrSector gcrSectors[2][12], DecodedSector decodedSectors[2][12], DiskImageMetadata* metadata);