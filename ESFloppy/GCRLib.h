// Header file for GCR encoding/decoding functions
#include "types.h"

// Lookup table for converting 6-bit nibbles to 8-bit GCR bytes; we need it here in the header so that the main program can use it
extern const uint8_t gcr_6to8[64];
// And the same thing for converting 8-bit GCR bytes back to 6-bit nibbles
extern const uint8_t gcr_8to6[256];

typedef uint8_t (*InterleaveTable)[22];

// We also need to make the interleave tables visible to the main program
// These cover both drive families: rows 8-12 are Sony and rows 15-22 are Twiggy, which fits in one table because the two ranges don't overlap
extern uint8_t interleave2to1[23][22];
extern uint8_t interleave4to1[23][22];
extern uint8_t interleave1to1[23][22];

// This function takes a decoded sector and encodes it into GCR format
void encodeSector(DecodedSector* decoded, GcrSector* gcr, DiskImageMetadata* metadata);

// And this one takes an encoded GCR sector and decodes it back into decoded format
// It returns true if the decoding was successful, or false if there was a checksum error or if any of the GCR bytes were invalid
bool decodeSector(GcrSector* gcr, DecodedSector* decoded, DiskImageMetadata* metadata);

// This function encodes an entire decoded track (all sectors) into GCR format
void encodeTrackToGCR(uint8_t track, DecodedSector decodedSectors[2][22], GcrSector gcrSectors[2][22], DiskImageMetadata* metadata);

// This function decodes an entire GCR track (all sectors) into decoded format
void decodeTrackFromGCR(uint8_t track, GcrSector gcrSectors[2][22], DecodedSector decodedSectors[2][22], DiskImageMetadata* metadata);

// Returns a pointer to the proper interleave table for the given disk image metadata
// The format parameter can come from either diskFormat in the metadata header (useMetadataFormat = true)
// Or from the uint8_t format parameter passed in (useMetadataFormat = false)
__attribute__((optimize("Ofast"))) IRAM_ATTR InterleaveTable getInterleaveTable(DiskImageMetadata* metadata, uint8_t format, bool useMetadataFormat);