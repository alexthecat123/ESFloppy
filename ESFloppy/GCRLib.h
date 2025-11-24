// Header file for GCR encoding/decoding functions
#include "types.h"

// This function takes a decoded sector and encodes it into GCR format
void encodeSector(DecodedSector* decoded, GcrSector* gcr);

// And this one takes an encoded GCR sector and decodes it back into decoded format
void decodeSector(GcrSector* gcr, DecodedSector* decoded);

// This function encodes an entire decoded track (all sectors) into GCR format
void encodeTrackToGCR(uint8_t track, DecodedSector decodedSectors[2][12], GcrSector gcrSectors[2][12], DiskImageMetadata* metadata);

// This function decodes an entire GCR track (all sectors) into decoded format
void decodeTrackFromGCR(uint8_t track, GcrSector gcrSectors[2][12], DecodedSector decodedSectors[2][12], DiskImageMetadata* metadata);