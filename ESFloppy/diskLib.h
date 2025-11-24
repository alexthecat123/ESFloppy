// Header file for disk image file handling functions
#include "types.h"

// Calculates and returns the data checksum for a DC42 image
__attribute__((optimize("Ofast"))) uint32_t calcDataChecksum(File32* disk, DiskImageMetadata* metadata);

// Calculates and returns the tag checksum for a DC42 image
__attribute__((optimize("Ofast"))) uint32_t calcTagChecksum(File32* disk, DiskImageMetadata* metadata);

// This function opens a disk image file and determines its type (raw or DC42)
// If there are any errors, it returns false; otherwise, it returns true
bool openImage(char* filename, File32* disk, DiskImageMetadata* metadata);

// Reads an entire track (both sides if 800K disk) from the disk image into a DecodedSector array
__attribute__((optimize("Ofast"))) void readTrack(uint8_t track, File32* disk, DecodedSector sectors[2][12], DiskImageMetadata* metadata);

// Writes an entire track (both sides if 800K disk) from a DecodedSector array back into the disk image
__attribute__((optimize("Ofast"))) void writeTrack(uint8_t track, File32* disk, DecodedSector sectors[2][12], DiskImageMetadata* metadata);

// Closes the disk image file, making sure to update the DC42 header if needed/applicable
void closeImage(File32* disk, DiskImageMetadata* metadata);