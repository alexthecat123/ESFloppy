#include <Arduino.h>
#include "SdFat.h"
#include "types.h"

// Contains all the disk image handling routines for ESFloppy

// These two buffers will be used to hold raw data and tags straight from the image on the SD card
// Once it's in the buffers, then we can format it into our DecodedSector structs (or whatever)
// It's way quicker to read all the data from the card at once into a buffer than to do multiple small reads
static uint8_t rawDataBuffer[12288];
static uint8_t rawTagBuffer[288];

// Calculates and returns the data checksum for a DC42 image
__attribute__((optimize("Ofast"))) uint32_t calcDataChecksum(File32* disk, DiskImageMetadata* metadata) {
    // The checksum algorithm is pretty darn simple: start with a 32-bit accumulator at 0
    // Then add each 16-bit BIG ENDIAN!!!! word of the data to the accumulator
    // And then rotate the accumulator right by 1 bit
    uint32_t dataChecksum = 0;
    // Now loop over all the sector data in 12288-byte chunks
    for (uint32_t offset = 0; offset < metadata->header.dataSize; offset += 12288) {
        // Seek to the proper spot in the disk image
        disk->seekSet(sizeof(DC42Header) + offset);
        // And now figure out how many bytes we should actually read
        // It might be less than 12288 if we're on the last iteration and the data size isn't a multiple of 12288
        uint32_t bytesToRead = min((uint32_t)12288, metadata->header.dataSize - offset);
        // Now go ahead and read that many bytes into our buffer
        disk->read(rawDataBuffer, bytesToRead);
        // We've got the data, so now compute the checksum over it
        for (uint32_t i = 0; i < bytesToRead; i += 2) {
            uint32_t word = (rawDataBuffer[i] << 8) | rawDataBuffer[i + 1]; // Combine two bytes into a big-endian word
            dataChecksum += word; // Add the word to the checksum
            dataChecksum = (dataChecksum >> 1) | (dataChecksum << 31); // Rotate the checksum right by 1 bit
        }
    }
    return dataChecksum; // Finally, return the computed checksum
}

// Calculates and returns the tag checksum for a DC42 image
__attribute__((optimize("Ofast"))) uint32_t calcTagChecksum(File32* disk, DiskImageMetadata* metadata) {
    // The tag checksum is the same, just we iterate over the tag data instead of the regular data
    // And apparently there was a bug in the original DC42 implementation where the first 12 tag bytes were skipped in the checksum
    // For the sake of compatibility, that convention has been preserved ever since, and so we'll do it here too
    uint32_t tagChecksum = 0;
    // First, make sure the image actually has tags; if not, we just leave the tag checksum as 0
    if (metadata->tagsPresent) {
        // Note that we're starting the offset at 12 to skip the first 12 tag bytes
        for (uint32_t offset = 12; offset < metadata->header.tagSize; offset += 12288) {
            // Seek to the proper spot in the disk image
            disk->seekSet(sizeof(DC42Header) + metadata->header.dataSize + offset);
            // Figure out how many bytes to read just like before
            uint32_t bytesToRead = min((uint32_t)12288, metadata->header.tagSize - offset);
            // Read that many bytes into our buffer
            disk->read(rawDataBuffer, bytesToRead);
            // And compute the checksum over the tag data
            for (uint32_t i = 0; i < bytesToRead; i += 2) {
                uint32_t word = (rawDataBuffer[i] << 8) | rawDataBuffer[i + 1]; // Combine two bytes into a big-endian word
                tagChecksum += word; // Add the word to the checksum
                tagChecksum = (tagChecksum >> 1) | (tagChecksum << 31); // Rotate the checksum right by 1 bit
            }
        }
    }
    return tagChecksum; // Return our checksum
}

// This function opens a disk image file and determines its type (raw or DC42)
// If there are any errors, it returns false; otherwise, it returns true
bool openImage(char* filename, File32* disk, DiskImageMetadata* metadata) {
    uint32_t startTime = micros();
    if(!disk->open(filename, O_RDWR)){ // Try opening the specified disk image file
        Serial.printf("Failed to open image file %s!\n", filename);
        return false; // Return false on failure
    }
    // If we succeed, we need to check if it's a DC42 image
    if(disk->fileSize() >= sizeof(DC42Header)) { // Make sure the file is big enough to contain a DC42 header
        disk->seekSet(0); // Seek to the start of the file
        disk->read(&metadata->header, sizeof(DC42Header)); // And read the header into our struct

        // A problem: the fields in the header are big-endian, but the ESP32 is little-endian
        // So we need to swap the byte order of the multi-byte fields
        metadata->header.dataSize = __builtin_bswap32(metadata->header.dataSize);
        metadata->header.tagSize = __builtin_bswap32(metadata->header.tagSize);
        metadata->header.dataChecksum = __builtin_bswap32(metadata->header.dataChecksum);
        metadata->header.tagChecksum = __builtin_bswap32(metadata->header.tagChecksum);
        metadata->header.dc42Magic = __builtin_bswap16(metadata->header.dc42Magic);

        if(metadata->header.dc42Magic == 0x0100) { // If the magic number is 0x0100, then it's a DC42 image
            metadata->imageType = DC42; // Set imageType to DC42
            Serial.println("Opened DC42 disk image!");
            Serial.print("Volume Name: ");
            for(int i = 0; i < metadata->header.nameLength; i++){
                Serial.print(metadata->header.volumeName[i]);
            }
            Serial.println();
            Serial.printf("Data Size: %d\n", metadata->header.dataSize);
            Serial.printf("Tag Size: %d\n", metadata->header.tagSize);
            Serial.printf("Data Checksum: %08x\n", metadata->header.dataChecksum);
            Serial.printf("Tag Checksum: %08x\n", metadata->header.tagChecksum);
            Serial.printf("Disk Encoding: %d\n", metadata->header.diskEncoding);
            Serial.printf("Disk Format: %02x\n", metadata->header.diskFormat);
            if (metadata->header.diskEncoding == 0x00 && metadata->header.dataSize == DATA_SIZE_400K) {
                metadata->driveType = Drive400;
                Serial.println("Drive Type: 400K");
                if (metadata->header.tagSize == TAG_SIZE_400K) {
                    metadata->tagsPresent = true;
                    Serial.println("Tags are present in this image.");
                }
                else if (metadata->header.tagSize == 0) {
                    metadata->tagsPresent = false;
                    Serial.println("No tags are present in this image.");
                }
                else {
                    Serial.printf("ERROR: Invalid tag size for 400K DC42 image; tag size is %d bytes!\n", metadata->header.tagSize);
                    return false;
                }
            }
            else if (metadata->header.diskEncoding == 0x01 && metadata->header.dataSize == DATA_SIZE_800K) {
                metadata->driveType = Drive800;
                Serial.println("Drive Type: 800K");
                if (metadata->header.tagSize == TAG_SIZE_800K) {
                    metadata->tagsPresent = true;
                    Serial.println("Tags are present in this image.");
                }
                else if (metadata->header.tagSize == 0) {
                    metadata->tagsPresent = false;
                    Serial.println("No tags are present in this image.");
                }
                else {
                    Serial.printf("ERROR: Invalid tag size for 800K DC42 image; tag size is %d bytes!\n", metadata->header.tagSize);
                    return false;
                }
            }
            else {
                Serial.printf("ERROR: Invalid disk encoding or data size for DC42 image; encoding is %02x and data size is %d bytes!\n", metadata->header.diskEncoding, metadata->header.dataSize);
                return false;
            }

            // Now calculate and verify the data and tag checksums
            uint32_t calculatedDataChecksum = calcDataChecksum(disk, metadata);
            uint32_t calculatedTagChecksum = calcTagChecksum(disk, metadata);

            if (calculatedDataChecksum != metadata->header.dataChecksum) {
                Serial.printf("WARNING: Data checksum mismatch! Expected %08x but calculated %08x\n", metadata->header.dataChecksum, calculatedDataChecksum);
            }
            else {
                Serial.println("Data checksum is good.");
            }

            if (calculatedTagChecksum != metadata->header.tagChecksum) {
                Serial.printf("WARNING: Tag checksum mismatch! Expected %08x but calculated %08x\n", metadata->header.tagChecksum, calculatedTagChecksum);
            }
            else {
                Serial.println("Tag checksum is good.");
            }

        }
        else { // The image doesn't have the correct magic number; not a DC42
            // Check if it's a DART image by looking at the first two bytes; if the first byte is 1, 2, or 3 and the second byte is 16, 17, or 18 (decimal), then it's a DART image
            // We don't support DART images because they use compression, so we have to reject them
            if ((metadata->header.nameLength == 1 || metadata->header.nameLength == 2 || metadata->header.nameLength == 3) && (metadata->header.volumeName[0] == 16 || metadata->header.volumeName[0] == 17 || metadata->header.volumeName[0] == 18)) {
                Serial.println("ERROR: DART disk images are not supported!");
                return false;
            }
            // Otherwise, it's probably just a raw image
            // Before we assume that though, let's make sure it's a valid size for either a 400K or 800K disk
            if (disk->fileSize() == DATA_SIZE_400K) {
                metadata->driveType = Drive400; // Set driveType to 400K
                Serial.println("Opened raw 400K disk image!");
            }
            else if (disk->fileSize() == DATA_SIZE_800K) {
                metadata->driveType = Drive800; // Set driveType to 800K
                Serial.println("Opened raw 800K disk image!");
            }
            else {
                // If it's neither, then it's an invalid image size and we'll have to reject it
                Serial.printf("ERROR: Image size isn't a valid floppy disk size; it's %d bytes!!!\n", disk->fileSize());
                return false; // So return false
            }
            // If we get here, it's a valid raw image
            metadata->imageType = RAW; // So set imageType to RAW to indicate raw
            metadata->tagsPresent = false; // And set tagsPresent to false since raw images can't have tags
        }
    }
    else { // The image is so small that it isn't even big enough to contain a DC42 header; it's definitely invalid
        Serial.printf("ERROR: Image size isn't a valid floppy disk size; it's only %d bytes!!!\n", disk->fileSize());
        return false; // So return false
    }
    metadata->diskInserted = true; // Mark the disk as inserted
    Serial.printf("Opened disk image in %d microseconds.\n", micros() - startTime);
    return true; // And return true on success
}

// Initial time: 39296
// Time after reading track as big block: 10004

// Reads an entire track (both sides if 800K disk) from the disk image into a DecodedSector array
__attribute__((optimize("Ofast"))) void readTrack(uint8_t track, File32* disk, DecodedSector sectors[2][12], DiskImageMetadata* metadata) {
    uint32_t startTime = micros();

    // First, figure out if it's a single-sided or double-sided disk
    uint32_t sides = (metadata->driveType == Drive800) ? 2 : 1;

    // Use our LUT to find the number of sectors in the current track (8, 9, 10, 11, or 12)
    uint32_t sectorsInTrack = sectorsPerTrack[track];
    // Also count up the total number of sectors on the disk up to this track for offset calculations later
    uint32_t totalSectorsBeforeTrack = 0;
    // Make sure to account for both sides if it's an 800K disk
    for (uint32_t i = 0; i < sides; i++) {
        for (uint32_t j = 0; j < track; j++) {
            totalSectorsBeforeTrack += sectorsPerTrack[j];
        }
    }

    // Now compute the offset to the start of the track's data and tags in the disk image
    // For raw images, the data offset is just the total sectors before this track times 512
    // And there's no tag data at all
    uint32_t readOffset = 0;

    if (metadata->imageType == RAW) {
        readOffset = (totalSectorsBeforeTrack * 512);
        disk->seekSet(readOffset); // Seek to the computed offset
        // Now read all the sector data for this track into our raw data buffer at once
        disk->read(rawDataBuffer, sectorsInTrack * 512 * sides);
    }
    // For DC42 images, we have to account for the header as well, and there might be a tag offset too
    else if (metadata->imageType == DC42) {
        readOffset = sizeof(DC42Header) + (totalSectorsBeforeTrack * 512);
        disk->seekSet(readOffset); // Seek to the computed offset
        // And read in all the data
        disk->read(rawDataBuffer, sectorsInTrack * 512 * sides);
        // Check if there are tags present
        if (metadata->tagsPresent) {
            // And if so, compute the offset to the start of the tags
            readOffset = sizeof(DC42Header) + (DATA_SIZE_400K * sides) + (totalSectorsBeforeTrack * 12);
            disk->seekSet(readOffset); // Seek to the tag offset
            // And read in all the tags
            disk->read(rawTagBuffer, sectorsInTrack * 12 * sides);
        }
    }

    for (uint32_t sector = 0; sector < sectorsInTrack; sector++) {
        for (uint32_t side = 0; side < sides; side++) {
            // Handle raw and DC42 images differently
            if (metadata->imageType == RAW) {
                // The first 12 bytes of each sector are the tag, which raw images don't have, so we just zero them out
                for (int i = 0; i < 12; i++) {
                    sectors[side][sector].data[i] = 0; // Zero out the tag bytes
                }
                // Now copy the 512 bytes of data from our raw data buffer into the sector's data array starting at position 12
                // The offset into the raw data buffer is computed based on the side, and current sector number from the for loop
                uint32_t bufferOffset = (sector * sides * 512) + (side * 512);
                // Use memcpy instead of a loop for speed
                memcpy(&sectors[side][sector].data[12], &rawDataBuffer[bufferOffset], 512);
                // Now set the other fields of our DecodedSector struct
                sectors[side][sector].track = track;
                sectors[side][sector].side = side;
                sectors[side][sector].sector = sector;
                // Unlike with DC42 images, we don't have a format byte stored anywhere, so just use context clues to set it as best we can
                // One thing that's for sure: we can set the side count bit based on whether it's an 800K or 400K disk
                sectors[side][sector].format = (metadata->driveType == Drive800) ? 0b00100000 : 0b00000000; // Bit 5 is side count
                // For the interleave factor, just assume it's the most common value of 2:1 interleave
                sectors[side][sector].format |= 0x02; // Bits [4:0] are interleave factor
            }
            else if (metadata->imageType == DC42) {
                // If it's a DC42, then start by copying over the 512 data bytes just like for raw images
                uint32_t bufferOffset = (sector * sides * 512) + (side * 512);
                // Use memcpy instead of a loop for speed
                memcpy(&sectors[side][sector].data[12], &rawDataBuffer[bufferOffset], 512);
                // Now handle the tags if they're present
                if (metadata->tagsPresent) {
                    // If they are, then we need to read the 12-byte tag for this sector
                    // So compute the offset into the raw tag buffer
                    bufferOffset = (sector * sides * 12) + (side * 12);
                    // And copy the 12 bytes from the raw tag buffer into the start of the sector's data array
                    memcpy(&sectors[side][sector].data[0], &rawTagBuffer[bufferOffset], 12);
                }
                else {
                    // If there aren't any tags, just zero out the first 12 bytes of the sector's data array like for the raw image
                    for (int i = 0; i < 12; i++) {
                        sectors[side][sector].data[i] = 0;
                    }
                }
                // Now set the other fields of our DecodedSector struct
                sectors[side][sector].track = track;
                sectors[side][sector].side = side;
                sectors[side][sector].sector = sector;
                // Unlike with raw images, we have the format byte stored in the DC42 header, so no need to do any guesswork there
                // Just pull it straight over
                sectors[side][sector].format = metadata->header.diskFormat;
            }
        }
    }
    Serial.printf("Read track %d in %d microseconds.\n", track, micros() - startTime);
}

// Writes an entire track (both sides if 800K disk) from a DecodedSector array back into the disk image
__attribute__((optimize("Ofast"))) void writeTrack(uint8_t track, File32* disk, DecodedSector sectors[2][12], DiskImageMetadata* metadata) {
    uint32_t startTime = micros();
    // This is going to be pretty darn similar to readTrack, but in reverse
    // First, figure out if it's a single-sided or double-sided disk
    uint32_t sides = (metadata->driveType == Drive800) ? 2 : 1;

    // Use our LUT to find the number of sectors in the current track (8, 9, 10, 11, or 12)
    uint32_t sectorsInTrack = sectorsPerTrack[track];
    // Also count up the total number of sectors on the disk up to this track for offset calculations later
    uint32_t totalSectorsBeforeTrack = 0;
    // Make sure to account for both sides if it's an 800K disk
    for (uint32_t i = 0; i < sides; i++) {
        for (uint32_t j = 0; j < track; j++) {
            totalSectorsBeforeTrack += sectorsPerTrack[j];
        }
    }

    // Before we write anything to disk, we need to grab all the sector data and put it in the rawDataBuffer
    // And put the tags in the rawTagBuffer if applicable
    for (uint32_t sector = 0; sector < sectorsInTrack; sector++) {
        for (uint32_t side = 0; side < sides; side++) {
            // Again, handle raw and DC42 images differently
            if (metadata->imageType == RAW) {
                // Compute the offset the same way as in readTrack
                uint32_t bufferOffset = (sector * sides * 512) + (side * 512);
                // And copy the 512 bytes of data from the sector's data array starting at position 12 into our raw data buffer
                memcpy(&rawDataBuffer[bufferOffset], &sectors[side][sector].data[12], 512);
                // For raw images, we don't have any tags to write, so we can skip that part
            }
            // If it's not raw, it must be DC42
            else if (metadata->imageType == DC42) {
                // First, let's update the diskFormat byte in the DC42 header to match the format byte of this sector
                // This assumes that all sectors on the disk have the same format byte, but that's a very reasonable assumption
                // Most likely, it'll be the same format byte we read when we opened the image, but there's a chance the host formatted the disk differently while in use
                // This is a bit inefficient since we're doing it for every sector, but there are only 12-24 sectors max per track
                metadata->header.diskFormat = sectors[side][sector].format;
                // Now find the data offset just like before
                uint32_t bufferOffset = (sector * sides * 512) + (side * 512);
                // And copy over the 512 bytes of sector data
                memcpy(&rawDataBuffer[bufferOffset], &sectors[side][sector].data[12], 512);
                // Now handle the tags if the image supports them
                if (metadata->tagsPresent) {
                    // If the DC42 image supports tags, compute the offset to the tag the same way as in readTrack
                    bufferOffset = (sector * sides * 12) + (side * 12);
                    // And copy the 12-byte tag from the start of the sector's data array into our raw tag buffer
                    memcpy(&rawTagBuffer[bufferOffset], &sectors[side][sector].data[0], 12);
                    // And we're done; no need to handle the "no tags" case since we don't have to write anything then
                }
            }
        }
    }

    // Now that we've got all the data and tags in our raw buffers, we can write them back to the disk image
    // First, compute the offset to the start of the track's data in the disk image
    uint32_t writeOffset = 0;
    // This is going to be different for raw and DC42 images
    if (metadata->imageType == RAW) {
        writeOffset = (totalSectorsBeforeTrack * 512);
        disk->seekSet(writeOffset); // Seek to the computed offset
        // Now write all the sector data for this track from our raw data buffer at once
        disk->write(rawDataBuffer, sectorsInTrack * 512 * sides);
    }
    else if (metadata->imageType == DC42) {
        writeOffset = sizeof(DC42Header) + (totalSectorsBeforeTrack * 512);
        disk->seekSet(writeOffset); // Seek to the computed offset
        // And write all the data
        disk->write(rawDataBuffer, sectorsInTrack * 512 * sides);
        // Check if there are tags present
        if (metadata->tagsPresent) {
            // And if so, compute the offset to the start of the tags
            writeOffset = sizeof(DC42Header) + (DATA_SIZE_400K * sides) + (totalSectorsBeforeTrack * 12);
            disk->seekSet(writeOffset); // Seek to the tag offset
            // And write all the tags
            disk->write(rawTagBuffer, sectorsInTrack * 12 * sides);
        }
    }
    Serial.printf("Wrote track %d in %d microseconds.\n", track, micros() - startTime);
}

// Closes the disk image file, making sure to update the DC42 header if needed/applicable
void closeImage(File32* disk, DiskImageMetadata* metadata) {
    uint32_t startTime = micros();
    // If it's a DC42 image, we need to update the header before closing
    if (metadata->imageType == DC42) {
        // All we need to do here is update the data and tag checksums in the DC42 header
        uint32_t dataChecksum = calcDataChecksum(disk, metadata);
        uint32_t tagChecksum = calcTagChecksum(disk, metadata);
        
        // Now that we've computed both checksums, update the DC42 header struct
        metadata->header.dataChecksum = dataChecksum;
        metadata->header.tagChecksum = tagChecksum;

        // Swap the byte order of the header back to big-endian for writing to the file
        metadata->header.dataSize = __builtin_bswap32(metadata->header.dataSize);
        metadata->header.tagSize = __builtin_bswap32(metadata->header.tagSize);
        metadata->header.dataChecksum = __builtin_bswap32(metadata->header.dataChecksum);
        metadata->header.tagChecksum = __builtin_bswap32(metadata->header.tagChecksum);
        metadata->header.dc42Magic = __builtin_bswap16(metadata->header.dc42Magic);

        // And seek back to the start of the file and write the updated header back to the disk image
        disk->seekSet(0);
        disk->write(&metadata->header, sizeof(DC42Header));
    }
    // And now actually close the image file and mark the disk as ejected; this is literally all we do here if it's a raw image
    metadata->diskInserted = false;
    disk->close();
    Serial.printf("Closed disk image in %d microseconds.\n", micros() - startTime);
}