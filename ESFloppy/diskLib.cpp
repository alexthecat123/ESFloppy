#include <Arduino.h>
#include "SdFat.h"
#include "types.h"

// Contains all the disk image handling routines for ESFloppy

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
                    Serial.println("ERROR: Invalid tag size for 400K DC42 image!");
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
                    Serial.println("ERROR: Invalid tag size for 800K DC42 image!");
                    return false;
                }
            }
            else {
                Serial.println("ERROR: Invalid disk encoding or data size for DC42 image!");
                return false;
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

// Reads an entire track (both sides if 800K disk) from the disk image into a DecodedSector array
void readTrack(uint8_t track, File32* disk, DecodedSector sectors[2][12], DiskImageMetadata* metadata) {
    uint32_t startTime = micros();
    // Use our LUT to find the number of sectors in the current track (8, 9, 10, 11, or 12)
    uint32_t sectorsInTrack = sectorsPerTrack[track];
    // Also count up the total number of sectors on the disk up to this track for offset calculations later
    uint32_t totalSectorsBeforeTrack = 0;
    for (uint32_t i = 0; i < track; i++) {
        totalSectorsBeforeTrack += sectorsPerTrack[i];
    }
    // Then figure out if it's a single-sided or double-sided disk
    uint32_t sides = (metadata->driveType == Drive800) ? 2 : 1;
    // Now we can loop through each side and each sector to read them all in
    for (uint32_t side = 0; side < sides; side++) {
        for (uint32_t sector = 0; sector < sectorsInTrack; sector++) {
            // The disk image could be either raw or DC42, and we need to read it differently depending on which it is
            if (metadata->imageType == RAW) {
                // For raw images, we just need to compute the offset based on track, side, and sector
                // I'm operating under the assumption that all the sectors of side 0 come first, then all the sectors of side 1
                // Hopefully this is right; I can't find any documentation on this for raw images or DC42 images
                // We take the side and multiply it by the size of a single-sided tagless image to get to the start of that side's data
                // And then add the total number of sectors before this track times 512 (sector size) to get to the start of this track
                // And finally, add the sector number times 512 to get to the start of this sector
                uint32_t readOffset = (side * DATA_SIZE_400K) + (totalSectorsBeforeTrack * 512) + (sector * 512);
                disk->seekSet(readOffset); // Seek to the computed offset
                // Now we need to read the data into sectors[side][sector].data, except start reading it into position 12
                // The first 12 bytes of each sector are the tag, which raw images don't have, so we just zero them out
                for (int i = 0; i < 12; i++) {
                    sectors[side][sector].data[i] = 0; // Zero out the tag bytes
                }
                disk->read(&sectors[side][sector].data[12], 512); // Then read the 512 bytes of data into the rest of the sector
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
                // If it's a DC42, then we've got a little more to do, given that there's a header and possibly tags
                // First, compute the offset to the start of the (data) sector we want to read, accounting for the header size
                uint32_t readOffset = sizeof(DC42Header) + (side * DATA_SIZE_400K) + (totalSectorsBeforeTrack * 512) + (sector * 512);
                // Then seek to that offset in the file and read it into the sector's data array, starting at position 12
                disk->seekSet(readOffset); // Seek to the computed offset
                disk->read(&sectors[side][sector].data[12], 512);
                // Now handle the tags if they're present
                if (metadata->tagsPresent) {
                    // If they are, then we need to read the 12-byte tag for this sector
                    // So compute the offset to the start of the tag; the tag data comes right after all the regular data in the DC42 file
                    readOffset = sizeof(DC42Header) + (sides * DATA_SIZE_400K) + (totalSectorsBeforeTrack * 12) + (sector * 12);
                    disk->seekSet(readOffset); // Seek to the computed offset
                    disk->read(&sectors[side][sector].data[0], 12); // And read the 12-byte tag into the start of the sector's data array
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
void writeTrack(uint8_t track, File32* disk, DecodedSector sectors[2][12], DiskImageMetadata* metadata) {
    uint32_t startTime = micros();
    // This is going to be pretty darn similar to readTrack, but in reverse
    // First, use our LUT to find the number of sectors in the current track
    uint32_t sectorsInTrack = sectorsPerTrack[track];
    // Also count up the total number of sectors on the disk up to this track for offset calculations later
    uint32_t totalSectorsBeforeTrack = 0;
    for (uint32_t i = 0; i < track; i++) {
        totalSectorsBeforeTrack += sectorsPerTrack[i];
    }
    // Then figure out if it's a single-sided or double-sided disk
    uint32_t sides = (metadata->driveType == Drive800) ? 2 : 1;
    // Now loop through the sides and sectors just like in readTrack
    for (uint32_t side = 0; side < sides; side++) {
        for (uint32_t sector = 0; sector < sectorsInTrack; sector++) {
            // Again, handle raw and DC42 images differently
            if (metadata->imageType == RAW) {
                // Compute the offset the same way as in readTrack
                uint32_t writeOffset = (side * DATA_SIZE_400K) + (totalSectorsBeforeTrack * 512) + (sector * 512);
                disk->seekSet(writeOffset);
                // There's no tags to write for raw images, so just write the 512 bytes of data starting at position 12
                // And then we're done
                disk->write(&sectors[side][sector].data[12], 512);
            }
            // If it's not raw, it must be DC42
            else if (metadata->imageType == DC42) {
                // First, let's update the diskFormat byte in the DC42 header to match the format byte of this sector
                // This assumes that all sectors on the disk have the same format byte, but that's a very reasonable assumption
                // Most likely, it'll be the same format byte we read when we opened the image, but there's a chance the host formatted the disk differently while in use
                // This is a bit inefficient since we're doing it for every sector, but there are only 12-24 sectors max per track
                metadata->header.diskFormat = sectors[side][sector].format;
                // Compute the offset to the data sector the same way as in readTrack for DC42s
                uint32_t writeOffset = sizeof(DC42Header) + (side * DATA_SIZE_400K) + (totalSectorsBeforeTrack * 512) + (sector * 512);
                disk->seekSet(writeOffset);
                // Write the 512 bytes of data starting at position 12
                disk->write(&sectors[side][sector].data[12], 512);
                // Now handle the tags if the image supports them
                if (metadata->tagsPresent) {
                    // If the DC42 image supports tags, compute the offset to the tag the same way as in readTrack
                    writeOffset = sizeof(DC42Header) + (sides * DATA_SIZE_400K) + (totalSectorsBeforeTrack * 12) + (sector * 12);
                    disk->seekSet(writeOffset);
                    // Write the 12-byte tag from the start of the sector's data array to the disk image
                    // And we're done; no need to handle the "no tags" case since we don't have to write anything then
                    disk->write(&sectors[side][sector].data[0], 12);
                }
            }
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
        // The checksum algorithm is pretty darn simple: start with a 32-bit accumulator at 0
        // Then add each 16-bit BIG ENDIAN!!!! word of the data to the accumulator
        // And then rotate the accumulator right by 1 bit
        uint8_t buffer[1024]; // Buffer to hold the sector data as we read it
        uint32_t dataChecksum = 0;
        // Now loop over all the sector data in 1024-byte chunks
        for (uint32_t offset = 0; offset < metadata->header.dataSize; offset += 1024) {
            // Seek to the proper spot in the disk image
            disk->seekSet(sizeof(DC42Header) + offset);
            // And now figure out how many bytes we should actually read
            // It might be less than 1024 if we're on the last iteration and the data size isn't a multiple of 1024
            uint32_t bytesToRead = min((uint32_t)1024, metadata->header.dataSize - offset);
            // Now go ahead and read that many bytes into our buffer
            disk->read(buffer, bytesToRead);
            // We've got the data, so now compute the checksum over it
            for (uint32_t i = 0; i < bytesToRead; i += 2) {
                uint32_t word = (buffer[i] << 8) | buffer[i + 1]; // Combine two bytes into a big-endian word
                dataChecksum += word; // Add the word to the checksum
                dataChecksum = (dataChecksum >> 1) | (dataChecksum << 31); // Rotate the checksum right by 1 bit
            }
        }
        // The tag checksum is the same, just we iterate over the tag data instead of the regular data
        // And apparently there was a bug in the original DC42 implementation where the first 12 tag bytes were skipped in the checksum
        // For the sake of compatibility, that convention has been preserved ever since, and so we'll do it here too
        uint32_t tagChecksum = 0;
        // First, make sure the image actually has tags; if not, we just leave the tag checksum as 0
        if (metadata->tagsPresent) {
            // Note that we're starting the offset at 12 to skip the first 12 tag bytes
            for (uint32_t offset = 12; offset < metadata->header.tagSize; offset += 1024) {
                // Seek to the proper spot in the disk image
                disk->seekSet(sizeof(DC42Header) + metadata->header.dataSize + offset);
                // Figure out how many bytes to read just like before
                uint32_t bytesToRead = min((uint32_t)1024, metadata->header.tagSize - offset);
                // Read that many bytes into our buffer
                disk->read(buffer, bytesToRead);
                // And compute the checksum over the tag data
                for (uint32_t i = 0; i < bytesToRead; i += 2) {
                    uint32_t word = (buffer[i] << 8) | buffer[i + 1]; // Combine two bytes into a big-endian word
                    tagChecksum += word; // Add the word to the checksum
                    tagChecksum = (tagChecksum >> 1) | (tagChecksum << 31); // Rotate the checksum right by 1 bit
                }
            }
        }
        // Now that we've computed both checksums, update the DC42 header struct
        metadata->header.dataChecksum = dataChecksum;
        metadata->header.tagChecksum = tagChecksum;
        // Now seek back to the start of the file and write the updated header back to the disk image
        disk->seekSet(0);
        disk->write(&metadata->header, sizeof(DC42Header));
    }
    // And now actually close the image file and mark the disk as ejected; this is literally all we do here if it's a raw image
    metadata->diskInserted = false;
    disk->close();
    Serial.printf("Closed disk image in %d microseconds.\n", micros() - startTime);
}