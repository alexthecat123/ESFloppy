#include <Arduino.h>
#include "SdFat.h"
#include "types.h"

// Contains all the disk image handling routines for ESFloppy

// These two buffers will be used to hold raw data and tags straight from the image on the SD card
// Once it's in the buffers, then we can format it into our DecodedSector structs (or whatever)
// It's way quicker to read all the data from the card at once into a buffer than to do multiple small reads
static uint8_t rawDataBuffer[12800];
static uint8_t rawTagBuffer[1024];

extern SdCard* card; // A pointer to the SdFat card object from ESFloppy_Main.cpp, so we can use it in this file

// Calculates and returns the data checksum for a DC42 image
__attribute__((optimize("Ofast"))) uint32_t calcDataChecksum(File32* disk, DiskImageMetadata* metadata) {
    // The checksum algorithm is pretty darn simple: start with a 32-bit accumulator at 0
    // Then add each 16-bit BIG ENDIAN!!!! word of the data to the accumulator
    // And then rotate the accumulator right by 1 bit
    uint32_t dataChecksum = 0;
    // Now loop over all the sector data in 12288-byte chunks
    for (uint32_t offset = 0; offset < metadata->header.dataSize; offset += 12288) {
        // Figure out how many bytes we should actually read
        // It might be less than 12288 if we're on the last iteration and the data size isn't a multiple of 12288
        uint32_t bytesToRead = min((uint32_t)12288, metadata->header.dataSize - offset);
        // Now compute the offset into the image at which we should start reading, accounting for the 84-byte DC42 header
        uint32_t readOffset = sizeof(DC42Header) + offset;
        // Also calculate the data skew, which is the offset into the sector at which the data actually starts
        // This is once again necessary because of the header, which prevents the data from being aligned to a sector boundary
        // Data skew will always be 84, but we calculate it here for good practice since it HAS to be calculated for the tags
        uint32_t dataSkew = readOffset & 511;
        // Use that info to figure out how many sectors we need to read; it may be one more than you'd expect if the data doesn't start on a sector boundary
        uint32_t sectorCount = (dataSkew + bytesToRead + 511) >> 9;
        // And read in all the data, taking into account the header situation
        card->readSectors(metadata->startAddress + (readOffset >> 9), rawDataBuffer, sectorCount);
        // We've got the data, so now compute the checksum over it
        for (uint32_t i = 0; i < bytesToRead; i += 2) {
            // Combine two bytes into a big-endian word, taking the skew into account
            uint32_t word = (rawDataBuffer[dataSkew + i] << 8) | rawDataBuffer[dataSkew + i + 1];
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
            // This is the same idea as for the data checksum; start by figuring out our read offset
            uint32_t readOffset = sizeof(DC42Header) + metadata->header.dataSize + offset;
            // And how many bytes to read
            uint32_t bytesToRead = min((uint32_t)12288, metadata->header.tagSize - offset);
            // Compute the tag skew and sector count just like with the data
            // Except the tag skew will actually vary since tags aren't exactly 512 bytes long
            uint32_t tagSkew = readOffset & 511;
            uint32_t sectorCount = (tagSkew + bytesToRead + 511) >> 9;
            // And finally read the tags
            card->readSectors(metadata->startAddress + (readOffset >> 9), rawDataBuffer, sectorCount);
            // And compute the checksum over the tag data
            for (uint32_t i = 0; i < bytesToRead; i += 2) {
                uint32_t word = (rawDataBuffer[tagSkew + i] << 8) | rawDataBuffer[tagSkew + i + 1]; // Combine two bytes into a big-endian word
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
    if(!disk->open(filename, O_RDWR)){ // Try opening the specified disk image file
        Serial.printf("Failed to open image file %s!\n", filename);
        disk->close();
        return false; // Return false on failure
    }
    if (!disk->contiguousRange(&metadata->startAddress, &metadata->endAddress)) {
        Serial.println("Image file isn't contiguous!");
        disk->close();
        return false;
    }
    // If we succeed, we need to check if it's a DC42 image
    if(disk->fileSize() >= sizeof(DC42Header)) { // Make sure the file is big enough to contain a DC42 header
        // Now that I think about it, I'm not even sure why I do that check at all; the file has to be a minimum of 512 bytes because that's the SD card sector size...
        // Whatever, might as well leave it there because I don't feel like deleting it

        // We need to read the DC42 header from the image into our metadata struct, but we can't read it directly
        // This is because we're using raw SD reads/writes for speed instead of going through the FS, and you have to read a whole sector at a time when going raw
        // So we'll need to copy a sector into a buffer and then copy the header from that buffer into our metadata struct
        // Just use the rawDataBuffer since we don't need it for anything else at this point
        card->readSectors(metadata->startAddress, rawDataBuffer, 1); // Read the first sector of the image into the buffer
        memcpy(&metadata->header, rawDataBuffer, sizeof(DC42Header)); // And copy the header from the buffer into our struct

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
                    disk->close();
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
                    disk->close();
                    return false;
                }
            }
            else {
                Serial.printf("ERROR: Invalid disk encoding or data size for DC42 image; encoding is %02x and data size is %d bytes!\n", metadata->header.diskEncoding, metadata->header.dataSize);
                disk->close();
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
                disk->close();
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
                disk->close();
                return false; // So return false
            }
            // If we get here, it's a valid raw image
            metadata->imageType = RAW; // So set imageType to RAW to indicate raw
            metadata->tagsPresent = false; // And set tagsPresent to false since raw images can't have tags
        }
    }
    else { // The image is so small that it isn't even big enough to contain a DC42 header; it's definitely invalid
        Serial.printf("ERROR: Image size isn't a valid floppy disk size; it's only %d bytes!!!\n", disk->fileSize());
        disk->close();
        return false; // So return false
    }
    metadata->diskInserted = true; // Mark the disk as inserted
    return true; // And return true on success
}

// Reads an entire track (both sides if 800K disk) from the disk image into a DecodedSector array
__attribute__((optimize("Ofast"))) void readTrack(uint8_t track, File32* disk, DecodedSector sectors[2][12], DiskImageMetadata* metadata) {
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
    uint32_t dataSkew = 0;
    uint32_t tagSkew = 0;

    if (metadata->imageType == RAW) {
        readOffset = (totalSectorsBeforeTrack * 512);
        // Read all the sector data for this track into our raw data buffer at once
        card->readSectors(metadata->startAddress + totalSectorsBeforeTrack, rawDataBuffer, sectorsInTrack * sides);
    }
    // For DC42 images, we have to account for the header as well, and there might be a tag offset too
    else if (metadata->imageType == DC42) {
        readOffset = sizeof(DC42Header) + (totalSectorsBeforeTrack * 512);
        // Given the 84-byte header, we need to read in an extra sector's worth of data to make sure we get all the data for this track
        // So compute a sectorCount that gets the total number of sectors to read in, including the extra one
        // First compute a skew factor; this will always be 84 for the data, but varies for the tags, so might as well compute it for both
        dataSkew = readOffset & 511;
        uint32_t sectorCount = (dataSkew + (sectorsInTrack * sides * 512) + 511) >> 9;
        // And read in all the data, taking into account the header situation
        card->readSectors(metadata->startAddress + (readOffset >> 9), rawDataBuffer, sectorCount);
        // Check if there are tags present
        if (metadata->tagsPresent) {
            // And if so, compute the offset to the start of the tags
            readOffset = sizeof(DC42Header) + (DATA_SIZE_400K * sides) + (totalSectorsBeforeTrack * 12);
            // Don't forget about the skew too; this is where it may not be 84 because tags are only 12 bytes each and so it'll differ for each track
            tagSkew = readOffset & 511;
            // Do the same thing as before where we add an extra sector if necessary to make sure we get all the tag data for this track
            sectorCount = (tagSkew + (sectorsInTrack * sides * 12) + 511) >> 9;
            // And read in all the tags
            card->readSectors(metadata->startAddress + (readOffset >> 9), rawTagBuffer, sectorCount);
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
                uint32_t bufferOffset = (side * sectorsInTrack * 512) + (sector * 512);
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
                // If it's a DC42, then start by copying over the 512 data bytes just like for raw images, but account for the 84-byte header too
                uint32_t bufferOffset = dataSkew + (side * sectorsInTrack * 512) + (sector * 512);
                // Use memcpy instead of a loop for speed
                memcpy(&sectors[side][sector].data[12], &rawDataBuffer[bufferOffset], 512);
                // Now handle the tags if they're present
                if (metadata->tagsPresent) {
                    // If they are, then we need to read the 12-byte tag for this sector
                    // So compute the offset into the raw tag buffer; once again, remember the header skew
                    bufferOffset = tagSkew + (side * sectorsInTrack * 12) + (sector * 12);
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
}

// Writes an entire track (both sides if 800K disk) from a DecodedSector array back into the disk image
__attribute__((optimize("Ofast"))) void writeTrack(uint8_t track, File32* disk, DecodedSector sectors[2][12], DiskImageMetadata* metadata) {
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

    // Before we do anything else, we need to figure out where in the disk image this track is located
    // We need the skews up front because the copy loop below has to write the sector data into the buffer at the right spot
    // And we need the sector counts up front because of the read-modify-write below
    uint32_t writeOffset = 0; // Byte offset into the image of the start of this track's data
    uint32_t dataSkew = 0; // Offset into the first sector at which this track's data actually starts
    uint32_t dataSectorNumber = 0; // Absolute SD card sector number of the start of this track's data
    uint32_t dataSectorCount = 0; // How many SD card sectors that data spans
    uint32_t tagSkew = 0; // And the same three things for the tags
    uint32_t tagSectorNumber = 0;
    uint32_t tagSectorCount = 0;

    if (metadata->imageType == RAW) {
        // Raw images have no header, so the track's data starts exactly on a sector boundary and spans a whole number of sectors
        // That means no skew, and no read-modify-write needed either since we overwrite every byte of every sector we touch
        writeOffset = (totalSectorsBeforeTrack * 512);
        dataSectorNumber = metadata->startAddress + totalSectorsBeforeTrack;
        dataSectorCount = sectorsInTrack * sides;
    }
    else if (metadata->imageType == DC42) {
        // For DC42 images, the 84-byte header misaligns everything, so we have to compute a skew and read an extra sector, just like in readTrack
        writeOffset = sizeof(DC42Header) + (totalSectorsBeforeTrack * 512);
        dataSkew = writeOffset & 511;
        dataSectorNumber = metadata->startAddress + (writeOffset >> 9);
        dataSectorCount = (dataSkew + (sectorsInTrack * sides * 512) + 511) >> 9;
        // The problem with raw writes is that we can only write whole sectors at a time, but the track's data doesn't start on a sector boundary
        // This is thanks to the dc42 header in the case of the data, and the header combined with the tags only being 12 bytes each in the case of the tags
        // What this means is that we can't just overwrite a whole SD card sector with our new data because that would clobber the data from other tracks that's partially in that sector
        // So read those sectors in first, and then the copy loop below only overwrites the parts that belong to this track
        card->readSectors(dataSectorNumber, rawDataBuffer, dataSectorCount);
        // Now do exactly the same thing for the tags if the image has them
        if (metadata->tagsPresent) {
            writeOffset = sizeof(DC42Header) + (DATA_SIZE_400K * sides) + (totalSectorsBeforeTrack * 12);
            tagSkew = writeOffset & 511;
            tagSectorNumber = metadata->startAddress + (writeOffset >> 9);
            tagSectorCount = (tagSkew + (sectorsInTrack * sides * 12) + 511) >> 9;
            card->readSectors(tagSectorNumber, rawTagBuffer, tagSectorCount);
        }
    }

    // Now that the buffers hold the surrounding sectors, we can grab all the sector data and overlay it into the rawDataBuffer
    // And put the tags in the rawTagBuffer if applicable
    for (uint32_t sector = 0; sector < sectorsInTrack; sector++) {
        for (uint32_t side = 0; side < sides; side++) {
            // Again, handle raw and DC42 images differently
            if (metadata->imageType == RAW) {
                // Compute the offset the same way as in readTrack
                uint32_t bufferOffset = (side * sectorsInTrack * 512) + (sector * 512);
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
                uint32_t bufferOffset = dataSkew + (side * sectorsInTrack * 512) + (sector * 512);
                // And copy over the 512 bytes of sector data
                memcpy(&rawDataBuffer[bufferOffset], &sectors[side][sector].data[12], 512);
                // Now handle the tags if the image supports them
                if (metadata->tagsPresent) {
                    // If the DC42 image supports tags, compute the offset to the tag the same way as in readTrack
                    bufferOffset = tagSkew + (side * sectorsInTrack * 12) + (sector * 12);
                    // And copy the 12-byte tag from the start of the sector's data array into our raw tag buffer
                    memcpy(&rawTagBuffer[bufferOffset], &sectors[side][sector].data[0], 12);
                    // And we're done; no need to handle the "no tags" case since we don't have to write anything then
                }
            }
        }
    }

    // Now that we've got all the data and tags in our raw buffers, we can write them back to the disk image
    // All of the offset math was done earlier, so all we need to do now is just send out the data
    card->writeSectors(dataSectorNumber, rawDataBuffer, dataSectorCount);
    // And write the tags too if this image has them
    if (tagSectorCount > 0) {
        card->writeSectors(tagSectorNumber, rawTagBuffer, tagSectorCount);
    }
}

// Closes the disk image file, making sure to update the DC42 header if needed/applicable
void closeImage(File32* disk, DiskImageMetadata* metadata) {
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

        // Given that we're doing raw writes, we have to write a whole sector at a time, not just the 84 bytes of the header
        // So we need to read in the first sector from the SD card, update the header, and then write it back out
        // Might as well just reuse the rawDataBuffer for this
        card->readSectors(metadata->startAddress, rawDataBuffer, 1); // Read the first sector into the buffer
        memcpy(rawDataBuffer, &metadata->header, sizeof(DC42Header)); // Copy the updated header into the start of the buffer
        card->writeSectors(metadata->startAddress, rawDataBuffer, 1); // And then write it back out again
    }
    // And now actually close the image file and mark the disk as ejected; this is literally all we do here if it's a raw image
    metadata->diskInserted = false;
    disk->close();
}