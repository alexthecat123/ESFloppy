#include <Arduino.h>
#include "SdFat.h"
#include "types.h"

// Contains all the disk image handling routines for ESFloppy

// These two buffers will be used to hold raw data and tags straight from the image on the SD card
// Once it's in the buffers, then we can format it into our DecodedSector structs (or whatever)
// It's way quicker to read all the data from the card at once into a buffer than to do multiple small reads
static uint8_t rawDataBuffer[23552];
static uint8_t rawTagBuffer[2048];

// The last track we read into the buffers; used to avoid unnecessary reads in writeTrack()
// Note that we don't have one per drive because the buffers are shared between both drives
static uint32_t lastReadTrack = 0xFFFFFFFF;
// The last drive that we read a track for; used in conjunction with lastReadTrack in multi-drive systems
// You can't just rely on lastReadTrack alone because it could be the same track number but on a different drive
// Twiggy drive 0 is the upper drive and 1 is the lower drive; for Sony drives this will always be 1
static uint32_t lastReadDrive = 0x01;

extern SdCard* card; // A pointer to the SdFat card object from ESFloppy_Main.cpp, so we can use it in this file

// Calculates and returns the data checksum for a DC42 image
__attribute__((optimize("Ofast"))) uint32_t calcDataChecksum(File32* disk, DiskImageMetadata* metadata) {
    // calcDataChecksum clobbers the rawDataBuffer, so set lastReadTrack to an invalid value so that writeTrack() will read the data sectors again
    lastReadTrack = 0xFFFFFFFF;
    lastReadDrive = metadata->driveIndex; // lastReadDrive just gets set to whatever the current drive is
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
    // calcTagChecksum clobbers the rawTagBuffer, so set lastReadTrack to an invalid value so that writeTrack() will read the tag sectors again
    lastReadTrack = 0xFFFFFFFF;
    lastReadDrive = metadata->driveIndex; // Set lastReadDrive too to mark this down for the appropriate drive
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
// If there are any errors, it returns what the error was; otherwise, it returns true
OpenResult openImage(char* filename, File32* disk, DiskImageMetadata* metadata) {
    // openImage clobbers the rawDataBuffer, so set lastReadTrack to an invalid value so that writeTrack() will read the data sectors again
    lastReadTrack = 0xFFFFFFFF;
    lastReadDrive = metadata->driveIndex; // And set lastReadDrive to the current drive
    metadata->diskInserted = false; // Mark the disk as not inserted until we successfully open it
    if(!disk->open(filename, O_RDWR)){ // Try opening the specified disk image file
        Serial.printf("Failed to open image file %s!\n", filename);
        disk->close();
        return ResultFailedOpen;
    }
    if (!disk->contiguousRange(&metadata->startAddress, &metadata->endAddress)) {
        Serial.println("Image file isn't contiguous!");
        disk->close();
        return ResultNotContiguous;
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
            // And print out a ton of info about the image just for fun
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
            // Now see if it's a 400K or 800K image based on the disk encoding and data size
            if (metadata->header.diskEncoding == DC42_400K_ENCODING && metadata->header.dataSize == DATA_SIZE_400K) {
                metadata->driveType = Drive400; // If it's 400K, then set the drive type to 400K
                Serial.println("Drive Type: 400K");
                // And then check to see if the tag size is valid for a 400K image; it should be either 0 or TAG_SIZE_400K bytes
                if (metadata->header.tagSize == TAG_SIZE_400K) {
                    metadata->tagsPresent = true;
                    Serial.println("Tags are present in this image.");
                }
                else if (metadata->header.tagSize == 0) {
                    metadata->tagsPresent = false;
                    Serial.println("No tags are present in this image.");
                }
                // If it's neither of those, then it's an invalid tag size and we need to reject the image
                else {
                    Serial.printf("ERROR: Invalid tag size for 400K DC42 image; tag size is %d bytes!\n", metadata->header.tagSize);
                    disk->close();
                    return ResultInvalidTagSize;
                }
            }
            // Now repeat that whole process for 800K images
            else if (metadata->header.diskEncoding == DC42_800K_ENCODING && metadata->header.dataSize == DATA_SIZE_800K) {
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
                    return ResultInvalidTagSize;
                }
            }
            // And finally one more time for Twiggy images
            else if (metadata->header.diskEncoding == DC42_TWIGGY_ENCODING && metadata->header.dataSize == DATA_SIZE_TWIGGY) {
                metadata->driveType = DriveTwiggy;
                Serial.println("Drive Type: Twiggy");
                // One addition for the Twiggy: check the diskFormat field to make sure that it's 0, 1, or 2
                // These correspond to Apple II or Apple ///, Lisa, and Mac, respectively, and anything else is invalid
                if (metadata->header.diskFormat > 2) {
                    Serial.printf("ERROR: Invalid disk format of %02x for Twiggy DC42 image!\n", metadata->header.diskFormat);
                    disk->close();
                    return ResultInvalidDiskFormat;
                }
                if (metadata->header.tagSize == TAG_SIZE_TWIGGY) {
                    metadata->tagsPresent = true;
                    Serial.println("Tags are present in this image.");
                }
                // Unlike previous cases where no tags is a valid option, it's not valid for Twiggies in 99% of cases
                // So flag it as such, but still let the user proceed if they want too
                else if (metadata->header.tagSize == 0) {
                    metadata->tagsPresent = false;
                    Serial.println("WARNING: No tags are present in this image. THIS IS ACTUALLY A PROBLEM FOR TWIGGY DISKS!");
                }
                else {
                    Serial.printf("ERROR: Invalid tag size for Twiggy DC42 image; tag size is %d bytes!\n", metadata->header.tagSize);
                    disk->close();
                    return ResultInvalidTagSize;
                }
            }
            else {
                Serial.printf("ERROR: Invalid disk encoding or data size for DC42 image; encoding is %02x and data size is %d bytes!\n", metadata->header.diskEncoding, metadata->header.dataSize);
                disk->close();
                return ResultInvalidDiskEncoding;
            }

            // Now calculate and verify the data and tag checksums
            /*uint32_t calculatedDataChecksum = calcDataChecksum(disk, metadata);
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
            }*/

        }
        else { // The image doesn't have the correct magic number; not a DC42
            // Check if it's a DART image by looking at the first two bytes; if the first byte is 1, 2, or 3 and the second byte is 16, 17, or 18 (decimal), then it's a DART image
            // We don't support DART images because they use compression, so we have to reject them
            if ((metadata->header.nameLength == 1 || metadata->header.nameLength == 2 || metadata->header.nameLength == 3) && (metadata->header.volumeName[0] == 16 || metadata->header.volumeName[0] == 17 || metadata->header.volumeName[0] == 18)) {
                Serial.println("ERROR: DART disk images are not supported!");
                disk->close();
                return ResultDARTNotSupported;
            }
            // Otherwise, it's probably just a raw image
            // Before we assume that though, let's make sure it's a valid size for a 400K, 800K, or Twiggy disk image (although I've never heard of a raw Twiggy image before)
            if (disk->fileSize() == DATA_SIZE_400K) {
                metadata->driveType = Drive400; // Set driveType to 400K
                Serial.println("Opened raw 400K disk image!");
            }
            else if (disk->fileSize() == DATA_SIZE_800K) {
                metadata->driveType = Drive800; // Set driveType to 800K
                Serial.println("Opened raw 800K disk image!");
            } else if (disk->fileSize() == DATA_SIZE_TWIGGY) {
                metadata->driveType = DriveTwiggy; // Set driveType to Twiggy
                // Print a warning here too, given that raw Twiggy images probably aren't actually a thing
                Serial.println("Opened raw Twiggy disk image! WARNING: I've never even heard of a raw Twiggy image before, so what is this???");
            }
            else {
                // If it's none of the above, then it's an invalid image size and we'll have to reject it
                Serial.printf("ERROR: Image size isn't a valid floppy disk size; it's %d bytes!!!\n", disk->fileSize());
                disk->close();
                return ResultInvalidImageSize; // So return false
            }
            // If we get here, it's a valid raw image
            metadata->imageType = RAW; // So set imageType to RAW to indicate raw
            metadata->tagsPresent = false; // And set tagsPresent to false since raw images can't have tags
        }
    }
    else { // The image is so small that it isn't even big enough to contain a DC42 header; it's definitely invalid
        Serial.printf("ERROR: Image size isn't a valid floppy disk size; it's only %d bytes!!!\n", disk->fileSize());
        disk->close();
        return ResultInvalidImageSize; // So return false
    }
    metadata->diskInserted = true; // Mark the disk as inserted
    return ResultSuccess; // And return true on success
}

// Reads an entire track (both sides if 800K or Twiggy disk) from the disk image into a DecodedSector array
__attribute__((optimize("Ofast"))) void readTrack(uint8_t track, File32* disk, DecodedSector sectors[2][22], DiskImageMetadata* metadata) {
    // First up, check to be sure that the track number is within bounds and return if not
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // For a Sony drive, the track must be 0-79
        if (track > 79) {
            return;
        }
    } else {
        // For a Twiggy drive, the track (carriage position) must be 0-45
        if (track > 45) {
            return;
        }
    }
    // Now, figure out if it's a single-sided or double-sided disk
    // Twiggies and 800K's are double-sided, 400K's are single-sided
    uint32_t sides = (metadata->driveType == Drive800 || metadata->driveType == DriveTwiggy) ? 2 : 1;

    // Use our LUT to find the number of sectors in the current track
    // sectorsInTrack is an array with 2 elements since Twiggy has a different number of sectors per track on each side thanks to the offset heads
    uint32_t sectorsPerTrack[2];
    if (metadata->driveType == DriveTwiggy) {
        // If the drive is a Twiggy, then use the Twiggy LUT to get the number of sectors for each side
        // track represents a carriage position which IS the track for the lower head, but is (45 - the track) for the upper head
        sectorsPerTrack[0] = sectorsPerTrackTwiggy[45 - track];
        sectorsPerTrack[1] = sectorsPerTrackTwiggy[track];
    } else {
        // Sony is easier; just use the same number of sectors for both sides
        sectorsPerTrack[0] = sectorsPerTrackSony[track];
        sectorsPerTrack[1] = sectorsPerTrackSony[track];
    }
    // Now count up the total number of sectors on the disk up to this track for offset calculations later
    // As with sectorsPerTrack, this is a 2-element array to allow for Twiggy weirdness
    uint32_t totalSectorsBeforeTrack[2] = {0, 0};
    // We have to do this differently on Sony vs Twiggy
    // On Sony images, data is stored in track-major, so side 0 track 0, side 1 track 0, side 0 track 1, side 1 track 1, and so on
    // So count up all the sectors on all the previous tracks for BOTH sides to get to the right offset
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        for (uint32_t i = 0; i < sides; i++) {
            for (uint32_t j = 0; j < track; j++) {
                totalSectorsBeforeTrack[0] += sectorsPerTrackSony[j];
                totalSectorsBeforeTrack[1] += sectorsPerTrackSony[j];
            }
        }
    }
    // Twiggy is in side-major order though, so side 0 track 0, side 0 track 1, ... side 0 track 45, side 1 track 0, and so on
    // So here we count up all the sectors before ours on side 0 to get side 0's offset
    // And then we count up all the sectors before ours on side 1 PLUS all of side 0 to get side 1's offset
    else if (metadata->driveType == DriveTwiggy) {
        // Here's side 0
        for (uint32_t j = 0; j < 45 - track; j++) {
            totalSectorsBeforeTrack[0] += sectorsPerTrackTwiggy[j];
        }
        // And here's side 1
        for (uint32_t j = 0; j < track; j++) {
            totalSectorsBeforeTrack[1] += sectorsPerTrackTwiggy[j];
        }
        // Don't forget to add all of side 0's sectors to side 1's total too, since side 1 comes after side 0 in the image
        // DATA_SIZE_TWIGGY >> 10 is the same as DATA_SIZE_TWIGGY / 2 / 512
        totalSectorsBeforeTrack[1] += DATA_SIZE_TWIGGY >> 10;
    }

    // Now compute the offset to the start of the track's data and tags in the disk image
    // For raw images, the data offset is just the total sectors before this track times 512
    // And there's no tag data at all
    uint32_t readOffset = 0;
    // Once again, we need two skews for the two different sides of the track on a Twiggy
    uint32_t dataSkew[2] = {0, 0};
    uint32_t tagSkew[2] = {0, 0};
    // We also need the base addresses of the data and tag areas in our buffer for each side
    uint32_t sideDataBase[2] = {0, 0};
    uint32_t sideTagBase[2] = {0, 0};

    // Now set up the data/tag base values based on our image and drive type
    if (metadata->driveType == DriveTwiggy) {
        // On a Twiggy, the data and tag base addresses for side 0 are 0
        // But on side 1, they're 23 and 2 sectors into the buffer, respectively
        // This is because these are the max numbers of sectors that we could possibly read in, when accounting for the potential extra one sector that we might have to read
        sideDataBase[1] = 23 * 512; // 23 sectors * 512 bytes/sector
        sideTagBase[1] = 2 * 512; // 2 sectors * 512 bytes/sector
    } else {
        // For Sony, things make a little more sense
        // We're reading everything sequentially instead of 2 different reads, so no need to make room for the extra sector when doing the offset
        sideDataBase[1] = sectorsPerTrack[0] * 512; // The data for side 1 starts right after the data for side 0
        sideTagBase[1] = sectorsPerTrack[0] * 12; // And the tags for side 1 start right after the tags for side 0
    }

    if (metadata->imageType == RAW) {
        // Raw reads are really easy on Sony images; just read in the data for both sides of the track
        if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
            // We can read it in all at once since the data for both sides of the track is contiguous
            card->readSectors(metadata->startAddress + totalSectorsBeforeTrack[0], rawDataBuffer + sideDataBase[0], sectorsPerTrack[0] * sides);
        } else {
            // But Twiggy raw reads have to be done one side at a time thanks to the data being non-contiguous in the image
            // First, side 0 (the upper/rear head)
            card->readSectors(metadata->startAddress + totalSectorsBeforeTrack[0], rawDataBuffer + sideDataBase[0], sectorsPerTrack[0]);
            // And then side 1 (the lower/front head), making sure to offset it into the buffer to avoid clobbering side 0
            card->readSectors(metadata->startAddress + totalSectorsBeforeTrack[1], rawDataBuffer + sideDataBase[1], sectorsPerTrack[1]);
        }
    }
    // For DC42 images, we have to account for the header as well, and there might be a tag offset too
    else if (metadata->imageType == DC42) {
        // For Sony images, we can read in all of the data for both sides at once just like with raw images
        if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
            readOffset = sizeof(DC42Header) + (totalSectorsBeforeTrack[0] * 512);
            // Given the 84-byte header, we need to read in an extra sector's worth of data to make sure we get all the data for this track
            // So compute a sectorCount that gets the total number of sectors to read in, including the extra one
            // First compute a skew factor; this will always be 84 for the data, but varies for the tags, so might as well compute it for both
            dataSkew[0] = readOffset & 511;
            uint32_t sectorCount = (dataSkew[0] + (sectorsPerTrack[0] * sides * 512) + 511) >> 9;
            // And read in all the data, taking into account the header situation
            card->readSectors(metadata->startAddress + (readOffset >> 9), rawDataBuffer + sideDataBase[0], sectorCount);
            // Check if there are tags present
            if (metadata->tagsPresent) {
                // And if so, compute the offset to the start of the tags, using the proper data size for the drive type
                readOffset = sizeof(DC42Header) + metadata->header.dataSize + (totalSectorsBeforeTrack[0] * 12);
                // Don't forget about the skew too; this is where it may not be 84 because tags are only 12 bytes each and so it'll differ for each track
                tagSkew[0] = readOffset & 511;
                // Do the same thing as before where we add an extra sector if necessary to make sure we get all the tag data for this track
                sectorCount = (tagSkew[0] + (sectorsPerTrack[0] * sides * 12) + 511) >> 9;
                // And read in all the tags
                card->readSectors(metadata->startAddress + (readOffset >> 9), rawTagBuffer + sideTagBase[0], sectorCount);
            }
            dataSkew[1] = dataSkew[0]; // This is Sony, so make sure the skews for both sides are the same
            tagSkew[1] = tagSkew[0];
        } else {
            // But for Twiggy, we have to split both the data AND tag reads into two separate reads each, one for each side
            // Start with side 0 (the upper/rear head)
            readOffset = sizeof(DC42Header) + (totalSectorsBeforeTrack[0] * 512);
            dataSkew[0] = readOffset & 511;
            uint32_t sectorCount = (dataSkew[0] + (sectorsPerTrack[0] * 512) + 511) >> 9;
            card->readSectors(metadata->startAddress + (readOffset >> 9), rawDataBuffer + sideDataBase[0], sectorCount);
            if (metadata->tagsPresent) {
                readOffset = sizeof(DC42Header) + metadata->header.dataSize + (totalSectorsBeforeTrack[0] * 12);
                tagSkew[0] = readOffset & 511;
                sectorCount = (tagSkew[0] + (sectorsPerTrack[0] * 12) + 511) >> 9;
                card->readSectors(metadata->startAddress + (readOffset >> 9), rawTagBuffer + sideTagBase[0], sectorCount);
            }
            // And now do side 1 (the lower/front head)
            readOffset = sizeof(DC42Header) + (totalSectorsBeforeTrack[1] * 512);
            dataSkew[1] = readOffset & 511;
            sectorCount = (dataSkew[1] + (sectorsPerTrack[1] * 512) + 511) >> 9;
            card->readSectors(metadata->startAddress + (readOffset >> 9), rawDataBuffer + sideDataBase[1], sectorCount);
            if (metadata->tagsPresent) {
                readOffset = sizeof(DC42Header) + metadata->header.dataSize + (totalSectorsBeforeTrack[1] * 12);
                tagSkew[1] = readOffset & 511;
                sectorCount = (tagSkew[1] + (sectorsPerTrack[1] * 12) + 511) >> 9;
                card->readSectors(metadata->startAddress + (readOffset >> 9), rawTagBuffer + sideTagBase[1], sectorCount);
            }
        }
    }
    // Now we need to take all of that raw data and format it into our DecodedSector structs for use by the rest of the emulator
    // The nice thing now is that, regardless of whether it was a raw or DC42 or Sony or Twiggy image, the data is now all structured the same way in the buffer
    // With side 0's data first, followed immediately by side 1's data
    // So iterate over all the sectors and sides
    for (uint32_t side = 0; side < sides; side++) {
        for (uint32_t sector = 0; sector < sectorsPerTrack[side]; sector++) {
            // Handle raw and DC42 images differently
            if (metadata->imageType == RAW) {
                // The first 12 bytes of each sector are the tag, which raw images don't have, so we just zero them out
                for (int i = 0; i < 12; i++) {
                    sectors[side][sector].data[i] = 0; // Zero out the tag bytes
                }
                // Now copy the 512 bytes of data from our raw data buffer into the sector's data array starting at position 12
                // The offset into the raw data buffer is computed based on the side, and current sector number from the for loop
                uint32_t bufferOffset = sideDataBase[side] + dataSkew[side] + (sector * 512);
                // Use memcpy instead of a loop for speed
                memcpy(&sectors[side][sector].data[12], &rawDataBuffer[bufferOffset], 512);
                // Now set the other fields of our DecodedSector struct
                // The track field varies depending on whether it's a Twiggy or a Sony
                if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
                    sectors[side][sector].track = track; // For Sony, the track is just the current track
                } else {
                    // For Twiggy, side 0's track is (45 - current track), side 1's track is just the current track
                    sectors[side][sector].track = (side == 0) ? (45 - track) : track;
                }
                sectors[side][sector].side = side;
                sectors[side][sector].sector = sector;
                // Unlike with DC42 images, we don't have a format byte stored anywhere, so just use context clues to set it as best we can
                // Let's start with the 400K/800K case
                if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
                    // One thing that's for sure: we can set the side count bit based on whether it's an 800K or 400K disk
                    sectors[side][sector].format = (metadata->driveType == Drive800) ? 0b00100000 : 0b00000000; // Bit 5 is side count
                    // For the interleave factor, just assume it's the most common value of 2:1 interleave
                    sectors[side][sector].format |= 0x02; // Bits [4:0] are interleave factor
                } else {
                    // For Twiggies, just always set the format to 0x01
                    // A format of 0x01 means it's a Lisa disk, which is obviously the safest bet for a Twiggy disk
                    sectors[side][sector].format = 0x01;
                }
            }
            else if (metadata->imageType == DC42) {
                // If it's a DC42, then start by copying over the 512 data bytes just like for raw images, but account for the 84-byte header too
                uint32_t bufferOffset = sideDataBase[side] + dataSkew[side] + (sector * 512);
                // Use memcpy instead of a loop for speed
                memcpy(&sectors[side][sector].data[12], &rawDataBuffer[bufferOffset], 512);
                // Now handle the tags if they're present
                if (metadata->tagsPresent) {
                    // If they are, then we need to read the 12-byte tag for this sector
                    // So compute the offset into the raw tag buffer; once again, remember the header skew
                    bufferOffset = sideTagBase[side] + tagSkew[side] + (sector * 12);
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
                // The track field varies depending on whether it's a Twiggy or a Sony
                if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
                    sectors[side][sector].track = track; // For Sony, the track is just the current track
                } else {
                    // For Twiggy, side 0's track is (45 - current track), side 1's track is just the current track
                    sectors[side][sector].track = (side == 0) ? (45 - track) : track;
                }
                sectors[side][sector].side = side;
                sectors[side][sector].sector = sector;
                // Unlike with raw images, we have the format byte stored in the DC42 header, so no need to do any guesswork there
                // Just pull it straight over
                sectors[side][sector].format = metadata->header.diskFormat;
            }
        }
    }
    lastReadTrack = track; // Remember the last track (or carriage position in the case of a Twiggy) we read so that writeTrack() can avoid unnecessary reads
    lastReadDrive = metadata->driveIndex; // And remember that it was read for this particular drive
}

// Writes an entire track (both sides if 800K or Twiggy disk) from a DecodedSector array back into the disk image
__attribute__((optimize("Ofast"))) void writeTrack(uint8_t track, File32* disk, DecodedSector sectors[2][22], DiskImageMetadata* metadata) {
    // This is going to be pretty darn similar to readTrack, but in reverse
    // First up, check to be sure that the track number is within bounds and return if not
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // For a Sony drive, the track must be 0-79
        if (track > 79) {
            return;
        }
    } else {
        // For a Twiggy drive, the track (carriage position) must be 0-45
        if (track > 45) {
            return;
        }
    }

    // Now, figure out if it's a single-sided or double-sided disk
    uint32_t sides = (metadata->driveType == Drive800 || metadata->driveType == DriveTwiggy) ? 2 : 1;

    // Use our LUT to find the number of sectors in the current track
    // sectorsPerTrack is an array with 2 elements since Twiggy has a different number of sectors per track on each side thanks to the offset heads
    uint32_t sectorsPerTrack[2];
    if (metadata->driveType == DriveTwiggy) {
        // If the drive is a Twiggy, then use the Twiggy LUT to get the number of sectors for each side
        // track represents a carriage position which IS the track for the lower head, but is (45 - the track) for the upper head
        sectorsPerTrack[0] = sectorsPerTrackTwiggy[45 - track];
        sectorsPerTrack[1] = sectorsPerTrackTwiggy[track];
    } else {
        // Sony is easier; just use the same number of sectors for both sides
        sectorsPerTrack[0] = sectorsPerTrackSony[track];
        sectorsPerTrack[1] = sectorsPerTrackSony[track];
    }
    // Now count up the total number of sectors on the disk up to this track for offset calculations later
    // As with sectorsPerTrack, this is a 2-element array to allow for Twiggy weirdness
    uint32_t totalSectorsBeforeTrack[2] = {0, 0};
    // We have to do this differently on Sony vs Twiggy
    // On Sony images, data is stored in track-major, so side 0 track 0, side 1 track 0, side 0 track 1, side 1 track 1, and so on
    // So count up all the sectors on all the previous tracks for BOTH sides to get to the right offset
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        for (uint32_t i = 0; i < sides; i++) {
            for (uint32_t j = 0; j < track; j++) {
                totalSectorsBeforeTrack[0] += sectorsPerTrackSony[j];
                totalSectorsBeforeTrack[1] += sectorsPerTrackSony[j];
            }
        }
    }
    // Twiggy is in side-major order though, so side 0 track 0, side 0 track 1, ... side 0 track 45, side 1 track 0, and so on
    // So here we count up all the sectors before ours on side 0 to get side 0's offset
    // And then we count up all the sectors before ours on side 1 PLUS all of side 0 to get side 1's offset
    else if (metadata->driveType == DriveTwiggy) {
        // Here's side 0
        for (uint32_t j = 0; j < 45 - track; j++) {
            totalSectorsBeforeTrack[0] += sectorsPerTrackTwiggy[j];
        }
        // And here's side 1
        for (uint32_t j = 0; j < track; j++) {
            totalSectorsBeforeTrack[1] += sectorsPerTrackTwiggy[j];
        }
        // Don't forget to add all of side 0's sectors to side 1's total too, since side 1 comes after side 0 in the image
        // DATA_SIZE_TWIGGY >> 10 is the same as DATA_SIZE_TWIGGY / 2 / 512
        totalSectorsBeforeTrack[1] += DATA_SIZE_TWIGGY >> 10;
    }

    // Now compute the offset to the start of the track's data and tags in the disk image
    // For raw images, the data offset is just the total sectors before this track times 512
    // And there's no tag data at all
    // Once again, we need two skews for the two different sides of the track on a Twiggy
    uint32_t dataSkew[2] = {0, 0}; // Offset into the first sector at which this track's data actually starts
    uint32_t tagSkew[2] = {0, 0};
    // We also need the base addresses of the data and tag areas in our buffer for each side
    uint32_t sideDataBase[2] = {0, 0};
    uint32_t sideTagBase[2] = {0, 0};
    // We also need the absolute SD card sector numbers of the start of the data and tag areas for each side
    uint32_t dataSectorNumber[2] = {0, 0};
    uint32_t tagSectorNumber[2] = {0, 0};
    // As well as the number of SD card sectors that the data and tag areas span for each side
    uint32_t dataSectorCount[2] = {0, 0};
    uint32_t tagSectorCount[2] = {0, 0};
    // And finally, a byte offset into the disk image at which to start writing each side of this track's data
    uint32_t writeOffset[2] = {0, 0};

    // Now set up the data/tag base values based on our image and drive type
    if (metadata->driveType == DriveTwiggy) {
        // On a Twiggy, the data and tag base addresses for side 0 are 0
        // But on side 1, they're 23 and 2 sectors into the buffer, respectively
        // This is because these are the max numbers of sectors that we could possibly read in, when accounting for the potential extra one sector that we might have to read
        sideDataBase[1] = 23 * 512; // 23 sectors * 512 bytes/sector
        sideTagBase[1] = 2 * 512; // 2 sectors * 512 bytes/sector
    } else {
        // For Sony, things make a little more sense
        // We're reading everything sequentially instead of 2 different reads, so no need to make room for the extra sector when doing the offset
        sideDataBase[1] = sectorsPerTrack[0] * 512; // The data for side 1 starts right after the data for side 0
        sideTagBase[1] = sectorsPerTrack[0] * 12; // And the tags for side 1 start right after the tags for side 0
    }

    if (metadata->imageType == RAW) {
        // Raw images have no header, so the track's data starts exactly on a sector boundary and spans a whole number of sectors
        // That means no skew, and no read-modify-write needed either since we overwrite every byte of every sector we touch
        // There are two different cases here though: Sony and Twiggy
        if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
            // For Sony, we can just write the whole track's data in one go since it's contiguous in the image
            writeOffset[0] = (totalSectorsBeforeTrack[0] * 512);
            dataSectorNumber[0] = metadata->startAddress + totalSectorsBeforeTrack[0];
            dataSectorCount[0] = sectorsPerTrack[0] * sides;
            // And make it the same for both sides of the disk
            writeOffset[1] = writeOffset[0];
            dataSectorNumber[1] = dataSectorNumber[0];
            dataSectorCount[1] = dataSectorCount[0];
        } else {
            // But for Twiggy, we have to write each side separately since the data for each side of a track is non-contiguous
            // Start with side 0 (the upper/rear head)
            writeOffset[0] = (totalSectorsBeforeTrack[0] * 512);
            dataSectorNumber[0] = metadata->startAddress + totalSectorsBeforeTrack[0];
            dataSectorCount[0] = sectorsPerTrack[0];
            // And then side 1 (the lower/front head)
            writeOffset[1] = (totalSectorsBeforeTrack[1] * 512);
            dataSectorNumber[1] = metadata->startAddress + totalSectorsBeforeTrack[1];
            dataSectorCount[1] = sectorsPerTrack[1];
        }
    }
    else if (metadata->imageType == DC42) {
        // For DC42 images, the 84-byte header misaligns everything, so we have to compute a skew and read an extra sector, just like in readTrack
        // There are separate Sony and Twiggy cases here too; let's start with Sony
        if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
            writeOffset[0] = sizeof(DC42Header) + (totalSectorsBeforeTrack[0] * 512);
            dataSkew[0] = writeOffset[0] & 511;
            dataSectorNumber[0] = metadata->startAddress + (writeOffset[0] >> 9);
            dataSectorCount[0] = (dataSkew[0] + (sectorsPerTrack[0] * sides * 512) + 511) >> 9;
            // The problem with raw writes is that we can only write whole sectors at a time, but the track's data doesn't start on a sector boundary
            // This is thanks to the dc42 header in the case of the data, and the header combined with the tags only being 12 bytes each in the case of the tags
            // What this means is that we can't just overwrite a whole SD card sector with our new data because that would clobber the data from other tracks that's partially in that sector
            // So read those sectors in first, and then the copy loop below only overwrites the parts that belong to this track

            // The good news though is that this read isn't really necessary most of the time, since we always read in the current track before writing it back out
            // Which means that the rawDataBuffer already contains the correct data for the sectors that aren't being overwritten
            // This assumption holds as long as readTrack is always called before writeTrack, which it is in the current implementation
            // So we'll just put a check around this readSectors call to make sure that the track that was read is the same track that's being written
            // And that the drive that was read is the same drive that's being written
            // The former is currently always the case, and the latter is likely to be the case as well, so we'll skip BOTH of these readSectors calls and save heaps of time
            if (lastReadTrack != track || lastReadDrive != metadata->driveIndex) {
                card->readSectors(dataSectorNumber[0], rawDataBuffer + sideDataBase[0], dataSectorCount[0]);
            }
            // Now do exactly the same thing for the tags if the image has them
            if (metadata->tagsPresent) {
                writeOffset[0] = sizeof(DC42Header) + metadata->header.dataSize + (totalSectorsBeforeTrack[0] * 12);
                tagSkew[0] = writeOffset[0] & 511;
                tagSectorNumber[0] = metadata->startAddress + (writeOffset[0] >> 9);
                tagSectorCount[0] = (tagSkew[0] + (sectorsPerTrack[0] * sides * 12) + 511) >> 9;
                // Same as above, we only need to read the tags if the last read track is not the same as the current track
                // Or if the last read drive isn't the same as this drive
                if (lastReadTrack != track || lastReadDrive != metadata->driveIndex) {
                    card->readSectors(tagSectorNumber[0], rawTagBuffer + sideTagBase[0], tagSectorCount[0]);
                }
            }
            // Just to be safe, make sure all our offsets and skews for side 1 are the same as side 0 since this is a Sony image
            writeOffset[1] = writeOffset[0];
            dataSkew[1] = dataSkew[0];
            tagSkew[1] = tagSkew[0];
            tagSectorCount[1] = tagSectorCount[0];
            dataSectorNumber[1] = dataSectorNumber[0];
            dataSectorCount[1] = dataSectorCount[0];
        } else {
            // Now onto the Twiggy case; it's basically all of that over again except we have to do it twice, once for each side
            // Start here with side 0 (the upper/rear head)
            writeOffset[0] = sizeof(DC42Header) + (totalSectorsBeforeTrack[0] * 512);
            dataSkew[0] = writeOffset[0] & 511;
            dataSectorNumber[0] = metadata->startAddress + (writeOffset[0] >> 9);
            dataSectorCount[0] = (dataSkew[0] + (sectorsPerTrack[0] * 512) + 511) >> 9;
            if (lastReadTrack != track || lastReadDrive != metadata->driveIndex) {
                card->readSectors(dataSectorNumber[0], rawDataBuffer + sideDataBase[0], dataSectorCount[0]);
            }
            if (metadata->tagsPresent) {
                writeOffset[0] = sizeof(DC42Header) + metadata->header.dataSize + (totalSectorsBeforeTrack[0] * 12);
                tagSkew[0] = writeOffset[0] & 511;
                tagSectorNumber[0] = metadata->startAddress + (writeOffset[0] >> 9);
                tagSectorCount[0] = (tagSkew[0] + (sectorsPerTrack[0] * 12) + 511) >> 9;
                if (lastReadTrack != track || lastReadDrive != metadata->driveIndex) {
                    card->readSectors(tagSectorNumber[0], rawTagBuffer + sideTagBase[0], tagSectorCount[0]);
                }
            }
            // And now do side 1 (the lower/front head)
            writeOffset[1] = sizeof(DC42Header) + (totalSectorsBeforeTrack[1] * 512);
            dataSkew[1] = writeOffset[1] & 511;
            dataSectorNumber[1] = metadata->startAddress + (writeOffset[1] >> 9);
            dataSectorCount[1] = (dataSkew[1] + (sectorsPerTrack[1] * 512) + 511) >> 9;
            if (lastReadTrack != track || lastReadDrive != metadata->driveIndex) {
                card->readSectors(dataSectorNumber[1], rawDataBuffer + sideDataBase[1], dataSectorCount[1]);
            }
            if (metadata->tagsPresent) {
                writeOffset[1] = sizeof(DC42Header) + metadata->header.dataSize + (totalSectorsBeforeTrack[1] * 12);
                tagSkew[1] = writeOffset[1] & 511;
                tagSectorNumber[1] = metadata->startAddress + (writeOffset[1] >> 9);
                tagSectorCount[1] = (tagSkew[1] + (sectorsPerTrack[1] * 12) + 511) >> 9;
                if (lastReadTrack != track || lastReadDrive != metadata->driveIndex) {
                    card->readSectors(tagSectorNumber[1], rawTagBuffer + sideTagBase[1], tagSectorCount[1]);
                }
            }
        }
    }

    // Now that the buffers hold the surrounding sectors, we can grab all the sector data and overlay it into the rawDataBuffer
    // And put the tags in the rawTagBuffer if applicable
    for (uint32_t side = 0; side < sides; side++) {
        for (uint32_t sector = 0; sector < sectorsPerTrack[side]; sector++) {
            // Again, handle raw and DC42 images differently
            if (metadata->imageType == RAW) {
                // Compute the offset the same way as in readTrack
                uint32_t bufferOffset = sideDataBase[side] + dataSkew[side] + (sector * 512);
                // And copy the 512 bytes of data from the sector's data array starting at position 12 into our raw data buffer
                memcpy(&rawDataBuffer[bufferOffset], &sectors[side][sector].data[12], 512);
                // For raw images, we don't have any tags to write, so we can skip that part
            }
            // If it's not raw, it must be DC42
            else if (metadata->imageType == DC42) {
                // First, let's update the diskFormat byte in the DC42 header to match the format byte of this sector
                // This assumes that all sectors on the disk have the same format byte, but that's a very reasonable assumption
                // Most likely, it'll be the same format byte we read when we opened the image, but there's a chance the host formatted the disk differently while in use
                // This is a bit inefficient since we're doing it for every sector, but there are only 44 sectors max per track in the Twiggy worst case
                // And really less than that because that assumes that both sides have 22 sectors, which is impossible given the offset heads
                metadata->header.diskFormat = sectors[side][sector].format;
                // Now find the data offset just like before
                uint32_t bufferOffset = sideDataBase[side] + dataSkew[side] + (sector * 512);
                // And copy over the 512 bytes of sector data
                memcpy(&rawDataBuffer[bufferOffset], &sectors[side][sector].data[12], 512);
                // Now handle the tags if the image supports them
                if (metadata->tagsPresent) {
                    // If the DC42 image supports tags, compute the offset to the tag the same way as in readTrack
                    bufferOffset = sideTagBase[side] + tagSkew[side] + (sector * 12);
                    // And copy the 12-byte tag from the start of the sector's data array into our raw tag buffer
                    memcpy(&rawTagBuffer[bufferOffset], &sectors[side][sector].data[0], 12);
                    // And we're done; no need to handle the "no tags" case since we don't have to write anything then
                }
            }
        }
    }

    // One more thing, when we're in Twiggy mode, carriage position 0 side 0 and carriage position 0 side 1 actually share part of the same SD sector
    // So this means that upper head track 45 and lower head track 0 share the same sector, and we have to be careful not to clobber one with the other
    // As-is right now, side 0 gets written first and then side 1, so the data for side 0 that's in the shared sector will be overwritten
    // The solution here is to detect this and merge the data together before the write
    if (metadata->driveType == DriveTwiggy) {
        // Only do this for Twiggy of course
        if (dataSectorNumber[0] + dataSectorCount[0] > dataSectorNumber[1]) {
            // If we end up here, then the data for side 0 and side 1 overlap
            // So merge them with memcpy
            memcpy(&rawDataBuffer[sideDataBase[1]], &rawDataBuffer[sideDataBase[0] + (dataSectorCount[0] - 1) * 512], dataSkew[1]);
            // And then decrement dataSectorCount[0] so that only side 1 writes the sector
            dataSectorCount[0]--;
        }
        // Do the same thing for the tags too
        if (metadata->tagsPresent && tagSectorNumber[0] + tagSectorCount[0] > tagSectorNumber[1]) {
            memcpy(&rawTagBuffer[sideTagBase[1]], &rawTagBuffer[sideTagBase[0] + (tagSectorCount[0] - 1) * 512], tagSkew[1]);
            tagSectorCount[0]--;
        }
        // Another thing I discovered later on is that carriage position 45 side 1 shares a sector with the start of the tag data
        // So we need to do something similar to the above for that case too
        // Only do it if the image has tags though
        if (metadata->tagsPresent && (dataSectorNumber[1] + dataSectorCount[1] > tagSectorNumber[0])) {
            // tagSkew[0] contains how many bytes of the shared sector are used by the data area
            // So copy that much data from the end of the side 1 data buffer into the tag buffer
            memcpy(&rawTagBuffer[sideTagBase[0]], &rawDataBuffer[sideDataBase[1] + (dataSectorCount[1] - 1) * 512], tagSkew[0]);
            // And then decrement dataSectorCount[1] so that only the tag write writes the sector
            dataSectorCount[1]--;
        }
    }

    // Now that we've got all the data and tags in our raw buffers, we can write them back to the disk image
    // All of the offset math was done earlier, so all we need to do now is just send out the data
    // For a Sony, this will be just one write each for the data and tags, but for a Twiggy, it'll be two each
    // Start with the Sony case
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // Write out the data for both sides
        card->writeSectors(dataSectorNumber[0], rawDataBuffer + sideDataBase[0], dataSectorCount[0]);
        // And write the tags too if this image has them
        if (tagSectorCount[0] > 0) {
            card->writeSectors(tagSectorNumber[0], rawTagBuffer + sideTagBase[0], tagSectorCount[0]);
        }
    } else {
        // And now the Twiggy case, starting with data side 0
        card->writeSectors(dataSectorNumber[0], rawDataBuffer + sideDataBase[0], dataSectorCount[0]);
        // And data side 1
        card->writeSectors(dataSectorNumber[1], rawDataBuffer + sideDataBase[1], dataSectorCount[1]);
        // Now do the tags if applicable, starting with side 0
        if (tagSectorCount[0] > 0) {
            card->writeSectors(tagSectorNumber[0], rawTagBuffer + sideTagBase[0], tagSectorCount[0]);
        }
        // And side 1
        if (tagSectorCount[1] > 0) {
            card->writeSectors(tagSectorNumber[1], rawTagBuffer + sideTagBase[1], tagSectorCount[1]);
        }
    }
}

// Closes the disk image file, making sure to update the DC42 header if needed/applicable
void closeImage(File32* disk, DiskImageMetadata* metadata) {
    // closeImage clobbers the rawDataBuffer, so set lastReadTrack to an invalid value so that writeTrack() will read the data sectors again
    lastReadTrack = 0xFFFFFFFF;
    lastReadDrive = metadata->driveIndex; // And set lastReadDrive to the current drive
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