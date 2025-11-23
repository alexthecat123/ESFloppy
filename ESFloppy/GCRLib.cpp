#include <Arduino.h>
#include "types.h"

// All the routines related to GCR encoding and decoding for ESFloppy

// Lookup tables for GCR encoding and decoding
// This table converts 6-bit nibbles to 8-bit GCR bytes
const uint8_t gcr_6to8[64] = {
    0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6,
    0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3,
    0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC,
    0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3,
    0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE,
    0xDF, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA, 0xEB, 0xEC,
    0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6,
    0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF
};

// And this one converts 8-bit GCR bytes back to 6-bit nibbles
const uint8_t gcr_8to6[256] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x01,
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x04, 0x05, 0x06,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x08,
    0xFF, 0xFF, 0xFF, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
    0xFF, 0xFF, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
    0xFF, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0x1B, 0xFF, 0x1C, 0x1D, 0x1E,
    0xFF, 0xFF, 0xFF, 0x1F, 0xFF, 0xFF, 0x20, 0x21,
    0xFF, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x29, 0x2A, 0x2B,
    0xFF, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32,
    0xFF, 0xFF, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
    0xFF, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F
};

// This function takes a decoded sector and encodes it into GCR format
void encodeSector(DecodedSector* decoded, GcrSector* gcr) {
    // First let's fill in the header fields
    gcr->loTrack = decoded->track & 0x3F; // Low 6 bits of track
    gcr->sector = decoded->sector; // Sector number gets pulled straight over
    gcr->hiTrackSide = ((decoded->track & 0x40) >> 6) | ((decoded->side & 0x01) << 5); // High track bit in bit 0, side in bit 5
    gcr->format = decoded->format; // Format byte gets pulled straight over too
    gcr->headerChecksum = gcr->loTrack ^ gcr->sector ^ gcr->hiTrackSide ^ gcr->format; // Simple XOR checksum of the header fields
    // Now onto the data stuff; starting with the sector number again
    gcr->sector_again = decoded->sector;
    // Time for the hard part: encoding the 524 bytes of decoded data into 699 bytes of GCR data
    // It sounds easy enough, just with some annoying bit manipulations, but no, Apple makes it hard on us
    // We have to do this dumb algorithm that calculates a checksum along the way and modifies the data according to it
    // So the data written to the disk isn't just a straight GCR encoding of the decoded data; it's a weird modified version of it
    // We process the decoded data in chunks of 3 bytes at a time; each 3-byte chunk becomes 4 GCR bytes
    // Since 524 isn't divisible by 3, the last chunk is only 2 bytes and so we set the final byte to 0
    uint16_t csumA = 0, csumB = 0;
    uint8_t csumC = 0;
    uint8_t carry;
    uint8_t byteA, byteB, byteC;
    uint8_t nibl1, nibl2, nibl3, nibl4;
    uint16_t i = 0; // Index for the decoded data array
    uint16_t j = 0; // Index for the GCR data array
    while (i < 522) { // Process the first 522 bytes (174 chunks of 3 bytes)
        byteA = decoded->data[i++];
        byteB = decoded->data[i++];
        byteC = decoded->data[i++];
        csumC = (csumC << 1) | (csumC >> 7); // Rotate csumC left by 1
        carry = csumC & 0x01; // Get the low (carry) bit of csumC
        csumA = csumA + byteA + carry; // Add byteA and carry to csumA
        carry = csumA & 0x100 ? 1 : 0; // Get the carry bit of csumA
        csumA = csumA & 0xFF; // And mask csumA to 8 bits to remove the carry
        byteA = byteA ^ csumC; // And then XOR byteA with csumC
        csumB = csumB + byteB + carry; // Add byteB and carry to csumB
        carry = csumB & 0x100 ? 1 : 0; // Get the carry bit of csumB
        csumB = csumB & 0xFF; // Mask csumB to 8 bits
        byteB = byteB ^ csumA; // XOR byteB with csumA
        csumC = csumC + byteC + carry; // Add byteC and carry to csumC
        byteC = byteC ^ csumB; // XOR byteC with csumB
        // Now we can GCR-encode the modified bytes
        // The first nibble is the high 2 bits of byteA, the high 2 bits of byteB, and the high 2 bits of byteC
        // A7 A6 B7 B6 C7 C6
        nibl1 = ((byteA & 0xC0) >> 2) | ((byteB & 0xC0) >> 4) | ((byteC & 0xC0) >> 6);
        // The next three nibbles are just the low 6 bits of each byte
        // A5 A4 A3 A2 A1 A0
        nibl2 = byteA & 0x3F;
        // B5 B4 B3 B2 B1 B0
        nibl3 = byteB & 0x3F;
        // C5 C4 C3 C2 C1 C0
        nibl4 = byteC & 0x3F;
        // Now write the GCR bytes to the output data array
        gcr->data[j++] = gcr_6to8[nibl1];
        gcr->data[j++] = gcr_6to8[nibl2];
        gcr->data[j++] = gcr_6to8[nibl3];
        gcr->data[j++] = gcr_6to8[nibl4];
    }
    // Now we have 2 bytes left to process (byte 522 and byte 523)
    byteA = decoded->data[i++]; // Grab A and B as usual
    byteB = decoded->data[i++];
    byteC = 0; // But set C to 0 since we don't have a third byte
    csumC = (csumC << 1) | (csumC >> 7); // Rotate csumC left by 1
    carry = csumC & 0x01; // Get the low (carry) bit of csumC
    csumA = csumA + byteA + carry; // Add byteA and carry to csumA
    carry = csumA & 0x100 ? 1 : 0; // Get the carry bit of csumA
    csumA = csumA & 0xFF; // And mask csumA to 8 bits to remove the carry
    byteA = byteA ^ csumC; // And then XOR byteA with csumC
    csumB = csumB + byteB + carry; // Add byteB and carry to csumB
    carry = csumB & 0x100 ? 1 : 0; // Get the carry bit of csumB
    csumB = csumB & 0xFF; // Mask csumB to 8 bits
    byteB = byteB ^ csumA; // XOR byteB with csumA
    csumC = csumC + byteC + carry; // Add byteC and carry to csumC
    byteC = byteC ^ csumB; // XOR byteC with csumB
    // Formulate the nibbles just as before, but skip anything C-related
    nibl1 = ((byteA & 0xC0) >> 2) | ((byteB & 0xC0) >> 4);
    // The next three nibbles are just the low 6 bits of each byte
    // A5 A4 A3 A2 A1 A0
    nibl2 = byteA & 0x3F;
    // B5 B4 B3 B2 B1 B0
    nibl3 = byteB & 0x3F;
    // Now write the GCR bytes to the output data array, but once again skip C (nibl4)
    gcr->data[j++] = gcr_6to8[nibl1];
    gcr->data[j++] = gcr_6to8[nibl2];
    gcr->data[j++] = gcr_6to8[nibl3];
    // Don't forget to fill in the data checksum at the end
    // It gets split into 4 GCR nybbles of course, and is encoded just like the data
    nibl1 = ((csumA & 0xC0) >> 2) | ((csumB & 0xC0) >> 4) | ((csumC & 0xC0) >> 6);
    nibl2 = csumA & 0x3F;
    nibl3 = csumB & 0x3F;
    nibl4 = csumC & 0x3F;
    gcr->dataChecksum[0] = gcr_6to8[nibl1];
    gcr->dataChecksum[1] = gcr_6to8[nibl2];
    gcr->dataChecksum[2] = gcr_6to8[nibl3];
    gcr->dataChecksum[3] = gcr_6to8[nibl4];
}

void decodeSector(GcrSector* gcr, DecodedSector* decoded) {
    // Decoding is (obviously) just the reverse of encoding; first do the header
    // Reconstruct the track number from the low and high bits
    decoded->track = gcr->loTrack | ((gcr->hiTrackSide & 0x01) << 6);
    // The sector comes straight over
    decoded->sector = gcr->sector;
    // The side is in bit 5 of hiTrackSide
    decoded->side = (gcr->hiTrackSide & 0x20) >> 5;
    // And the format byte comes straight over too
    decoded->format = gcr->format;
    // Now we've got to do the stupid data decoding algorithm
    uint16_t csumA = 0, csumB = 0;
    uint8_t csumC = 0;
    uint8_t carry;
    uint8_t byteA, byteB, byteC;
    uint8_t nibl1, nibl2, nibl3, nibl4;
    uint16_t i = 0; // Index for the GCR data array
    uint16_t j = 0; // Index for the decoded data array
    // Now we can start decoding the data itself
    while (j < 522) { // Process the first 522 bytes (174 chunks of 3 bytes)
        // First read 4 GCR bytes and convert them back to nibbles
        nibl1 = gcr_8to6[gcr->data[i++]];
        nibl2 = gcr_8to6[gcr->data[i++]];
        nibl3 = gcr_8to6[gcr->data[i++]];
        nibl4 = gcr_8to6[gcr->data[i++]];
        // Now reconstruct the bytes from those nibbles; grab the high 2 bits from nibl1 and the low 6 bits from nibl2, nibl3, and nibl4
        byteA = ((nibl1 & 0x30) << 2) | (nibl2 & 0x3F);
        byteB = ((nibl1 & 0x0C) << 4) | (nibl3 & 0x3F);
        byteC = ((nibl1 & 0x03) << 6) | (nibl4 & 0x3F);
        // Now we have to undo the modifications made during encoding
        csumC = (csumC << 1) | (csumC >> 7); // Rotate csumC left by 1
        carry = csumC & 0x01; // Get the low (carry) bit of csumC
        byteA = byteA ^ csumC; // XOR byteA with csumC
        csumA = csumA + byteA + carry; // Add byteA and carry to csumA
        carry = csumA & 0x100 ? 1 : 0; // Get the carry bit of csumA
        csumA = csumA & 0xFF; // And mask csumA to 8 bits to remove the carry
        byteB = byteB ^ csumA; // XOR byteB with csumA
        csumB = csumB + byteB + carry; // Add byteB and carry to csumB
        carry = csumB & 0x100 ? 1 : 0; // Get the carry bit of csumB
        csumB = csumB & 0xFF; // And mask csumB to 8 bits to remove the carry
        byteC = byteC ^ csumB; // XOR byteC with csumB
        csumC = csumC + byteC + carry; // Add byteC and carry to csumC
        // Now write the decoded bytes to the output data array
        decoded->data[j++] = byteA;
        decoded->data[j++] = byteB;
        decoded->data[j++] = byteC;
    }
    // Now we have 2 bytes left to process (byte 522 and byte 523)
    // Do the same thing as before, but make anything C-related zero
    nibl1 = gcr_8to6[gcr->data[i++]];
    nibl2 = gcr_8to6[gcr->data[i++]];
    nibl3 = gcr_8to6[gcr->data[i++]];
    // Reconstruct bytes A and B
    byteA = ((nibl1 & 0x30) << 2) | (nibl2 & 0x3F);
    byteB = ((nibl1 & 0x0C) << 4) | (nibl3 & 0x3F);
    byteC = 0; // Set C to 0 since we don't have a third byte
    // Undo the modifications made during encoding
    csumC = (csumC << 1) | (csumC >> 7); // Rotate csumC left by 1
    carry = csumC & 0x01; // Get the low (carry) bit of csumC
    byteA = byteA ^ csumC; // XOR byteA with csumC
    csumA = csumA + byteA + carry; // Add byteA and carry to csumA
    carry = csumA & 0x100 ? 1 : 0; // Get the carry bit of csumA
    csumA = csumA & 0xFF; // And mask csumA to 8 bits to remove the carry
    byteB = byteB ^ csumA; // XOR byteB with csumA
    csumB = csumB + byteB + carry; // Add byteB and carry to csumB
    carry = csumB & 0x100 ? 1 : 0; // Get the carry bit of csumB
    csumB = csumB & 0xFF; // And mask csumB to 8 bits to remove the carry
    byteC = byteC ^ csumB; // XOR byteC with csumB
    // Now write the decoded bytes to the output data array
    decoded->data[j++] = byteA;
    decoded->data[j++] = byteB;
}

// This function encodes an entire decoded track (all sectors) into GCR format
// It takes the track number and a pointer to an array of DecodedSector structs as input
// And outputs an array of GcrSector structs
void encodeTrackToGCR(uint8_t track, DriveType driveType, DecodedSector decodedSectors[2][12], GcrSector gcrSectors[2][12]) {
    uint32_t startTime = micros();
    // First we need to figure out how many sectors are on this track
    uint8_t sectorCount = 0;
    if (track <= 15) {
        sectorCount = 12;
    }
    else if (track <= 31) {
        sectorCount = 11;
    }
    else if (track <= 47) {
        sectorCount = 10;
    }
    else if (track <= 63) {
        sectorCount = 9;
    }
    else {
        sectorCount = 8;
    }
    // Now encode each sector into GCR format and put it in the output array
    // If it's a double-sided disk, we have to do both sides
    if (driveType == Drive800) {
        for(int i = 0; i < 2; i++) {
            for (int j = 0; j < sectorCount; j++) {
                encodeSector(&decodedSectors[i][j], &gcrSectors[i][j]);
            }
        }
    }
    // Otherwise it's a single-sided disk, so just do side 0
    else {
        for (int i = 0; i < sectorCount; i++) {
            encodeSector(&decodedSectors[0][i], &gcrSectors[0][i]);
        }
    }
    Serial.printf("Track encoding to GCR took %u microseconds\n", micros() - startTime);
}