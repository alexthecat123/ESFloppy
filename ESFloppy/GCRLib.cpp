#include <Arduino.h>
#include "types.h"

// All the routines related to GCR encoding and decoding for ESFloppy

// Lookup tables for GCR encoding and decoding
// This table converts 6-bit nibbles to 8-bit GCR bytes
extern const uint8_t gcr_6to8[64] = {
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
extern const uint8_t gcr_8to6[256] = {
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

// We also need two interleave lookup tables, one for 2:1 interleave and one for 4:1 interleave
// The DC42 format technically supports other interleave factors too, but I've never seen them used
// So anything that's not 1:1, 2:1, or 4:1 will just be treated as 2:1
// These interleave tables have to be valid for all 5 track sizes (12, 11, 10, 9, and 8 sectors per track)
// Each table is indexed like table[sectorsPerTrack][slot] where sectorsPerTrack is the number of sectors on the current track and slot is the physical slot number (0-11) that we're trying to find the logical sector for
extern const uint8_t interleave2to1[13][12] = {
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 0 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 1 sector/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 2 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 3 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 4 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 5 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 6 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 7 sectors/track (unused)
    {   0,    4,    1,    5,    2,    6,    3,    7, 0xFF, 0xFF, 0xFF, 0xFF}, // 8 sectors/track (tracks 64-79)
    {   0,    5,    1,    6,    2,    7,    3,    8,    4, 0xFF, 0xFF, 0xFF}, // 9 sectors/track (tracks 48-63)
    {   0,    5,    1,    6,    2,    7,    3,    8,    4,    9, 0xFF, 0xFF}, // 10 sectors/track (tracks 32-47)
    {   0,    6,    1,    7,    2,    8,    3,    9,    4,   10,    5, 0xFF}, // 11 sectors/track (tracks 16-31)
    {   0,    6,    1,    7,    2,    8,    3,    9,    4,   10,    5,   11}, // 12 sectors/track (tracks 0-15)
};

// Same idea for 4:1 interleave
// Note that 4 and 8 share a common factor, so a true 4:1 layout is impossible on an 8-sector track;
// that row ends up alternating 4 and 5 slots between consecutive logical sectors, which is as close as it gets
extern const uint8_t interleave4to1[13][12] = {
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 0 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 1 sector/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 2 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 3 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 4 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 5 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 6 sectors/track (unused)
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 7 sectors/track (unused)
    {   0,    2,    4,    6,    1,    3,    5,    7, 0xFF, 0xFF, 0xFF, 0xFF}, // 8 sectors/track (tracks 64-79)
    {   0,    7,    5,    3,    1,    8,    6,    4,    2, 0xFF, 0xFF, 0xFF}, // 9 sectors/track (tracks 48-63)
    {   0,    5,    3,    8,    1,    6,    4,    9,    2,    7, 0xFF, 0xFF}, // 10 sectors/track (tracks 32-47)
    {   0,    3,    6,    9,    1,    4,    7,   10,    2,    5,    8, 0xFF}, // 11 sectors/track (tracks 16-31)
    {   0,    3,    6,    9,    1,    4,    7,   10,    2,    5,    8,   11}, // 12 sectors/track (tracks 0-15)
};

// This function takes a decoded sector and encodes it into GCR format
void encodeSector(DecodedSector* decoded, GcrSector* gcr) {
    // First let's fill in the header fields, making sure to GCR-encode each one along the way
    uint8_t loTrack = decoded->track & 0x3F; // Low 6 bits of track
    uint8_t sectorNum = decoded->sector & 0x3F; // Sector number gets pulled straight over, with a mask to keep it within 6 bits
    uint8_t hiTrackSide = ((decoded->track & 0x40) >> 6) | ((decoded->side & 0x01) << 5); // High track bit in bit 0, side in bit 5
    uint8_t format = decoded->format & 0x3F; // Format byte gets pulled straight over too; just isolate the low 6 bits to fit with GCR
    gcr->loTrack = gcr_6to8[loTrack]; // Low 6 bits of track
    gcr->sector = gcr_6to8[sectorNum]; // Sector number gets pulled straight over
    gcr->hiTrackSide = gcr_6to8[hiTrackSide]; // High track bit in bit 0, side in bit 5
    gcr->format = gcr_6to8[format]; // Format byte gets pulled straight over too
    // Simple XOR checksum of the header fields; compute the checksum of the raw (non-GCR) values and then encode it as GCR
    gcr->headerChecksum = gcr_6to8[loTrack ^ sectorNum ^ hiTrackSide ^ format];
    // Now onto the data stuff; starting with the sector number again
    gcr->sector_again = gcr_6to8[sectorNum];
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
    // No byteC or csumC since there isn't a third byte on this last chunk, so we can skip that part
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

bool decodeSector(GcrSector* gcr, DecodedSector* decoded) {
    // Make a backup copy of the current contents of decoded so we can restore it if the sector we're decoding now is invalid
    DecodedSector backup;
    memcpy(&backup, decoded, sizeof(DecodedSector));
    // Decoding is (obviously) just the reverse of encoding; first do the header
    bool sectorValid = true; // Assume the sector is valid until we find a problem
    // Reconstruct the track number from the low and high bits, decoding the GCR along the way
    decoded->track = gcr_8to6[gcr->loTrack] | ((gcr_8to6[gcr->hiTrackSide] & 0x01) << 6);
    // The sector comes straight over
    decoded->sector = gcr_8to6[gcr->sector];
    // The side is in bit 5 of hiTrackSide
    decoded->side = (gcr_8to6[gcr->hiTrackSide] & 0x20) >> 5;
    // And the format byte comes straight over too
    decoded->format = gcr_8to6[gcr->format];
    // Now check the header checksum; it should be the XOR of the other header fields
    if (gcr_8to6[gcr->headerChecksum] != (gcr_8to6[gcr->loTrack] ^ gcr_8to6[gcr->sector] ^ gcr_8to6[gcr->hiTrackSide] ^ gcr_8to6[gcr->format])) {
        sectorValid = false; // If not, then the header is invalid and we should mark the sector as bad
    }
    // Also check to make sure that all of the header fields are valid GCR bytes; if any of them are bigger than 6 bits, then they aren't
    if ((gcr_8to6[gcr->loTrack] | gcr_8to6[gcr->sector] | gcr_8to6[gcr->hiTrackSide] | gcr_8to6[gcr->format] | gcr_8to6[gcr->headerChecksum]) & 0xC0) {
        sectorValid = false;
    }
    // Also make sure that sector_again is the same as sector; sector_again isn't covered by the checksum
    if (decoded->sector != gcr_8to6[gcr->sector_again]) {
        sectorValid = false; // If not, then we're once again invalid
    }
    // Now we've got to do the stupid data decoding algorithm
    uint16_t csumA = 0, csumB = 0;
    uint8_t csumC = 0;
    uint8_t carry;
    uint8_t byteA, byteB, byteC;
    uint8_t nibl1, nibl2, nibl3, nibl4;
    uint8_t badNibls = 0; // Allows us to see if any of the data nibbles are invalid GCR bytes
    uint16_t i = 0; // Index for the GCR data array
    uint16_t j = 0; // Index for the decoded data array
    // Now we can start decoding the data itself
    while (j < 522) { // Process the first 522 bytes (174 chunks of 3 bytes)
        // First read 4 GCR bytes and convert them back to nibbles
        nibl1 = gcr_8to6[gcr->data[i++]];
        nibl2 = gcr_8to6[gcr->data[i++]];
        nibl3 = gcr_8to6[gcr->data[i++]];
        nibl4 = gcr_8to6[gcr->data[i++]];
        badNibls |= nibl1 | nibl2 | nibl3 | nibl4; // If any of the nibbles are invalid GCR bytes, then the high 2 bits of badNibls will be set
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
    // Undo the modifications made during encoding
    // No byteC or csumC since there isn't a third byte on this last chunk, so we can skip that part
    csumC = (csumC << 1) | (csumC >> 7); // Rotate csumC left by 1
    carry = csumC & 0x01; // Get the low (carry) bit of csumC
    byteA = byteA ^ csumC; // XOR byteA with csumC
    csumA = csumA + byteA + carry; // Add byteA and carry to csumA
    carry = csumA & 0x100 ? 1 : 0; // Get the carry bit of csumA
    csumA = csumA & 0xFF; // And mask csumA to 8 bits to remove the carry
    byteB = byteB ^ csumA; // XOR byteB with csumA
    csumB = csumB + byteB + carry; // Add byteB and carry to csumB
    csumB = csumB & 0xFF; // And mask csumB to 8 bits to remove the carry
    // Now write the decoded bytes to the output data array
    decoded->data[j++] = byteA;
    decoded->data[j++] = byteB;

    // Finally, check the data checksum for validity
    nibl1 = gcr_8to6[gcr->dataChecksum[0]]; // Get the 4 GCR nibbles of the data checksum and convert them back to 6-bit values
    nibl2 = gcr_8to6[gcr->dataChecksum[1]];
    nibl3 = gcr_8to6[gcr->dataChecksum[2]];
    nibl4 = gcr_8to6[gcr->dataChecksum[3]];
    badNibls |= nibl1 | nibl2 | nibl3 | nibl4; // If any of the nibbles are invalid GCR bytes, then the high 2 bits of badNibls will be set
    uint8_t storedCsumA = ((nibl1 & 0x30) << 2) | (nibl2 & 0x3F); // Now build the stored checksums from the nibbles
    uint8_t storedCsumB = ((nibl1 & 0x0C) << 4) | (nibl3 & 0x3F);
    uint8_t storedCsumC = ((nibl1 & 0x03) << 6) | (nibl4 & 0x3F);
    if ((badNibls & 0xC0) || storedCsumA != csumA || storedCsumB != csumB || storedCsumC != csumC) {
        sectorValid = false; // If any of the data nibbles are invalid GCR bytes, or if the checksums don't match, then the data is invalid
    }
    // If the sector was invalid, restore the backup copy of the decoded sector and return false
    if (!sectorValid) {
        memcpy(decoded, &backup, sizeof(DecodedSector));
        return false;
    }
    return true; // Otherwise return true
}

// This function encodes an entire decoded track (all sectors) into GCR format
// It takes the track number and a pointer to an array of DecodedSector structs as input
// And outputs an array of GcrSector structs
void encodeTrackToGCR(uint8_t track, DecodedSector decodedSectors[2][12], GcrSector gcrSectors[2][12], DiskImageMetadata* metadata) {
    // First we need to figure out how many sectors are on this track
    uint32_t sectorCount = sectorsPerTrack[track];
    // Now encode each sector into GCR format and put it in the output array
    // If it's a double-sided disk, we have to do both sides
    if (metadata->driveType == Drive800) {
        for(int i = 0; i < 2; i++) {
            for (int j = 0; j < sectorCount; j++) {
                // Check which interleave table to use based on the interleave factor in the metadata
                if ((decodedSectors[0][0].format & 0x1F) == 0x04) {
                    // We have a case for 4:1 interleave
                    encodeSector(&decodedSectors[i][interleave4to1[sectorCount][j]], &gcrSectors[i][j]);
                } else if ((decodedSectors[0][0].format & 0x1F) == 0x01) {
                    // And another case for 1:1 interleave, which doesn't need an interleave table at all
                    encodeSector(&decodedSectors[i][j], &gcrSectors[i][j]);
                } else {
                    // Otherwise we default to 2:1 interleave
                    encodeSector(&decodedSectors[i][interleave2to1[sectorCount][j]], &gcrSectors[i][j]);
                }
            }
        }
    }
    // Otherwise it's a single-sided disk, so just do side 0
    else {
        for (int i = 0; i < sectorCount; i++) {
            if ((decodedSectors[0][0].format & 0x1F) == 0x04) {
                encodeSector(&decodedSectors[0][interleave4to1[sectorCount][i]], &gcrSectors[0][i]);
            } else if ((decodedSectors[0][0].format & 0x1F) == 0x01) {
                encodeSector(&decodedSectors[0][i], &gcrSectors[0][i]);
            } else {
                encodeSector(&decodedSectors[0][interleave2to1[sectorCount][i]], &gcrSectors[0][i]);
            }
        }
    }
}

// This function decodes an entire GCR track (all sectors) into decoded format
void decodeTrackFromGCR(uint8_t track, GcrSector gcrSectors[2][12], DecodedSector decodedSectors[2][12], DiskImageMetadata* metadata) {
    // First we need to figure out how many sectors are on this track
    uint32_t sectorCount = sectorsPerTrack[track];
    // Now decode each sector from GCR format into decoded format
    // If it's a double-sided disk, we have to do both sides
    if (metadata->driveType == Drive800) {
        for(int i = 0; i < 2; i++) {
            for (int j = 0; j < sectorCount; j++) {
                // Check which interleave table to use based on the interleave factor in the metadata
                // Use the format from gcrSectors instead of decodedSectors since gcrSectors might have been modified by a format op from the Lisa
                // And this change won't have been committed to decodedSectors yet
                if ((gcr_8to6[gcrSectors[i][j].format] & 0x1F) == 0x04) {
                    // We have a case for 4:1 interleave
                    decodeSector(&gcrSectors[i][j], &decodedSectors[i][interleave4to1[sectorCount][j]]);
                } else if ((gcr_8to6[gcrSectors[i][j].format] & 0x1F) == 0x01) {
                    // And another case for 1:1 interleave, which doesn't need an interleave table at all
                    decodeSector(&gcrSectors[i][j], &decodedSectors[i][j]);
                } else {
                    // Otherwise we default to 2:1 interleave
                    decodeSector(&gcrSectors[i][j], &decodedSectors[i][interleave2to1[sectorCount][j]]);
                }
            }
        }
    }
    // Otherwise it's a single-sided disk, so just do side 0
    else {
        for (int i = 0; i < sectorCount; i++) {
            if ((gcr_8to6[gcrSectors[0][i].format] & 0x1F) == 0x04) {
                decodeSector(&gcrSectors[0][i], &decodedSectors[0][interleave4to1[sectorCount][i]]);
            } else if ((gcr_8to6[gcrSectors[0][i].format] & 0x1F) == 0x01) {
                decodeSector(&gcrSectors[0][i], &decodedSectors[0][i]);
            } else {
                decodeSector(&gcrSectors[0][i], &decodedSectors[0][interleave2to1[sectorCount][i]]);
            }
        }
    }
}