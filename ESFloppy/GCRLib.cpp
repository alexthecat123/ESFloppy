#include <Arduino.h>
#include "GCRLib.h"
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
// These interleave tables have to be valid for all the track sizes, both Twiggy and Sony
// Each table is indexed like table[sectorsPerTrack][slot] where sectorsPerTrack is the number of sectors on the current track and slot is the physical slot number (0-21) that we're trying to find the logical sector for
// A little convenience here: Sony drives go from 8-12 sectors, and the Twiggy drives go from 15-22 sectors, so we can actually use the same table for both since there's no overlap
// That leaves rows 0-7, 13 and 14 with nothing to describe, so they're filled with all 0xFF's
#define INTERLEAVE_NONE {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

extern uint8_t interleave2to1[23][22] = {
    INTERLEAVE_NONE, // 0 sectors/track (unused)
    INTERLEAVE_NONE, // 1 sector/track (unused)
    INTERLEAVE_NONE, // 2 sectors/track (unused)
    INTERLEAVE_NONE, // 3 sectors/track (unused)
    INTERLEAVE_NONE, // 4 sectors/track (unused)
    INTERLEAVE_NONE, // 5 sectors/track (unused)
    INTERLEAVE_NONE, // 6 sectors/track (unused)
    INTERLEAVE_NONE, // 7 sectors/track (unused)
    {   0,    4,    1,    5,    2,    6,    3,    7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 8 sectors/track (Sony tracks 64-79)
    {   0,    5,    1,    6,    2,    7,    3,    8,    4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 9 sectors/track (Sony tracks 48-63)
    {   0,    5,    1,    6,    2,    7,    3,    8,    4,    9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 10 sectors/track (Sony tracks 32-47)
    {   0,    6,    1,    7,    2,    8,    3,    9,    4,   10,    5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 11 sectors/track (Sony tracks 16-31)
    {   0,    6,    1,    7,    2,    8,    3,    9,    4,   10,    5,   11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 12 sectors/track (Sony tracks 0-15)
    INTERLEAVE_NONE, // 13 sectors/track (unused)
    INTERLEAVE_NONE, // 14 sectors/track (unused)
    {   0,    8,    1,    9,    2,   10,    3,   11,    4,   12,    5,   13,    6,   14,    7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 15 sectors/track (Twiggy tracks 42-45)
    {   0,    8,    1,    9,    2,   10,    3,   11,    4,   12,    5,   13,    6,   14,    7,   15, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 16 sectors/track (Twiggy tracks 35-41)
    {   0,    9,    1,   10,    2,   11,    3,   12,    4,   13,    5,   14,    6,   15,    7,   16,    8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 17 sectors/track (Twiggy tracks 29-34)
    {   0,    9,    1,   10,    2,   11,    3,   12,    4,   13,    5,   14,    6,   15,    7,   16,    8,   17, 0xFF, 0xFF, 0xFF, 0xFF}, // 18 sectors/track (Twiggy tracks 23-28)
    {   0,   10,    1,   11,    2,   12,    3,   13,    4,   14,    5,   15,    6,   16,    7,   17,    8,   18,    9, 0xFF, 0xFF, 0xFF}, // 19 sectors/track (Twiggy tracks 17-22)
    {   0,   10,    1,   11,    2,   12,    3,   13,    4,   14,    5,   15,    6,   16,    7,   17,    8,   18,    9,   19, 0xFF, 0xFF}, // 20 sectors/track (Twiggy tracks 11-16)
    {   0,   11,    1,   12,    2,   13,    3,   14,    4,   15,    5,   16,    6,   17,    7,   18,    8,   19,    9,   20,   10, 0xFF}, // 21 sectors/track (Twiggy tracks 4-10)
    {   0,   11,    1,   12,    2,   13,    3,   14,    4,   15,    5,   16,    6,   17,    7,   18,    8,   19,    9,   20,   10,   21}, // 22 sectors/track (Twiggy tracks 0-3)
};

// Same idea for 4:1 interleave
extern uint8_t interleave4to1[23][22] = {
    INTERLEAVE_NONE, // 0 sectors/track (unused)
    INTERLEAVE_NONE, // 1 sector/track (unused)
    INTERLEAVE_NONE, // 2 sectors/track (unused)
    INTERLEAVE_NONE, // 3 sectors/track (unused)
    INTERLEAVE_NONE, // 4 sectors/track (unused)
    INTERLEAVE_NONE, // 5 sectors/track (unused)
    INTERLEAVE_NONE, // 6 sectors/track (unused)
    INTERLEAVE_NONE, // 7 sectors/track (unused)
    {   0,    2,    4,    6,    1,    3,    5,    7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 8 sectors/track (Sony tracks 64-79)
    {   0,    7,    5,    3,    1,    8,    6,    4,    2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 9 sectors/track (Sony tracks 48-63)
    {   0,    5,    3,    8,    1,    6,    4,    9,    2,    7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 10 sectors/track (Sony tracks 32-47)
    {   0,    3,    6,    9,    1,    4,    7,   10,    2,    5,    8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 11 sectors/track (Sony tracks 16-31)
    {   0,    3,    6,    9,    1,    4,    7,   10,    2,    5,    8,   11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 12 sectors/track (Sony tracks 0-15)
    INTERLEAVE_NONE, // 13 sectors/track (unused)
    INTERLEAVE_NONE, // 14 sectors/track (unused)
    {   0,    4,    8,   12,    1,    5,    9,   13,    2,    6,   10,   14,    3,    7,   11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 15 sectors/track (Twiggy tracks 42-45)
    {   0,    4,    8,   12,    1,    5,    9,   13,    2,    6,   10,   14,    3,    7,   11,   15, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 16 sectors/track (Twiggy tracks 35-41)
    {   0,   13,    9,    5,    1,   14,   10,    6,    2,   15,   11,    7,    3,   16,   12,    8,    4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 17 sectors/track (Twiggy tracks 29-34)
    {   0,    9,    5,   14,    1,   10,    6,   15,    2,   11,    7,   16,    3,   12,    8,   17,    4,   13, 0xFF, 0xFF, 0xFF, 0xFF}, // 18 sectors/track (Twiggy tracks 23-28)
    {   0,    5,   10,   15,    1,    6,   11,   16,    2,    7,   12,   17,    3,    8,   13,   18,    4,    9,   14, 0xFF, 0xFF, 0xFF}, // 19 sectors/track (Twiggy tracks 17-22)
    {   0,    5,   10,   15,    1,    6,   11,   16,    2,    7,   12,   17,    3,    8,   13,   18,    4,    9,   14,   19, 0xFF, 0xFF}, // 20 sectors/track (Twiggy tracks 11-16)
    {   0,   16,   11,    6,    1,   17,   12,    7,    2,   18,   13,    8,    3,   19,   14,    9,    4,   20,   15,   10,    5, 0xFF}, // 21 sectors/track (Twiggy tracks 4-10)
    {   0,   11,    6,   17,    1,   12,    7,   18,    2,   13,    8,   19,    3,   14,    9,   20,    4,   15,   10,   21,    5,   16}, // 22 sectors/track (Twiggy tracks 0-3)
};

// We'll also make a straight-through mapping for 1:1 just so that getInterleaveTable() can return a valid pointer for it
extern uint8_t interleave1to1[23][22] = {
    INTERLEAVE_NONE, // 0 sectors/track (unused)
    INTERLEAVE_NONE, // 1 sector/track (unused)
    INTERLEAVE_NONE, // 2 sectors/track (unused)
    INTERLEAVE_NONE, // 3 sectors/track (unused)
    INTERLEAVE_NONE, // 4 sectors/track (unused)
    INTERLEAVE_NONE, // 5 sectors/track (unused)
    INTERLEAVE_NONE, // 6 sectors/track (unused)
    INTERLEAVE_NONE, // 7 sectors/track (unused)
    {   0,    1,    2,    3,    4,    5,    6,    7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 8 sectors/track (Sony tracks 64-79)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 9 sectors/track (Sony tracks 48-63)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 10 sectors/track (Sony tracks 32-47)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 11 sectors/track (Sony tracks 16-31)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 12 sectors/track (Sony tracks 0-15)
    INTERLEAVE_NONE, // 13 sectors/track (unused)
    INTERLEAVE_NONE, // 14 sectors/track (unused)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 15 sectors/track (Twiggy tracks 42-45)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 16 sectors/track (Twiggy tracks 35-41)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // 17 sectors/track (Twiggy tracks 29-34)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17, 0xFF, 0xFF, 0xFF, 0xFF}, // 18 sectors/track (Twiggy tracks 23-28)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18, 0xFF, 0xFF, 0xFF}, // 19 sectors/track (Twiggy tracks 17-22)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18,   19, 0xFF, 0xFF}, // 20 sectors/track (Twiggy tracks 11-16)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18,   19,   20, 0xFF}, // 21 sectors/track (Twiggy tracks 4-10)
    {   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18,   19,   20,   21}, // 22 sectors/track (Twiggy tracks 0-3)
};

// This function takes a decoded sector and encodes it into GCR format
void encodeSector(DecodedSector* decoded, GcrSector* gcr, DiskImageMetadata* metadata) {
    // First let's fill in the header fields, making sure to GCR-encode each one along the way
    uint8_t loTrack = decoded->track & 0x3F; // Low 6 bits of track (which is the full track number for Twiggy drives)
    uint8_t sectorNum = decoded->sector & 0x3F; // Sector number gets pulled straight over, with a mask to keep it within 6 bits
    // hiTrackSide differs depending on whether this is a Sony or Twiggy drive
    // For Sony drives, the high track bit is in bit 0, side is in bit 5
    // For Twiggies, the side is in bit 0 and everything else is 0
    uint8_t hiTrackSide = metadata->driveType == DriveTwiggy ? ((decoded->side & 0x01) << 0) : (((decoded->track & 0x40) >> 6) | ((decoded->side & 0x01) << 5));
    uint8_t format = decoded->format & 0x3F; // Format byte gets pulled straight over; just isolate the low 6 bits to fit with GCR
    gcr->loTrack = gcr_6to8[loTrack]; // Low 6 bits of track
    gcr->sector = gcr_6to8[sectorNum]; // Sector number gets pulled straight over
    gcr->hiTrackSide = gcr_6to8[hiTrackSide]; // High track and/or side depending on drive type
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

bool decodeSector(GcrSector* gcr, DecodedSector* decoded, DiskImageMetadata* metadata) {
    // Make a backup copy of the current contents of decoded so we can restore it if the sector we're decoding now is invalid
    DecodedSector backup;
    memcpy(&backup, decoded, sizeof(DecodedSector));
    // Decoding is (obviously) just the reverse of encoding; first do the header
    bool sectorValid = true; // Assume the sector is valid until we find a problem
    // Reconstruct the track number from the low and high bits (Sony) or just the low bits (Twiggy), decoding the GCR along the way
    decoded->track = metadata->driveType == DriveTwiggy ? (gcr_8to6[gcr->loTrack]) : (gcr_8to6[gcr->loTrack] | ((gcr_8to6[gcr->hiTrackSide] & 0x01) << 6));
    // The sector comes straight over
    decoded->sector = gcr_8to6[gcr->sector];
    // The side is in bit 0 (Twiggy) or bit 5 (Sony) of hiTrackSide depending on the drive type
    decoded->side = metadata->driveType == DriveTwiggy ? (gcr_8to6[gcr->hiTrackSide] & 0x01) : ((gcr_8to6[gcr->hiTrackSide] & 0x20) >> 5);
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
// It takes the track number (or carriage position for Twiggy) and a pointer to an array of DecodedSector structs as input
// And outputs an array of GcrSector structs
void encodeTrackToGCR(uint8_t track, DecodedSector decodedSectors[2][22], GcrSector gcrSectors[2][22], DiskImageMetadata* metadata) {
    // Start by making sure that the track provided to us is in bounds for our given drive type; just return if not
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // So that's 0-79 on Sony
        if (track > 79) {
            return;
        }
    } else {
        // And 0-45 on Twiggy
        if (track > 45) {
            return;
        }
    }
    
    // Next we need to figure out how many sectors are on this track
    uint32_t sectorCount[2] = {0, 0};
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // For Sony, this is really easy; both sides have the same number of sectors
        sectorCount[0] = sectorsPerTrackSony[track];
        sectorCount[1] = sectorCount[0];
    } else {
        // For Twiggy, we have to check each side separately since they have different numbers of sectors thanks to the offset heads
        sectorCount[0] = sectorsPerTrackTwiggy[45 - track]; // Side 0 is 45 - the carriage position
        sectorCount[1] = sectorsPerTrackTwiggy[track]; // And side 1 is just the carriage position
    }
    // Now encode each sector into GCR format and put it in the output array
    // If it's a double-sided disk (Twiggy or 800K), we have to do both sides
    if ((metadata->driveType == DriveTwiggy) || (metadata->driveType == Drive800)) {
        // Check which interleave table to use; this gets really annoying to do over and over again
        // So just call the getInterleaveTable function to get the proper one for this drive type and sector format byte
        // The false tells it to use the decodedSectors format byte instead of the metadata one
        InterleaveTable interleaveTable = getInterleaveTable(metadata, decodedSectors[0][0].format, false);
        for(int i = 0; i < 2; i++) {
            // And now loop over every sector and every side and encode them into GCR using that interleave table
            // Make sure to also pick the proper sectorCount for each side since this matters for Twiggy
            for (int j = 0; j < sectorCount[i]; j++) {
                encodeSector(&decodedSectors[i][interleaveTable[sectorCount[i]][j]], &gcrSectors[i][j], metadata);
            }
        }
    }
    // Otherwise it's a single-sided disk, so only put actual data on side 0
    // No need to do the Sony vs Twiggy check here since we already know it's single-sided and Twiggies are double-sided
    else {
        InterleaveTable interleaveTable = getInterleaveTable(metadata, decodedSectors[0][0].format, false);
        for (int i = 0; i < sectorCount[0]; i++) {
            encodeSector(&decodedSectors[0][interleaveTable[sectorCount[0]][i]], &gcrSectors[0][i], metadata);
        }
        // This doesn't mean that our work is over though; we need to fill side 1 with empty sectors
        // The reason we need to do this is really stupid; the Lisa 800K disk ROM will try and format both sides of a disk regardless of whether it's 400K/800K
        // So when it goes to do the verify pass, it expects to find fresh empty sectors on side 1, and will fail if not
        // So on a 400K disk, we need to spoof fake empty sectors on side 1 so that the verify pass will succeed
        // This is the only time where the floppy controller will actually use side 1 of a 400K disk (unless an OS totally killed itself or something), so no need to worry about any other cases
        for (int i = 0; i < sectorCount[1]; i++) {
            // Iterate through all of the sectors on side 1 and start by constructing their headers
            // The headers are the same as the corresponding sectors on side 0, except the side bit is set to 1
            // And of course this also means that we need to recompute the header checksum
            gcrSectors[1][i].loTrack = gcrSectors[0][i].loTrack;
            gcrSectors[1][i].sector = gcrSectors[0][i].sector;
            gcrSectors[1][i].hiTrackSide = gcr_6to8[(gcr_8to6[gcrSectors[0][i].hiTrackSide] | 0x20)]; // Set the side bit to 1
            gcrSectors[1][i].format = gcrSectors[0][i].format;
            gcrSectors[1][i].headerChecksum = gcr_6to8[gcr_8to6[gcrSectors[1][i].loTrack] ^ gcr_8to6[gcrSectors[1][i].sector] ^ gcr_8to6[gcrSectors[1][i].hiTrackSide] ^ gcr_8to6[gcrSectors[1][i].format]]; // Recompute the header checksum with the side bit set to 1
            gcrSectors[1][i].sector_again = gcrSectors[0][i].sector_again;
            // Now fill the data and data checksum portion of the sector with whatever was on side 0
            // In the format case, this will be all 0's, and that's the only case we care about
            memcpy(gcrSectors[1][i].data, gcrSectors[0][i].data, sizeof(gcrSectors[0][i].data));
            memcpy(gcrSectors[1][i].dataChecksum, gcrSectors[0][i].dataChecksum, sizeof(gcrSectors[0][i].dataChecksum));
        }
    }

    // One additional nuance here in the Twiggy case
    // All Twiggy tracks have a sequence of "timing bytes" (0xA9) right before the header prologue
    // But ONLY on logical sector 0 of each track
    // The easiest way to handle this is to just check for Twiggy, and then just replace the last 10 sync bytes with A9's if so
    if (metadata->driveType == DriveTwiggy) {
        // First, get a pointer to 10 bytes before the header prologue of sector 0, ensuring that we use logical not physical sector 0
        InterleaveTable interleaveTable = getInterleaveTable(metadata, gcr_8to6[gcrSectors[0][0].format], false);
        // Find the proper physical slot that holds logical sector 0 the same way that we do in fluxRW.cpp
        uint32_t slot = 0;
        for (uint32_t i = 0; i < sectorCount[0]; i++) {
            if (interleaveTable[sectorCount[0]][i] == 0) {
                slot = i;
                break;
            }
        }
        uint8_t* timingBytes = gcrSectors[0][slot].headerPrologue - 10;
        for (int i = 0; i < 10; i++) {
            timingBytes[i] = 0xA9; // And replace all 10 of them with A9's
        }
        // Now repeat for side 1 as well
        for (uint32_t i = 0; i < sectorCount[1]; i++) {
            if (interleaveTable[sectorCount[1]][i] == 0) {
                slot = i;
                break;
            }
        }
        timingBytes = gcrSectors[1][slot].headerPrologue - 10;
        for (int i = 0; i < 10; i++) {
            timingBytes[i] = 0xA9;
        }
    }
}

// This function decodes an entire GCR track (all sectors) into decoded format
void decodeTrackFromGCR(uint8_t track, GcrSector gcrSectors[2][22], DecodedSector decodedSectors[2][22], DiskImageMetadata* metadata) {
    // Same as with encodeTrackToGCR, start by making sure that track is in bounds
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // So that's 0-79 on Sony
        if (track > 79) {
            return;
        }
    } else {
        // And 0-45 on Twiggy
        if (track > 45) {
            return;
        }
    }

    // Now figure out how many sectors are on each side of this track
    uint32_t sectorCount[2] = {0, 0};
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // Both sides have the same number of sectors on Sony
        sectorCount[0] = sectorsPerTrackSony[track];
        sectorCount[1] = sectorCount[0];
    } else {
        // Each side has a different number of sectors on Twiggy
        sectorCount[0] = sectorsPerTrackTwiggy[45 - track];
        sectorCount[1] = sectorsPerTrackTwiggy[track];
    }
    // Now decode each sector from GCR format into decoded format
    // If it's a double-sided disk (Twiggy/800K Sony), we have to do both sides
    if (metadata->driveType == DriveTwiggy || metadata->driveType == Drive800) {
        // As in encodeTrackToGCR, use getInterleaveTable to figure out which interleave table to use for this sector
        // Use the format from gcrSectors instead of decodedSectors since gcrSectors might have been modified by a format op from the Lisa
        // And this change won't have been committed to decodedSectors yet
        InterleaveTable interleaveTable = getInterleaveTable(metadata, gcr_8to6[gcrSectors[0][0].format], false);
        for(int i = 0; i < 2; i++) {
            for (int j = 0; j < sectorCount[i]; j++) {
                // And decode the sector accordingly, using the proper side of sectorCount
                decodeSector(&gcrSectors[i][j], &decodedSectors[i][interleaveTable[sectorCount[i]][j]], metadata);
            }
        }
    }
    // Otherwise it's a single-sided disk, so just do side 0
    // As with encoding, no need to do the Twiggy vs Sony check here since Twiggies can't be single-sided
    else {
        InterleaveTable interleaveTable = getInterleaveTable(metadata, gcr_8to6[gcrSectors[0][0].format], false);
        for (int i = 0; i < sectorCount[0]; i++) {
            decodeSector(&gcrSectors[0][i], &decodedSectors[0][interleaveTable[sectorCount[0]][i]], metadata);
        }
    }
}

// Returns a pointer to the proper interleave table for the given disk image metadata
// The format parameter can come from either diskFormat in the metadata header (useMetadataFormat = true)
// Or from the uint8_t format parameter passed in (useMetadataFormat = false)
__attribute__((optimize("Ofast"))) IRAM_ATTR InterleaveTable getInterleaveTable(DiskImageMetadata* metadata, uint8_t format, bool useMetadataFormat) {
    // For Sony drives, we can check the format byte to directly determine the interleave
    // But for Twiggies, it's never given anywhere and so we kind of just have to guess
    // I'm going to assume 2:1 interleave for Twiggy on Mac/Lisa and 4:1 on Apple II and Apple ///
    // I can confirm that 2:1 is correct for the Lisa at the very least, but I have no clue about the others
    uint8_t formatByte = useMetadataFormat ? metadata->header.diskFormat : format;
    // First, handle the Twiggy case
    if (metadata->driveType == DriveTwiggy) {
        if ((formatByte & 0x3F) == 0x00) {
            return interleave4to1; // For Apple II and Apple ///, assume 4:1 interleave
        }
        // For all other cases (Lisa, Mac, and unknown), use 2:1 interleave
        return interleave2to1;
    }
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // For Sony drives, we can use the format byte to determine the interleave directly
        if ((formatByte & 0x1F) == 0x04) {
            return interleave4to1; // Format byte of 0x04 indicates 4:1 interleave
        } else if ((formatByte & 0x1F) == 0x01) {
            return interleave1to1; // Format byte of 0x01 indicates 1:1 interleave
        } else {
            return interleave2to1; // All other format bytes indicate 2:1 interleave
        }
    }
    return interleave2to1; // Default to 2:1 interleave for any other drive types, although we should never end up here
}