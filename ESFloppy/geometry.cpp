#include <Arduino.h>

// Drive geometry LUTs for the various different drive types that we support

// Lookup table for number of sectors per track for each of the 80 tracks on a standard 400K/800K floppy
uint32_t sectorsPerTrackSony[80] = {
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, // Tracks 0-15
    11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, // Tracks 16-31
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, // Tracks 32-47
    9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9, // Tracks 48-63
    8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8, // Tracks 64-79
};

// Same thing, but for the Twiggy drive, which has 46 tracks and a VERY different sector layout
uint32_t sectorsPerTrackTwiggy[46] = {
    22, 22, 22, 22, // Tracks 0-3
    21, 21, 21, 21, 21, 21, 21, // Tracks 4-10
    20, 20, 20, 20, 20, 20, // Tracks 11-16
    19, 19, 19, 19, 19, 19, // Tracks 17-22
    18, 18, 18, 18, 18, 18, // Tracks 23-28
    17, 17, 17, 17, 17, 17, // Tracks 29-34
    16, 16, 16, 16, 16, 16, 16, // Tracks 35-41
    15, 15, 15, 15 // Tracks 42-45
};

// Another set of LUTs for the tachometer pulse frequency that's needed for each track
// These are only used for Sony drives; Twiggy regulates its speed internally
// Are these right? One source (the 800K drive spec) says this, another (the 400K spec) is slightly different...
// And I can't find the final source, but I got something from somewhere else that breaks it down quite differently:
/*
    Track/RPM pairings
    0...9 363
    10...25 393
    26...40 429
    41...55 472
    56...71 524
    72...79 590
*/
uint32_t tachPulsesPerTrackMac[80] = {
    394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, 394, // Tracks 0-15
    429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, 429, // Tracks 16-31
    472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, 472, // Tracks 32-47
    525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, 525, // Tracks 48-63
    578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, // Tracks 64-79
};

// For the sake of speed, we can't be converting these TACH RPM values into LEDC divider values on the fly
// So precompute the dividers and store them in another LUT
uint32_t tachDividerPerTrackMac[80] = {
    203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, 203045, // Tracks 0-15
    186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, 186480, // Tracks 16-31
    169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, 169491, // Tracks 32-47
    152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, 152380, // Tracks 48-63
    138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, 138408, // Tracks 64-79
};

// The Lisa uses a slightly different set of TACH pulse frequencies, so here's a separate set of LUTs for those
uint32_t tachPulsesPerTrackLisa[80] = {
    407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, 407, // Tracks 0-15
    443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, 443, // Tracks 16-31
    489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, 489, // Tracks 32-47
    545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, 545, // Tracks 48-63
    613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, 613, // Tracks 64-79
};

uint32_t tachDividerPerTrackLisa[80] = {
    196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, 196560, // Tracks 0-15
    180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, 180586, // Tracks 16-31
    163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, 163599, // Tracks 32-47
    146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, 146788, // Tracks 48-63
    130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, 130505, // Tracks 64-79
};