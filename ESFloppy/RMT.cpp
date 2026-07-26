#include <Arduino.h>
#include "types.h"
// This file contains everything related to controlling the RMT peripheral that we use for sending/receiving the floppy bitstream

// GCR is 1 bit every 2us so this is what we need to set the RMT for
// A 1 bit is represented by a falling edge on RDA; raise it 1us later (halfway thru the 2us bit time) to prep for the next bit
// A 0 bit is represented by no falling edge on RDA; just keep it high for the full 2us bit time
// Here are the RMTDataItems for a GCR 1 bit and a GCR 0 bit:
RMTDataItem gcrBitOne = { .duration0 = 40, .level0 = 0, .duration1 = 40, .level1 = 1 }; // 1us low, 1us high
RMTDataItem gcrBitZero = { .duration0 = 40, .level0 = 1, .duration1 = 40, .level1 = 1 }; // 2us high, no second part

// We have room for 192 RMT data items, so that's 192/2 = 96 bits we can send before we have to refill the RMT memory
// At 2us per bit, that gives us 192us of free time to refill the RMT memory before it runs out of data

// To save time in our GCR to RMT conversion, we'll make a lookup table of RMTDataItem arrays for each possible byte value (0-255)
// Each entry will be an array of 8 RMTDataItems representing the 8 bits of that byte in GCR format
// The math for this hurt my head, so I'll be honest: I had ChatGPT write this function for me
RMTDataItem bitPatterns[256][8];
void initBitPatterns() {
    for (uint32_t v = 0; v < 256; v++) {
        for (int bit = 7; bit >= 0; bit--) {
            bitPatterns[v][7 - bit] = (v & (1 << bit)) ? gcrBitOne : gcrBitZero;
        }
    }
}

// This function initializes the RMT peripheral using direct register access to get it ready for transmitting floppy data
void initRMT() {
    // First, we'll enable the RMT clock and take it out of reset
    // Which is done by setting bit 9 of the SYSTEM_PERIP_CLK_EN0_REG to turn on the clock
    REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, 1 << 9);
    // And then setting and clearing bit 9 of the SYSTEM_PERIP_RST_EN0_REG reset the RMT
    REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 9);
    REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 9);

    // Now we can set up the RMT registers themselves
    // We'll start with RMT_SYS_CONF_REG
    // Set RMT_CLK_EN to enable the clock to all the RMT registers
    REG_SET_BIT(RMT_SYS_CONF_REG, 1 << 31);
    // Set RMT_MEM_CLK_FORCE_ON to enable the clock to the RMT memory
    REG_SET_BIT(RMT_SYS_CONF_REG, 1 << 1);
    // Set RMT_APB_FIFO_MASK to enable writing directly to the RMT RAM without the FIFO
    // Just kidding, we're actually going to use the FIFO for simplicity
    //REG_SET_BIT(RMT_SYS_CONF_REG, 1 << 0);

    // Now onto RMT_CH0CONF0_REG
    // Whenever we're changing the RMT_CH0CONF0_REG or RMT_CH0_TX_LIM_REG, we have to set the RMT_CONF_UPDATE_CH0 bit first
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 24);
    // Enable the RMT_MEM_TX_WRAP_EN_CH0 bit to turn on wraparound in transmitter memory
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 4);
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 24);
    // Change RMT_MEM_SIZE_CH0 to 4 (bits [19:16]) to allocate 4 blocks (48 words each) to the transmitter
    REG_WRITE(RMT_CH0CONF0_REG, (REG_READ(RMT_CH0CONF0_REG) & 0xFFF0FFFF) | (4 << 16));
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 24);
    // And finally clear RMT_CARRIER_EN_CH0 to disable carrier modulation
    REG_CLR_BIT(RMT_CH0CONF0_REG, 1 << 21);

    // Now onto RMT_CH0_TX_LIM_REG
    // First set RMT_CONF_UPDATE_CH0 to allow updating this register
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 24);
    // Now we need to set RMT_TX_LIM_CH0 (bits [8:0]) to an appropriate threshold value that will set the flag to tell us to refill the transmit memory buffer
    // The RMT transmitter has a memory capacity of mem_size * 48 words; with mem_size = 4, that's 192 words
    // Let's say we want to refill when it's half empty, so set the threshold to 96
    // It's okay to wipe out the other bits since they all default to 0 anyway
    REG_WRITE(RMT_CH0_TX_LIM_REG, 96);

    // Just to be safe, let's clear any pending RMT_CH0_TX_THR_EVENT_INT interrupts in RMT_INT_CLR_REG
    REG_SET_BIT(RMT_INT_CLR_REG, 1 << 8);

    // And finally, initialize the lookup table of bit patterns for GCR to RMT conversion
    initBitPatterns();
}

// This function converts a GcrSector into its corresponding RMTSector
// We have to run this on-the-fly as the disk "rotates" between sectors since we don't have enough RAM to precompute all the RMT data for an entire track
// So we need this to be as fast as possible; this current version is about 67us, which is good enough
// First up, we have to optimize it aggressively with the -Ofast flag
__attribute__((optimize("Ofast"))) void convertGCRToRMT(GcrSector* gcr, RMTSector* rmt) {
    uint32_t startTime = micros();
    // Init a pointer to the GCR data for easy access; we don't care about the different data fields so just treat it as a flat array
    uint8_t* gcrPtr = (uint8_t*)gcr; 
    // Make a pointer to the RMT data in the RMTSector too
    // Note that we're making the pointer a uint32_t instead of an RMTDataItem to make the copying faster
    // Not sure why, but the compiler seems to optimize it better this way
    uint32_t* out = (uint32_t*)rmt->data;
    
    // Now iterate over all 733 GCR bytes in the sector
    for (int i = 0; i < 733; i++) {
        // And copy the 8 RMTDataItems corresponding to the current GCR byte from the bitPatterns lookup table to the output
        // Once again, we're treating the RMTDataItems as uint32_t values for faster copying
        uint32_t* src = (uint32_t*)bitPatterns[gcrPtr[i]];
        out[0] = src[0];
        out[1] = src[1];
        out[2] = src[2];
        out[3] = src[3];
        out[4] = src[4];
        out[5] = src[5];
        out[6] = src[6];
        out[7] = src[7];
        out += 8;
    }
    //Serial.printf("GCR to RMT conversion took %u microseconds\n", micros() - startTime);
}