// Header file for RMT functions
#include "types.h"

// This function constructs a lookup table of RMTDataItem arrays for each possible byte value (0-255)
void initBitPatterns();

// This function initializes the RMT peripheral using direct register access to get it ready for transmitting floppy data
void initRMT();

// The following functions are all inline to maximize speed
// And since they're inline, we have to both declare AND define them here in the header file

// Starts an RMT transmission by setting the RMT_TX_START_CH0 bit
inline __attribute__((__always_inline__)) void startRMT() {
    // Don't forget to set the CONF_UPDATE bit first
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 24);
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 0);
}

// Stops an RMT transmission by setting the RMT_TX_STOP_CH0 bit
inline __attribute__((__always_inline__)) void stopRMT() {
    // Set the CONF_UPDATE bit first
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 24);
    REG_SET_BIT(RMT_CH0CONF0_REG, 1 << 7);
}

// Returns the status of the RMT_CH0_TX_THR_EVENT_INT_RAW bit
inline __attribute__((__always_inline__)) bool rmtNeedsData() {
    return (REG_READ(RMT_INT_RAW_REG) & (1 << 8)) != 0;
}

// Clears the RMT_CH0_TX_THR_EVENT_INT_RAW bit
inline __attribute__((__always_inline__)) void clearRMTInt() {
    REG_SET_BIT(RMT_INT_CLR_REG, 1 << 8);
}

// Gives the RMT control over the RDA pin
inline __attribute__((__always_inline__)) void RMTControl() {
    // Route RMT channel 0 to GPIO4 (RDA pin)
    REG_WRITE(GPIO_FUNC4_OUT_SEL_CFG_REG, ((REG_READ(GPIO_FUNC4_OUT_SEL_CFG_REG) & 0xFFFFFE00) | 81));
    // Print out GPIO_FUNC4_OUT_SEL_CFG_REG in binary to verify our settings
    Serial.printf("GPIO_FUNC4_OUT_SEL_CFG_REG: %x\n", REG_READ(GPIO_FUNC4_OUT_SEL_CFG_REG));
}

// Gives GPIO control over the RDA pin
inline __attribute__((__always_inline__)) void GPIOControl() {
    // Route GPIO4 to normal GPIO function
    REG_WRITE(GPIO_FUNC4_OUT_SEL_CFG_REG, ((REG_READ(GPIO_FUNC4_OUT_SEL_CFG_REG) & 0xFFFFFE00) | 256));
}

// This function converts a GcrSector into its corresponding RMTSector
__attribute__((optimize("Ofast"))) void convertGCRToRMT(GcrSector* gcr, RMTSector* rmt);