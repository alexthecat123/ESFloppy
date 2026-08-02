#include <Arduino.h>
#include <esp_cpu.h>
#include "GCRLib.h"
#include "GPIO.h"
#include "SDTask.h"
#include "types.h"

// This file contains the routines for reading and writing flux data to and from the Lisa's floppy drive interface

#define SEEK_LEADIN_SECTORS 5 // How many sectors before sector 0 to reset the sector counter to when seeking to a new track

static char debugString[MAX_DEBUG_STRING_LENGTH]; // A string buffer for sending debug messages to the SD card task for printing over serial

// This struct is used to store a sector that we've received from the Lisa, but that we can't write to the track buffer yet because it's in use by the SD task
struct SectorStashEntry {
    WriteState state; // The state of the sector (whether it's a header or data)
    uint32_t gpio; // The state of the GPIO pins when we got the sector; used to determine the side number from HDS in certain cases
    uint8_t data[704]; // The actual sector data itself
};

// This function gets called whenever the Lisa tries to read data from the floppy drive
// It checks if it's time to send out a new flux transition, and if so, it bit-bangs it out on the RDA pin
// For the sake of speed, we don't return from here until the Lisa stops reading from the drive
// We need speed here, so make sure to stick it in IRAM and optimize it as much as possible
__attribute__((optimize("Ofast"))) IRAM_ATTR void transmitTrack(GcrSector trackBufferGCR[2][22], volatile SdTaskInterface *sdTaskInterface, TrackParams* trackParams, DiskImageMetadata* metadata) {
    // Each bit time is 2us long; for a 1 bit, we send a falling edge at the start, followed by a rising edge 1us later
    // For a 0 bit, we just keep it high the whole time

    // We'll use this variable to keep track of how long it's been since we sent the last bit
    static uint32_t prevBitTime = 0;

    static uint32_t currentSector = 0; // The sector that we're currently sending data for
    static uint32_t inSectorIndex = 0; // How many bits of that sector we've sent so far

    // The number of sectors per track for both sides of the current track based on the drive type
    static uint32_t sectorsPerTrack[2] = {0, 0};

    // Grab the side number from the low HDS bit; we'll need it for a lot of stuff here in a moment
    uint32_t gpioIn = REG_READ(GPIO_IN_REG); // Read the GPIO input register
    uint32_t side = (gpioIn & 1 << HDS) ? 1 : 0;

    // On a real drive, the disk doesn't stop turning just because we exited transmitTrack
    // So if the Lisa exits transmitTrack to check status and reenters, we need to catch up to the current time to simulate the spinning disk
    // You'd think that we'd be able to just pick up where we left off, which we can most of the time, but there's a problem with that
    // The Lisa periodically polls /DRVIN once per sector, and tries to do it in phase with the sector sync field
    // This way, the only data missed is some of the sync data, and we can recover with the remaining sync data when we return
    // But sometimes the /DRVIN check gets a little out of phase and happens in the middle of data instead of sync
    // Now we have a problem because the Lisa can't recover from that since there's no sync
    // And retrying won't help because the /DRVIN will still be out of phase and hit the data again
    // So the simple fix is to simulate the disk spinning while we're not in transmitTrack so that the phase constantly drifts instead of getting stuck out of sync
    // This will lead to occasional errors, but they should be recoverable in one retry and won't be frequent
    uint32_t currentTime = esp_cpu_get_cycle_count(); // To do this, start by getting the current time
    uint32_t timeAway = currentTime - prevBitTime; // And then get the number of CPU cycles since the last bit time (the time that we were away)
    if (timeAway > 480 && trackParams->trackChanged == false) {
        // If we were away for more than 480 cycles (one full bit time), then we have catch-up work to do
        // There's a good chance that RDA will be low coming into here if we've been away for a while
        // So go ahead and set it high to start with so that we're all ready to send out a falling edge for a 1 bit if we need to
        // Otherwise, if the first bit we send is a 1 bit, it'll be missed because we can't send a falling edge if RDA is already low
        writeRDA(true);
        inSectorIndex += timeAway / 480; // Increment the in-sector index by the number of full bit times that have passed
        // But if this pushes it past the end of the sector, then we have even more work to do
        if (inSectorIndex >= BITS_PER_SECTOR) {
            // Increment the current sector by the number of full sectors that have passed, wrapping around to sector 0 if necessary
            currentSector = (currentSector + (inSectorIndex / BITS_PER_SECTOR)) % sectorsPerTrack[side];
            inSectorIndex %= BITS_PER_SECTOR; // And then set the in-sector index to the appropriate spot within the new sector
        }
        prevBitTime = currentTime; // And finally, update the previous time to now
    } else if (trackParams->trackChanged == true) {
        // If the track has changed, then we have a couple things to do
        // First update sectorsPerTrack to the correct value for the new track
        // On a Sony, this is easy
        if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
            // Both sides of a Sony track have the same number of sectors, so just use side 0's value for both sides
            sectorsPerTrack[0] = sectorsPerTrackSony[trackParams->currentTrack];
            sectorsPerTrack[1] = sectorsPerTrack[0];
        } else {
            // But on a Twiggy, we have to do it for both sides because they each have a different number of sectors
            sectorsPerTrack[0] = sectorsPerTrackTwiggy[45 - trackParams->currentTrack]; // Side 0's (upper/rear head) track is (45 - carriage position)
            sectorsPerTrack[1] = sectorsPerTrackTwiggy[trackParams->currentTrack]; // And side 1's (lower/front head) is the carriage position
        }
        // Now we need to reset the in-sector index and current sector to 0 and not do a catch-up
        // This isn't necessary, but it speeds things up since the FDC normally starts reading new tracks from sector 0, so might as well start there
        // Except we don't actually want to immediately go to sector 0 because the controller won't start reading immediately
        // It'll need a few ms to get ready, so if we start with sector 0 immediately, then it'll miss it and have to wait a whole revolution
        // That's a huge waste of time, so instead start SEEK_LEADIN_SECTORS before sector 0 to give it time to get ready
        // And remember that logical sectors differ from physical sectors, so we need to use our interleave LUT to convert between them
        // Use getInterleaveTable to find out what interleave table to use for our disk metadata
        InterleaveTable interleaveTable = getInterleaveTable(metadata, 0, true);
        uint32_t sector0Slot = 0;
        // And now scan through the interleave table to find out which slot sector 0 is in
        for (uint32_t i = 0; i < sectorsPerTrack[side]; i++) {
            if (interleaveTable[sectorsPerTrack[side]][i] == 0) {
                sector0Slot = i;
                break;
            }
        }
        // Once we know where it is, we can set the current sector to SEEK_LEADIN_SECTORS before it, wrapping around if necessary
        currentSector = (sector0Slot + sectorsPerTrack[side] - SEEK_LEADIN_SECTORS) % sectorsPerTrack[side];
        inSectorIndex = 0; // And reset the in-sector index to 0 as well
        prevBitTime = currentTime; // Set the previous time to now
        trackParams->trackChanged = false; // Finally, mark that we've handled the track change
    }

    while (1) {
        // Before we do anything, we need to check to see if the Lisa is still listening to us to begin with
        // "Listening to us" looks different on a Sony versus on a Twiggy
        // For the sake of speed, do a raw REG_READ here instead of using any helper functions
        gpioIn = REG_READ(GPIO_IN_REG); // Read the GPIO input register
        if (metadata->driveType == DriveTwiggy) {
            // On a Twiggy, the Lisa is always listening to us on RDA as long as the drive is enabled and the motor is on
            if (!(!(gpioIn & (1 << DR1)) && trackParams->motorOn)) {
                // If either of those conditions are not met, then the Lisa is no longer listening to us, so we need to exit
                return;
            }
        } else {
            // On a Sony, the Lisa is listening if it's accessing read registers 8 or 9
            // We don't care about the low side select bit; we just need to make sure the high 3 bits {PH2, PH1, PH0} are 100
            // And we also need to be sure that LSTRB (PH3) isn't high; if it is, then we might miss a write to regs 0 or 1 which look like regs 8 and 9 in write mode
            if (!(gpioIn & 1 << PH2 && !(gpioIn & 1 << PH1) && !(gpioIn & 1 << PH0)) || (gpioIn & 1 << PH3)) {
                // If not, then return
                return;
            }
        }
        // We also need to check to see if WRQ went low, meaning that the Lisa is trying to write to the drive
        if (!(gpioIn & 1 << WRQ)) {
            // If so, then get out of here and let the write handler (receiveSector) take over
            return;
        }

        // Otherwise, grab the side number from the low HDS bit and continue
        side = (gpioIn & 1 << HDS) ? 1 : 0;

        // When we arrive here, we'll be on the first half of a bit, so go ahead and prep that bit to be sent out on RDA
        // We need to extract the bit from trackBufferGCR[side][currentSector]
        // But there's a catch: trackBufferGCR may not be ready yet if the SD task on the other core is still running
        // So we need to wait until it's finished before we read the track buffer and just send out the FF sync pattern until then
        // The Lisa won't care; it'll just think that the disk hasn't spun around to the next sector yet and will patiently wait for us
        static const uint8_t syncPattern[5] = {0xFF, 0x3F, 0xCF, 0xF3, 0xFC};
        uint8_t currentByte;
        if (sdTaskInterface->finished && !trackParams->pendingDispatch) {
            // If the SD task is finished AND we're not waiting for a pending dispatch (meaning that trackBufferGCR is up to date), then we can sedn out the next bit from the track buffer
            // Make sure we don't go out of bounds if the side changed in the middle of this track
            // If it did and this is a Twiggy, there's a chance that the current sector is now out of bounds for the new sidee
            // In which case we just reset to slot 0 in the GCR buffer again
            uint32_t slot = (currentSector < sectorsPerTrack[side]) ? currentSector : 0;
            currentByte = ((uint8_t*)&trackBufferGCR[side][slot])[inSectorIndex >> 3]; // Get the byte that contains the bit we want
        } else {
            // If the SD task is not finished, then we need to send out the sync pattern
            currentByte = syncPattern[(inSectorIndex >> 3) % 5];
        }
        bool bit = (currentByte >> (7 - (inSectorIndex & 0x07))) & 0x01; // Extract the bit we want from the byte

        // We truly want to send the bit out the MOMENT that the bit time is up
        // So let's even precompute the register that we need to write to in order to send the bit out on RDA
        // This way, we don't have to call writeRDA() and have it waste time calculating this after the bit time is up
        uint32_t writeReg = bit ? GPIO_OUT_W1TC_REG : GPIO_OUT_W1TS_REG;

        // Now that the bit is all ready to go, we need to wait until it's time to send it out
        prevBitTime += 240;
        while ((int32_t)(esp_cpu_get_cycle_count() - prevBitTime) < 0); // Get the number of CPU cycles between now and the last bit time; esp_cpu_get_cycle_count() is faster than ESP.getCycleCount()
        // Once that while loop finishes (240 cycles at 240MHz is 1us), it's time to send out the first half of our bit
        // Since we already precomputed everything, writing it out is as easy as:
        REG_WRITE(writeReg, 1 << RDA);
        
        // Now we need to wait for the second half of the bit time, which is another 1us
        prevBitTime += 240;
        while ((int32_t)(esp_cpu_get_cycle_count() - prevBitTime) < 0);
        // Time to send the second half of the bit; if it was a 1, we need to send a rising edge; if it was a 0, we just keep it high
        // We don't need to retrieve the bit again and check its value because regardless of whether it was a 0 or a 1, we need to set it high!
        writeRDA(true); // Nice and easy!
        
        // We now need to increment to the next bit in the sector
        inSectorIndex++;
        if (inSectorIndex >= BITS_PER_SECTOR) {
            // If we're about to go past the end of the sector, we need to move to the next one
            currentSector++;
            if (currentSector >= sectorsPerTrack[side]) {
                currentSector = 0; // Or wrap back to sector 0 if we're at the end of the track
            }
            inSectorIndex = 0; // Don't forget to reset the in-sector index since we're starting a new sector
        }
        // If we're emulating a Twiggy, then return back to the main loop after every bit so that we can do some housekeeping stuff
        if (metadata->driveType == DriveTwiggy) {
            return;
        }
    }
}

// This helper function gets called by receiveSector to interpret the raw data coming from the Lisa, extract prologues, and stick everything into the data buffer
// It returns true once the full header or sector data has been received, and false if we need to keep receiving more data
// It also modifies the writeState variable to indicate to the caller whether we're still in the prologue, reading the header, or reading the data
__attribute__((optimize("Ofast"))) IRAM_ATTR bool processRawWriteData(uint32_t rawInputData, uint8_t* dataBuffer, WriteState& writeState, bool newReception) {
    static uint32_t bitCounter = 0; // This keeps track of how many bits we've received so far for the current byte
    static uint32_t byteCounter = 0; // And how many bytes we've received so far in total

    if (newReception) {
        // If this is the start of a new reception, then we need to reset the bit and byte counters back to 0
        // We have to do this because they're static and would persist otherwise
        // And we can't rely on just resetting them in the "return true" case below because there's a chance that we only get a partial/corrupt sector and never get there
        bitCounter = 0;
        byteCounter = 0;
    }

    if (writeState == PROLOGUE) {
        // If we're in the prologue state, then we need to check if the data matches the header or data prologue
        if ((rawInputData & 0xFFFFFF) == DATA_PROLOGUE) {
            writeState = DATA; // Update the writeState accordingly
        } else if ((rawInputData & 0xFFFFFF) == HEADER_PROLOGUE) {
            writeState = HEADER;
        }
    } else if (writeState == DATA || writeState == HEADER) {
        // If we're in the data or header state, then we need to check two things
        // First, we need to check if we've received enough data to form a full byte to copy into the data buffer
        // And second, we need to check if we've received enough data to have the full header or sector data, and exit if so
        bitCounter++; // Increment the bit counter since we just received a bit
        // Start with the first check; if we've received 8 bits, then it's time to copy a byte into the buffer
        if (bitCounter == 8) {
            dataBuffer[byteCounter] = rawInputData & 0xFF; // Copy the LSB of rawInputData into the data buffer
            bitCounter = 0; // Reset the bit counter back to the start of the next byte
            byteCounter++; // And the byte counter since we just received a full byte
            // Now check to see if we've received enough bytes to have the full header or sector data
            if ((writeState == DATA && byteCounter >= 704) || (writeState == HEADER && byteCounter >= 5)) {
                // If so, then we can return true to indicate that we're done receiving data
                return true;
            }
        }
    }
    return false; // Return false if we haven't filled up the header/data buffer yet
}

// This function gets called whenever the Lisa pulls WRQ low and starts writing data to the disk
// Note that it's called receiveSector instead of receiveTrack because the Lisa only writes one sector at a time
// This makes our life a LOT easier
__attribute__((optimize("Ofast"))) IRAM_ATTR void receiveSector(GcrSector trackBufferGCR[2][22], volatile SdTaskInterface *sdTaskInterface, TrackParams* trackParams, DiskImageMetadata* metadata) {
    // The write process, despite sounding scary at first, is actually not too bad and looks like this:
        // 1. The Lisa lowers WRQ and starts sending valid data on WRD; each edge represents a 1, and the absence of an edge for 2us represents a 0
        // 2. The start of the data is the classic FF FF 3F CF F3 FC FF FF self-sync pattern, which we can safely ignore; we'll be fine even if we don't start sampling until midway through it
        // 3. After self-sync, the Lisa sends the D5 AA AD data prologue, which is our queue to start sampling
        // 4. Then we get sector_again, which we can use to determine which sector we're writing to
        // 5. Now we get our 699 bytes of GCR-encoded data, the 4-byte checksum, and the DE AA FF data epilogue, which gets cut off midway through the FF and we can ignore
        // 6. That can all just go straight into a buffer, get copied into a DecodedSector struct, and then get written to the image file
        // Notice that the header wasn't touched here at all; standard write ops completely ignore it
        // Format ops, on the other hand, overwrite BOTH the header and the data
        // A format works like this:
        // 1. The Lisa lowers WRQ and starts WRD data
        // 2. We get the self-sync pattern
        // 3. The Lisa sends the D5 AA 96 header prologue, which is our queue to start sampling the header
        // 4. We read in the header and its checksum, followed by the DE AA header epilogue, which we can ignore
        // 5. We can ignore most of the header, just paying attention to the sector number, side, and format to figure out where it needs to go and update our disk image
        // 6. WRQ goes high for a bit
        // 7. WRQ goes low again, and we get a standard data write as described above to clear out the data portion of the sector
        // So we can really just handle the header and data portions of things separately, and don't care that the data is tied to a format at all

    // Anyway, time to get started on all of that

    // This holds what state of the write op we're in; we start in PROLOGUE since the first thing we do is hunt for the prologue
    // We never modify this directly; we pass it to processRawWriteData, which updates it as needed
    WriteState writeState = PROLOGUE;

    // Get the current and previous times in CPU cycles so that we can measure the time between edges on WRD
    uint32_t currTime = 0;
    uint32_t prevTime = esp_cpu_get_cycle_count();

    // We also need a firstTime variable to know when we see the first edge on WRD so that we can start measuring time between edges
    bool firstTime = true;
    
    // This will hold the current state of the GPIO input register so that we can check the WRD and WRQ lines
    uint32_t gpioData = 0;

    bool prevWRD = REG_READ(GPIO_IN_REG) & (1 << WRD); // This will hold the previous state of WRD so that we can detect edges
    bool currWRD = prevWRD; // And the current state

    // Get the number of sectors per track for both sides of the current track based on the drive type
    uint32_t sectorsPerTrack[2] = {0, 0};
    if (metadata->driveType == Drive400 || metadata->driveType == Drive800) {
        // Both sides of a Sony track have the same number of sectors, so just use side 0's value for both sides
        sectorsPerTrack[0] = sectorsPerTrackSony[trackParams->currentTrack];
        sectorsPerTrack[1] = sectorsPerTrack[0];
    } else {
        // But on a Twiggy, we have to do it for both sides because they each have a different number of sectors
        sectorsPerTrack[0] = sectorsPerTrackTwiggy[45 - trackParams->currentTrack]; // Side 0's (upper/rear head) track is (45 - carriage position)
        sectorsPerTrack[1] = sectorsPerTrackTwiggy[trackParams->currentTrack]; // And side 1's (lower/front head) is the carriage position
    }

    // This buffer will hold the raw WRD data that we're currently receiving from the Lisa, and we'll use it like a shift register
    // It's 4 bytes long so that we can hold a full D5 AA AD or D5 AA 96 prologue that we use to detect the start of data or header
    uint32_t rawInputData = 0;

    // This is the big buffer that we copy the data into once we've seen the prologue and know that we're in a data or header write
    // It's 704 bytes long so that it can hold sector_again, the 699 bytes of data, and the 4-byte checksum
    uint8_t dataBuffer[704];
    
    // Here's another data buffer that we can stick the write data into if we've accumulated a full header/sector, but the SD card task hasn't finished loading the track yet
    // We can stick up to 16 sectors in here until the SD card task is finished, and then copy them into the track buffer when it's ready
    static SectorStashEntry sectorStash[16];

    // The stash needs to be a FIFO to make sure that we write the sectors in the order that they were received to make sure that lastWriteWasHeader works
    // So we need static head and tail indices for it
    static uint32_t stashHead = 0;
    static uint32_t stashTail = 0;

    // This flag is set if the last thing we received (on the last call to receiveSector) was a header; it's used to determine where to get the side number and interleave from
    static bool lastWriteWasHeader = false;
    static uint32_t prevSideNum = 0; // This is the side number from the last header
    static uint32_t prevFormat = 0; // And the format byte

    static uint32_t lostSectorCount = 0; // The number of sectors that we've lost because the stash overflowed

    gpioData = REG_READ(GPIO_IN_REG); // Read the GPIO input register to get the current state of WRD and WRQ

    // Skip all of the read logic here if WRQ is high, meaning that the Lisa isn't writing to us anymore
    // If we're here and WRQ is high, then it means that we need to write out the stash, so no need to try and read in any sectors
    if (!(gpioData & (1 << WRQ))) {
        // First, just put RDA in a defined 0 state so that we're not sending garbage during a write
        writeRDA(false);
        // Now we need to sync with the prologue and then do the actual data/header read
        while (1) {
            // As mentioned earlier, a 1 is represented by an edge on WRD, and a 0 is represented by the absence of an edge
            // The easiest way to detect edges is to start a timer when we see the first edge, and then constantly poll for edges on WRD
            // When we see the next edge, we check the time since the last edge and figure out what multiple of 2us it is (with some tolerance)
            // If it's just 2us, then we shift in a 1; if it's 4us, then we shift in a 01; if it's 6us, then we shift in a 001
            // The problem with this strategy is that 0 bits will only be committed when we see the next edge, so if the data ends with a 0, we won't see it
            // But this is fine because the end of the data is always the epilogue anyway, so we can just stop once we get through the data itself
            gpioData = REG_READ(GPIO_IN_REG); // Read the GPIO input register to get the current state of WRD and WRQ
            if ((gpioData & (1 << WRQ))) {
                // If WRQ goes high, then the Lisa has stopped writing and we can just return
                // This either means that the Lisa is being stupid and never sent all the data, or that we missed it
                return;
            }
            prevWRD = currWRD; // Update the previous WRD state to the current outdated one
            currWRD = gpioData & (1 << WRD); // And then update the current state of WRD to what it actually is
            if (currWRD == prevWRD) {
                continue; // If there was no change in WRD, then just continue to the next iteration of the loop and skip everything below here
            }
            // Otherwise, we saw an edge on WRD, so we need to check the time since the last edge to see what bit combination to shift in
            if (firstTime) {
                // If it's the first time through the loop, then we don't care about the time since the last edge
                // So just shift in a 1 bit and mark that we've seen the first edge
                rawInputData = (rawInputData << 1) | 1;
                prevTime = esp_cpu_get_cycle_count(); // Set prevTime to now so that we can measure the time to the next edge
                // Go process that data based on the current writeState to see if we've received the prologue or the full header/data yet
                // We pass true for the newReception parameter so that processRawWriteData knows to reset its bit and byte counters
                processRawWriteData(rawInputData, dataBuffer, writeState, true); 
                firstTime = false; // And mark that we've seen the first edge
            } else {
                // Otherwise, we need to check the time since the last edge
                currTime = esp_cpu_get_cycle_count(); // Get the current time in CPU cycles
                if (currTime - prevTime < BIT_TIME_1) {
                    // If the time since the last edge is less than our 2us with some tolerance, then shift in a 1 bit
                    rawInputData = (rawInputData << 1) | 1;
                    prevTime = currTime; // And update prevTime to now so that we can measure the time to the next edge
                    // Then go interpret the data based on the current writeState
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break; // If processRawWriteData returns true, then we've received the full header/data and can go deal with it
                    }          
                } else if (currTime - prevTime < BIT_TIME_01) {
                    // Same for 4us; shift in a 01
                    // We have to do this in 2 steps (shift in the 0, then the 1) so that we can call processRawWriteData after each bit
                    rawInputData = (rawInputData << 1);
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    rawInputData = (rawInputData << 1) | 1;
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    prevTime = currTime;
                } else if (currTime - prevTime < BIT_TIME_001) {
                    // And 6us; shift in a 001
                    // As before, we have to do this in steps; 3 steps this time
                    rawInputData = (rawInputData << 1);
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    rawInputData = (rawInputData << 1);
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    rawInputData = (rawInputData << 1) | 1;
                    if (processRawWriteData(rawInputData, dataBuffer, writeState, false)) {
                        break;
                    }
                    prevTime = currTime;
                } else {
                    // Something weird happened if we end up here; there should never be a gap of more than 6us between edges
                    // This points to data corruption, so there's no point in continuing to receive data
                    // Just return and let the Lisa retry the write if it feels like it
                    lastWriteWasHeader = false; // Make sure to reset this so that we don't try to use a header that's stale
                    interrupts();
                    Serial.println("MORE THAN 6US");
                    noInterrupts();
                    return;
                }
            }
        }
        // Last but not least, we need to write out the data that we've received
        // If the SD card task is still busy, then we need to stash the data away for later
        // That second condition of stashCount > 0 might seem a little weird because why would we ever want to stash data if the SD card isn't busy?
        // Well, it's because data must be written out in the order it's received in order for the lastWriteWasHeader logic to work
        // So if the stash already has stuff in it, then we need to keep filling it instead of bypassing it to maintain that order
        if (!sdTaskInterface->finished || trackParams->stashCount > 0) {
            if (trackParams->stashCount >= 16) {
                // If the stash is full, then we have a problem and need to return without doing anything else
                lostSectorCount++;
                snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "ERROR: Sector stash overflow! Lost %d sectors so far.\n", lostSectorCount);
                debugPrint(debugString, strlen(debugString));
                return;
            }
            else if (trackParams->stashCount > 0) {
                snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Stash count: %d\n", trackParams->stashCount);
                debugPrint(debugString, strlen(debugString));
            }
            // Go ahead and copy the data into the stash, remembering that it's a FIFO
            memcpy(sectorStash[stashHead].data, dataBuffer, 704);
            sectorStash[stashHead].state = writeState; // Don't forget to copy writeState and gpioData too
            sectorStash[stashHead].gpio = gpioData;
            trackParams->stashCount++; // Increment the stash count to mark that we've added an entry
            stashHead = (stashHead + 1) % 16; // Move the head to the next position
            return; // And then return so that we don't touch trackBufferGCR whatsoever; the loop can call us again once the task is done
        }
    } else {
        // A reminder: That entire while loop gets skipped if we already have a non-empty sectorStash to write out
        // We end up here in that case, where we need to double-check that it's safe to write to trackBufferGCR
        if (!sdTaskInterface->finished) {
            // If the task is still busy, then we just need to return again
            return;
        } else {
            // Otherwise, it's safe to proceed and write the stash to trackBufferGCR
            // But only if the stash is non-empty; double-check to be safe
            if (trackParams->stashCount == 0) {
                return;
            }
            // Copy one stash entry per call to avoid blocking for too long; the loop will call us again to write the next one
            // Once again, remember that the stash is a FIFO; we need to write out the oldest entry first
            memcpy(dataBuffer, sectorStash[stashTail].data, 704); // Copy the oldest stash entry into the main data buffer
            // And also restore writeState and gpioData from the stash entry so that we can write it to the proper GcrSector in trackBufferGCR
            writeState = sectorStash[stashTail].state;
            gpioData = sectorStash[stashTail].gpio;
            trackParams->stashCount--; // Decrement the stash count to mark that we've written one entry
            stashTail = (stashTail + 1) % 16; // And move the tail to the next position
        }
    }

    // Now that we have all of our header data or data data, we need write it to the proper GcrSector in trackBufferGCR
    // Note that we DO NOT write it to the disk image file yet because it would take too long
    // That's handled when the Lisa seeks to another track, turns off the motor, or ejects the disk
    if (writeState == DATA) {
        // If this is a data write, then we need to copy sector_again, the data, and the checksum
        // This is as easy as doing a memcpy into the proper GcrSector in trackBufferGCR, starting at the sector_again field
        uint32_t sectorNum = gcr_8to6[dataBuffer[0]]; // The first byte of the data buffer is the sector number (sector_again)
        // The side number is a little trickier; if the last write was a header, then we can read it from that header
        // And this is required during formats because HDS won't reflect the new side number during a format op
        // But if the last write was a data write, then we can just read it from HDS like normal
        uint32_t sideNum = (lastWriteWasHeader) ? (prevSideNum) : ((gpioData & (1 << HDS)) ? 1 : 0);
        // One more issue: we can't do a memcpy directly into trackBufferGCR[sideNum][sectorNum] because of the order in which trackbufferGCR stores sectors
        // It stores them in physical order, NOT logical order, so trackBufferGCR[sideNum][1] with 2:1 interleave on a 12-sector track would actually be sector 6
        // And we'd overwrite sector 6 with sector 1's data
        // So we actually need to run our sectorNum through the interleave table to figure out which physical slot to write it into
        uint32_t slot = 0xFFFFFFFF; // Initialize to an invalid value so that we can check later if we found a valid slot
        // Now determine which interleave table we'll need to use based on the format byte
        uint32_t currentFormat = 0;
        // If we're in the middle of a format, then use the interleave from the last header that was formatted
        if (lastWriteWasHeader) {
            currentFormat = prevFormat;
        // But if we're not in the middle of a format, then just use the interleave from literally any sector on the track since they're all the same 
        } else {
            currentFormat = gcr_8to6[trackBufferGCR[0][0].format];
        }
        // We don't have to do this manually anymore; now we have our fancy getInterleaveTable function that does it for us
        // False tells it to use the format from the currentFormat byte instead of the metadata
        InterleaveTable interleaveTable = getInterleaveTable(metadata, currentFormat, false);
        // Iterate through each sector in the current track
        for (uint32_t i = 0; i < sectorsPerTrack[sideNum]; i++) {
            if (interleaveTable[sectorsPerTrack[sideNum]][i] == sectorNum) {
                slot = i;
                break;
            }
        }
        // Now that we've used it, reset lastWriteWasHeader to false so that we don't accidentally use a stale header for the next data write if we end up aborting right now
        lastWriteWasHeader = false;
        if (slot == 0xFFFFFFFF) {
            // If we didn't find a valid slot, then something went wrong (perhaps a corrupted header), so just abort
            interrupts();
            Serial.println("NO VALID DATASLOT");
            noInterrupts();
            return;
        }
        // Now copy the 704 bytes of data into the proper GcrSector, using the slot we just found as the physical sector number
        memcpy(&trackBufferGCR[sideNum][slot].sector_again, &dataBuffer[0], 704);
        trackParams->dirty = true; // Don't forget to set dirty to true so that we know to write the track back to the disk image later
    } else if (writeState == HEADER) {
        // If this is a header write, then we need to copy the 4 bytes of header data and the 1-byte header checksum into the proper GcrSector
        // And we'll actually update the format byte and checksum in ALL of the sectors, not just the one we're writing to, to make sure it's always consistent across the whole track
        // Since the format byte controls interleave, we don't want a situation where part of a track uses one interleave and part uses another mid-format, so might as well just update them all
        // Before we do any of that though, make sure that all the bytes in the header are valid GCR bytes; if not, then abort
        uint8_t validGCR = 0;
        validGCR = gcr_8to6[dataBuffer[0]] | gcr_8to6[dataBuffer[1]] | gcr_8to6[dataBuffer[2]] | gcr_8to6[dataBuffer[3]] | gcr_8to6[dataBuffer[4]];
        if (validGCR & 0xC0) {
            // If any of the bytes have a 1 in the top 2 bits, then it's not valid GCR and we need to abort
            interrupts();
            Serial.println("BAD HEADER GCR");
            noInterrupts();
            return;
        }
        // Now make sure that the new track number in the header matches currentTrack
        // If it doesn't, then either the Lisa is going crazy or we missed a seek, but either way we need to abort
        // The track number encoding differs between Sony and Twiggy drives, so reconstruct it according to drive type
        uint32_t trackNum = metadata->driveType == DriveTwiggy ? (gcr_8to6[dataBuffer[0]]) : (gcr_8to6[dataBuffer[0]] | ((gcr_8to6[dataBuffer[2]] & 1) << 6));
        if (trackNum != trackParams->currentTrack) {
            interrupts();
            Serial.println("HEADER TRACK MISMATCH");
            noInterrupts();
            return;
        }
        uint32_t sectorNum = gcr_8to6[dataBuffer[1]]; // The second byte of the data buffer is the sector number
        // We can read the side number from hiTrackSide in the new header instead of HDS this time; just make sure to use the proper format for Sony vs Twiggy
        uint32_t sideNum = metadata->driveType == DriveTwiggy ? (gcr_8to6[dataBuffer[2]]) : ((gcr_8to6[dataBuffer[2]] & (1 << 5)) ? 1 : 0);
        prevSideNum = sideNum; // Store the side number for the next data write in this format op
        prevFormat = gcr_8to6[dataBuffer[3]]; // And store the format for the next data write in this format op
        // As we did for the data write, we need to map the logical sector number to a physical slot using the appropriate interleave table
        // The nice part here is that we ALWAYS have a valid format byte to use (stored in prevFormat right above), so no need to worry about where to get the format byte from
        // Aside from that, it's literally the exact same logic as for the data write, so nothing else to say here
        uint32_t slot = 0xFFFFFFFF;
        InterleaveTable interleaveTable = getInterleaveTable(metadata, prevFormat, false);
        for (uint32_t i = 0; i < sectorsPerTrack[sideNum]; i++) {
            if (interleaveTable[sectorsPerTrack[sideNum]][i] == sectorNum) {
                slot = i;
                break;
            }
        }
        if (slot == 0xFFFFFFFF) {
            // Abort on an invalid slot as before
            interrupts();
            Serial.println("NO VALID HEADERSLOT");
            noInterrupts();
            return;
        }
        // Now write our data to the sector headers using the physical slot number we just found
        for (uint32_t i = 0; i < sectorsPerTrack[sideNum]; i++) {
            trackBufferGCR[sideNum][i].format = dataBuffer[3]; // Update the format byte for all sectors on this side of the track
            // And then update the header checksums of all those sectors to reflect the new format byte
            trackBufferGCR[sideNum][i].headerChecksum = gcr_6to8[gcr_8to6[trackBufferGCR[sideNum][i].loTrack] ^ gcr_8to6[trackBufferGCR[sideNum][i].sector] ^ gcr_8to6[trackBufferGCR[sideNum][i].hiTrackSide] ^ gcr_8to6[trackBufferGCR[sideNum][i].format]];
        }
        // And then memcpy over the full sector header for the particular sector that we just wrote to
        memcpy(&trackBufferGCR[sideNum][slot].loTrack, &dataBuffer[0], 5);
        lastWriteWasHeader = true; // Mark that the last write was a header
        trackParams->dirty = true; // Once again, make sure to indicate that this track needs to be written back
    }
}
