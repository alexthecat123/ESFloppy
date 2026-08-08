#include <Arduino.h>
#include "fluxRW.h"
#include "GPIO.h"
#include "LEDC.h"
#include "SDTask.h"
#include "types.h"

// The routines for emulating the Sony drive's register-level interface with the Lisa's floppy controller

static char debugString[MAX_DEBUG_STRING_LENGTH]; // A string buffer for sending debug messages to the SD card task for printing over serial

bool prevLSTRB = 0;
bool currLSTRB = 0;

bool ledcAttached = false;
StepDirection stepDirection = OUT;
bool stepComplete = true;

bool ejectPending = false;
uint32_t ejectStartTime;
uint32_t tachFreq = 0;

// Create a new trackParams struct to hold the current state of the track we're on and any pending operations on it
static TrackParams trackParams = {1, false, false, 0, 0, true};

static BufferStatus bufferStatus = {0, 1, false, 0}; // The current state of the track buffer and which drive owns it (although there's only one drive here)

// The main register-access loop for the Sony drive interface
__attribute__((optimize("Ofast"))) IRAM_ATTR void sonyLoop(volatile SdTaskInterface* sdTaskInterface, GcrSector trackBufferGCR[2][22], DiskImageMetadata* metadata[2]) {
    // Don't do anything unless the drive is enabled (DR1 low)
        // When it's enabled, we check for commands to the drive
        // The read/write register comes in on CA2-CA0 (PH2-PH0) and the low bit (SEL) is on HDS
        // As long as the drive is enabled and LSTRB (PH3) is low, we put the contents of that register on the bus
        // But when LSTRB goes high, we write whatever's on the bus into a register
        // Now the data to write is on CA2 (PH2) and the register is determined by CA1-CA0 (PH1-PH0) and HDS
        // Let's edge-trigger on LSTRB going high instead of level-trigger
        // Reading EJECT always returns 0, to eject a disk you have to write a 1 to EJECT but LSTRB has to be high for at least 500ms
        // MOTORON shouldn't do anything unless a disk is inserted
        // The host sets STEP to 0 to step the heads, but the drive must set it back to 1 again within 12ms. I'm guessing the host polls for this, and since we're planning on writing to the SD card during steps, we need to make sure to only set it back to 1 after the SD card write is finished
    
    // Run everything in here in a while(1) so that we don't waste time returning to the main loop after each iteration
    while (1) {
        // First up, read in the states of all the I/O pins that we care about
        uint32_t gpioIn = REG_READ(GPIO_IN_REG); // Read the GPIO input register

        // Set the side in trackParams to the current state of HDS
        trackParams.side = (gpioIn & 1 << HDS) ? 1 : 0;

        // Every loop iteration, try to dispatch any pending SD card ops from previous seeks, if there are any
        if (tryToStartSD(sdTaskInterface, &trackParams, &bufferStatus)) {
            stepComplete = true; // If we successfully started a new operation, set stepComplete to true if we were in the middle of a step
            trackParams.pendingDispatch = false; // Clear the pendingDispatch flag since we just dispatched it
        }

        if ((gpioIn & (1 << DR1)) == 0) { // If the drive is enabled, then we need to check for commands

            // If WRQ is low (Lisa is trying to write) or we have stashed sectors to write out, then call receiveSector to handle it
            if (((gpioIn & (1 << WRQ)) == 0) || bufferStatus.stashCount > 0) {
                receiveSector(trackBufferGCR, sdTaskInterface, &trackParams, &bufferStatus, metadata[1]);
                return; // This makes sure that we refresh gpioIn with an updated read after we get back from receiveSector
            }
                
            currLSTRB = (gpioIn & (1 << PH3)) ? 1 : 0; // Read the current state of LSTRB (PH3)
            if (currLSTRB == 0) { // If LSTRB is low, then we need to put the selected register on the bus
                // Figure out which register the host wants to read from
                uint8_t regNum = ((gpioIn & (1 << PH2)) ? 8 : 0) | ((gpioIn & (1 << PH1)) ? 4 : 0) | ((gpioIn & (1 << PH0)) ? 2 : 0) | ((gpioIn & (1 << HDS)) ? 1 : 0);
                // Now we need to figure out what data to send for that register
                if (regNum != 7 && ledcAttached) { // If we're not reading the TACH register, make sure to stop any ongoing tach pulse generation
                    // Only do this if it's not already detached since repeatedly detaching it wastes tons of time
                    ledcAttached = false;
                    GPIOControl(RDA); // Give the GPIO registers control of the RDA pin back so we can bit-bang data to it
                }
                switch (regNum) {
                    case 0: // /DIRTN register (head step direction)
                        writeRDA(stepDirection);
                        break;
                    case 1: // /CSTIN register (false if disk inserted, else true)
                        writeRDA(metadata[1]->diskInserted ? 0 : 1);
                        break;
                    case 2: // /STEP register (host sets low to step heads, drive sets high when step is complete)
                        writeRDA(stepComplete);
                        break;
                    case 3: // /WRPROT register (low if disk is write-protected, else high)
                        writeRDA(1); // We never have write protection enabled, so always return high
                        break;
                    case 4: // /MOTORON register (low if motor is on, else high)
                        writeRDA(trackParams.motorOn ? 0 : 1);
                        break;
                    case 5: // /TK0 register (low if heads are on track 0, else high)
                        writeRDA((trackParams.currentTrack == 0) ? 0 : 1);
                        break;
                    case 6: // /EJECT register (write-only, always returns 0)
                        writeRDA(0);
                        break;
                    case 7: // /TACH register (produces 60 pulses per revolution when motor is on)
                        // Use our LUT to figure out what the LEDC divider value should be for the current track's TACH frequency
                        tachFreq = tachDividerPerTrackLisa[trackParams.currentTrack];
                        // Only output TACH pulses if the motor is on (DUH), and only start the LEDC if it's not already running
                        if (trackParams.motorOn && !ledcAttached) {
                            setFreqRaw(tachFreq, 8); // Set the LEDC divider value to our TACH frequency with 8-bit duty resolution
                            LEDCControl(RDA); // And give the LEDC control of the RDA pin so it can output the TACH pulses
                            ledcAttached = true; // Mark that the LEDC is attached so we don't attach/detach it unnecessarily
                        }
                        // If the motor is off, just output a constant low
                        else if (!trackParams.motorOn) {
                            writeRDA(0);
                        }
                        break;
                    case 8: // RDDATA register for head 0
                        // If the drive's motor is running, then transmit data for side 0; otherwise, do nothing
                        if (trackParams.motorOn) {
                            transmitTrack(trackBufferGCR, sdTaskInterface, &trackParams, &bufferStatus, metadata[1]);
                        }
                        break;
                    case 9: // RDDATA register for head 1; only valid for 800k drives
                        // If the drive's motor is running, then transmit data for side 1; otherwise, do nothing
                        if (trackParams.motorOn) {
                            transmitTrack(trackBufferGCR, sdTaskInterface, &trackParams, &bufferStatus, metadata[1]);
                        }
                        break;
                    case 12:
                    case 13: // SIDES register (duplicated on both addresses 12 and 13); returns 0 for 400K drives, 1 for 800K drives
                        writeRDA(metadata[1]->driveType == Drive800 ? 1 : 0);
                        break;
                    case 14:
                    case 15: // /DRVIN register (duplicated on both addresses 14 and 15); hard-coded to 0 as a way for the host to detect a drive connected
                        writeRDA(0);
                        break;
                    default: // If an invalid register is selected, just return 0
                        writeRDA(0);
                        break;
                }
            }
            // Otherwise, if LSTRB (PH3) is on a rising edge, or LSTRB is just high period while an eject is pending, then we need to write to a register
            else if ((prevLSTRB == 0 && currLSTRB == 1) || (currLSTRB == 1 && ejectPending == true)) {
                // So figure out which register the host wants to write to
                uint8_t regNum = ((gpioIn & (1 << PH1)) ? 4 : 0) | ((gpioIn & (1 << PH0)) ? 2 : 0) | ((gpioIn & (1 << HDS)) ? 1 : 0);
                bool regData = (gpioIn & (1 << PH2)) ? 1 : 0; // The data to write is on CA2 (PH2)
                // Now write the data to the selected register
                switch (regNum) {
                    case 0: // /DIRTN register (head step direction, low for IN, high for OUT)
                        stepDirection = regData ? OUT : IN;
                        break;
                    case 2: // /STEP register (host sets low to step heads, drive sets high when step is complete)
                        if (regData == 0) { // If the host is trying to step the heads
                            // We don't necessraily always want to step the heads here; if the SD card task is still busy, then we can't step until it's done
                            if (!trackParams.pendingDispatch) {
                                // If a dispatch to the SD task isn't already pending, then we can set up a pending one with the current seek
                                trackParams.pendingDispatch = true; // Mark that a dispatch is pending so we don't overwrite the pending command with a new one if the host seeks again
                            }

                            // Now we can actually perform the step; this way, currentTrack keeps up with the actual track but we don't disturb the SD task until it's done
                            if (stepDirection == IN) { // If stepping IN, increment the track
                                if (trackParams.currentTrack < 79) {
                                    trackParams.currentTrack++;
                                }
                            }
                            else { // If stepping OUT, decrement the track
                                if (trackParams.currentTrack > 0) {
                                    trackParams.currentTrack--;
                                }
                            }

                            trackParams.trackChanged = true; // Mark that the track has changed so we can reset the sector back to 0 in transmitTrack
                            stepComplete = false; // Mark that a step is in progress; it won't be done until the SD card task finishes THIS step
                            // Try to start the read and/or write for this seek on the SD card task if it isn't busy
                            if (tryToStartSD(sdTaskInterface, &trackParams, &bufferStatus)) {
                                stepComplete = true; // And mark that we're done with the current step if we were able to start the operation
                                trackParams.pendingDispatch = false; // Clear the pendingDispatch flag since we just dispatched it
                            }
                        }
                        break;
                    case 4: // /MOTORON register (low to turn motor on, high to turn motor off)
                        trackParams.motorOn = (regData == 0);
                        // Turn on the activity LED whenever the motor is on too
                        if (trackParams.motorOn) {
                            REG_WRITE(GPIO_OUT_W1TS_REG, 1 << LED);
                        }
                        else {
                            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << LED);
                            // If the motor just turned off, this is a prime opportunity to write the current track to the image if necessary
                            // This works basically the same as in the step case above, except that we don't need to change the track number since we're not stepping
                            if (bufferStatus.bufferDirty == true && !trackParams.pendingDispatch) {
                                trackParams.pendingDispatch = true;
                                if (tryToStartSD(sdTaskInterface, &trackParams, &bufferStatus)) {
                                    stepComplete = true;
                                    trackParams.pendingDispatch = false;
                                }
                            }
                        }
                        break;
                    case 6: // EJECT register (write-only, write a 1 to eject the disk)
                        // The catch here is that the 1 must be held for at least 500ms to actually eject the disk
                        // So we need to start a timer when we see a 1 written, and if it stays 1 for 500ms, we eject the disk
                        // Start by checking if the written data is a 1 and if we're not already pending an eject
                        if (regData == 1 && ejectPending == false) {
                            ejectPending = true; // If so, mark that an eject is pending
                            ejectStartTime = millis(); // And record the start time
                        }
                        else if (regData == 0) { // If the host writes a 0, we need to cancel any pending eject
                            ejectPending = false;
                        }
                        else if (ejectPending == true) { // Otherwise, if an eject is pending, check if 500ms has passed
                            if (millis() - ejectStartTime >= 500) {
                                // If so, eject the disk
                                // First make sure that any pending writes are flushed to the image; dispatch the SD task the same way as usual
                                if (bufferStatus.bufferDirty == true) {
                                    // Wait until the SD card task is finished with its current command before we dispatch it again
                                    // It's okay to block like this in the eject handler because we're ejecting the disk anyway
                                    while (sdTaskInterface->finished == false);
                                    sdTaskInterface->writeTrack = bufferStatus.bufferOwnerTrack;
                                    sdTaskInterface->writeDrive = bufferStatus.bufferOwnerDrive;
                                    sdTaskInterface->readDrive = trackParams.drive;
                                    sdTaskInterface->command = WRITE_READ_TRACK;
                                    sdTaskInterface->readTrack = trackParams.currentTrack;
                                    bufferStatus.bufferDirty = false;
                                    sdTaskInterface->finished = false;
                                    __sync_synchronize();
                                    sdTaskInterface->start = true;
                                }
                                // Wait until the SD card task is done with its current command
                                while (sdTaskInterface->finished == false);
                                trackParams.pendingDispatch = false; // Clear the pendingDispatch flag since we're about to eject the disk
                                sdTaskInterface->command = CLOSE_IMAGE; // Now tell it to close the image
                                sdTaskInterface->finished = false;
                                sdTaskInterface->writeDrive = bufferStatus.bufferOwnerDrive;
                                sdTaskInterface->readDrive = trackParams.drive;
                                __sync_synchronize();
                                sdTaskInterface->start = true; // And start the task
                                trackParams.motorOn = false; // Turn off the motor too
                                ejectPending = false;
                                snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Disk Ejected!\n");
                                debugPrint(debugString, strlen(debugString));
                            }
                        }
                        break;
                    default:
                        // Do nothing for writes to other (invalid) registers
                        break;
                }
            }
            prevLSTRB = currLSTRB; // Update prevLSTRB for the next iteration
        } else {
            // If the drive is disabled, make sure the step direction is OUT (this is the idle state)
            stepDirection = OUT;
            // And sit here until it's enabled again
        }
    }
}