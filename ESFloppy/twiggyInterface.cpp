#include <Arduino.h>
#include "fluxRW.h"
#include "GPIO.h"
#include "SDTask.h"
#include "types.h"

// The routines for emulating the Twiggy drive's register-level interface with the Lisa's floppy controller

#define MAX_MICROSTEP_COUNT 432 // The maximum number of microsteps that we can do before we hit the end of our stepper range and eject

static char debugString[MAX_DEBUG_STRING_LENGTH]; // A string buffer for sending debug messages to the SD card task for printing over serial

bool ejectPressed = false; // A flag to indicate whether the eject button has been pressed

bool trackLoaded = false; // A flag to indicate whether the current track has been loaded into the track buffer yet

uint32_t motorStartTime = 0; // The time that MT went high; used to determine when to actually turn the motor on

uint32_t gpioIn = 0; // The current state of GPIO_IN_REG
uint32_t gpioIn1 = 0; // And GPIO_IN1_REG

// Create a new trackParams struct to hold the current state of the track we're on and any pending operations on it
static TrackParams trackParams = {false, 0, READ_TRACK, false, 45, true, 0, false};

// The current microstep that we're on; each track consists of 8 of these
// Start on lower (front) head track 45, the innermost track
// The formula for converting this to a track number (for the lower head) is: track = (microStepCount - 16) >> 3
int32_t microStepCount = 376;

// A LUT to convert the current state of PH[0:3] into a step state (0-3) or an invalid state (0xFF)
// See the loop below for an explanation of how this works
uint32_t phaseLUT[16] = {
    0xFF, // PH[0:3] = 0000, invalid state
    0xFF, // PH[0:3] = 0001, invalid state
    0xFF, // PH[0:3] = 0010, invalid state
    2, // PH[0:3] = 0011, PH2+PH3
    0xFF, // PH[0:3] = 0100, invalid state
    0xFF, // PH[0:3] = 0101, invalid state
    1, // PH[0:3] = 0110, PH1+PH2
    0xFF, // PH[0:3] = 0111, invalid state
    0xFF, // PH[0:3] = 1000, invalid state
    3, // PH[0:3] = 1001, PH3+PH0
    0xFF, // PH[0:3] = 1010, invalid state
    0xFF, // PH[0:3] = 1011, invalid state
    0, // PH[0:3] = 1100, PH0+PH1
    0xFF, // PH[0:3] = 1101, invalid state
    0xFF, // PH[0:3] = 1110, invalid state
    0xFF  // PH[0:3] = 1111, invalid state
};

uint32_t prevStepState = 0; // The previous step state derived from the above LUT

// The previous raw track number, used to detect when the track has changed
int32_t prevRawTrack = 45;

uint32_t stepOverflowCount = 0; // How many tracks we've stepped past the end of the disk; used to determine when to eject it

uint32_t loopCounter = 0; // Increments on every twiggyLoop iteration; used for sequencing certain ops that don't need to happen every time

// Current and previous states of the SNS pin
bool currentSNS = false;
bool prevSNS = true;

uint32_t startTime = 0;

// The main register-access loop for the Twiggy drive interface
__attribute__((optimize("Ofast"))) IRAM_ATTR void twiggyLoop(volatile SdTaskInterface* sdTaskInterface, GcrSector trackBufferGCR[2][22], DiskImageMetadata* metadata) {
    // The Twiggy interface is quite a bit different from the Sony interface, and easier in most ways
    // The biggest difference is that RDA is no longer a multiplexed pin for reading registers and flux data
    // On the Twiggy, it's dedicated to disk data, along with WRD and WRQ just like on the Sony
    // This doesn't mean that Twiggies don't have registers though; they do, but far fewer and they're accessed on a different line
    // This other line is called SNS, and it can provide you with one of 3 different register values
    // The particular value that it outputs is determined by the state of HDS and PH0, as follows:
        // PH0 = 0, HDS = 0: Write protect (high if write-protected, low if not)
        // PH0 = 0, HDS = 1: Eject button pressed (high if pressed, low if not)
        // PH0 = 1, HDS = 0: CAL (the track 0 sensor), except it actually goes high when the heads are a little behind track 0, like track -1 or so
        // PH0 = 1, HDS = 1: Disk in place (high if in place, low if not)
    // One interesting thing to note: the eject button register actually latches the state of the button when the user presses it
    // The host can clear the latch by setting PH2 high
    // Another interesting thing to not here: There's no register to command ejecting or clamping a disk
    // This is because it's done by moving the heads around; if you step the heads past track 45, then the disk will get unclamped and ejected
    // And stepping them in the other direction will clamp the disk back in place
    // Motor speed control is also handled quite differently from the Sony
    // The Sony has no speed regulation at all and relies on the TACH feedback loop with the Lisa to adjust speed
    // But the Twiggy regulates speed internally; you shift it an 8-bit speed value on PH0 using MT as the shift clock, and it handles everything for you
    // MT also serves as the "motor on" signal; setting it high turns the motor on
    // So no need to use the LEDC to generate TACH pulses here!!!
    // We shouldn't even need to worry about reading in the shift pattern because we don't need it anyway; all we need to know is what track we're on
    // All of that is pretty darn easy to implement; the only real annoyance is the way that the Twiggy handles stepping the heads
    // Instead of a step register like the Sony, the Twiggy has four phase lines PH0-PH3 that you have to toggle in sequence to step the heads
    // To step them away from track 0, you assert them in the order 0-1-2-3, and to step toward track 0, you do 3-2-1-0
    // But it takes 8 steps to go from one track to another, so we have to count steps and make sure to read in the next track at the right time
    // To make matters worse, Twiggy always seeks 4 steps past the desired track and then steps back 4 steps to minimize overshoot
    // And there's also a recal sequence where it expects to start receiving garbage data around the point of that 4th step
    // So that's something to consider too...

    // With that rather massive introduction out of the way, let's get to the actual implementation!
    // This whole thing goes in a big while(1) loop so that we don't have to waste time returning to the main loop and coming back
    while (1) {
        //startTime = esp_cpu_get_cycle_count();

        // First up, read in the states of all the I/O pins
        gpioIn = REG_READ(GPIO_IN_REG);
        if (loopCounter % 256 == 0) {
            // Only read in gpioIn1 every 256 iterations to save time; it's just for the SEL button which can be checked infrequently
            gpioIn1 = REG_READ(GPIO_IN1_REG);
        }

        // Every loop iteration, try to dispatch any pending SD card ops from previous seeks, if there are any
        // Only do this if the SD task is finished and we're not already waiting for a pending dispatch
        // This check isn't really necessary, it just saves us a few cycles by not calling tryToStartSD when we know it won't do anything
        if (trackParams.pendingDispatch && sdTaskInterface->finished) {
            if(tryToStartSD(sdTaskInterface, &trackParams)) {
                trackParams.pendingDispatch = false; // If it succeeded, then clear the pendingDispatch flag since we just dispatched it
            }
        }

        // Next let's handle the eject button latch; this gets done regardless of whether the drive is enabled or not
        // If the user has pressed the eject (SEL) button, then latch ejectPressed to true
        // Note that SEL is active-low
        if (!(gpioIn1 & (1 << (SEL - 32)))) {
            ejectPressed = true;
        }

        // On even iterations, check the state of MT to see if we need to turn the motor on or off
        // This is done regardless of whether the drives are enabled or not; Twiggy doesn't care about DE for the motor
        if (loopCounter % 2 == 0) {
            // Now let's check the state of MT to see if the motor should be on or off
            // We can't just say motorOn = MT because MT is also used as the shift clock and that would make it go on and off rapidly
            // So instead start a timer and only turn the motor on if MT has been high for at least 240000 cycles (1ms at 240MHz)
            // Same deal with turning it off; do it on a 240000-cycle delay
            bool currMT = (gpioIn & (1 << MT1));
            if (currMT && !trackParams.motorOn && motorStartTime == 0) {
                // If MT is high, the motor is off, and the timer hasn't started yet, then start the timer
                motorStartTime = esp_cpu_get_cycle_count();
            } else if (currMT && !trackParams.motorOn && (esp_cpu_get_cycle_count() - motorStartTime) >= 240000) {
                // Otherwise, if MT is high, the motor is off, and the timer has been running for at least 240000 cycles, then turn the motor on
                motorStartTime = 0;
                trackParams.motorOn = true;
                REG_WRITE(GPIO_OUT_W1TS_REG, 1 << LED); // Now that the motor is on, turn on the LED
            } else if (!currMT && trackParams.motorOn && motorStartTime == 0) {
                // If MT is low, the motor is on, and the timer hasn't started yet, then start the timer
                motorStartTime = esp_cpu_get_cycle_count();
            } else if (!currMT && trackParams.motorOn && (esp_cpu_get_cycle_count() - motorStartTime) >= 240000) {
                // If MT is low, the motor is on, and the timer has been running for at least 240000 cycles, then turn the motor off
                motorStartTime = 0;
                trackParams.motorOn = false;
                REG_WRITE(GPIO_OUT_W1TC_REG, 1 << LED); // Now that the motor is off, turn off the LED
            }
        }

        // Now check to see if the drive is enabled; if not, then we have nothing else to do
        if ((gpioIn & (1 << DR1)) == 0) {
            // If it is enabled, then start by checking for write operations
            // If WRQ is low (Lisa is trying to write) or we have stashed sectors to write out, then call receiveSector to handle it
            if (((gpioIn & (1 << WRQ)) == 0) || trackParams.stashCount > 0) {
                receiveSector(trackBufferGCR, sdTaskInterface, &trackParams, metadata);
                gpioIn = REG_READ(GPIO_IN_REG); // Refresh gpioIn after returning from receiveSector
            }
            
            // Once we've handled any writes (which are the most urgent), we need to handle reads
            // If the motor is on, then keep transmitting the current track to the Lisa
            // Wait, is this a problem? I think transmitTrack blocks right now, which was fine on Sony, but won't work very well here...
            if (trackParams.motorOn) {
                transmitTrack(trackBufferGCR, sdTaskInterface, &trackParams, metadata);
                gpioIn = REG_READ(GPIO_IN_REG); // Refresh gpioIn after returning from transmitTrack
            }

            // Now let's check the state of PH0 and HDS to output the appropriate register value on SNS
            // I'm doing some of the math ahead of time to save a few cycles; that's why you don't see the PH0 or HDS defines here
            uint32_t regAddress = ((gpioIn >> 11) & 2) | ((gpioIn >> 8) & 1);
            switch (regAddress) {
                case 0:
                    // Register 0 is the write-protect register; we don't have write-protection so always set SNS low here
                    currentSNS = false;
                    break;
                case 1:
                    // Register 1 is the eject button register; set it to our latched ejectPressed value
                    currentSNS = ejectPressed;
                    break;
                case 2:
                    // Register 2 is the CAL track 0 sensor; set it high whenever microStepCount is <= 12 and low otherwise
                    currentSNS = (microStepCount <= 12);
                    break;
                case 3:
                    // Register 3 is the disk-in-place register; use the metadata to determine its value
                    currentSNS = metadata->diskInserted;
                    break;
            }
            if (currentSNS != prevSNS) {
                writeSNS(currentSNS); // If the SNS state has changed, then write it out to the pin
            }
            prevSNS = currentSNS; // And update the previous state to the current one for the next iteration

            // All of the stuff above here is time-sensitive stuff that needs to be done on every loop iteration
            // But everything below here is less sensitive and can be done less frequently
            // We need to stagger execution so that only some of it happens on each iteration; otherwise transmitTrack won't keep up
            // So let's split it so that motor control and eject stuff happens on even iterations
            // And seeking/stepping stuff happens on odd iterations
            // Motor control happens regardless of whether the drive is enabled or not so that's handled up above
            if (loopCounter % 2 == 0) {
                // Here's the even iteration stuff; start with the eject latch

                // Unlike setting the eject latch, which happens even when the drive is deselected, resetting it requires the drive to be enabled
                // Only reset the latch if the host has set PH2 high AND the user isn't actively pressing eject
                if (gpioIn & (1 << PH2) && (gpioIn1 & (1 << (SEL - 32)))) {
                    ejectPressed = false; // So be a nice obedient Twiggy and comply
                }

                // Finally, if our microStepCount has hit its maximum (meaning the heads are well off the disk), then we need to eject it
                // This simulates the "move the heads past track 45 to eject" behavior
                if (microStepCount >= MAX_MICROSTEP_COUNT && metadata->diskInserted) {
                    // Mark that the disk is no longer inserted
                    // We need to do this first because the Lisa is polling SNS for it, and we're about to block for a while
                    metadata->diskInserted = false;
                    microStepCount = 376; // Reset the microstep count to track 45 now that we're actually ejecting to prep for the next disk
                    if (trackParams.dirty == true) {
                        // If the current track is dirty, then we need to write it out before we eject the disk
                        // Wait until the SD card task is finished with its current command before we dispatch it again
                        // It's okay to block like this in the eject handler because we're ejecting the disk anyway
                        while (sdTaskInterface->finished == false);
                        sdTaskInterface->writeTrack = trackParams.pendingTrackToWrite;
                        sdTaskInterface->command = WRITE_READ_TRACK;
                        sdTaskInterface->readTrack = trackParams.currentTrack;
                        trackParams.dirty = false;
                        sdTaskInterface->finished = false;
                        __sync_synchronize();
                        sdTaskInterface->start = true;
                    }
                    // Now that we've written out the current track if necessary, we can actually eject the disk
                    // So wait until the SD task is done with whatever it's doing
                    while (sdTaskInterface->finished == false);
                    trackParams.pendingDispatch = false; // Clear the pendingDispatch flag since we're about to eject the disk
                    sdTaskInterface->command = CLOSE_IMAGE; // Now tell it to close the image
                    sdTaskInterface->finished = false;
                    __sync_synchronize();
                    sdTaskInterface->start = true; // And start the task
                    trackParams.motorOn = false; // Turn off the motor too if it happens to be on
                    snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Disk Ejected!\n");
                    debugPrint(debugString, strlen(debugString));
                }
            } else {
                // And now it's time for the odd iteration stuff
                // Here, we need to handle head stepping; this is the annoying part
                // Make a variable to hold the state of PH[3:0]
                // But it's actually in the order PH[0:3] in gpioIn
                // For the sake of speed, we'll just leave it like that instead of trying to reorder it
                uint32_t currPH = (gpioIn >> PH3) & 0x0F; // Get the current state of PH[0:3] from gpioIn
                // Now we need to figure out the current position of the motor
                // Because of how stepper motors work, the phase can be in one of 4 valid states that advance the motor
                // And any other state won't advance the motor at all, and states have to be sequential; you can't skip one
                // The states (which PH signals are asserted simultaneously) are:
                    // PH0+PH1, PH1+PH2, PH2+PH3, PH3+PH0
                // We could use if statements for all this, but a LUT is faster, so we'll do that instead
                uint32_t stepState = phaseLUT[currPH]; // Look up the current step state from the phase LUT

                // Now we need to figure out how the step state has changed since the last time we checked
                // Only compute a delta if the step state was valid; otherwise leave it at 0
                uint32_t stepDelta = 0;
                if (stepState != 0xFF) {
                    // The +4 and %4 is necessary to prevent negative values from messing things up
                    stepDelta = (stepState - prevStepState + 4) % 4;
                }
                // If the delta is 1, then we stepped forward one microstep, so increment the microstep count
                if (stepDelta == 1) {
                    // Cap it at the maximum microstep count so we don't go too far off the disk though
                    if (microStepCount < MAX_MICROSTEP_COUNT) {
                        microStepCount++;
                    }
                } else if (stepDelta == 3) { // If the delta is 3, then we stepped backward one microstep, so decrement the microstep count
                    // It's okay for this to go negative; the drive will do this during the recal sequence
                    microStepCount--;
                }
                // Otherwise, the delta is either 0 (no change) or 2 (we skipped a step), so just hold position
                // Now that we're done, update prevStepState to the current step state, but only if the current step state was valid
                if (stepState != 0xFF) {
                    prevStepState = stepState;
                }

                // Now convert that microstep count into a track number
                // The formula for this differs between the upper (rear) and lower (front) heads
                // We use the lower head (side 1, HDS high) as our frame of reference, so its formula is: track = (microStepCount - 12) >> 3
                // And as an aside, the upper head (side 0, HDS low) is: track = 45 - ((microStepCount - 12) >> 3)
                // Notice that we subtract 12 from the microstep count
                // This is because microstep 12 represents the middle of track 0, and we want it to be able to go to track -1 for the CAL sequence
                int32_t rawTrack = (microStepCount - 12) >> 3;
                // currentTrack can be either -1 (the Twiggy timing track) or 0-45 (the normal tracks), so we need to clamp it to that range
                if (rawTrack < -1) {
                    trackParams.currentTrack = -1;
                } else if (rawTrack > 45) {
                    trackParams.currentTrack = 45;
                } else {
                    trackParams.currentTrack = rawTrack;
                }

                // Now detect if the track has changed since the last time we checked
                if (rawTrack != prevRawTrack) {
                    // If it has, then mark that we've changed tracks
                        snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Track %d\n", rawTrack);
                        debugPrint(debugString, strlen(debugString));
                        trackParams.trackChanged = true; // Tell the flux transmission code that the track has changed so it can reset to sector 0
                        trackLoaded = false; // And also mark that the track is no longer loaded since we just changed tracks
                }

                // Update prevRawTrack to the current raw track number now
                prevRawTrack = rawTrack;

                // If the track has changed, but the step hasn't completed yet, then we need to try to start the SD card task with the appropriate read/write operation
                // Note that we don't guard this with a check for the track being >= 0 because a -1 timing track read/write will get auto-rejected by the SD task
                if (trackParams.trackChanged && !trackLoaded) {
                    if (!trackParams.pendingDispatch) {
                        // If a dispatch to the SD task isn't already pending, then we can set up a pending one with the current seek
                        trackParams.pendingCommand = trackParams.dirty ? WRITE_READ_TRACK : READ_TRACK; // If the track is dirty, then we need to write it out first, otherwise we can just read in the new track
                        // Keep in mind that pendingTrackToWrite itself is set in tryToStartSD, so that's why we don't set it here
                        trackParams.dirty = false; // Clear the dirty bit since the task already knows about it now
                        trackParams.pendingDispatch = true; // Mark that a dispatch is pending so we don't overwrite the pending command with a new one if the host seeks again
                    }
                    // Try to start the read and/or write for this seek on the SD card task if it isn't busy
                    if (tryToStartSD(sdTaskInterface, &trackParams)) {
                        trackParams.pendingDispatch = false; // Clear the pendingDispatch flag since we just dispatched it
                        trackLoaded = true; // Mark that the track is now loaded so we don't try to load it again until the next seek
                    }
                }
            }
        } else {
            // If the drive is disabled, set RDA high and SNS low
            writeRDA(true);
            writeSNS(false);
            prevSNS = false;
        }

        loopCounter++; // Last but not least, increment the loop counter
        //snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "%d\n", esp_cpu_get_cycle_count() - startTime);
        //debugPrint(debugString, strlen(debugString));
    }
}