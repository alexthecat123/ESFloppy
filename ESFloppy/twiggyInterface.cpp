#include <Arduino.h>
#include "fluxRW.h"
#include "GPIO.h"
#include "SDTask.h"
#include "types.h"

// The routines for emulating the Twiggy drive's register-level interface with the Lisa's floppy controller

#define MAX_MICROSTEP_COUNT 432 // The maximum number of microsteps that we can do before we hit the end of our stepper range and eject

static char debugString[MAX_DEBUG_STRING_LENGTH]; // A string buffer for sending debug messages to the SD card task for printing over serial

bool ejectPressed[2] = {false, false}; // A flag to indicate whether the eject button has been pressed on each drive

uint32_t motorStartTime[2] = {0, 0}; // The time that MT went high; used to determine when to actually turn the motor on for each drive

uint32_t gpioIn = 0; // The current state of GPIO_IN_REG
uint32_t gpioIn1 = 0; // And GPIO_IN1_REG

bool ph2CameIn[2] = {false, false}; // A flag for each drive to indicate whether PH2 has come in since the last time we checked; used for clearing the eject button latch

// Create a new trackParams struct to hold the current state of the track we're on and any pending operations on it for each drive
static TrackParams trackParams[2] = {{0, false, false, 45, 0, true}, 
                                    {1, false, false, 45, 0, true}};

static BufferStatus bufferStatus = {0, 1, false, 0}; // The current state of the track buffer and which drive owns it

// The current microstep that we're on for each drive; each track consists of 8 of these
// Start on lower (front) head track 45, the innermost track
// The formula for converting this to a track number (for the lower head) is: track = (microStepCount - 16) >> 3
int32_t microStepCount[2] = {376, 376};

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

uint32_t prevStepState[2] = {0, 0}; // The previous step state for each drive derived from the above LUT
bool needsResync[2] = {false, false}; // Flags for each drive to indicate whether the phase lines are out of sync with the motor position for each drive

uint32_t loopCounter = 0; // Increments on every twiggyLoop iteration; used for sequencing certain ops that don't need to happen every time

// The current and previous drive select values; 3 means both drives disabled
uint32_t driveSelect = 3;
uint32_t prevDriveSelect = 3;
uint32_t driveSelectDebounced = 3; // The debounced drive select value; used to avoid switching buffer ownership too quickly
uint32_t prevDriveSelectDebounce = 3; // The previous value of driveSelect; used in the debounce logic
uint32_t driveStartTime = 0; // The time that driveSelect changed; also for debouncing purposes

// Table to hold the four different possible SNS register values for both drives, indexed by [driveSelect][{PH0, HDS}]
bool snsTable[2][4] = {{false, false, false, false}, {false, false, false, false}};

// Current and previous states of the SNS pin
bool currentSNS = false;
bool prevSNS = true;

uint32_t startTime = 0;

// The main register-access loop for the Twiggy drive interface
__attribute__((optimize("Ofast"))) IRAM_ATTR void twiggyLoop(volatile SdTaskInterface* sdTaskInterface, GcrSector trackBufferGCR[2][22], DiskImageMetadata* metadata[2]) {
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
    // There's more weirdness here too: Given that one head is on one side of the disk and the other is on the other side, the lower head will be on track T while the upper will be on track 45-T
    // This means that the Twiggy's track number is more like a carriage position that we convert to a track for each of the heads
    // The sector encoding is identical to Sony, except that Twiggy requires a series of 10 0xA9 speed sync bytes at the start of sector 0 on each track
    // The controller uses these to adjust its speed during a format op
    // It also requires that each track be a very specific length in order for it to accept the timing of the speed sync bytes
    // If the track doesn't take pretty much exactly the right amount of time between consecutive speed sync bytes, then the controller won't format it
    // So we'll have to pad out tracks with sync bytes to make them the right length, with a different padding length for each track
    // And another annoyance: Twiggy also has a "track -1" that it uses as a timing track during formatting
    // At the start of a format op, it seeks to track -1 and writes 14 sectors, then 15, and then 21 sectors to the track, and then times the disk speed
    // So we have to implement that track (which isn't saved in the image) too

    // With that rather massive introduction out of the way, let's get to the actual implementation!
    // This whole thing goes in a big while(1) loop so that we don't have to waste time returning to the main loop and coming back
    while (1) {
        //startTime = esp_cpu_get_cycle_count();

        // First up, read in the states of all the I/O pins
        gpioIn = REG_READ(GPIO_IN_REG);

        // Set the ph2CameIn flag for the current drive if PH2 is high so that the eject button latch knows about it
        if (gpioIn & (1 << PH2)) {
            if (driveSelect == 0 || driveSelect == 2) {
                ph2CameIn[0] = true;
            }
            if (driveSelect == 1 || driveSelect == 2) {
                ph2CameIn[1] = true;
            }
        }

        // There's only three things that really need to happen every single loop itration
        // The first is to check the state of PH0 and HDS and output the appropriate register value on SNS
        // Only do this if a drive is selected of course
        if (driveSelect < 2) {
            // Look up the current state of SNS in the snsTable to determine what to output on the pin for the current drive
            currentSNS = snsTable[driveSelect][((gpioIn >> 11) & 2) | ((gpioIn >> 8) & 1)];
            if (currentSNS != prevSNS) {
                writeSNS(currentSNS); // If the SNS state has changed, then write it out to the pin
            }
            prevSNS = currentSNS; // And update the previous state to the current one for the next iteration

            // The second thing that needs to happen every loop iteration (also only if a drive is selected) is to check if the Lisa is trying to write to the disk
            // If WRQ is low (Lisa is trying to write) or we have stashed sectors to write out, then call receiveSector to handle it
            if (((gpioIn & (1 << WRQ)) == 0) || bufferStatus.stashCount > 0) {
                receiveSector(trackBufferGCR, sdTaskInterface, &trackParams[driveSelect], &bufferStatus, metadata[driveSelect]);
                gpioIn = REG_READ(GPIO_IN_REG); // Refresh gpioIn after returning from receiveSector
            }

            // And the third thing is to send out data over RDA if the Lisa is trying to read from the disk
            // Do this after writes since they're the most urgent
            // As long as the motor is on, then keep transmitting the current track to the Lisa
            if (trackParams[driveSelect].motorOn) {
                // Set the side in trackParams to the current state of HDS
                trackParams[driveSelect].side = (gpioIn & 1 << HDS) ? 1 : 0;
                transmitTrack(trackBufferGCR, sdTaskInterface, &trackParams[driveSelect], &bufferStatus, metadata[driveSelect]);
                gpioIn = REG_READ(GPIO_IN_REG); // Refresh gpioIn after returning from transmitTrack
            }
        } else {
            // If no drive is selected, then just set SNS low and RDA high
            writeRDA(true);
            writeSNS(false);
            prevSNS = false;
        }

        // Everything else is less time-sensitive, and can be done less frequently

        // The most time-sensitive thing is updating the state of driveSelect; do it every 4 loop iterations
        if ((loopCounter & 3) == 0) {
            // W can get away with only reading gpioIn1 here since the only thing it's used for is DR0
            // And the buttons too, but those are really not time-senitive AT ALL
            gpioIn1 = REG_READ(GPIO_IN1_REG); // Start by getting the state of GPIO_IN1_REG
            // Now generate a driveSelect value based on the states of the two DR pins to determine which drive is selected
            bool DE1 = (gpioIn & (1 << DR1)) != 0;
            bool DE0 = ((gpioIn1 & (1 << (DR0 - 32))) != 0);
            // These first two cases are obvious
            if (!DE1 && DE0) {
                driveSelect = 1;
            } else if (DE1 && !DE0) {
                driveSelect = 0;
            } else if (!DE1 && !DE0) {
                // But it's also possible on very rare occasion for both drives to be selected
                // Give that a driveSelect value of 2
                driveSelect = 2;
            } else {
                // And if neither drive is selected, then give it a value of 3
                driveSelect = 3;
            }

            // Now that we've computed driveSelect, we also need to compute a debounced version of it
            // We can't just use driveSelect directly for certain things (like dispatching SD card ops) because it can change rapidly
            // Like when the floppy controller is reading SNS for one drive and then the other, and we wouldn't want to invalidate the track buffer in this case
            // The debounced version will ensure that we only do something drastic like that if driveSelect has been stable for a while (1ms)
            if (driveSelect != prevDriveSelectDebounce) {
                // If driveSelect has changed, then start a timer
                driveStartTime = esp_cpu_get_cycle_count();
                prevDriveSelectDebounce = driveSelect; // And update prevDriveSelectDebounce to the new value
            } else if ((driveSelect < 2) && (esp_cpu_get_cycle_count() - driveStartTime > 240000)) {
                // If driveSelect has been stable for more than 1ms, and a drive is selected, then set driveSelectDebounced to the debounced value
                driveSelectDebounced = driveSelect;
            }

        }

        // Everything else is even less crucial; split the loop into a 63-iteration cycle and do things on certain iteration numbers
        // Just make sure that these iteration numbers aren't multiples of 8 so that we're not doing them at the same time as the driveSelect stuff
        switch (loopCounter & 63) {
            case 4: {
                // On iteration 4, handle the logic for dispatching any pending SD card ops from previous seeks, if any
                // Only do this if the SD task is finished and we're not already waiting for a pending dispatch
                // This check isn't really necessary, it just saves us a few cycles by not calling tryToStartSD when we know it won't do anything
                // A check that IS necessary though is to only do this if just a single drive is selected
                // Because if both or neither are selected, then we don't know which drive to dispatch for
                if (driveSelect < 2) {
                    if (trackParams[driveSelect].pendingDispatch && sdTaskInterface->finished) {
                        if(tryToStartSD(sdTaskInterface, &trackParams[driveSelect], &bufferStatus)) {
                            trackParams[driveSelect].pendingDispatch = false; // If it succeeded, then clear the pendingDispatch flag since we just dispatched it
                            // And print out the seek that we just did
                            //snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Drive %d Track %d\n", driveSelect, trackParams[driveSelect].currentTrack);
                            //debugPrint(debugString, strlen(debugString));
                        }
                    }
                }
                break;
            }
            case 12: {
                // On iteration 12, handle head stepping for the selected drive
                if (driveSelect < 2) {
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
                    // Only compute a delta if the step state was valid; otherwise ignore it
                    if (stepState != 0xFF) {
                        if (needsResync[driveSelect] || driveSelect != prevDriveSelect) {
                            // If we resynced the step state for this drive last time, or we just switched to it, then clear its resync flag
                            needsResync[driveSelect] = false;
                        } else {
                            // Otherwise, compute the delta between the current and previous step states
                            uint32_t stepDelta = (stepState - prevStepState[driveSelect] + 4) % 4;
                            // If the delta is 1, then we stepped forward one microstep, so increment the microstep count
                            if (stepDelta == 1) {
                                // Cap it at the maximum microstep count so we don't go too far off the disk though
                                if (microStepCount[driveSelect] < MAX_MICROSTEP_COUNT) {
                                    microStepCount[driveSelect]++;
                                }
                            } else if (stepDelta == 3) { // If the delta is 3, then we stepped backward one microstep, so decrement the microstep count
                                // It's okay for this to go negative; the drive will do this during the recal sequence
                                microStepCount[driveSelect]--;
                            }
                        }
                        prevStepState[driveSelect] = stepState;
                    } else if (driveSelect != prevDriveSelect) {
                        // If the step state is invalid, and we just switched to this drive, then we need to resync the state to the new drive
                        needsResync[driveSelect] = true;
                    }
                    // Otherwise, do absolutely nothing at all

                    // Now convert that microstep count into a track number
                    // The formula for this differs between the upper (rear) and lower (front) heads
                    // We use the lower head (side 1, HDS high) as our frame of reference, so its formula is: track = (microStepCount - 12) >> 3
                    // And as an aside, the upper head (side 0, HDS low) is: track = 45 - ((microStepCount - 12) >> 3)
                    // Notice that we subtract 12 from the microstep count
                    // This is because microstep 12 represents the middle of track 0, and we want it to be able to go to track -1 for the CAL sequence
                    int32_t rawTrack = (microStepCount[driveSelect] - 12) >> 3;
                    // currentTrack can be either -1 (the Twiggy timing track) or 0-45 (the normal tracks), so we need to clamp it to that range
                    if (rawTrack < -1) {
                        trackParams[driveSelect].currentTrack = -1;
                    } else if (rawTrack > 45) {
                        trackParams[driveSelect].currentTrack = 45;
                    } else {
                        trackParams[driveSelect].currentTrack = rawTrack;
                    }

                    // Also update the CAL register in the snsTable based on whether we're at track -1 (microstep 12 or less) or not
                    snsTable[0][2] = (microStepCount[0] <= 12);
                    snsTable[1][2] = (microStepCount[1] <= 12);

                    prevDriveSelect = driveSelect; // Last but not least, update prevDriveSelect to the current driveSelect
                }
                break;
            }
            case 20: {
                // On iteration 20, check if the track has changed and if we need to dispatch a read/write to the SD card task
                // Basically part 2 of iteration 16

                // Make sure to only do any of this if a single drive is selected and has been stable for at least 1ms
                // We don't want to flip back and forth reloading the buffer if the host is rapidly switching between drives to read SNS or something
                if (driveSelect < 2 && driveSelect == driveSelectDebounced) {
                    // Detect if we need to reload the buffer; this happens whenever we change drives or tracks
                    if ((bufferStatus.bufferOwnerDrive != driveSelect) || (bufferStatus.bufferOwnerTrack != trackParams[driveSelect].currentTrack)) {
                        // If it has, then mark that we've changed tracks
                        trackParams[driveSelect].trackChanged = true; // Tell the flux transmission code that the track has changed so it can reset to sector 0
                    }

                    // If the track has changed, but the step hasn't completed yet, then we need to try to start the SD card task with the appropriate read/write operation
                    // Note that we don't guard this with a check for the track being >= 0 because a -1 timing track read/write will get auto-rejected by the SD task
                    // We also want to do this if we just switched from one drive to another, because the new drive's track isn't loaded and the old drive's track need to be written
                    // Make sure that the stash is empty before we dispatch a write though; otherwise we'll overwrite it
                    if ((trackParams[driveSelect].trackChanged && ((bufferStatus.bufferOwnerDrive != driveSelect) || (bufferStatus.bufferOwnerTrack != trackParams[driveSelect].currentTrack)) && bufferStatus.stashCount == 0)) {
                        if (!trackParams[driveSelect].pendingDispatch) {
                            // If a dispatch to the SD task isn't already pending, then we can set up a pending one with the current seek
                            trackParams[driveSelect].pendingDispatch = true; // Mark that a dispatch is pending so we don't overwrite the pending command with a new one if the host seeks again
                        }
                        // Try to start the read and/or write for this seek on the SD card task if it isn't busy
                        if (tryToStartSD(sdTaskInterface, &trackParams[driveSelect], &bufferStatus)) {
                            trackParams[driveSelect].pendingDispatch = false; // Clear the pendingDispatch flag since we just dispatched it
                            // And print out the seek that we just did
                            //snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Drive %d Track %d\n", driveSelect, trackParams[driveSelect].currentTrack);
                            //debugPrint(debugString, strlen(debugString));
                        }
                    }
                }
                break;
            }
            case 28: {
                // On iteration 28, handle the spindle motor enable/disable for drive 0 (the upper drive)
                // This is done regardless of whether the drive is enabled or not; Twiggy doesn't care about DE for the motor
                // Check the state of the MT line to see if the motor should be on or off
                // We can't just say motorOn = MT because MT is also used as the shift clock and that would make it go on and off rapidly
                // So instead start a timer and only turn the motor on if MT has been high for at least 240000 cycles (1ms at 240MHz)
                // Same deal with turning it off; do it on a 240000-cycle delay
                bool currMT0 = (gpioIn & (1 << MT0));
                if (currMT0 && !trackParams[0].motorOn && motorStartTime[0] == 0) {
                    // If MT is high, the motor is off, and the timer hasn't started yet, then start the timer
                    motorStartTime[0] = esp_cpu_get_cycle_count();
                } else if (currMT0 && !trackParams[0].motorOn && (esp_cpu_get_cycle_count() - motorStartTime[0]) >= 240000) {
                    // Otherwise, if MT is high, the motor is off, and the timer has been running for at least 240000 cycles, then turn the motor on
                    motorStartTime[0] = 0;
                    trackParams[0].motorOn = true;
                    REG_WRITE(GPIO_OUT_W1TS_REG, 1 << LED); // Now that the motor is on, turn on the LED
                } else if (!currMT0 && trackParams[0].motorOn && motorStartTime[0] == 0) {
                    // If MT is low, the motor is on, and the timer hasn't started yet, then start the timer
                    motorStartTime[0] = esp_cpu_get_cycle_count();
                } else if (!currMT0 && trackParams[0].motorOn && (esp_cpu_get_cycle_count() - motorStartTime[0]) >= 240000) {
                    // If MT is low, the motor is on, and the timer has been running for at least 240000 cycles, then turn the motor off
                    motorStartTime[0] = 0;
                    trackParams[0].motorOn = false;
                    if (trackParams[1].motorOn == false) {
                        // Now that the upper drive's motor is off, turn off the LED
                        // But only if the lower drive's motor is also off
                        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << LED);
                    }
                }
                break;
            }
            case 36: {
                // On iteration 36, handle the spindle motor for the lower drive (drive 1)
                bool currMT1 = (gpioIn & (1 << MT1));
                if (currMT1 && !trackParams[1].motorOn && motorStartTime[1] == 0) {
                    // If MT is high, the motor is off, and the timer hasn't started yet, then start the timer
                    motorStartTime[1] = esp_cpu_get_cycle_count();
                } else if (currMT1 && !trackParams[1].motorOn && (esp_cpu_get_cycle_count() - motorStartTime[1]) >= 240000) {
                    // Otherwise, if MT is high, the motor is off, and the timer has been running for at least 240000 cycles, then turn the motor on
                    motorStartTime[1] = 0;
                    trackParams[1].motorOn = true;
                    REG_WRITE(GPIO_OUT_W1TS_REG, 1 << LED); // Now that the motor is on, turn on the LED
                } else if (!currMT1 && trackParams[1].motorOn && motorStartTime[1] == 0) {
                    // If MT is low, the motor is on, and the timer hasn't started yet, then start the timer
                    motorStartTime[1] = esp_cpu_get_cycle_count();
                } else if (!currMT1 && trackParams[1].motorOn && (esp_cpu_get_cycle_count() - motorStartTime[1]) >= 240000) {
                    // If MT is low, the motor is on, and the timer has been running for at least 240000 cycles, then turn the motor off
                    motorStartTime[1] = 0;
                    trackParams[1].motorOn = false;
                    if (trackParams[0].motorOn == false) {
                        // Now that the lower drive's motor is off, turn off the LED
                        // But only if the upper drive's motor is also off
                        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << LED);
                    }
                }
                break;
            }
            case 44: {
                // On iteration 44, handle the eject button latch for both drives

                // First handle latching button presses; this gets done regardless of whether the drives are enabled or not
                // If the user has pressed either of the eject buttons, then latch ejectPressed to true for the corresponding drive
                // The upper drive uses LEFT for eject and the lower drive uses RIGHT for eject
                // Note that the buttons are active-low
                if (!(gpioIn1 & (1 << (LEFT - 32)))) {
                    ejectPressed[0] = true;
                }
                if (!(gpioIn1 & (1 << (RIGHT - 32)))) {
                    ejectPressed[1] = true;
                }

                // That's the code for pressing the buttons to set the latch, now let's handle clearing the latch
                // Remember, this is done by the host asserting PH2, and a drive has to be enabled for its latch to clear
                // We can't just directly check PH2 here because its pulse width is so short that we'll miss it given how infrequently this is executed
                // So instead check the ph2CameIn flag that gets set in the code that's executed every iteration
                // Only clear the latch if the user isn't actively pressing eject on either drive
                if ((gpioIn1 & (1 << (LEFT - 32))) && ph2CameIn[0]) {
                    ejectPressed[0] = false;
                }
                if ((gpioIn1 & (1 << (RIGHT - 32))) && ph2CameIn[1]) {
                    ejectPressed[1] = false;
                }
                ph2CameIn[0] = false; // And clear the ph2CameIn flags so we don't keep clearing the latch(es) on every iteration
                ph2CameIn[1] = false;

                // Update the eject button register values in the snsTable based on the state of the latches for both drives
                snsTable[0][1] = ejectPressed[0];
                snsTable[1][1] = ejectPressed[1];

                break;
            }
            case 52: {
                // And finally, on iteration 52, we have the logic for actually ejecting a disk
                // If our microStepCount has hit its maximum (meaning the heads are well off the disk), then we need to eject it
                // This simulates the "move the heads past track 45 to eject" behavior
                if (driveSelect < 2) {
                    if (microStepCount[driveSelect] >= MAX_MICROSTEP_COUNT && metadata[driveSelect]->diskInserted) {
                        // Mark that the disk is no longer inserted in this drive
                        // We need to do this first because the Lisa is polling SNS for it, and we're about to block for a while
                        metadata[driveSelect]->diskInserted = false;
                        microStepCount[driveSelect] = 376; // Reset the microstep count to track 45 now that we're actually ejecting
                        if (bufferStatus.bufferDirty == true && bufferStatus.bufferOwnerDrive == trackParams[driveSelect].drive) {
                            // If the current track on one of the drives is dirty, then we need to write it out before we truly close the disk
                            // Wait until the SD card task is finished with its current command before we dispatch it again
                            // It's okay to block like this in the eject handler because we're ejecting the disk anyway
                            while (sdTaskInterface->finished == false);
                            sdTaskInterface->command = WRITE_READ_TRACK;
                            sdTaskInterface->writeTrack = bufferStatus.bufferOwnerTrack;
                            sdTaskInterface->readTrack = trackParams[driveSelect].currentTrack;
                            sdTaskInterface->writeDrive = bufferStatus.bufferOwnerDrive;
                            sdTaskInterface->readDrive = trackParams[driveSelect].drive;
                            sdTaskInterface->finished = false;
                            __sync_synchronize();
                            sdTaskInterface->start = true;
                        }
                        // Now that we've written out the current track if necessary, we can actually eject the disk
                        // So wait until the SD task is done with whatever it's doing
                        while (sdTaskInterface->finished == false);
                        if (bufferStatus.bufferDirty == true && bufferStatus.bufferOwnerDrive == trackParams[driveSelect].drive) {
                            bufferStatus.bufferDirty = false; // Now that we've written it out (if necessary), clear bufferDirty
                        }
                        sdTaskInterface->writeDrive = bufferStatus.bufferOwnerDrive; // writeDrive doesn't matter here
                        sdTaskInterface->readDrive = trackParams[driveSelect].drive; // Set readDrive to the drive we're ejecting
                        trackParams[driveSelect].pendingDispatch = false; // Clear the pendingDispatch flag since we're about to eject the disk
                        sdTaskInterface->command = CLOSE_IMAGE; // Now tell it to close the image
                        sdTaskInterface->finished = false;
                        __sync_synchronize();
                        sdTaskInterface->start = true; // And start the task
                        trackParams[driveSelect].motorOn = false; // Turn off the motor too if it happens to be on
                        snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "Disk Ejected!\n");
                        debugPrint(debugString, strlen(debugString));
                        // And finally, set bufferOwnerTrack to an invalid value to indicate that the buffer is no longer valid for this drive, if this drive is currently active
                        // This will prevent any of the logic that relies on it from running when the disk is no longer present
                        if (bufferStatus.bufferOwnerDrive == trackParams[driveSelect].drive) {
                            bufferStatus.bufferOwnerTrack = 100;
                        }
                    }

                    // This is also a good place to update the disk in place registers in the snsTable based on the latest metadata
                    snsTable[0][3] = metadata[0]->diskInserted;
                    snsTable[1][3] = metadata[1]->diskInserted;

                }
                break;
            }
        }

        loopCounter++; // On every loop iteration, increment the loop counter

        //snprintf(debugString, MAX_DEBUG_STRING_LENGTH, "%d %d\n", trackParams[driveSelect].motorOn, (esp_cpu_get_cycle_count() - startTime));
        //debugPrint(debugString, strlen(debugString));
    }
}