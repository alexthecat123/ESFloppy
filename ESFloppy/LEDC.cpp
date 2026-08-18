// Main source file for LEDC functions; we have to handle the LEDC manually to achieve the speed that we need
#include <Arduino.h>
#include "LEDC.h"

// Initializes the LEDC peripheral and routes it to the specified GPIO pin
void initLEDC(uint32_t pin) {
    // First, we need to turn the LEDC on
    REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, 1 << 11); // Enable the clock to the LEDC
    REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 11); // And bring it in and then out of reset
    REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, 1 << 11);

    // Now we can configure the LEDC itself, starting with some global config settings
    REG_SET_BIT(LEDC_CONF_REG, 1 << 31); // Turn on the clock to the LEDC so that we can write to its registers
    // And then select the 80MHz APB_CLK as our clock source for the LEDC timers
    REG_SET_BIT(LEDC_CONF_REG, 1 << 0);

    // We'll wait and configure the timer later in the setFreq() function

    // Now we can configure LEDC channel 0 that we'll use to output our PWM signal
    // It's hooked to timer 0 by default, so no need to change that
    // And it's already set to idle low when the output is disabled, so no need to change that either
    // And we don't care about overflow counting, so there's actually nothing to do in LEDC_CH0_CONF0_REG for now

    // Same goes for LEDC_CH0_CONF1_REG; it controls duty cycle fading which we don't need and it's off by default

    // We always want the output to go high at the start of each PWM cycle, so set the high point to 0
    REG_WRITE(LEDC_CH0_HPOINT_REG, 0);
    // And set LEDC_PARA_UP_CH0 in LEDC_CH0_CONF0_REG to commit that change
    REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 4);

    // And that's all the configuration we need to do
    // To use the LEDC, just set the duty cycle in LEDC_CH0_DUTY_REG and enable the output in LEDC_CH0_CONF0_REG
}

// Sets the frequency (and resolution, they kind of go together) of the LEDC's timer and PWM pulse
void setFreq(uint32_t freq, uint32_t resolution) {
    // Convert the the frequency (and resolution) to a clock divider value using the formula in the ESP32 technical reference
    double divider = 80000000/(pow(2, resolution)*freq);
    int whole = (int)divider; // This is a floating-point number, but we need it in fixed-point, so take out the whole part
    double fractional = divider - int(divider); // And isolate the fractional part too
    // Convert the fractional part to a numerator over 256 for the (18, 8) fixed-point representation that we need
    int numerator = (int)(fractional * 256); 
    // And put the whole and numerator parts together to get our fixed-point number that we can put into the divider register
    uint32_t finalDivider = (whole << 8) | numerator;
    //Serial.printf("Freq: %d, Divider: %d\n", freq, finalDivider);
    // First, set the frequency to that final computed value value
    REG_WRITE(LEDC_TIMER0_CONF_REG, ((REG_READ(LEDC_TIMER0_CONF_REG) & 0xFFC0000F) | (finalDivider << 4)));
    // And then set the timer resolution to whatever's specified in our resolution variable
    REG_WRITE(LEDC_TIMER0_CONF_REG, ((REG_READ(LEDC_TIMER0_CONF_REG) & 0xFFFFFFF0) | resolution));
    // Then start the timer by pulling it out of reset
    REG_CLR_BIT(LEDC_TIMER0_CONF_REG, 1 << 23);
    // And finally, set the LEDC_TIMER0_PARA_UP bit to commit these timer configuration changes
    REG_SET_BIT(LEDC_TIMER0_CONF_REG, 1 << 25);
}

// Sets the frequency and resolution, but takes raw divider values instead of computing them; use this when you need max speed
void setFreqRaw(uint32_t divider, uint32_t resolution) {
    // First, set the frequency to that final computed value value
    REG_WRITE(LEDC_TIMER0_CONF_REG, ((REG_READ(LEDC_TIMER0_CONF_REG) & 0xFFC0000F) | (divider << 4)));
    // And then set the timer resolution to whatever's specified in our resolution variable
    REG_WRITE(LEDC_TIMER0_CONF_REG, ((REG_READ(LEDC_TIMER0_CONF_REG) & 0xFFFFFFF0) | resolution));
    // Then start the timer by pulling it out of reset
    REG_CLR_BIT(LEDC_TIMER0_CONF_REG, 1 << 23);
    // And finally, set the LEDC_TIMER0_PARA_UP bit to commit these timer configuration changes
    REG_SET_BIT(LEDC_TIMER0_CONF_REG, 1 << 25);
}

// Sets the duty cycle of the LEDC's PWM pulse
void setDuty(uint32_t duty) {
        // Set LEDC channel 0's duty cycle to duty, making sure to shift left by 4 to account for the unused fixed-point bits
        REG_WRITE(LEDC_CH0_DUTY_REG, duty << 4);
        // And set LEDC_PARA_UP_CH0 in LEDC_CH0_CONF0_REG to commit that change
        REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 4);
        // Also set LEDC_DUTY_START to commit the changes
        REG_SET_BIT(LEDC_CH0_CONF1_REG, 1 << 31);
}

// Enables or disables the output of the LEDC
void enableLEDCOutput(bool enabled) {
    if (enabled == true) {
        // If we want to enable the output, then go ahead and do it
        REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 2);
        // And set LEDC_PARA_UP_CH0 in LEDC_CH0_CONF0_REG to commit that change
        REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 4);
    } else {
        // Same goes for disabling it
        REG_CLR_BIT(LEDC_CH0_CONF0_REG, 1 << 2);
        // And don't forget to commit that change too
        REG_SET_BIT(LEDC_CH0_CONF0_REG, 1 << 4);
    }
}