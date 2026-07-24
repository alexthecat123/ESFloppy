// Header file for LEDC functions; we have to handle the LEDC manually to achieve the speed that we need

// Some defines for register addresses
#define LEDC_BASE 0x60019000
#define SYSTEM_BASE 0x600C0000
#define GPIO_BASE 0x60004000

#define SYSTEM_PERIP_CLK_EN0_REG (SYSTEM_BASE + 0x18)
#define SYSTEM_PERIP_RST_EN0_REG (SYSTEM_BASE + 0x20)

#define LEDC_CH0_CONF0_REG (LEDC_BASE + 0x00)
#define LEDC_CH0_CONF1_REG (LEDC_BASE + 0x0C)
#define LEDC_CONF_REG (LEDC_BASE + 0xD0)
#define LEDC_CH0_HPOINT_REG (LEDC_BASE + 0x04)
#define LEDC_CH0_DUTY_REG (LEDC_BASE + 0x08)
#define LEDC_CH0_DUTY_R_REG (LEDC_BASE + 0x10)
#define LEDC_TIMER0_CONF_REG (LEDC_BASE + 0xA0)
#define LEDC_TIMER0_VALUE_REG (LEDC_BASE + 0xA4)
#define LEDC_INT_RAW_REG (LEDC_BASE + 0xC0)
#define LEDC_INT_CLR_REG (LEDC_BASE + 0xCC)

// This function gives the LEDC control over the specified pin
// Since it's inline, we have to both declare AND define it here in the header file
inline __attribute__((__always_inline__)) void LEDCControl(uint32_t pin) {
    // Route LEDC channel 0 to the specified GPIO pin
    REG_WRITE((GPIO_BASE + 0x554 + (pin * 4)), ((REG_READ((GPIO_BASE + 0x554 + (pin * 4))) & 0xFFFFFE00) | 73));
}

// This function gives GPIO control over the specified pin
// As with the above function, it's inline so we have to both declare and define it here in the header file
inline __attribute__((__always_inline__)) void GPIOControl(uint32_t pin) {
    // Route the specified GPIO pin to normal GPIO function
    REG_WRITE((GPIO_BASE + 0x554 + (pin * 4)), ((REG_READ((GPIO_BASE + 0x554 + (pin * 4))) & 0xFFFFFE00) | 256));
}

// Initializes the LEDC peripheral and routes it to the specified pin
void initLEDC(uint32_t pin);

// Sets the frequency (and resolution, they kind of go together) of the LEDC's timer and PWM pulse
void setFreq(uint32_t freq, uint32_t resolution);

// Sets the frequency and resolution, but takes raw divider values instead of computing them; use this when you need max speed
void setFreqRaw(uint32_t divider, uint32_t resolution);

// Sets the duty cycle of the LEDC's PWM pulse
void setDuty(uint32_t duty);

// Enables or disables the output of the LEDC
void enableLEDCOutput(bool enabled);