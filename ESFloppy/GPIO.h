// Header file for GPIO functions
#include "types.h"

// All of these functions except initPins are inline to maximize speed
// And since they're inline, we have to both declare AND define them here in the header file
// We'll define initPins here too, just to avoid having a separate GPIO.cpp file for a single function

// All this code is stolen almost verbatim from ESProFile, just with the pins changed
// They're all inline functions to maximize speed

// Sets the direction of the parallel bus going to the FPGA
inline __attribute__((__always_inline__)) void setParallelDir(bool dir){
    if(dir == 0){ // Set to an input if dir is 0
        REG_WRITE(GPIO_ENABLE1_W1TC_REG, 0b11111111 << 2);
    }
    else if(dir == 1){ // And an output if dir is 1
        REG_WRITE(GPIO_ENABLE1_W1TS_REG, 0b11111111 << 2);
    }
}

// A user-friendly way to send data over the parallel bus in less time-sensitive situations
inline __attribute__((__always_inline__)) void sendData(uint8_t parallelBits){
    REG_WRITE(GPIO_OUT1_W1TS_REG, parallelBits << 2); // Write W1TS with the data to set all the bits that need to be set
    REG_WRITE(GPIO_OUT1_W1TC_REG, ((byte)~parallelBits << 2)); // Write W1TC with the inverted data to clear all the bits that need to be cleared
}

// A user-friendly way to receive data over the parallel bus in less time-sensitive situations
inline __attribute__((__always_inline__)) uint8_t receiveData(){
    return REG_READ(GPIO_IN1_REG) >> 2; // Return the 8-bit value on the bus
}

// The only output pins on the floppy drive are RDA, SNS, and the LED, and these functions set/clear them with direct register writes
inline __attribute__((__always_inline__)) void writeRDA(bool state){
    if(state){
        REG_WRITE(GPIO_OUT_W1TS_REG, 0b1 << RDA);
    }
    else{
        REG_WRITE(GPIO_OUT_W1TC_REG, 0b1 << RDA);
    }
}

inline __attribute__((__always_inline__)) void writeSNS(bool state){
    if(state){
        REG_WRITE(GPIO_OUT_W1TS_REG, 0b1 << SNS);
    }
    else{
        REG_WRITE(GPIO_OUT_W1TC_REG, 0b1 << SNS);
    }
}

inline __attribute__((__always_inline__)) void writeLED(bool state){
    if(state){
        REG_WRITE(GPIO_OUT_W1TS_REG, 0b1 << LED);
    }
    else{
        REG_WRITE(GPIO_OUT_W1TC_REG, 0b1 << LED);
    }
}

// We've got lots of inputs though; these read functions just return the state of their corresponding control signals
inline __attribute__((__always_inline__)) bool readWRD(){
    return bitRead(REG_READ(GPIO_IN_REG), WRD);
}

inline __attribute__((__always_inline__)) bool readSNS(){
    return bitRead(REG_READ(GPIO_IN_REG), SNS);
}

inline __attribute__((__always_inline__)) bool readWRQ(){
    return bitRead(REG_READ(GPIO_IN_REG), WRQ);
}

inline __attribute__((__always_inline__)) bool readHDS(){
    return bitRead(REG_READ(GPIO_IN_REG), HDS);
}

inline __attribute__((__always_inline__)) bool readPH3(){
    return bitRead(REG_READ(GPIO_IN_REG), PH3);
}

inline __attribute__((__always_inline__)) bool readPH2(){
    return bitRead(REG_READ(GPIO_IN_REG), PH2);
}

inline __attribute__((__always_inline__)) bool readPH1(){
    return bitRead(REG_READ(GPIO_IN_REG), PH1);
}

inline __attribute__((__always_inline__)) bool readPH0(){
    return bitRead(REG_READ(GPIO_IN_REG), PH0);
}

// This reads all the phase pins at once and returns them as a single 4-bit value
inline __attribute__((__always_inline__)) uint8_t readPhases(){
    uint8_t reversed_val = (REG_READ(GPIO_IN_REG) >> PH0) & 0b00001111;
    return ((reversed_val & 0b00000001) << 3) | ((reversed_val & 0b00000010) << 1) | ((reversed_val & 0b00000100) >> 1) | ((reversed_val & 0b00001000) >> 3);
}

inline __attribute__((__always_inline__)) bool readMT1(){
    return bitRead(REG_READ(GPIO_IN_REG), MT1);
}

inline __attribute__((__always_inline__)) bool readMT0(){
    return bitRead(REG_READ(GPIO_IN_REG), MT0);
}

inline __attribute__((__always_inline__)) bool readDR1(){
    return bitRead(REG_READ(GPIO_IN_REG), DR1);
}

inline __attribute__((__always_inline__)) bool readDR0(){
    return bitRead(REG_READ(GPIO_IN1_REG), DR0 - 32);
}

inline __attribute__((__always_inline__)) bool readPWM(){
    return bitRead(REG_READ(GPIO_IN1_REG), PWM - 32);
}

// And finally, a function to set all the pin modes (inputs/outputs) correctly
inline void initPins() {
    pinMode(LED, OUTPUT);
    pinMode(RDA, OUTPUT);
    pinMode(WRD, INPUT);
    pinMode(SNS, OUTPUT);
    pinMode(WRQ, INPUT);
    pinMode(HDS, INPUT);
    pinMode(PH3, INPUT);
    pinMode(PH2, INPUT);
    pinMode(PH1, INPUT);
    pinMode(PH0, INPUT);
    pinMode(MT1, INPUT);
    pinMode(MT0, INPUT);
    pinMode(DR1, INPUT);
    pinMode(DR0, INPUT);
    //setParallelDir(0); // Set the FPGA bus to input mode
    pinMode(LEFT, INPUT_PULLUP);
    pinMode(SEL, INPUT_PULLUP);
    pinMode(RIGHT, INPUT_PULLUP);
    pinMode(PWM, INPUT);
    digitalWrite(LED, LOW); // Turn off the LED initially
    digitalWrite(RDA, HIGH); // And keep RDA high to signify no drive connected until everything is set up
    digitalWrite(SNS, LOW); // Drive SNS low to avoid asserting any of the Twiggy registers
}