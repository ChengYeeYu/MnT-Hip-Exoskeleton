#include <Arduino.h>

#include "FSR.h"

// Helper Functions
// Read Raw Analog Data and normalise to 0–255
void FSR::_readRawData() {
    _value = (uint8_t)(analogRead(_pin) >> 2);
}

// Constructor
FSR::FSR(int pin) {
    _pin     = pin;
    _value   = 0;
    _contact = false;
}

// Public Methods
// FSR setup
void FSR::init(){
    pinMode(_pin, INPUT);
}

// Read latest sensor data and write directly into the VolatileData.cpp
void FSR::read(volatile uint8_t* value) {
    _readRawData();
    *value = _value;
}
