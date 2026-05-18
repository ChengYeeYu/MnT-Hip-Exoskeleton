#include <Arduino.h>

#include "FSR.h"

// Helper Functions
// Read ADC and normalise to 0–255
// Teensy 4.1 default ADC resolution: 10-bit (0–1023)
// Right-shift by 2 maps 0–1023 → 0–255
uint8_t FSR::_readRawData() {
    return (uint8_t)(analogRead(_pin) >> 2);
}

// Constructor
FSR::FSR(int pin, uint8_t threshold) { 
    _pin                = pin;
    _threshold          = threshold;
    _value              = 0;
    _contact            = false;
    _debounce_count     = 0; 
    _debounce_candidate = false; 
}

// Public Methods
// FSR setup
void FSR::init(){
    pinMode(_pin, INPUT);
}

// Read ADC, update _value, run 3-count debounce on contact state.
void FSR::update() {
    _value = _readRawData();

    // 3-count debounce: require 3 consecutive matching reads before flipping _contact
    bool raw_contact = (_value > _threshold);

    if (raw_contact == _debounce_candidate) {
        if (_debounce_count < 3) _debounce_count++;
        if (_debounce_count >= 3) _contact = _debounce_candidate;
    } else {
        _debounce_candidate = raw_contact;
        _debounce_count = 1;
    }
}

uint8_t FSR::getValue() {
    return _value;
}

bool FSR::isContact() {
    return _contact;
}