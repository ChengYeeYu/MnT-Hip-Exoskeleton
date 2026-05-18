#pragma once

#include <Arduino.h>

class FSR {
    private:
        int     _pin;               // Teensy analog pin
        uint8_t _threshold;         // contact threshold (0–255)

        // Sensor data
        uint8_t _value;             // normalised reading (0–255)
        bool    _contact;           // debounced contact state

        // Debounce state
        uint8_t _debounce_count;    // consecutive reads matching _debounce_candidate
        bool    _debounce_candidate;// contact state being evaluated

        // Helper Functions
        uint8_t _readRawData(); // analogRead → normalise to 0–255

    public:
        // Constructor
        // pin       : Teensy analog pin number
        // threshold : contact threshold 0–255
        FSR(int pin, uint8_t threshold);

        // Public Methods
        void init();
        void update();    // read ADC + run debounce, call each sensor task tick
        uint8_t getValue();  // returns normalised FSR value 0–255
        bool isContact(); // returns debounced contact state
};
