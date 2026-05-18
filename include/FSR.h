#pragma once

#include <Arduino.h>

#include "VolatileData.h"

class FSR {
    private:
        int _pin;              // Teensy analog pin

        // Sensor data
        uint8_t _value;            // normalised reading (0–255)
        bool    _contact;          // debounced contact state

        // Helper Functions
        // Read Raw Analog Data and normalise to 0–255
        void _readRawData(); 

    public:
        // Constructor
        FSR(int pin);

        // Public Methods
        void init();
        void read(volatile uint8_t* value);
};
