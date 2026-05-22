#pragma once

#include <Arduino.h>

#include "VolatileData.h"

class FSR {
    private:
        // Internal Variables
        // --------------------------------------------------------------------------------------------
        // Sensor configuration
        int    _pin;              // Analog pin
        String _sensorName;       // For clean debug logs
        uint8_t _threshold;        // Contact threshold (0–255)

        // Sensor data
        uint8_t _value;            // normalised reading (0–255)
        bool    _contact;          // debounced contact state



        // Helper Functions
        // --------------------------------------------------------------------------------------------
        // Read Raw Analog Data and normalise to 0–255
        void _readRawData(); 

        // Update Contact State
        void _updateContactState();
        
        // Update Threshold based on the min and max values
        void _updateThreshold();

    public:
        // Constructor
        // --------------------------------------------------------------------------------------------
        FSR(String name, int pin);

        // Public Methods
        // --------------------------------------------------------------------------------------------
        // FSR setup
        void init();

        // Read latest sensor data and write directly into the VolatileData.cpp
        void read(volatile uint8_t* value, volatile bool* contact);

        // Serial Monitor (for debugging only)
        // --------------------------------------------------------------------------------------------
        void printAnalogData();

};
