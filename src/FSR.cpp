#include <Arduino.h>

#include "FSR.h"

// Helper Functions
// --------------------------------------------------------------------------------------------
// Read Raw Analog Data and normalise to 0–255
void FSR::_readRawData() {
    _value = (uint8_t)(analogRead(_pin) >> 2);
}

// Update Contact State
void FSR::_updateContactState() {
    _updateThreshold();

    if (_value > _threshold + 10) {
        _contact = true;
    }
    else if (_value < _threshold - 10) {
        _contact = false;
    }
}

// Update Threshold based on the min and max values
void FSR::_updateThreshold() {
    
    // TODO: Implement a more robust thresholding algorithm that adapts to changing conditions, such as sensor drift or varying contact pressures. For example, you could maintain a rolling average of the sensor readings and set the threshold based on this average plus a fixed offset. This would allow the system to adjust to slow changes in the sensor's baseline reading while still detecting significant increases that indicate contact.
    // _threshold = (_threshold + _value) / 2;
}

// Constructor
// --------------------------------------------------------------------------------------------
FSR::FSR(String name, int pin) {
    _sensorName = name;
    _pin     = pin;
    _value   = 0;
    _contact = false;
    _threshold = 50; 
}

// Public Methods
// --------------------------------------------------------------------------------------------
// FSR setup
void FSR::init(){
    pinMode(_pin, INPUT);
}

// Read latest sensor data and write directly into the VolatileData.cpp
void FSR::read(volatile uint8_t* value, volatile bool* contact) {
    _readRawData();
    _updateContactState();
    *value = _value;
    *contact = _contact;
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
// Prints sensor name, raw value, threshold and contact state to Serial Monitor
void FSR::printAnalogData() {
    Serial.println(_sensorName);
    Serial.print(": ");
    Serial.println(_value);
    Serial.print(" | ");
    Serial.println(_threshold);
    Serial.print(" | ");
    Serial.println(_contact);

}

void FSR::printTeleplot() {
    Serial.print(">");
    Serial.print(_sensorName); Serial.print("_value:");
    Serial.print(_value);

    Serial.print("|>");
    Serial.print(_sensorName); Serial.print("_threshold:");
    Serial.print(_threshold);

    Serial.print("|>");
    Serial.print(_sensorName); Serial.print("_contact:");
    Serial.println(_contact ? 1 : 0);
}