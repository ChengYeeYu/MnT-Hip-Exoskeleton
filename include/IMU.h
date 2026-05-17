#pragma once

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class IMU {
    private:
        Adafruit_MPU6050 _mpu; // Object composition instead of inheritance (Adafruit Library)
        uint8_t _address;       // Sensor I2C address (0x68 or 0x69) (Check hardware description)
        TwoWire* _wireBus;      // Pointer to the chosen Wire bus
        String _sensorName;     // For clean debug logs

        // Sensor Data
        struct _accelerometerData {
            float x;
            float y;
            float z;
        } _accData;

        struct _gyroscopeData {
            float x;
            float y;
            float z;
        } _gyroData;

        // Helper Functions
        bool _begin();      // IMU setup and initialization (return False when failed)
        void _configure();  // 6050 IMU Configuration
        void _readData();   // Read data from the IMU sensor and update internal struct

    public:
        // Constructor
        IMU(String name, uint8_t address, TwoWire* bus);

        // Public Methods
        // Initialize the IMU sensor and configure it. Returns true if successful, false otherwise.
        void init();
    };