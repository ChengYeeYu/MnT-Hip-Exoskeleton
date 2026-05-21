#pragma once

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "VolatileData.h"

class IMU {
    private:
        // Internal Variables
        // --------------------------------------------------------------------------------------------
        // Sensor configuration
        Adafruit_MPU6050 _mpu;        // Object composition instead of inheritance (Adafruit Library)
        uint8_t          _address;    // Sensor I2C address (0x68 or 0x69) (Check hardware description)
        TwoWire*         _wireBus;    // Pointer to the chosen Wire bus
        String           _sensorName; // For clean debug logs
        bool             _initialized = false;

        // Sensor Data
        struct { float x; float y; float z; } _accData;
        struct { float x; float y; float z; } _gyroData;

        // Helper Functions
        // --------------------------------------------------------------------------------------------
        bool _begin();      // IMU setup and initialization (return False when failed)
        void _configure();  // 6050 IMU Configuration
        void _readData();   // Read sensor and update _accData / _gyroData

    public:
        // Constructor
        // --------------------------------------------------------------------------------------------
        IMU(String name, uint8_t address, TwoWire* bus);

        // Public Methods
        // --------------------------------------------------------------------------------------------
        // Initialize the IMU sensor and configure it
        void init(); 

        // Read latest sensor data and write directly into the VolatileData.cpp
        void read(volatile AccelData* accel, volatile GyroData* gyro);

        // Serial Monitor (for debugging only)
        // --------------------------------------------------------------------------------------------
        void printData();
        void printTeleplot();   // sends accel xyz + gyro xyz to Teleplot
};