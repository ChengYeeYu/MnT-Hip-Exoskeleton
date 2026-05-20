#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "IMU.h"

// Helper Functions
// --------------------------------------------------------------------------------------------
// IMU setup and initialization (return False when failed)
bool IMU::_begin() {
    return _mpu.begin(_address, _wireBus);
}

// 6050 IMU Configuration
void IMU::_configure() {
    _mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // Accelerometer Range
    _mpu.setGyroRange(MPU6050_RANGE_250_DEG);      // Gyroscope Range
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Filter Bandwidth
}

// Read data from the IMU sensor and update internal struct
void IMU::_readData() {
    sensors_event_t a, g, temp;
    _mpu.getEvent(&a, &g, &temp);

    // Linear acceleration in m/s^2
    _accData.x = a.acceleration.x;
    _accData.y = a.acceleration.y;
    _accData.z = a.acceleration.z;

    // Gyroscope data in rad/s
    _gyroData.x = g.gyro.x;
    _gyroData.y = g.gyro.y;
    _gyroData.z = g.gyro.z;
}

// Constructor
// --------------------------------------------------------------------------------------------
IMU::IMU(String name, uint8_t address, TwoWire* bus) {
    _sensorName = name;
    _address    = address;
    _wireBus    = bus;
}

// Public Methods
// --------------------------------------------------------------------------------------------
// Initialize the IMU sensor and configure it
void IMU::init() {
    if (_begin() == false) {
        Serial.print("Failed to initialize IMU: ");
        Serial.println(_sensorName);
    }
    else {
        Serial.print("Successfully initialize IMU: ");
        Serial.println(_sensorName);
        _configure();
    }
}

// Read latest sensor data and write directly into the VolatileData.cpp
void IMU::read(volatile AccelData* accel, volatile GyroData* gyro) {
    _readData();
    accel->x = _accData.x;
    accel->y = _accData.y;
    accel->z = _accData.z;
    gyro->x  = _gyroData.x;
    gyro->y  = _gyroData.y;
    gyro->z  = _gyroData.z;
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
// Prints sensor name, raw linear acceleration and angular velocity values to Serial Monitor
void IMU::printData() {
    Serial.println(_sensorName);
    Serial.print(": ");
    Serial.println(_accData.x);
    Serial.print(" | ");
    Serial.println(_accData.y);
    Serial.print(" | ");
    Serial.println(_accData.z);
    Serial.print(" m/s^2 | ");
    Serial.println(_gyroData.x);
    Serial.print(" | ");
    Serial.println(_gyroData.y);
    Serial.print(" | ");
    Serial.println(_gyroData.z);
    Serial.print(" rad/s ");

}

void IMU::printTeleplot() {
    Serial.print(">");
    Serial.print(_sensorName); Serial.print("_AccX:");
    Serial.print(_accData.x, 4);

    Serial.print("|>");
    Serial.print(_sensorName); Serial.print("_AccY:");
    Serial.print(_accData.y, 4);

    Serial.print("|>");
    Serial.print(_sensorName); Serial.print("_AccZ:");
    Serial.print(_accData.z, 4);

    Serial.print("|>");
    Serial.print(_sensorName); Serial.print("_GyroX:");
    Serial.print(_gyroData.x, 4);

    Serial.print("|>");
    Serial.print(_sensorName); Serial.print("_GyroY:");
    Serial.print(_gyroData.y, 4);

    Serial.print("|>");
    Serial.print(_sensorName); Serial.print("_GyroZ:");
    Serial.println(_gyroData.z, 4);
}