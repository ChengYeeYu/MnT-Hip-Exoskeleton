#pragma once

#include <Arduino.h>
#include <Wire.h>

// E-stop Pin
const unsigned int PIN_ESTOP = 2;

// IMU Pins
// inline: C++17 — allows safe inclusion in multiple .cpp files (one definition rule)
// constexpr: addresses are compile-time constants, safe in headers
inline TwoWire* const I2C_Bus      = &Wire;
constexpr uint8_t     IMU1_address = 0x68;
constexpr uint8_t     IMU2_address = 0x69;

//FSR Pins
const unsigned int LEFT_HEEL_FSR_PIN  = A14;
const unsigned int LEFT_TOE_FSR_PIN   = A15;
const unsigned int RIGHT_HEEL_FSR_PIN = A5;
const unsigned int RIGHT_TOE_FSR_PIN  = A4;

// SD Card Pins


// Motor Driver Pins