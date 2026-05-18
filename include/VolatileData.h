#pragma once

#include <Arduino.h>

// Shared volatile state — extern declarations only.
// Definitions are in VolatileData.cpp.
// Including this header in multiple .cpp files is safe.

// E-Stop
extern volatile bool     estop_triggered;   // written by GPIO ISR,    read by safetyTask

// IMU data — written by sensorTask after calling imu_left.read() / imu_right.read()
// Struct types for shared IMU data
struct AccelData { 
    float x; 
    float y; 
    float z; 
};
struct GyroData  { 
    float x; 
    float y; 
    float z; 
};

extern volatile AccelData imu_hip_accel;   // read by controlTask
extern volatile GyroData  imu_hip_gyro;

// Gait
extern volatile uint8_t  gait_phase;        // written by sensorTask,  read by controlTask

// Motor control
extern volatile float    tau_cmd_L;         // written by controlTask, read by safetyTask
extern volatile float    tau_cmd_R;         // written by controlTask, read by safetyTask

// Companion link
extern volatile float    assistive_gain;    // written by commsTask,   read by controlTask
extern volatile float    stiffness;         // written by commsTask,   read by controlTask
extern volatile uint32_t last_companion_ms; // written by commsTask,   read by safetyTask
