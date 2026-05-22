#pragma once

#include <Arduino.h>

// Shared volatile state — extern declarations only.
// Definitions are in VolatileData.cpp.
// Including this header in multiple .cpp files is safe.

// E-Stop
// --------------------------------------------------------------------------------------------
extern volatile bool estop_triggered;   // written by GPIO ISR,    read by safetyTask

// IMU data 
// Struct types for shared IMU data
// --------------------------------------------------------------------------------------------
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

// Raw IMU data
extern volatile AccelData imu_hip_accel;
extern volatile GyroData  imu_hip_gyro;
extern volatile AccelData imu_thigh_accel;
extern volatile GyroData  imu_thigh_gyro;

// Filtered IMU data (Madgwick output)
// --------------------------------------------------------------------------------------------
struct QuaternionData {
    float w;
    float x;
    float y;
    float z;
};

extern volatile QuaternionData imu_hip_quaternion;     // orientation quaternion [w, x, y, z]
extern volatile float          imu_hip_flex_angle;     // sagittal-plane hip flexion, degrees
extern volatile QuaternionData imu_thigh_quaternion;
extern volatile float          imu_thigh_flex_angle;   // sagittal-plane thigh flexion, degrees

// FSR data
// --------------------------------------------------------------------------------------------
// Raw FSR values (0–255)
extern volatile uint8_t fsr_left_heel_value; 
extern volatile uint8_t fsr_left_toe_value;  
extern volatile uint8_t fsr_right_heel_value; 
extern volatile uint8_t fsr_right_toe_value;  

// Contact states
extern volatile bool fsr_left_heel_contact; 
extern volatile bool fsr_left_toe_contact;  
extern volatile bool fsr_right_heel_contact; 
extern volatile bool fsr_right_toe_contact;  

// Gait FSM  (written by sensorTask, read by controlTask)
// --------------------------------------------------------------------------------------------
// State: 0=STANCE, 1=PUSH_OFF, 2=SWING  (matches GaitState enum)
extern volatile uint8_t gait_state_left;
extern volatile uint8_t gait_state_right;
// Normalised swing phase [0.0–1.0]; 0 at heel-strike, 1 at end-of-swing.
// Only meaningful while gait_state_* == SWING.
extern volatile float gait_phi_left;
extern volatile float gait_phi_right;

// Motor control
// --------------------------------------------------------------------------------------------
extern volatile float tau_cmd_L;         // written by controlTask, read by safetyTask
extern volatile float tau_cmd_R;         // written by controlTask, read by safetyTask

// Companion link
// --------------------------------------------------------------------------------------------
extern volatile float    assistive_gain;    // written by commsTask,   read by controlTask
extern volatile float    stiffness;         // written by commsTask,   read by controlTask
extern volatile uint32_t last_companion_ms; // written by commsTask,   read by safetyTask
