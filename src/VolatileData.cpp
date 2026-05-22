#include "VolatileData.h"

// Actual definitions — only compiled once.
// All other files see only the extern declarations in VolatileData.h.

// E-Stop data
// --------------------------------------------------------------------------------------------
volatile bool estop_triggered   = false;

// IMU data
// --------------------------------------------------------------------------------------------
// Raw IMU data
volatile AccelData imu_hip_accel   = {};
volatile GyroData  imu_hip_gyro    = {};
volatile AccelData imu_thigh_accel = {};
volatile GyroData  imu_thigh_gyro  = {};

// Filtered IMU data (Madgwick output)
volatile QuaternionData imu_hip_quaternion   = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float          imu_hip_flex_angle   = 0.0f;
volatile QuaternionData imu_thigh_quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float          imu_thigh_flex_angle = 0.0f;

// FSR data
// --------------------------------------------------------------------------------------------
// Raw FSR values (0–255)
volatile uint8_t fsr_left_heel_value = {}; 
volatile uint8_t fsr_left_toe_value = {};
volatile uint8_t fsr_right_heel_value = {};
volatile uint8_t fsr_right_toe_value = {};

// Contact states
extern volatile bool fsr_left_heel_contact = {}; 
extern volatile bool fsr_left_toe_contact = {};  
extern volatile bool fsr_right_heel_contact = {}; 
extern volatile bool fsr_right_toe_contact = {}; 

// Gait FSM
// --------------------------------------------------------------------------------------------
volatile uint8_t gait_state_left  = 0;   // STANCE
volatile uint8_t gait_state_right = 0;
volatile float   gait_phi_left    = 0.0f;
volatile float   gait_phi_right   = 0.0f;

// Motor control
// --------------------------------------------------------------------------------------------
volatile float tau_cmd_L = 0.0f;
volatile float tau_cmd_R = 0.0f;

// Companion link
// --------------------------------------------------------------------------------------------
volatile float    assistive_gain    = 0.5f;
volatile float    stiffness         = 10.0f;
volatile uint32_t last_companion_ms = 0;
