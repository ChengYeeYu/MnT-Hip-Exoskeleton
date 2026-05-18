#include "VolatileData.h"

// Actual definitions — only compiled once.
// All other files see only the extern declarations in VolatileData.h.

// E-Stop
volatile bool     estop_triggered   = false;

// IMU data
volatile AccelData imu_hip_accel   = {};
volatile GyroData  imu_hip_gyro    = {};


// Gait
volatile uint8_t  gait_phase        = 0;

// Motor control
volatile float    tau_cmd_L         = 0.0f;
volatile float    tau_cmd_R         = 0.0f;

// Companion link
volatile float    assistive_gain    = 0.5f;
volatile float    stiffness         = 10.0f;
volatile uint32_t last_companion_ms = 0;
