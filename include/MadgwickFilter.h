#pragma once

#include <Arduino.h>

#include "VolatileData.h"

class MadgwickFilter {
    private:
        String _name;           // label prefix used in Teleplot output
        float  _beta;           // algorithm gain — higher trusts accel more, lower trusts gyro more
        float  _dt;             // sample period in seconds (1 / sampleHz)

        // Internal quaternion state [w, x, y, z]
        float _q0, _q1, _q2, _q3;

    public:
        // Constructor
        // --------------------------------------------------------------------------------------------
        // name      : label prefix for Teleplot output (e.g. "hip")
        // beta      : gradient descent step size (0.1 typical for slow/medium motion)
        // sampleHz  : must match the rate at which update() is called (default 100 Hz)
        MadgwickFilter(String name, float beta = 0.1f, float sampleHz = 100.0f);

        // Public Methods
        // --------------------------------------------------------------------------------------------

        // Run one filter step for the given IMU.
        // Reads raw accel + gyro from the supplied VolatileData pointers,
        // then writes the updated quaternion and sagittal-plane hip flex angle back.
        void update(volatile AccelData*     accel,
                    volatile GyroData*      gyro,
                    volatile QuaternionData* quat,
                    volatile float*          flex_angle);

        // Euler angles derived from the internal quaternion state (ZYX convention).
        // All values in degrees.
        float getRoll();    // rotation about X-axis (lateral tilt)
        float getPitch();   // rotation about Y-axis (forward/back — hip flexion)
        float getYaw();     // rotation about Z-axis (horizontal turn)

        void setBeta(float beta)   { _beta = beta; }
        void setSampleHz(float hz) { _dt   = 1.0f / hz; }

        // Serial Monitor (for debugging only)
        // --------------------------------------------------------------------------------------------
        void printData();
};