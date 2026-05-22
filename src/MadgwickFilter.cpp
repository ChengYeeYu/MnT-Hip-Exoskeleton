#include <Arduino.h>

#include "MadgwickFilter.h"

// Constructor
// --------------------------------------------------------------------------------------------
MadgwickFilter::MadgwickFilter(String name, float beta, float sampleHz) {
    _name = name;
    _beta = beta;
    _dt   = 1.0f / sampleHz;
    _q0   = 1.0f;
    _q1   = 0.0f;
    _q2   = 0.0f;
    _q3   = 0.0f;
}

// Public Methods
// --------------------------------------------------------------------------------------------

// Madgwick AHRS — IMU mode (accelerometer + gyroscope, no magnetometer).
// Algorithm: S. Madgwick, "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays", x-io Technologies, 2010.
//
// Accel input : m/s²  (Adafruit MPU6050 default)
// Gyro  input : rad/s (Adafruit MPU6050 default)
//
// Hip flex angle convention: positive = flexion (forward lift), negative = extension.
// Adjust axis selection below if the IMU is mounted in a different orientation.
void MadgwickFilter::update(volatile AccelData*     accel,
                            volatile GyroData*      gyro,
                            volatile QuaternionData* quat,
                            volatile float*          flex_angle) {
    // Copy from volatile to local to minimise critical section duration
    float ax = accel->x;
    float ay = accel->y;
    float az = accel->z;
    float gx = gyro->x;
    float gy = gyro->y;
    float gz = gyro->z;

    float q0 = _q0, q1 = _q1, q2 = _q2, q3 = _q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope (pure quaternion product)
    qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // Apply accelerometer correction only when accel vector is non-zero
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer
        recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to reduce repeated multiplications
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient descent corrective step (objective: align estimated gravity with measured)
        s0 =  _4q0*q2q2 + _2q2*ax  + _4q0*q1q1 - _2q1*ay;
        s1 =  _4q1*q3q3 - _2q3*ax  + 4.0f*q0q0*q1 - _2q0*ay
             - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 =  4.0f*q0q0*q2 + _2q0*ax  + _4q2*q3q3  - _2q3*ay
             - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 =  4.0f*q1q1*q3 - _2q1*ax  + 4.0f*q2q2*q3 - _2q2*ay;

        // Normalise gradient step
        recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Subtract gradient step from gyro-derived rate of change
        qDot0 -= _beta * s0;
        qDot1 -= _beta * s1;
        qDot2 -= _beta * s2;
        qDot3 -= _beta * s3;
    }

    // Integrate rate of change to yield new quaternion
    q0 += qDot0 * _dt;
    q1 += qDot1 * _dt;
    q2 += qDot2 * _dt;
    q3 += qDot3 * _dt;

    // Normalise quaternion
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    _q0 = q0 * recipNorm;
    _q1 = q1 * recipNorm;
    _q2 = q2 * recipNorm;
    _q3 = q3 * recipNorm;

    // Write quaternion back to the supplied VolatileData pointer
    quat->w = _q0;
    quat->x = _q1;
    quat->y = _q2;
    quat->z = _q3;

    // Extract sagittal-plane hip flexion angle (pitch about Y-axis, ZYX Euler convention).
    // Positive = hip flexion (forward leg lift), negative = extension.
    // If the IMU is mounted with a different axis forward, swap the quaternion terms below.
    *flex_angle = asinf(
        constrain(2.0f * (_q0*_q2 - _q3*_q1), -1.0f, 1.0f)
    ) * 57.2958f;
}

// Euler Angle Extraction (ZYX convention)
// --------------------------------------------------------------------------------------------
// All return degrees.

float MadgwickFilter::getRoll() {
    return atan2f(2.0f*(_q0*_q1 + _q2*_q3), 1.0f - 2.0f*(_q1*_q1 + _q2*_q2)) * 57.2958f;
}

float MadgwickFilter::getPitch() {
    return asinf(constrain(2.0f*(_q0*_q2 - _q3*_q1), -1.0f, 1.0f)) * 57.2958f;
}

float MadgwickFilter::getYaw() {
    return atan2f(2.0f*(_q0*_q3 + _q1*_q2), 1.0f - 2.0f*(_q2*_q2 + _q3*_q3)) * 57.2958f;
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void MadgwickFilter::printData() {
    Serial.print(_name);
    Serial.print(" | q: [");
    Serial.print(_q0, 4); Serial.print(", ");
    Serial.print(_q1, 4); Serial.print(", ");
    Serial.print(_q2, 4); Serial.print(", ");
    Serial.print(_q3, 4);
    Serial.print("] | roll: ");  Serial.print(getRoll(),  2);
    Serial.print(" | pitch: "); Serial.print(getPitch(), 2);
    Serial.print(" | yaw: ");   Serial.print(getYaw(),   2);
    Serial.println(" deg");
}

void MadgwickFilter::printTeleplot() {
    Serial.print(">");  Serial.print(_name); Serial.print("_roll:");  Serial.print(getRoll(),  2);
    Serial.print("|>"); Serial.print(_name); Serial.print("_pitch:"); Serial.print(getPitch(), 2);
    Serial.print("|>"); Serial.print(_name); Serial.print("_yaw:");   Serial.println(getYaw(), 2);
}