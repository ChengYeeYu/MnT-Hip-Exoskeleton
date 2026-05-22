#pragma once

#include <Arduino.h>

// Gait phase state machine — one instance per leg.
//
// State diagram (biomechanical ZYX convention):
//
//   STANCE ──► PUSH_OFF ──► SWING ──► STANCE
//                  │                     ▲
//                  └─── (re-plant) ──────┘
//
// phi: normalised swing phase [0.0–1.0]; 0 = heel-strike, 1 = end of swing.
//      Meaningful only in SWING state; held at 0.0 in STANCE / PUSH_OFF.

enum class GaitState : uint8_t {
    STANCE   = 0,   // heel or toe contact — foot loaded
    PUSH_OFF = 1,   // toe-only contact, high pitch velocity — leg leaving ground
    SWING    = 2,   // both FSRs clear — foot airborne, phi accumulating
};

class GaitFSM {
    public:
        // name     : label prefix used in Teleplot / Serial output  (e.g. "left", "right")
        // dt       : sample period in seconds — must match the rate update() is called at
        GaitFSM(String name, float dt = 0.01f);

        // Call once per sensor cycle (after FSR and IMU reads).
        // gyro_pitch : sagittal-plane angular velocity in rad/s (positive = flexion)
        void update(bool heel_contact, bool toe_contact, float gyro_pitch);

        GaitState getState()     const { return _state; }
        float     getPhi()       const { return _phi;   }

        // True for exactly one cycle immediately after heel strike.
        bool      heelStrike()   const { return _heel_strike; }

        // Serial Monitor (for debugging only)
        void printData();


    private:
        String    _name;
        GaitState _state;
        float     _phi;
        float     _dt;
        bool      _heel_strike;

        // Minimum sagittal pitch rate (rad/s) needed to confirm push-off intent.
        // Prevents slow weight-shifts from falsely triggering PUSH_OFF.
        static constexpr float GYRO_PUSHOFF_THRESH = 0.35f;

        // Estimated swing half-period (s) used to normalise phi.
        // Tune to the subject's comfortable cadence (~120 bpm → ~0.25 s swing).
        static constexpr float NOMINAL_SWING_S = 0.45f;
    };
