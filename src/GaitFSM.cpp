#include <Arduino.h>

#include "GaitFSM.h"


// TODO: Add IMU + Encoder data to the FSM update logic, for more robust state transitions and richer Teleplot output.
GaitFSM::GaitFSM(String name, float dt)
    : _name(name)
    , _state(GaitState::STANCE)
    , _phi(0.0f)
    , _dt(dt)
    , _heel_strike(false)
{}

void GaitFSM::update(bool heel_contact, bool toe_contact, float gyro_pitch) {
    _heel_strike = false;

    switch (_state) {

        case GaitState::STANCE:
            // Heel lifts while toe stays down AND pitch velocity confirms active push-off.
            if (!heel_contact && toe_contact && fabsf(gyro_pitch) > GYRO_PUSHOFF_THRESH) {
                _state = GaitState::PUSH_OFF;
            }
            break;

        case GaitState::PUSH_OFF:
            // Both sensors clear → fully airborne, begin swing-phase accumulation.
            if (!heel_contact && !toe_contact) {
                _state = GaitState::SWING;
                _phi   = 0.0f;
                break;
            }
            // Foot re-planted (stumble or aborted step) → return to stance.
            if (heel_contact) {
                _state = GaitState::STANCE;
            }
            break;
        
        // TODO: Add mid-swing event (e.g. for triggering active assistance) once IMU + encoder data are integrated into the FSM logic.
        case GaitState::SWING:
            // Heel strike: end of swing cycle.
            if (heel_contact) {
                _state      = GaitState::STANCE;
                _heel_strike = true;
                _phi        = 0.0f;
                break;
            }
            // Accumulate normalised swing phase; clamp to 1.0.
            _phi += _dt / NOMINAL_SWING_S;
            if (_phi > 1.0f) _phi = 1.0f;
            break;
    }
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void GaitFSM::printData() const {
    static const char* const STATE_NAMES[] = { "STANCE", "PUSH_OFF", "SWING" };

    Serial.print(_name);
    Serial.print(" | state: ");
    Serial.print(STATE_NAMES[static_cast<uint8_t>(_state)]);
    Serial.print(" | phi: ");
    Serial.print(_phi, 3);
    Serial.print(" | heel_strike: ");
    Serial.println(_heel_strike ? "YES" : "no");
}

void GaitFSM::printTeleplot() const {
    static uint8_t skip = 0;
    if (++skip < 10) return;
    skip = 0;

    String name = _name;
    name.replace(" ", "_");

    Serial.print(">");  Serial.print(name); Serial.print("_state:"); Serial.print(static_cast<uint8_t>(_state));
    Serial.print("|>"); Serial.print(name); Serial.print("_phi:");   Serial.println(_phi, 3);
}
