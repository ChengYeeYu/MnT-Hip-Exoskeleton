#include <Arduino.h>

#include "Controller.h"

Controller::Controller(String name, float peak_torque_nm, float direction)
    : _name(name), _peak_Nm(peak_torque_nm), _dir(direction)
{}

float Controller::compute(uint8_t gait_state, float phi, float gain) {
    float tau = 0.0f;

    if (gait_state == GAIT_SWING) {
        // Single sine pulse centred on mid-swing.
        // phi=0 (heel-strike) → tau=0
        // phi=0.5 (mid-swing) → tau=peak
        // phi=1.0 (end-swing) → tau=0
        tau = _peak_Nm * gain * sinf(PI * phi);
    }

    // Apply motor mounting direction then hard-clamp.
    tau *= _dir;
    return constrain(tau, -MAX_TORQUE_NM, MAX_TORQUE_NM);
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void Controller::printData(float tau) const {
    Serial.print(_name);
    Serial.print(" | tau: ");
    Serial.print(tau, 3);
    Serial.println(" Nm");
}

void Controller::printTeleplot(const String& pfx, float tau) const {
    Serial.print(">"); Serial.print(pfx); Serial.print("_tau:"); Serial.println(tau, 3);
}
