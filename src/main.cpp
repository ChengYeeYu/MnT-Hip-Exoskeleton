#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#include "Pin.h"
#include "VolatileData.h"

#include "IMU.h"
#include "FSR.h"
#include "MadgwickFilter.h"

// Forward declarations 
// -------------------------------------------------------------------------------------------
void ESTOP_ISR();

// Global Peripherals Objects
// -------------------------------------------------------------------------------------------
// Constructors must NOT make hardware calls (Wire, SPI, Serial, pinMode).
// Hardware init happens in setup() via initIMUs() / initFSRs().
// IMUs Objects
IMU imu_hip ("Hip IMU",  IMU1_address, I2C_Bus);

// Madgwick Filter Object (β=0.1, 100 Hz — must match sensorTask rate)
MadgwickFilter madgwick("hip", 0.1f, 100.0f);

// FSR Objects
FSR fsr_left_heel ("Left Heel FSR",  LEFT_HEEL_FSR_PIN);
FSR fsr_left_toe  ("Left Toe FSR",   LEFT_TOE_FSR_PIN);
FSR fsr_right_heel("Right Heel FSR", RIGHT_HEEL_FSR_PIN);
FSR fsr_right_toe ("Right Toe FSR",  RIGHT_TOE_FSR_PIN);


//  Peripheral Initialization Functions
// -------------------------------------------------------------------------------------------

// Initiate Serial Monitor
static void initSerial() {
    Serial.begin(115200);
}

// Setup E-Stop Button and ISR
static void initEStop() {
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), ESTOP_ISR, FALLING);
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 0); // Set GPIO interrupt to highest priority (0)
}

// Initiate IMU Objects 
static void initIMUs() {
    imu_hip.init();
}

// Initiate FSR Objects 
static void initFSRs() {
    fsr_left_heel.init();
    fsr_left_toe.init();
    fsr_right_heel.init();
    fsr_right_toe.init();
}

// Initiate SD Card Objects 
static void initSDCard() {

}

// Initiate Motor Driver Objects 
static void initMotorDriver() {

}

//  E-stop ISR 
// -------------------------------------------------------------------------------------------
void ESTOP_ISR() {
    estop_triggered = true;
}

// FreeRTOS Scheduled Tasks
// -------------------------------------------------------------------------------------------
// ──────────────────────────────────────────────
//  safetyTask — 1 kHz, priority 10 (highest)
//  Clamp, estop check, companion watchdog.
//  Preempts all other tasks within next 1 ms tick.
// ──────────────────────────────────────────────
static void safetyTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // E-stop Check — spin forever sending zero torque, never return from a FreeRTOS task
        while (estop_triggered) {
            // TODO: Serial2.print("c 0 0.0\n");
            // TODO: Serial3.print("c 0 0.0\n");
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // TODO: clamp |tau_cmd_L| and |tau_cmd_R| to MAX_TORQUE (15.0 Nm) → set flag bit 2
        // TODO: rate-limit torque delta: if (Δτ/Δt > 50 Nm/s) → rate-limit command
        // TODO: encoder jump check: if (|Δθ| > 30°/ms) → estop, set flag bit 3
        // TODO: companion watchdog: if (millis() - last_companion_ms > 500) → safe defaults, flag bit 1
        // TODO: UART error check → log, retry ×3, then zero torque, set flag bit 4

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

// ──────────────────────────────────────────────
//  controlTask — 200 Hz, priority 8
//  Re-reads encoders at full rate, runs AdaptiveHip,
//  sends ODrive ASCII torque commands over UART2 + UART3.
// ──────────────────────────────────────────────
static void controlTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // 5a — re-read AS5047P encoders at 200 Hz (SPI, CS_L pin 10, CS_R pin 9)
        // TODO: send 0x3FFF → read 14-bit count → θ_joint = (count/16384) × 2π / 10.0
        // TODO: ω = (θ_current - θ_prev) / 0.005s; handle wrap-around

        // TODO: read volatile: gait_phase, assistive_gain

        // 5b — spline baseline torque
        // TODO: τ_base = torque_profile[profile_id][phi_index] × MAX_TORQUE × 0.30

        // 5c — velocity admittance scaling
        // TODO: Δω = ω_measured - ω_nominal(phi)
        // TODO: τ_out = τ_base × (1.0 + G_ext × Δω)   during extension
        // TODO: τ_out = τ_base × (1.0 + G_flex × Δω)  during flexion

        // 5d — ML assistive gain
        // TODO: τ_final = τ_out × assistive_gain

        // Stage 6a — pre-clamp before UART TX
        // TODO: τ_L = constrain(τ_final_L, -MAX_TORQUE, +MAX_TORQUE)
        // TODO: τ_R = constrain(τ_final_R, -MAX_TORQUE, +MAX_TORQUE)

        // Step 7 — ODrive ASCII torque TX (stagger L then R by ~1.1 ms)
        // TODO: Serial2.print("c 0 <tau_L>\n")  → XDrive Left
        // TODO: (after ~1.1 ms stagger)
        // TODO: Serial3.print("c 0 <tau_R>\n")  → XDrive Right

        // Update shared volatile for safety task
        // tau_cmd_L = τ_L;
        // tau_cmd_R = τ_R;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
    }
}

// ──────────────────────────────────────────────
//  sensorTask — 100 Hz, priority 6
//  IMU read, ADS1115 FSR, encoder SPI, gait FSM update.
// ──────────────────────────────────────────────
static void sensorTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // SPI — AS5047P ×2 (θ, ω at sensor rate)
        // TODO: assert CS_L (pin 10) low → send 0x3FFF → read 14-bit count → deassert
        // TODO: repeat for CS_R (pin 9)
        // TODO: θ_joint = (count / 16384.0) × 2π / gear_ratio (10:1)
        // TODO: ω = (θ_current - θ_prev) / 0.01s; handle ±π wrap-around
        
        // IMU + Madgwick Filter
        imu_hip.read(&imu_hip_accel, &imu_hip_gyro);
        madgwick.update(&imu_hip_accel, &imu_hip_gyro, &imu_hip_quaternion, &imu_hip_flex_angle);
        
        // FSRs 
        fsr_left_heel.read(&fsr_left_heel_value, &fsr_left_heel_contact);
        fsr_left_toe.read(&fsr_left_toe_value, &fsr_left_toe_contact);
        fsr_right_heel.read(&fsr_right_heel_value, &fsr_right_heel_contact);
        fsr_right_toe.read(&fsr_right_toe_value, &fsr_right_toe_contact);

        // Gait FSM update (runs after all sensor reads)
        // TODO: STANCE     → TRANSITION: toe FSR drops below threshold AND ω > ω_threshold
        // TODO: TRANSITION → SWING:      heel FSR=0 AND toe FSR=0 (foot fully airborne)
        // TODO: SWING      → STANCE:     heel FSR rises (heel strike) → reset φ=0
        // TODO: SWING: φ += ω_imu × Δt  (accumulate gait phase %)
        // TODO: write volatile: gait_phase, phi


        // Serial Monitor (for debugging only)
        // imu_hip.printData();
        // magdwick.printData();
        // fsr_left_heel.printAnalogData();
        // fsr_left_toe.printAnalogData();
        // fsr_right_heel.printAnalogData();
        // fsr_right_toe.printAnalogData();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

// ──────────────────────────────────────────────
//  loggerTask — 50 Hz, priority 2 (lowest)
//  Append 64-byte record to RAM buffer.
//  Flush to SD card only when buffer is full (~1280 ms).
//  Never runs inside the control path.
// ──────────────────────────────────────────────
static void loggerTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // TODO: append 64-byte log record to RAM ring buffer (~10 µs)
        // TODO: if (buffer >= 4096 bytes) → SD.write() flush (~5–20 ms, this task only)

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}

// ──────────────────────────────────────────────
//  commsTask — 50 Hz, priority 4
//  TX 68-byte TeensyPacket to companion.
//  RX + validate 16-byte CompanionPacket from companion.
// ──────────────────────────────────────────────
static void commsTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // Extra: Implementation of Rasberry Pi 
        
        // TX — pack TeensyPacket (68 bytes) and send over Serial1 at 921600 baud
        // TODO: fill sync[2]={0xAA,0xFF}, timestamp_ms, imu_L[6], imu_R[6]
        // TODO: fill encoder_L_rad, encoder_R_rad, fsr[4], gait_phase, tau_cmd_L/R, flags
        // TODO: compute XOR checksum over all preceding bytes
        // TODO: Serial1.write((uint8_t*)&pkt, sizeof(TeensyPacket))
        // RX — check for incoming CompanionPacket (16 bytes) from Raspberry Pi
        // TODO: read Serial1 into CompanionPacket buffer
        // TODO: validate sync bytes 0xBB 0xEE + XOR checksum + range checks
        // TODO: on valid packet: assistive_gain = pkt.assistive_gain
        //                        stiffness      = pkt.stiffness
        //                        profile_id     = pkt.profile_id
        //                        last_companion_ms = millis()

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}

//  Setup / FreeRTOS Scheduler Start
// -------------------------------------------------------------------------------------------
void setup() {

    // Peripheral Initialization
    initSerial();
    initEStop();
    initIMUs();
    initFSRs();
    initSDCard();
    initMotorDriver();

    // Create FreeRTOS tasks with appropriate stack sizes and priorities
    // Stack sizes in words (4 bytes each on Cortex-M7). Priority: higher = more urgent
    xTaskCreate(safetyTask,  "safety",  512,  nullptr, 10, nullptr);
    xTaskCreate(controlTask, "control", 2048, nullptr, 8,  nullptr);
    xTaskCreate(sensorTask,  "sensor",  2048, nullptr, 6,  nullptr);
    // xTaskCreate(commsTask,   "comms",   1024, nullptr, 4,  nullptr); // Notes: Currently not considering having a second Raspberry Pi
    xTaskCreate(loggerTask,  "logger",  1024, nullptr, 2,  nullptr);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
    // Never reached — scheduler takes over
}

void loop() {
    // Intentionally empty. FreeRTOS scheduler runs after vTaskStartScheduler().
}