#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#include "Pin.h"
#include "IMU.h"
#include "FSR.h"

// ──────────────────────────────────────────────
//  Shared volatile state (spec §4.2)
//  Written/read across tasks — all declared volatile.
// ──────────────────────────────────────────────
volatile float    assistive_gain    = 0.5f;   // written by commsTask,   read by controlTask
volatile float    stiffness         = 10.0f;  // written by commsTask,   read by controlTask
volatile uint8_t  gait_phase        = 0;      // written by sensorTask,  read by controlTask
volatile float    tau_cmd_L         = 0.0f;   // written by controlTask, read by safetyTask
volatile float    tau_cmd_R         = 0.0f;   // written by controlTask, read by safetyTask
volatile bool     estop_triggered   = false;  // written by GPIO ISR,    read by safetyTask
volatile uint32_t last_companion_ms = 0;      // written by commsTask,   read by safetyTask

// ──────────────────────────────────────────────
//  Peripheral init stubs
// ──────────────────────────────────────────────
static void initSPI() {
    // TODO: SPI0 — AS5047P Left (CS pin 10), Right (CS pin 9), SD card (CS pin 8)
    // TODO: SPI mode 1 (CPOL=0, CPHA=1) at 8 MHz for encoders; mode 0 at 25 MHz for SD
}

static void initI2C() {
    // TODO: Wire.begin() at 400 kHz fast mode
    // TODO: IMU Left 0x68 (AD0=GND), IMU Right 0x69 (AD0=VCC), ADS1115 0x48 (ADDR=GND)
    // TODO: 4.7 kΩ pull-ups to 3.3V on SDA (pin 18) and SCL (pin 19)
}

static void initGPIO() {
    // TODO: pinMode(PIN_ESTOP, INPUT_PULLUP)
    // TODO: attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), ESTOP_ISR, FALLING)
    // TODO: set interrupt priority to highest (0)
}

static void initSerial() {
    // TODO: Serial1.begin(921600) — UART1 companion link (TX1 pin 1, RX1 pin 0 → Raspberry Pi)
}

static void initUART() {
    // TODO: Serial2.begin(115200) — UART2 XDrive Left  (TX2 pin 10 → XDrive Left  RX)
    // TODO: Serial3.begin(115200) — UART3 XDrive Right (TX3 pin 14 → XDrive Right RX)
}

// ──────────────────────────────────────────────
//  E-stop ISR — fires within ~1 µs of button press (NC button, falling edge)
// ──────────────────────────────────────────────
void ESTOP_ISR() {
    estop_triggered = true;
}

// ──────────────────────────────────────────────
//  safetyTask — 1 kHz, priority 10 (highest)
//  Clamp, estop check, companion watchdog.
//  Preempts all other tasks within next 1 ms tick.
// ──────────────────────────────────────────────
static void safetyTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // TODO: if (estop_triggered) → Serial2.print("c 0 0.0\n"); Serial3.print("c 0 0.0\n"); return
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

        // I²C — IMU ×2 burst read (register 0x3B, 14 bytes each, addresses 0x68 + 0x69)
        // TODO: convert raw int16 → accel [m/s²] = raw × (9.81 / 16384.0)  [±2g]
        // TODO: convert raw int16 → gyro  [rad/s] = raw × (π / (180 × 131)) [±250°/s]
        // TODO: run Madgwick filter (β=0.1) → quaternion → hip_flex_angle (sagittal)

        // I²C — ADS1115 FSR (pipelined, non-blocking, 1 channel per tick)
        // TODO: read conversion result register from previous cycle → FSR[ch]
        // TODO: trigger next single-shot conversion on next channel
        // TODO: rotate: Heel_L → Toe_L → Heel_R → Toe_R (25 Hz effective per sensor)
        // TODO: contact = (fsr_value > threshold); debounce 3 consecutive reads

        // Gait FSM update (runs after all sensor reads)
        // TODO: STANCE     → TRANSITION: toe FSR drops below threshold AND ω > ω_threshold
        // TODO: TRANSITION → SWING:      heel FSR=0 AND toe FSR=0 (foot fully airborne)
        // TODO: SWING      → STANCE:     heel FSR rises (heel strike) → reset φ=0
        // TODO: SWING: φ += ω_imu × Δt  (accumulate gait phase %)
        // TODO: write volatile: gait_phase, phi

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
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
//  setup / loop
// ──────────────────────────────────────────────
void setup() {
    Serial.begin(115200);  // USB debug only

    initSPI();
    initI2C();
    initGPIO();
    initSerial();
    initUART();

    // Stack sizes in words (4 bytes each on Cortex-M7). Priority: higher = more urgent.
    xTaskCreate(safetyTask,  "safety",  512,  nullptr, 10, nullptr);
    xTaskCreate(controlTask, "control", 2048, nullptr, 8,  nullptr);
    xTaskCreate(sensorTask,  "sensor",  2048, nullptr, 6,  nullptr);
    xTaskCreate(commsTask,   "comms",   1024, nullptr, 4,  nullptr);
    xTaskCreate(loggerTask,  "logger",  1024, nullptr, 2,  nullptr);

    vTaskStartScheduler();
    // Never reached — scheduler takes over.
}

void loop() {
    // Intentionally empty. FreeRTOS scheduler runs after vTaskStartScheduler().
}