#pragma once

#include "pico/stdlib.h"
#include "servo2040.hpp"

// ----------------------------------------------------------------------------
// Servo pin range — from Pimoroni servo2040.hpp
// ----------------------------------------------------------------------------
// Wiring convention (Phase 7 stage F; host-side, NOT enforced here):
//   pin 0,1,2    → leg 1 (front-right)  coxa, femur, tibia
//   pin 3,4,5    → leg 2 (mid-right)    coxa, femur, tibia
//   pin 6,7,8    → leg 3 (back-right)   coxa, femur, tibia
//   pin 9,10,11  → leg 4 (back-left)    coxa, femur, tibia
//   pin 12,13,14 → leg 5 (mid-left)     coxa, femur, tibia
//   pin 15,16,17 → leg 6 (front-left)   coxa, femur, tibia
// Canonical mapping incl. direction/calibration: contrib/servo_mapping.yaml
// The firmware deliberately knows nothing about joints — it only sees indices.
// ----------------------------------------------------------------------------
constexpr uint START_PIN  = servo::servo2040::SERVO_1;
constexpr uint END_PIN    = servo::servo2040::SERVO_18;
constexpr uint NUM_SERVOS = (END_PIN - START_PIN) + 1;

// ----------------------------------------------------------------------------
// Tick & timing (Phase 7 stage C)
// ----------------------------------------------------------------------------
namespace cfg {
    constexpr uint32_t TICK_HZ              = 100;
    constexpr uint32_t TICK_PERIOD_US       = 1'000'000 / TICK_HZ;  // 10_000 us
    constexpr uint32_t WATCHDOG_TIMEOUT_MS  = 200;

    // Stage C.3 soft-ramp: max pulse-µs change per tick.
    // 20 µs/tick @ 100 Hz = 2 000 µs/s — about one full travel/sec.
    constexpr int16_t MAX_DELTA_PULSE_PER_TICK_US = 20;

    // Default per-servo pulse calibration (SET_CALIBRATION overrides later).
    constexpr int16_t DEFAULT_PULSE_MIN_US  = 500;
    constexpr int16_t DEFAULT_PULSE_MAX_US  = 2500;
    constexpr int16_t DEFAULT_PULSE_ZERO_US = 1500;

    // Boot stagger: ms between enabling consecutive servos (stage D).
    constexpr uint32_t BOOT_STAGGER_MS = 50;

    // Onboard WS2812 LED count.
    constexpr uint32_t NUM_LEDS = 6;

    // ------------------------------------------------------------------------
    // Stage E — current / voltage sensing thresholds
    //
    // The Servo2040 ADC mux exposes a *single* CURRENT_SENSE_ADDR for the
    // total rail current — no per-servo sensing in hardware. The thresholds
    // below are for the 2× MG996R bench test (6.0 V PSU). Re-tune for the
    // full 18-servo robot in Phase 10.
    // ------------------------------------------------------------------------
    constexpr uint32_t TOTAL_CURRENT_MAX_MA  = 3500;  // 2× MG996R: ~1 A normal, ~5 A dual-stall
    constexpr uint16_t UNDERVOLTAGE_WARN_MV  = 5500;  // 6.0 V nominal → 5.5 V warn (-8%)
    constexpr uint16_t UNDERVOLTAGE_CRIT_MV  = 5000;  // → 5.0 V crit (-17%)

    // Sample current + voltage every N ticks (N=5 @ 100 Hz tick = 20 Hz sense rate).
    constexpr uint8_t  SENSE_SAMPLE_EVERY_TICKS = 5;
    // IIR low-pass: smooth = (smooth*7 + sample) / 8  → 1/8 weight on new sample.
    // Time constant ≈ 8 samples / 20 Hz = 400 ms.
}  // namespace cfg

// ----------------------------------------------------------------------------
// Wire-protocol opcodes — see PROTOCOL.md §3
// ----------------------------------------------------------------------------
namespace cmd {
    // 0x01-0x0F: servo control + state roundtrip
    constexpr uint8_t SET_TARGETS     = 0x01;
    constexpr uint8_t GET_STATE       = 0x02;
    constexpr uint8_t STATE_RESPONSE  = 0x82;

    // 0x10-0x1F: servo configuration
    constexpr uint8_t SET_CALIBRATION  = 0x10;
    constexpr uint8_t SET_CURRENT_LIMIT = 0x11;  // payload: u16 LE mA (overrides TOTAL_CURRENT_MAX_MA until power cycle)

    // 0x20-0x2F: servo enable/disable
    constexpr uint8_t ENABLE_SERVO    = 0x20;

    // 0x30-0x3F: LEDs (6 onboard WS2812)
    constexpr uint8_t SET_LED         = 0x30;
    constexpr uint8_t SET_LEDS_ALL    = 0x31;

    // 0x40-0x4F: inputs (sensor pins + USER_SW)
    constexpr uint8_t GET_INPUTS      = 0x40;
    constexpr uint8_t INPUTS_RESPONSE = 0xC0;

    // 0x50-0x5F: system
    constexpr uint8_t RESET           = 0x50;

    // FW -> Host
    constexpr uint8_t ERROR_REPORT    = 0x7F;
    constexpr uint8_t NACK            = 0xFE;
    constexpr uint8_t ACK             = 0xFF;
}  // namespace cmd

// ----------------------------------------------------------------------------
// Error codes (used in ERROR_REPORT / NACK reason) — see PROTOCOL.md §3.4
// ----------------------------------------------------------------------------
namespace err {
    constexpr uint8_t FRAME_CRC          = 0x01;
    constexpr uint8_t FRAME_MALFORMED    = 0x02;
    constexpr uint8_t UNKNOWN_OPCODE     = 0x03;
    constexpr uint8_t PAYLOAD_LEN        = 0x04;
    constexpr uint8_t PULSE_OUT_OF_RANGE = 0x10;
    constexpr uint8_t SERVO_OVERCURRENT  = 0x20;
    constexpr uint8_t TOTAL_OVERCURRENT  = 0x21;
    constexpr uint8_t UNDERVOLTAGE       = 0x30;
    constexpr uint8_t WATCHDOG_TRIPPED   = 0x40;
}  // namespace err

// ----------------------------------------------------------------------------
// Status flag bits (status_flags byte in STATE response) — see PROTOCOL.md §3.1
// ----------------------------------------------------------------------------
namespace status {
    constexpr uint8_t WATCHDOG_TRIPPED              = 1u << 0;
    constexpr uint8_t UNDERVOLTAGE_TRIPPED          = 1u << 1;
    constexpr uint8_t TOTAL_OVERCURRENT_TRIPPED     = 1u << 2;
    constexpr uint8_t ANY_SERVO_OVERCURRENT_TRIPPED = 1u << 3;  // reserved (no per-servo sense HW)
    constexpr uint8_t ANY_SERVO_DISABLED            = 1u << 4;
    constexpr uint8_t UNDERVOLTAGE_WARNING          = 1u << 5;  // warn-only, no servo disable
}  // namespace status
