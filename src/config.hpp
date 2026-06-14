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

// Phase 13 Stage 0.1 — Relay power-gate on the A0 header (GP26). High-trigger,
// normally-open relay in the servo V+ rail: GP26 LOW = open = servos unpowered,
// GP26 HIGH = closed = servos powered. Default LOW (fail-safe). See
// docs_raspi/phase_13_stage_0_1_relay_plan.md and pimoroni_servo_fix/src/
// test_relay_power_sequence.cpp.
constexpr uint RELAY_PIN  = servo::servo2040::ADC0;   // GP26 = A0 header pin

// Switch test (temporary wiring-check). External switch on the A1 header:
//   switch pin 1 -> 3.3V, switch pin 2 -> A1 (GP27).
// Read as a plain digital input with the RP2040's internal pull-down enabled,
// so OPEN = 0 (pulled to GND) and CLOSED = 1 (3.3V). A 0->1 rising edge lights
// the onboard WS2812 LED bar. GP27 = ADC1 is a free user pin (sensing uses GP29).
constexpr uint SWITCH_PIN = servo::servo2040::ADC1;   // GP27 = A1 header pin

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
    // total rail current — no per-servo sensing in hardware.
    // TOTAL_CURRENT_MAX_MA = 18-servo robot. 2026-05-31 user-set 10000 mA, damit
    // der Phase-13-Stage-0.7-Vergleich (altes joint-space-Aufstehen B-T3 zieht
    // ~10 A) nicht trippt; cartesian-Aufstehen (B-T2) bleibt darunter. War 7000
    // (und davor 3500, 2× MG996R bench). Software trip only — die PSU/Verkabelung
    // muss 10 A liefern können; weniger Schutz, bewusst fuer den Mess-Vergleich.
    // NOTE: UNDERVOLTAGE_* below are still on the old 6.0 V bench PSU; re-tune
    // for the 2S-LiPo (7.4 V nom) when that rail is wired (separate change).
    // ------------------------------------------------------------------------
    constexpr uint32_t TOTAL_CURRENT_MAX_MA  = 10000;  // 18-servo robot, user-set 2026-05-31 (war 7000/3500)
    // 2026-05-31 user-set auf 4 V herab, damit der Stage-0.7-B-T3-Vergleich
    // (joint-space-Aufstehen bricht die PSU-Spannung ein) nicht per Undervoltage-
    // Trip abbricht. ⚠️ 4 V ist UNTER der Servo-Min-Spec (4.8 V) — bewusst nur
    // fuer den Mess-Vergleich; danach wieder hochsetzen.
    constexpr uint16_t UNDERVOLTAGE_WARN_MV  = 4500;  // war 5500
    constexpr uint16_t UNDERVOLTAGE_CRIT_MV  = 4000;  // war 5000 — latched Trip @ 4.0 V

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
    constexpr uint8_t RELAY_CONTROL   = 0x51;  // payload: 1 byte (1=on/HIGH, 0=off/LOW)

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
    constexpr uint8_t RELAY_ON                      = 1u << 6;  // Stage 0.1: relay power-gate closed
}  // namespace status
