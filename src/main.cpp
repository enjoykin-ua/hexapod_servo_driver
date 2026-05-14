#include <stdio.h>

#include "pico/stdlib.h"
#include "servo2040.hpp"

#include "config.hpp"
#include "proto/frame.hpp"

// =============================================================================
// Phase 7 Stage C.3 — hard-clamp + watchdog + soft-ramp + ServoCluster
//
// Implemented:
//   - C.0: COBS+CRC frame layer, 100 Hz non-blocking main loop, GET_STATE, RESET
//   - C.1: ServoCluster init for all 18 channels, per-servo pulse_min/max/zero
//          (defaults), SET_TARGETS with hard-clamp, ENABLE_SERVO
//   - C.2: Watchdog — disables all servos after 200 ms without a valid frame,
//          sends unsolicited ERROR_REPORT, cleared by RESET
//   - C.3: Soft-ramp — limits pulse change to MAX_DELTA_PULSE_PER_TICK_US per
//          tick; current chases target at 2000 µs/s @ 100 Hz
//
// NOT YET implemented (later stages):
//   - SET_CALIBRATION (stage F) / SET_LED / SET_LEDS_ALL / GET_INPUTS (stage G+)
//   - Current / voltage sensing (stage E)
// =============================================================================

namespace {

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------
int16_t  target_pulse_us  [NUM_SERVOS] = {};
int16_t  current_pulse_us [NUM_SERVOS] = {};
int16_t  pulse_min_us     [NUM_SERVOS];
int16_t  pulse_max_us     [NUM_SERVOS];
int16_t  pulse_zero_us    [NUM_SERVOS];
uint16_t last_current_ma  [NUM_SERVOS] = {};
bool     servo_enabled    [NUM_SERVOS] = {};

uint16_t rail_voltage_mv = 0;
uint8_t  status_flags    = 0;

uint8_t out_buf[proto::MAX_FRAME_LEN_WIRE];

// ServoCluster pointer — set in main() once the cluster is live.
servo::ServoCluster* g_servos = nullptr;

// Watchdog: armed only after the first valid frame is received, so the
// board doesn't trip immediately at boot before a host has a chance to
// say hello.
bool            watchdog_armed = false;
absolute_time_t last_valid_frame_time;

// -----------------------------------------------------------------------------
// Frame send helpers
// -----------------------------------------------------------------------------
void send_frame(uint8_t seq, uint8_t opcode, const uint8_t* payload, uint8_t len) {
    proto::Frame f{};
    f.seq = seq;
    f.cmd = opcode;
    f.len = len;
    for (uint8_t i = 0; i < len; ++i) f.payload[i] = payload[i];

    size_t n = proto::encode_frame(f, out_buf, sizeof(out_buf));
    if (n == 0) return;
    for (size_t i = 0; i < n; ++i) {
        putchar_raw(out_buf[i]);
    }
    stdio_flush();
}

void send_ack(uint8_t seq, uint8_t original_cmd) {
    uint8_t p[1] = { original_cmd };
    send_frame(seq, cmd::ACK, p, 1);
}

void send_nack(uint8_t seq, uint8_t original_cmd, uint8_t reason) {
    uint8_t p[2] = { original_cmd, reason };
    send_frame(seq, cmd::NACK, p, 2);
}

void send_error(uint8_t seq, uint8_t err_code, uint8_t servo_idx, int16_t aux) {
    uint8_t p[4] = {
        err_code,
        servo_idx,
        static_cast<uint8_t>(aux & 0xFF),
        static_cast<uint8_t>((aux >> 8) & 0xFF),
    };
    send_frame(seq, cmd::ERROR_REPORT, p, 4);
}

// -----------------------------------------------------------------------------
// Hard-clamp (Safety layer 1, PROTOCOL.md §7 "Pulse-Wertebereich")
// -----------------------------------------------------------------------------
int16_t clamp_pulse(uint8_t i, int16_t pulse, bool& clamped_out) {
    if (pulse < pulse_min_us[i]) { clamped_out = true; return pulse_min_us[i]; }
    if (pulse > pulse_max_us[i]) { clamped_out = true; return pulse_max_us[i]; }
    clamped_out = false;
    return pulse;
}

// -----------------------------------------------------------------------------
// Status flag bookkeeping
// -----------------------------------------------------------------------------
void update_disabled_flag() {
    bool any_disabled = false;
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        if (!servo_enabled[i]) { any_disabled = true; break; }
    }
    if (any_disabled) status_flags |=  status::ANY_SERVO_DISABLED;
    else              status_flags &= ~status::ANY_SERVO_DISABLED;
}

// -----------------------------------------------------------------------------
// Command handlers
// -----------------------------------------------------------------------------
void handle_get_state(uint8_t seq) {
    uint8_t payload[75];
    size_t off = 0;

    for (uint i = 0; i < NUM_SERVOS; ++i) {
        int16_t v = current_pulse_us[i];
        payload[off++] = static_cast<uint8_t>(v & 0xFF);
        payload[off++] = static_cast<uint8_t>((v >> 8) & 0xFF);
    }
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        uint16_t v = last_current_ma[i];
        payload[off++] = static_cast<uint8_t>(v & 0xFF);
        payload[off++] = static_cast<uint8_t>((v >> 8) & 0xFF);
    }
    payload[off++] = static_cast<uint8_t>(rail_voltage_mv & 0xFF);
    payload[off++] = static_cast<uint8_t>((rail_voltage_mv >> 8) & 0xFF);
    payload[off++] = status_flags;

    send_frame(seq, cmd::STATE_RESPONSE, payload, static_cast<uint8_t>(off));
}

void handle_reset(uint8_t seq) {
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        if (g_servos) g_servos->disable(i, false);
        servo_enabled[i]    = false;
        target_pulse_us[i]  = pulse_zero_us[i];
        current_pulse_us[i] = pulse_zero_us[i];
    }
    if (g_servos) g_servos->load();
    // Disarm watchdog (a fresh RESET is the recovery path from a trip).
    watchdog_armed = false;
    status_flags   = status::ANY_SERVO_DISABLED;
    send_ack(seq, cmd::RESET);
}

void handle_set_targets(uint8_t seq, const uint8_t* p, uint8_t len) {
    if (len != 36) {
        send_error(seq, err::PAYLOAD_LEN, 0, 36);
        return;
    }
    bool    any_clamped       = false;
    uint8_t first_clamped_idx = 0;
    int16_t first_clamped_raw = 0;
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        int16_t raw = static_cast<int16_t>(
            static_cast<uint16_t>(p[2*i])
          | (static_cast<uint16_t>(p[2*i + 1]) << 8));
        bool clamped = false;
        target_pulse_us[i] = clamp_pulse(static_cast<uint8_t>(i), raw, clamped);
        if (clamped && !any_clamped) {
            any_clamped       = true;
            first_clamped_idx = static_cast<uint8_t>(i);
            first_clamped_raw = raw;
        }
    }
    if (any_clamped) {
        send_error(seq, err::PULSE_OUT_OF_RANGE,
                   first_clamped_idx, first_clamped_raw);
    } else {
        send_ack(seq, cmd::SET_TARGETS);
    }
}

void handle_enable_servo(uint8_t seq, const uint8_t* p, uint8_t len) {
    if (len != 2) {
        send_error(seq, err::PAYLOAD_LEN, 0, 2);
        return;
    }
    // PROTOCOL.md §6: while WATCHDOG_TRIPPED is set, ENABLE_SERVO is
    // refused — host must send RESET first.
    if (status_flags & status::WATCHDOG_TRIPPED) {
        send_nack(seq, cmd::ENABLE_SERVO, err::WATCHDOG_TRIPPED);
        return;
    }
    uint8_t idx = p[0];
    uint8_t en  = p[1];
    if (idx >= NUM_SERVOS) {
        send_nack(seq, cmd::ENABLE_SERVO, err::PAYLOAD_LEN);
        return;
    }
    servo_enabled[idx] = (en != 0);
    if (g_servos) {
        if (en) g_servos->enable(idx, true);
        else    g_servos->disable(idx, true);
    }
    update_disabled_flag();
    send_ack(seq, cmd::ENABLE_SERVO);
}

void dispatch(const proto::Frame& f) {
    // Every valid (CRC-checked) frame keeps the watchdog happy.
    last_valid_frame_time = get_absolute_time();
    watchdog_armed        = true;

    switch (f.cmd) {
        case cmd::GET_STATE:
            if (f.len != 0) { send_error(f.seq, err::PAYLOAD_LEN, 0, 0); return; }
            handle_get_state(f.seq);
            return;
        case cmd::RESET:
            if (f.len != 0) { send_error(f.seq, err::PAYLOAD_LEN, 0, 0); return; }
            handle_reset(f.seq);
            return;
        case cmd::SET_TARGETS:
            handle_set_targets(f.seq, f.payload, f.len);
            return;
        case cmd::ENABLE_SERVO:
            handle_enable_servo(f.seq, f.payload, f.len);
            return;

        // Stages F (calibration) / later (LEDs, inputs)
        case cmd::SET_CALIBRATION:
        case cmd::SET_LED:
        case cmd::SET_LEDS_ALL:
        case cmd::GET_INPUTS:
            send_nack(f.seq, f.cmd, err::UNKNOWN_OPCODE);
            return;

        default:
            send_error(f.seq, err::UNKNOWN_OPCODE, 0, 0);
            return;
    }
}

// -----------------------------------------------------------------------------
// Tick
// -----------------------------------------------------------------------------
void on_tick() {
    if (!g_servos) return;

    // C.2 — Watchdog: if armed and no valid frame within the timeout,
    // disable everything and set the trip flag. Recovery requires RESET.
    if (watchdog_armed && !(status_flags & status::WATCHDOG_TRIPPED)) {
        int64_t age_us = absolute_time_diff_us(last_valid_frame_time,
                                               get_absolute_time());
        if (age_us > static_cast<int64_t>(cfg::WATCHDOG_TIMEOUT_MS) * 1000) {
            for (uint i = 0; i < NUM_SERVOS; ++i) {
                g_servos->disable(static_cast<uint8_t>(i), /*load=*/false);
                servo_enabled[i] = false;
            }
            g_servos->load();
            status_flags |= status::WATCHDOG_TRIPPED | status::ANY_SERVO_DISABLED;
            // Unsolicited error report (seq = 0).
            send_error(0, err::WATCHDOG_TRIPPED, 0, 0);
        }
    }

    // C.3 — Soft-ramp: limit pulse change per tick to MAX_DELTA_PULSE_PER_TICK_US.
    // No matter how big the host's jump in target is, current chases it at
    // most MAX_DELTA_PULSE_PER_TICK_US per tick (= 2000 µs/s @ 100 Hz default).
    constexpr int16_t step = cfg::MAX_DELTA_PULSE_PER_TICK_US;
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        int16_t delta = static_cast<int16_t>(target_pulse_us[i] - current_pulse_us[i]);
        if      (delta >  step) current_pulse_us[i] += step;
        else if (delta < -step) current_pulse_us[i] -= step;
        else                    current_pulse_us[i]  = target_pulse_us[i];
        g_servos->pulse(static_cast<uint8_t>(i),
                        static_cast<float>(current_pulse_us[i]),
                        /*load=*/false);
    }
    g_servos->load();

    // Stage E (current/voltage sensing) → here.
}

}  // namespace

// =============================================================================
// Entry point
// =============================================================================
int main() {
    stdio_init_all();

    // Wait for the host to actually open /dev/ttyACM* before printing the
    // boot banner — otherwise the bytes are lost. The board stays reachable
    // via picotool throughout this wait.
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(50);

    // Init per-servo state (defaults — overridden by SET_CALIBRATION later).
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        pulse_min_us[i]     = cfg::DEFAULT_PULSE_MIN_US;
        pulse_max_us[i]     = cfg::DEFAULT_PULSE_MAX_US;
        pulse_zero_us[i]    = cfg::DEFAULT_PULSE_ZERO_US;
        target_pulse_us[i]  = cfg::DEFAULT_PULSE_ZERO_US;
        current_pulse_us[i] = cfg::DEFAULT_PULSE_ZERO_US;
        servo_enabled[i]    = false;
    }
    status_flags = status::ANY_SERVO_DISABLED;

    // Init ServoCluster (PIO0 SM0, 18 servos starting at SERVO_1 = GPIO 0).
    // All servos start disabled — host must explicitly send ENABLE_SERVO.
    static servo::ServoCluster servos(pio0, 0, START_PIN, NUM_SERVOS);
    servos.init();
    g_servos = &servos;

    // Banner — keep "Servo2040 USB-UART Communication Started" prefix
    // so tools/flash_and_verify.py keeps matching.
    printf("Servo2040 USB-UART Communication Started (vC.3 soft-ramp)\n");

    proto::FrameAssembler decoder;
    proto::Frame frame;

    absolute_time_t next_tick = make_timeout_time_us(cfg::TICK_PERIOD_US);

    while (true) {
        int ch;
        while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if (decoder.feed(static_cast<uint8_t>(ch & 0xFF), frame)) {
                dispatch(frame);
            }
        }

        if (time_reached(next_tick)) {
            on_tick();
            next_tick = delayed_by_us(next_tick, cfg::TICK_PERIOD_US);
        }

        tight_loop_contents();
    }
}
