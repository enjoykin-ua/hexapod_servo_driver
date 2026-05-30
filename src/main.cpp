#include <stdio.h>
#include <algorithm>

#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "analog.hpp"
#include "analogmux.hpp"

#include "config.hpp"
#include "proto/frame.hpp"

// =============================================================================
// Phase 7 Stage E — current / voltage sensing + total-current + undervoltage trip
//
// Implemented:
//   - C.0–C.3: COBS+CRC, 100 Hz loop, hard-clamp, watchdog, soft-ramp
//   - D.1:    Per-servo ENABLE_SERVO with staged boot (host-driven, 50 ms stagger)
//   - E.1:    Total rail current trip (TOTAL_CURRENT_MAX_MA, hardware does NOT
//             expose per-servo current — only total via CURRENT_SENSE_ADDR mux)
//   - E.2:    Undervoltage warn (UNDERVOLTAGE_WARN_MV, auto-clearing) + critical
//             trip (UNDERVOLTAGE_CRIT_MV, RESET to clear)
//
// NOT YET implemented (later stages):
//   - SET_CALIBRATION (stage F) / SET_LED / SET_LEDS_ALL / GET_INPUTS (stage G+)
//   - Per-servo stall detection via software (would need pulse-vs-target chase
//     timing; deferred to Phase 10 if needed)
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

// Stage E — ADC mux + analog sensing.
pimoroni::AnalogMux* g_mux       = nullptr;
pimoroni::Analog*    g_cur_sense = nullptr;
pimoroni::Analog*    g_vol_sense = nullptr;

// IIR-smoothed total rail current (mA). Stored as 32-bit to keep the
// (smooth*7 + sample) intermediate within range; cast to uint16 for GET_STATE.
uint32_t rail_current_ma_smooth = 0;
bool     sense_seeded           = false;
uint8_t  sense_tick_counter     = 0;
uint8_t  sense_warmup_samples   = 0;  // gate trip logic until filter has settled

// Runtime-overridable current trip threshold. Initialised from the compile-time
// default but can be lowered (e.g. by tests) via SET_CURRENT_LIMIT. Resets to
// the compile-time default on power cycle, NOT on RESET.
uint32_t runtime_current_max_ma = cfg::TOTAL_CURRENT_MAX_MA;

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
    // Phase 13 FW-Fix: explicit per-pin disable BEFORE state-reset is
    // critical. Pimoroni's ServoCluster::pulse() auto-enables the pin on
    // any value >= MIN_VALID_PULSE — so without this explicit disable,
    // the very next on_tick would re-enable pins via pulse() and emit
    // pulse_zero (~1500 µs = horizontal) on physically-powered servos.
    // The on_tick path also now skips pulse() for disabled pins (see
    // change in on_tick below), but issuing disable() here additionally
    // commits PWM-out=0 immediately, not waiting for the next tick.
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        if (g_servos) g_servos->disable(i, false);
        servo_enabled[i]    = false;
        target_pulse_us[i]  = pulse_zero_us[i];
        current_pulse_us[i] = pulse_zero_us[i];
    }
    if (g_servos) g_servos->load();
    // Disarm watchdog (a fresh RESET is the recovery path from a trip).
    watchdog_armed = false;
    // Clear all trip flags; ANY_SERVO_DISABLED stays set (everything is disabled now).
    // UNDERVOLTAGE_WARNING is auto-managed by the sense loop so we leave it alone.
    status_flags = status::ANY_SERVO_DISABLED |
                   (status_flags & status::UNDERVOLTAGE_WARNING);
    // Reset the IIR-smoothed rail current AND re-arm the warmup gate.
    // Otherwise a stale post-trip value (~τ = 400 ms to decay) would re-trip
    // TOTAL_OVERCURRENT on the very next sense tick after RESET clears the
    // flag — even though the actual rail current already dropped to 0 mA
    // once the trip disabled all servos. Resetting `sense_warmup_samples`
    // additionally suppresses trip checks for the next 8 samples (400 ms),
    // giving the IIR plenty of time to re-seed with the fresh ADC reading
    // and the user time to release a stalled test servo.
    rail_current_ma_smooth = 0;
    sense_seeded           = false;
    sense_warmup_samples   = 0;
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

        // Phase 13 FW-Fix: while a pin is disabled, soft-ramp in on_tick
        // is a no-op (we skip pulse() for disabled pins). Sync current to
        // target so the very first PWM emitted after ENABLE_SERVO is
        // already at target, not at the pulse_zero stuck in current from
        // RESET. The actual Pimoroni-state sync (last_enabled_pulse=target)
        // happens in handle_enable_servo below where we call pulse()
        // directly instead of enable() — that avoids any MID-fallback.
        if (!servo_enabled[i]) {
            current_pulse_us[i] = target_pulse_us[i];
        }
    }
    if (any_clamped) {
        send_error(seq, err::PULSE_OUT_OF_RANGE,
                   first_clamped_idx, first_clamped_raw);
    } else {
        send_ack(seq, cmd::SET_TARGETS);
    }
}

void handle_set_current_limit(uint8_t seq, const uint8_t* p, uint8_t len) {
    if (len != 2) {
        send_error(seq, err::PAYLOAD_LEN, 0, 2);
        return;
    }
    uint16_t limit_ma = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    runtime_current_max_ma = limit_ma;
    send_ack(seq, cmd::SET_CURRENT_LIMIT);
}

void handle_enable_servo(uint8_t seq, const uint8_t* p, uint8_t len) {
    if (len != 2) {
        send_error(seq, err::PAYLOAD_LEN, 0, 2);
        return;
    }
    // PROTOCOL.md §6: while any latched trip is set, ENABLE_SERVO is refused —
    // host must send RESET first (and fix the underlying cause).
    if (status_flags & status::WATCHDOG_TRIPPED) {
        send_nack(seq, cmd::ENABLE_SERVO, err::WATCHDOG_TRIPPED);
        return;
    }
    if (status_flags & status::TOTAL_OVERCURRENT_TRIPPED) {
        send_nack(seq, cmd::ENABLE_SERVO, err::TOTAL_OVERCURRENT);
        return;
    }
    if (status_flags & status::UNDERVOLTAGE_TRIPPED) {
        send_nack(seq, cmd::ENABLE_SERVO, err::UNDERVOLTAGE);
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
        if (en) {
            // Phase 13 FW-Fix v2 (double-pulse workaround):
            //
            // Empirically observed (2026-05-28, diagnose_single_pin_pwm.py):
            // the FIRST g_servos->pulse() call for a freshly-RESET pin
            // lands at PWM ≈ 1500 µs (MID), regardless of what value we
            // pass in. Subsequent pulse() calls hit the correct target.
            // Root cause not pinpointed in Pimoroni source — likely a
            // PIO-DMA initialization side-effect or a Pimoroni-internal
            // "first-write defaults to MID" behaviour we don't see in
            // the source path.
            //
            // Workaround: pulse() twice, with a 20 ms sleep (= one PWM
            // period at 50 Hz) in between. The first call gets eaten by
            // whatever causes the MID-glitch; the second call hits the
            // actual target. Cost: 18 × 20 ms = 360 ms added to on_activate
            // (total ~1.3 s) — acceptable for one-time boot.
            //
            // Cleaner long-term: instrument the PWM pin with an
            // oscilloscope to identify what really happens on the first
            // write, then either patch Pimoroni or restructure FW init.
            g_servos->pulse(idx,
                            static_cast<float>(current_pulse_us[idx]),
                            /*load=*/true);
            sleep_ms(20);
            g_servos->pulse(idx,
                            static_cast<float>(current_pulse_us[idx]),
                            /*load=*/true);
        } else {
            g_servos->disable(idx, true);
        }
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
        case cmd::SET_CURRENT_LIMIT:
            handle_set_current_limit(f.seq, f.payload, f.len);
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
    //
    // Phase 13 FW-Fix: only emit PWM for enabled pins. Pimoroni's
    // ServoCluster::pulse(idx, val) calls ServoState::set_pulse_with_return
    // which auto-enables the pin (sets enabled=true) on any val >=
    // MIN_VALID_PULSE — see pimoroni-pico/drivers/servo/servo_state.cpp.
    // Without this servo_enabled[] check we would re-enable pins on every
    // tick, ignoring handle_enable_servo(false) / watchdog-trip / under-
    // voltage-trip / handle_reset's intent to keep PWM off.
    // Soft-ramp still runs on current_pulse_us for disabled pins so that a
    // later ENABLE_SERVO finds a sane state — but no PWM is emitted until
    // the pin is explicitly re-enabled.
    constexpr int16_t step = cfg::MAX_DELTA_PULSE_PER_TICK_US;
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        int16_t delta = static_cast<int16_t>(target_pulse_us[i] - current_pulse_us[i]);
        if      (delta >  step) current_pulse_us[i] += step;
        else if (delta < -step) current_pulse_us[i] -= step;
        else                    current_pulse_us[i]  = target_pulse_us[i];
        if (servo_enabled[i]) {
            g_servos->pulse(static_cast<uint8_t>(i),
                            static_cast<float>(current_pulse_us[i]),
                            /*load=*/false);
        }
    }
    g_servos->load();

    // -------------------------------------------------------------------------
    // E.1/E.2 — Current + voltage sensing (every SENSE_SAMPLE_EVERY_TICKS ticks)
    // -------------------------------------------------------------------------
    if (!g_mux || !g_cur_sense || !g_vol_sense) return;
    if (++sense_tick_counter < cfg::SENSE_SAMPLE_EVERY_TICKS) return;
    sense_tick_counter = 0;

    // ---- Current ----
    g_mux->select(servo::servo2040::CURRENT_SENSE_ADDR);
    float current_a = g_cur_sense->read_current();
    if (current_a < 0.0f) current_a = 0.0f;
    uint16_t current_ma_sample = static_cast<uint16_t>(
        std::min(current_a * 1000.0f, 65535.0f));

    if (!sense_seeded) {
        rail_current_ma_smooth = current_ma_sample;
        sense_seeded = true;
    } else {
        // IIR: smooth = (smooth * 7 + sample) / 8   → α = 1/8
        rail_current_ma_smooth =
            (rail_current_ma_smooth * 7u + current_ma_sample) >> 3;
    }
    // Store total rail current in slot 0; slots 1..17 stay zero (no per-servo HW).
    last_current_ma[0] = static_cast<uint16_t>(rail_current_ma_smooth);

    // ---- Voltage ----
    g_mux->select(servo::servo2040::VOLTAGE_SENSE_ADDR);
    float voltage_v = g_vol_sense->read_voltage();
    if (voltage_v < 0.0f) voltage_v = 0.0f;
    rail_voltage_mv = static_cast<uint16_t>(
        std::min(voltage_v * 1000.0f, 65535.0f));

    // ---- Trip logic — gated until the filter has settled (~8 samples = 400 ms) ----
    if (sense_warmup_samples < 8) {
        ++sense_warmup_samples;
        return;
    }

    // E.1 — total current trip (latched, RESET to clear).
    if (!(status_flags & status::TOTAL_OVERCURRENT_TRIPPED) &&
        rail_current_ma_smooth > runtime_current_max_ma) {
        for (uint i = 0; i < NUM_SERVOS; ++i) {
            g_servos->disable(static_cast<uint8_t>(i), /*load=*/false);
            servo_enabled[i] = false;
        }
        g_servos->load();
        status_flags |= status::TOTAL_OVERCURRENT_TRIPPED | status::ANY_SERVO_DISABLED;
        send_error(0, err::TOTAL_OVERCURRENT, 0,
                   static_cast<int16_t>(rail_current_ma_smooth & 0x7FFF));
    }

    // E.2 — undervoltage warn (auto-clearing) + critical trip (latched).
    if (rail_voltage_mv < cfg::UNDERVOLTAGE_CRIT_MV &&
        !(status_flags & status::UNDERVOLTAGE_TRIPPED)) {
        for (uint i = 0; i < NUM_SERVOS; ++i) {
            g_servos->disable(static_cast<uint8_t>(i), /*load=*/false);
            servo_enabled[i] = false;
        }
        g_servos->load();
        status_flags |= status::UNDERVOLTAGE_TRIPPED | status::ANY_SERVO_DISABLED;
        send_error(0, err::UNDERVOLTAGE, 0,
                   static_cast<int16_t>(rail_voltage_mv & 0x7FFF));
    } else if (rail_voltage_mv < cfg::UNDERVOLTAGE_WARN_MV) {
        if (!(status_flags & status::UNDERVOLTAGE_WARNING)) {
            status_flags |= status::UNDERVOLTAGE_WARNING;
            // Send a one-shot warn so the host knows immediately, even if it
            // isn't polling GET_STATE.  servo_idx = 0xFF marks "warn, not trip".
            send_error(0, err::UNDERVOLTAGE, 0xFF,
                       static_cast<int16_t>(rail_voltage_mv & 0x7FFF));
        }
    } else {
        // Voltage healthy again — auto-clear the warning. (The latched
        // UNDERVOLTAGE_TRIPPED bit still needs an explicit RESET.)
        status_flags &= ~status::UNDERVOLTAGE_WARNING;
    }
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

    // Init ADC mux + analog sensing (Stage E).
    // mux drives ADDR_0/1/2; CURRENT_SENSE_ADDR=0b111, VOLTAGE_SENSE_ADDR=0b110.
    // Both Analog instances read SHARED_ADC (GPIO29) — the mux picks which signal
    // sits on that pin at any given time.
    static pimoroni::AnalogMux mux(servo::servo2040::ADC_ADDR_0,
                                   servo::servo2040::ADC_ADDR_1,
                                   servo::servo2040::ADC_ADDR_2,
                                   PIN_UNUSED,
                                   servo::servo2040::SHARED_ADC);
    static pimoroni::Analog cur_sense(servo::servo2040::SHARED_ADC,
                                      servo::servo2040::CURRENT_GAIN,
                                      servo::servo2040::SHUNT_RESISTOR,
                                      servo::servo2040::CURRENT_OFFSET);
    static pimoroni::Analog vol_sense(servo::servo2040::SHARED_ADC,
                                      servo::servo2040::VOLTAGE_GAIN);
    g_mux       = &mux;
    g_cur_sense = &cur_sense;
    g_vol_sense = &vol_sense;

    // Banner — keep "Servo2040 USB-UART Communication Started" prefix
    // so tools/flash_and_verify.py keeps matching.
    printf("Servo2040 USB-UART Communication Started (vE total-current + undervoltage)\n");

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
