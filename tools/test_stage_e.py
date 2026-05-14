#!/usr/bin/env python3
"""Stage-E tests for the Servo2040 firmware.

E.1  Total rail current monitoring + overcurrent trip
E.2  Undervoltage warning (auto-clearing) + critical trip

Hardware required:
  - 2× MG996R on outputs 0 and 1
  - Bench-PSU: 6.0 V / 3.0 A connected to servo rail

E.1 and E.2 tests require manual interaction (stalling a servo / turning down
the PSU).  Follow the on-screen prompts.

Usage:
    python3 tools/test_stage_e.py [/dev/ttyACMx]

Run AFTER tools/flash_and_verify.py.  Exit 0 on PASS, non-zero on first FAIL.
"""
from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Reuse proto helpers from test_servo2040.py without copy-pasting
# ---------------------------------------------------------------------------
_tools_dir = os.path.dirname(os.path.abspath(__file__))
if _tools_dir not in sys.path:
    sys.path.insert(0, _tools_dir)

from test_servo2040 import (  # noqa: E402
    crc16, encode_frame, decode_frame,
    Link, next_seq,
    send_reset, send_get_state, send_set_targets, send_enable,
    get_state, hold_target, expect_frame, parse_state,
    step, ok, info, fail,
    CMD_ACK, CMD_NACK, CMD_ERROR_REPORT,
    CMD_RESET, CMD_ENABLE_SERVO, CMD_SET_TARGETS,
    DEFAULT_PULSE_ZERO, NUM_SERVOS, DEFAULT_TTY,
    STATUS_ANY_SERVO_DISABLED,
)

# SET_CURRENT_LIMIT opcode (introduced for Stage E so tests can lower the
# trip threshold to a hand-stallable value without re-flashing firmware).
CMD_SET_CURRENT_LIMIT = 0x11

# Lowered threshold used during the overcurrent trip test — the user can
# reach ~800 mA stall current by hand, so 600 mA gives ~200 mA headroom over
# idle (~200–400 mA when the two MG996R are enabled at neutral) without
# false-tripping.
TEST_CURRENT_LIMIT_MA = 600

# Production threshold restored after the test finishes — keep in sync with
# cfg::TOTAL_CURRENT_MAX_MA in src/config.hpp.
PROD_CURRENT_LIMIT_MA = 3500

# ---------------------------------------------------------------------------
# Stage-E specific constants (must match src/config.hpp + src/config.hpp)
# ---------------------------------------------------------------------------
ERR_TOTAL_OVERCURRENT = 0x21
ERR_UNDERVOLTAGE      = 0x30

STATUS_UNDERVOLTAGE_TRIPPED      = 1 << 1
STATUS_TOTAL_OVERCURRENT_TRIPPED = 1 << 2
STATUS_UNDERVOLTAGE_WARNING      = 1 << 5

TOTAL_CURRENT_MAX_MA = 3500   # cfg::TOTAL_CURRENT_MAX_MA
UNDERVOLTAGE_WARN_MV = 5500   # cfg::UNDERVOLTAGE_WARN_MV
UNDERVOLTAGE_CRIT_MV = 5000   # cfg::UNDERVOLTAGE_CRIT_MV

# IIR warmup: 8 samples × 50 ms = 400 ms before trip logic activates.
SENSE_WARMUP_S = 0.5

N_TEST_SERVOS = 2   # outputs 0 and 1


# ---------------------------------------------------------------------------
# Keep-alive helper (feeds watchdog while waiting for Enter)
# ---------------------------------------------------------------------------
def _wait_enter(link: Link, prompt: str):
    """Print prompt and wait for Enter, feeding SET_TARGETS every 100 ms."""
    print(f"     >>> {prompt}", flush=True)
    print("         Press Enter when ready…", end="", flush=True)
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.10)
        if r:
            sys.stdin.readline()
            break
        send_set_targets(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS)
    link.drain(0.05)


def send_set_current_limit(link: Link, limit_ma: int) -> int:
    """Send SET_CURRENT_LIMIT and wait for ACK. Returns the seq used."""
    seq = next_seq()
    payload = struct.pack("<H", limit_ma)
    link.write(encode_frame(seq, CMD_SET_CURRENT_LIMIT, payload))
    f = expect_frame(
        link,
        lambda s, c, p, _s=seq: (s == _s and c == CMD_ACK
                                 and len(p) >= 1 and p[0] == CMD_SET_CURRENT_LIMIT),
        timeout_s=1.0,
    )
    if f is None:
        fail(f"no ACK for SET_CURRENT_LIMIT({limit_ma} mA) — old firmware without 0x11?")
    return seq


def _enable_test_servos(link: Link):
    """RESET then enable servos 0 and 1 at neutral."""
    send_reset(link)
    time.sleep(0.05)
    link.drain(0.1)
    hold_target(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS, duration_s=0.3)
    for i in range(N_TEST_SERVOS):
        seq = send_enable(link, i, True)
        f = expect_frame(link,
                         lambda s, c, p, _s=seq: s == _s and c == CMD_ACK,
                         timeout_s=1.0)
        if f is None:
            fail(f"no ACK for ENABLE_SERVO({i})")


# ---------------------------------------------------------------------------
# Test E.0 — Sensing sanity (automated, no stall required)
# ---------------------------------------------------------------------------
def test_sensing_sanity(link: Link):
    step("E.0 sensing sanity: verify ADC readings are plausible (no servos loaded)")

    send_reset(link)
    time.sleep(0.05)
    link.drain(0.1)

    # Wait for IIR filter to warm up (8 samples × 50 ms = ~400 ms).
    info(f"warming up ADC filter ({SENSE_WARMUP_S:.1f} s)…")
    hold_target(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS, duration_s=SENSE_WARMUP_S)

    _, currents, voltage, flags = get_state(link)
    rail_current = currents[0]

    if not (3000 <= voltage <= 9000):
        fail(f"rail voltage out of expected range: {voltage} mV "
             f"(expect 3000–9000 mV for a 6 V PSU)")
    ok(f"rail voltage = {voltage} mV  ({voltage / 1000:.2f} V)")

    if flags & STATUS_TOTAL_OVERCURRENT_TRIPPED:
        fail(f"TOTAL_OVERCURRENT_TRIPPED already set at idle! flags=0x{flags:02X}")
    if flags & STATUS_UNDERVOLTAGE_TRIPPED:
        fail(f"UNDERVOLTAGE_TRIPPED already set! flags=0x{flags:02X}")
    ok(f"status flags = 0x{flags:02X}  (no spurious trips)")

    info(f"rail current at idle (servos disabled) = {rail_current} mA")

    # Enable both servos, settle, verify current > idle and < trip threshold.
    _enable_test_servos(link)
    hold_target(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS, duration_s=1.0)

    _, currents, voltage, flags = get_state(link)
    rail_current = currents[0]
    ok(f"rail current with {N_TEST_SERVOS} servos at neutral = {rail_current} mA")
    ok(f"rail voltage = {voltage} mV")

    if rail_current >= TOTAL_CURRENT_MAX_MA:
        fail(f"current already at/above trip threshold at neutral! "
             f"{rail_current} mA ≥ {TOTAL_CURRENT_MAX_MA} mA")

    send_reset(link)
    time.sleep(0.05)


# ---------------------------------------------------------------------------
# Test E.1 — Total overcurrent trip (manual: stall servo 0)
# ---------------------------------------------------------------------------
def test_overcurrent_trip(link: Link):
    step(f"E.1 overcurrent trip: stall servo 0 to exceed a TEST threshold "
         f"({TEST_CURRENT_LIMIT_MA} mA, hand-reachable)")
    info(f"production threshold ({PROD_CURRENT_LIMIT_MA} mA) is too high to "
         f"reach by hand — using SET_CURRENT_LIMIT to lower temporarily")

    try:
        # Lower the trip threshold so a single manual stall (~800 mA total) triggers.
        send_set_current_limit(link, TEST_CURRENT_LIMIT_MA)
        ok(f"trip threshold lowered to {TEST_CURRENT_LIMIT_MA} mA for this test")

        _enable_test_servos(link)
        hold_target(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS, duration_s=0.5)
        _, currents, _, _ = get_state(link)
        info(f"baseline current = {currents[0]} mA (servos at neutral)")
        if currents[0] >= TEST_CURRENT_LIMIT_MA - 100:
            fail(f"baseline {currents[0]} mA already too close to test threshold "
                 f"{TEST_CURRENT_LIMIT_MA} mA — raise TEST_CURRENT_LIMIT_MA or "
                 f"check why idle current is so high")

        _wait_enter(link,
                    f"GRIP servo 0's output shaft and stall it firmly.  "
                    f"Trip threshold is now {TEST_CURRENT_LIMIT_MA} mA, "
                    f"a sustained ~800 mA pull should trigger.  "
                    f"Hold until the test reports [OK].")

        # Feed watchdog while waiting for the unsolicited ERROR_REPORT (up to 15 s).
        info("waiting for ERROR_REPORT/TOTAL_OVERCURRENT (up to 15 s)…")
        deadline = time.monotonic() + 15.0
        trip_frame = None
        while time.monotonic() < deadline:
            send_set_targets(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS)
            frames = link.read_frames(time.monotonic() + 0.10)
            for s, c, p in frames:
                if c == CMD_ERROR_REPORT and len(p) >= 1 and p[0] == ERR_TOTAL_OVERCURRENT:
                    trip_frame = (s, c, p)
                    break
            if trip_frame:
                break

        if trip_frame is None:
            fail("no ERROR_REPORT/TOTAL_OVERCURRENT received within 15 s — "
                 "was the servo stalled hard enough?")

        _, _, p = trip_frame
        measured_ma = struct.unpack("<h", bytes(p[2:4]))[0] if len(p) >= 4 else 0
        ok(f"ERROR_REPORT/TOTAL_OVERCURRENT received  (measured ≈ {measured_ma} mA)")

        _, currents, _, flags = get_state(link)
        if not (flags & STATUS_TOTAL_OVERCURRENT_TRIPPED):
            fail(f"TOTAL_OVERCURRENT_TRIPPED not set after trip! flags=0x{flags:02X}")
        ok(f"status flags = 0x{flags:02X}  (TOTAL_OVERCURRENT_TRIPPED + ANY_SERVO_DISABLED)")

        # RESET must clear the trip (note: RESET does NOT reset the runtime limit).
        # During the post-RESET diagnostic we bump the threshold to 5000 mA so
        # the trip CANNOT re-fire regardless of what the ADC reports. That way
        # we can observe the actual rail-current trajectory over ~1.5 s and
        # cross-check against the PSU ammeter (ground truth).
        send_reset(link)
        send_set_current_limit(link, 5000)
        info("waiting 1500 ms post-RESET with threshold raised to 5000 mA")
        info(">>> RELEASE the servo AND WATCH THE PSU AMMETER during the next 1.5 s")
        info("    if PSU shows ~0 mA but firmware reports ~800 mA → measurement bug")
        info("    if PSU shows ~800 mA → real current (servo internal MCU is fighting?)")
        for i in range(15):
            send_set_targets(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS)
            time.sleep(0.05)
            _, currents, voltage, flags = get_state(link)
            tripped = "TRIP" if (flags & STATUS_TOTAL_OVERCURRENT_TRIPPED) else "ok  "
            info(f"  t={i*100:4d} ms: current={currents[0]:5d} mA, "
                 f"voltage={voltage:5d} mV, flags=0x{flags:02X} [{tripped}]")
            time.sleep(0.05)
        link.drain(0.05)

        _, currents, _, flags = get_state(link)
        if flags & STATUS_TOTAL_OVERCURRENT_TRIPPED:
            fail(f"TOTAL_OVERCURRENT_TRIPPED still set even with 5000 mA threshold! "
                 f"flags=0x{flags:02X}, current={currents[0]} mA — "
                 f"unexpected, the firmware shouldn't trip at 5000 mA")
        ok(f"after RESET: flags = 0x{flags:02X}, current = {currents[0]} mA  (trip cleared)")

    finally:
        # ALWAYS restore the production threshold, even if the test failed
        # halfway — otherwise a subsequent test run (or normal operation) would
        # false-trip on light load.
        send_set_current_limit(link, PROD_CURRENT_LIMIT_MA)
        ok(f"trip threshold restored to production value ({PROD_CURRENT_LIMIT_MA} mA)")


# ---------------------------------------------------------------------------
# Test E.2 — Undervoltage warning + critical trip (manual: lower PSU)
# ---------------------------------------------------------------------------
def test_undervoltage(link: Link):
    step("E.2 undervoltage: lower PSU to trigger warn then critical trip")

    _enable_test_servos(link)
    hold_target(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS, duration_s=0.5)
    _, _, voltage, _ = get_state(link)
    info(f"baseline rail voltage = {voltage} mV  ({voltage / 1000:.2f} V)")

    # ---- Warning phase ----
    _wait_enter(link,
                f"Slowly lower the PSU voltage from 6.0 V toward "
                f"{UNDERVOLTAGE_WARN_MV / 1000:.1f} V  "
                f"(warn threshold).  Stop as soon as you see [OK].")

    info(f"waiting for UNDERVOLTAGE warning (servo_idx = 0xFF, up to 20 s)…")
    deadline = time.monotonic() + 20.0
    warn_frame = None
    while time.monotonic() < deadline:
        send_set_targets(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS)
        for s, c, p in link.read_frames(time.monotonic() + 0.10):
            if (c == CMD_ERROR_REPORT and len(p) >= 2
                    and p[0] == ERR_UNDERVOLTAGE and p[1] == 0xFF):
                warn_frame = (s, c, p)
                break
        if warn_frame:
            break

    if warn_frame is None:
        fail(f"no UNDERVOLTAGE warning received within 20 s — "
             f"was the voltage actually lowered below {UNDERVOLTAGE_WARN_MV} mV?")

    _, _, p = warn_frame
    mv = struct.unpack("<h", bytes(p[2:4]))[0] if len(p) >= 4 else 0
    ok(f"UNDERVOLTAGE warning received  ({mv} mV)")

    _, _, voltage, flags = get_state(link)
    if not (flags & STATUS_UNDERVOLTAGE_WARNING):
        fail(f"STATUS_UNDERVOLTAGE_WARNING not set! flags=0x{flags:02X}")
    ok(f"status flags = 0x{flags:02X}  (UNDERVOLTAGE_WARNING set, no trip yet)")

    # ---- Critical trip phase ----
    _wait_enter(link,
                f"Continue lowering the PSU to below "
                f"{UNDERVOLTAGE_CRIT_MV / 1000:.1f} V  "
                f"(critical threshold).  Stop as soon as you see [OK].")

    info(f"waiting for UNDERVOLTAGE critical trip (servo_idx = 0x00, up to 20 s)…")
    deadline = time.monotonic() + 20.0
    crit_frame = None
    while time.monotonic() < deadline:
        # Watchdog keep-alive not needed here if trip already fired, but send anyway.
        send_set_targets(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS)
        for s, c, p in link.read_frames(time.monotonic() + 0.10):
            if (c == CMD_ERROR_REPORT and len(p) >= 2
                    and p[0] == ERR_UNDERVOLTAGE and p[1] == 0x00):
                crit_frame = (s, c, p)
                break
        if crit_frame:
            break

    if crit_frame is None:
        fail(f"no UNDERVOLTAGE critical trip received within 20 s — "
             f"was the voltage lowered below {UNDERVOLTAGE_CRIT_MV} mV?")

    _, _, p = crit_frame
    mv = struct.unpack("<h", bytes(p[2:4]))[0] if len(p) >= 4 else 0
    ok(f"UNDERVOLTAGE critical trip received  ({mv} mV)")

    _, _, _, flags = get_state(link)
    if not (flags & STATUS_UNDERVOLTAGE_TRIPPED):
        fail(f"STATUS_UNDERVOLTAGE_TRIPPED not set after trip! flags=0x{flags:02X}")
    ok(f"status flags = 0x{flags:02X}  (UNDERVOLTAGE_TRIPPED + ANY_SERVO_DISABLED)")

    # ---- Recovery ----
    _wait_enter(link, "Turn PSU back up to 6.0 V, then press Enter.")

    send_reset(link)
    time.sleep(0.2)
    link.drain(0.2)
    _, _, voltage, flags = get_state(link)
    if flags & STATUS_UNDERVOLTAGE_TRIPPED:
        fail(f"UNDERVOLTAGE_TRIPPED still set after RESET! flags=0x{flags:02X}")
    ok(f"after RESET: flags = 0x{flags:02X}  voltage = {voltage} mV  (trip cleared)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Servo2040 stage-E tests (current/voltage sensing).")
    parser.add_argument("tty", nargs="?", default=DEFAULT_TTY,
                        help=f"serial device (default {DEFAULT_TTY})")
    args = parser.parse_args()

    print(f"=== Servo2040 stage-E tests on {args.tty} ===")
    print( "    PSU: 6.0 V / 3.0 A   |   2× MG996R on outputs 0 and 1\n")

    try:
        link = Link(args.tty)
    except OSError as e:
        print(f"[FAIL] cannot open {args.tty}: {e}")
        sys.exit(1)

    t0 = time.monotonic()
    try:
        test_sensing_sanity(link)
        print()
        test_overcurrent_trip(link)
        print()
        test_undervoltage(link)
    except (AssertionError, TimeoutError) as e:
        elapsed = time.monotonic() - t0
        print(f"\n=== X STAGE-E TESTS FAILED after {elapsed:.1f}s: {e} ===")
        sys.exit(1)
    finally:
        try:
            send_reset(link)
            time.sleep(0.05)
        finally:
            link.close()

    elapsed = time.monotonic() - t0
    print(f"\n=== OK ALL STAGE-E TESTS PASSED in {elapsed:.1f}s ===")


if __name__ == "__main__":
    main()
