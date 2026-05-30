#!/usr/bin/env python3
"""Phase 13 FW-fix diagnose — directly drive FW without the ROS2 plugin.

Sequence (mimicking what the hexapod_hardware plugin does in on_activate):
    1. RESET
    2. SET_TARGETS with per-pin "suspended" PWMs (hardcoded from
       servo_mapping.yaml + Plugin's radians_to_pulse_us for rad=+1.45
       femur / 0 coxa / 0 tibia)
    3. 18× ENABLE_SERVO with 50 ms stagger
    4. GET_STATE — print current_pulse_us for all 18 pins

If FW-fix is working: physically the legs go straight down. GET_STATE
shows current_pulse_us == sent target.

If FW-fix is NOT working: legs go to horizontal (MID-fallback). GET_STATE
will reveal what current_pulse_us actually is.

Usage:
    python3 tools/diagnose_phase13_fix.py [/dev/ttyACMx]

Requires: firmware up (run flash_and_verify.py first), plugin/ROS2 NOT running.
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from test_servo2040 import (
    Link, NUM_SERVOS, DEFAULT_TTY,
    send_reset, send_set_targets, send_enable, get_state,
)

# Hardcoded suspended-PWM table per pin (servo-output index 0..17).
# Derived from src/hexapod_hardware/config/servo_mapping.yaml +
# Plugin's radians_to_pulse_us for the Phase 13 Stage A suspended preset:
#   coxa  = 0.0    rad  →  pulse_zero per pin
#   femur = +1.45  rad  →  near pulse_max for right legs (dir=+1) or
#                          near pulse_min for left legs (dir=-1)
#   tibia = 0.0    rad  →  pulse_zero per pin
#
# Layout: 3 joints per leg, sequential.
#   pin 0..2:  leg 1 (front-right)
#   pin 3..5:  leg 2 (mid-right)
#   pin 6..8:  leg 3 (back-right)
#   pin 9..11: leg 4 (back-left)
#   pin 12..14: leg 5 (mid-left)
#   pin 15..17: leg 6 (front-left)
SUSPENDED_PWMS = [
    # leg 1 — right (femur dir=+1)
    1460,   # pin 0 — leg_1_coxa  (rad=0   → pulse_zero=1460)
    2101,   # pin 1 — leg_1_femur (rad=1.45 → near pulse_max=2120)
    1680,   # pin 2 — leg_1_tibia (rad=0   → pulse_zero=1680)
    # leg 2 — right
    1575,   # pin 3 — leg_2_coxa
    2172,   # pin 4 — leg_2_femur (rad=1.45, dir=+1, pulse_zero=1550, pulse_max=2190)
    1680,   # pin 5 — leg_2_tibia
    # leg 3 — right
    1410,   # pin 6 — leg_3_coxa
    2081,   # pin 7 — leg_3_femur
    1620,   # pin 8 — leg_3_tibia
    # leg 4 — left (femur dir=-1)
    1520,   # pin 9  — leg_4_coxa
    890,    # pin 10 — leg_4_femur (rad=1.45, dir=-1, pulse_zero=1560, pulse_min=870)
    1320,   # pin 11 — leg_4_tibia
    # leg 5 — left
    1550,   # pin 12 — leg_5_coxa
    879,    # pin 13 — leg_5_femur
    1390,   # pin 14 — leg_5_tibia
    # leg 6 — left
    1530,   # pin 15 — leg_6_coxa
    860,    # pin 16 — leg_6_femur (rad=1.45, dir=-1, pulse_zero=1540, pulse_min=840)
    1340,   # pin 17 — leg_6_tibia
]

# Pin names for human-readable output.
PIN_NAMES = [
    "leg_1_coxa", "leg_1_femur", "leg_1_tibia",
    "leg_2_coxa", "leg_2_femur", "leg_2_tibia",
    "leg_3_coxa", "leg_3_femur", "leg_3_tibia",
    "leg_4_coxa", "leg_4_femur", "leg_4_tibia",
    "leg_5_coxa", "leg_5_femur", "leg_5_tibia",
    "leg_6_coxa", "leg_6_femur", "leg_6_tibia",
]


def main() -> int:
    tty = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TTY

    assert len(SUSPENDED_PWMS) == NUM_SERVOS, f"PWM table size mismatch ({len(SUSPENDED_PWMS)})"

    print(f"[..] Opening {tty}")
    link = Link(tty)

    # === Step 0: get state BEFORE any commands ===
    print("\n=== STEP 0: GET_STATE (pre-test, FW should be in fresh-boot state) ===")
    try:
        pulses_pre, currents_pre, voltage_pre, flags_pre = get_state(link)
        print(f"    voltage={voltage_pre} mV   flags=0x{flags_pre:02X}")
        print(f"    current_pulse_us per pin (pin: PWM):")
        for pin in range(NUM_SERVOS):
            print(f"      pin {pin:2d}  {PIN_NAMES[pin]:14s}  {pulses_pre[pin]}")
    except Exception as e:
        print(f"    [FAIL] GET_STATE: {e}")

    # === Step 1: RESET ===
    print("\n=== STEP 1: RESET ===")
    send_reset(link)
    time.sleep(0.05)
    link.drain(0.1)
    print("    OK — FW state should be: target=current=pulse_zero=1500, servo_enabled=false")

    # === Step 2: SET_TARGETS with suspended PWMs ===
    print("\n=== STEP 2: SET_TARGETS (suspended PWMs) ===")
    print(f"    Sending: {SUSPENDED_PWMS}")
    send_set_targets(link, SUSPENDED_PWMS)
    time.sleep(0.05)
    link.drain(0.1)

    # GET_STATE post-SET_TARGETS — verifies FW Fix 3.2 (current=target sync for disabled)
    print("\n    GET_STATE after SET_TARGETS (pre-enable):")
    try:
        pulses_set, _, voltage, flags = get_state(link)
        print(f"      voltage={voltage} mV   flags=0x{flags:02X}")
        all_match = True
        for pin in range(NUM_SERVOS):
            expected = SUSPENDED_PWMS[pin]
            got = pulses_set[pin]
            mark = "OK" if got == expected else "!!"
            if got != expected:
                all_match = False
            print(f"      {mark} pin {pin:2d}  {PIN_NAMES[pin]:14s}  expected={expected}  got={got}")
        if all_match:
            print("    OK — all 18 pins have current_pulse_us == suspended (Fix 3.2 working)")
        else:
            print("    !! current_pulse_us != target for some pins — Fix 3.2 may not be working")
    except Exception as e:
        print(f"    [FAIL] GET_STATE: {e}")

    # === Step 3: ENABLE_SERVO × 18 with 50 ms stagger ===
    print("\n=== STEP 3: ENABLE_SERVO × 18 with 50 ms stagger ===")
    print("    Watch the legs! If FW Fix 3.4 is working, each leg should activate")
    print("    pointing DOWN (femur → boden) — not horizontal.")
    for pin in range(NUM_SERVOS):
        send_enable(link, pin, True)
        time.sleep(0.05)
    link.drain(0.2)
    print("    All 18 servos enabled.")

    # === Step 4: GET_STATE after enables ===
    print("\n=== STEP 4: GET_STATE (post-enable) ===")
    try:
        pulses_post, currents_post, voltage_post, flags_post = get_state(link)
        print(f"    voltage={voltage_post} mV   current_total={currents_post[0]} mA   flags=0x{flags_post:02X}")
        all_match = True
        for pin in range(NUM_SERVOS):
            expected = SUSPENDED_PWMS[pin]
            got = pulses_post[pin]
            mark = "OK" if got == expected else "!!"
            if got != expected:
                all_match = False
            print(f"      {mark} pin {pin:2d}  {PIN_NAMES[pin]:14s}  expected={expected}  got={got}")
        print()
        if all_match:
            print("    ✓ FW state correct — current_pulse_us == suspended for all 18 pins.")
            print("    If servos are NOT physically at suspended, the bug is in Pimoroni or PWM hardware.")
        else:
            print("    ✗ FW state mismatch — some pins have current_pulse_us != suspended.")
            print("    Bug is in FW handle_set_targets / handle_enable_servo / on_tick.")
    except Exception as e:
        print(f"    [FAIL] GET_STATE: {e}")

    # === Step 5: wait + final GET_STATE ===
    print("\n=== STEP 5: wait 2 s then re-poll (verify steady-state) ===")
    time.sleep(2.0)
    link.drain(0.5)
    try:
        pulses_final, _, voltage_final, flags_final = get_state(link)
        print(f"    voltage={voltage_final} mV   flags=0x{flags_final:02X}")
        for pin in (1, 4, 7, 10, 13, 16):  # only femurs (most diagnostic)
            print(f"      pin {pin:2d}  {PIN_NAMES[pin]:14s}  current_pulse_us={pulses_final[pin]}")
    except Exception as e:
        print(f"    [FAIL] GET_STATE: {e}")

    # === Step 6: cleanup — RESET so the user is back in a known state ===
    print("\n=== STEP 6: cleanup RESET ===")
    send_reset(link)
    time.sleep(0.1)
    link.drain(0.1)
    print("    OK — FW back to disabled state.")
    print()
    print("Done. Plug in plugin again with: ros2 launch hexapod_bringup real.launch.py")

    return 0


if __name__ == "__main__":
    sys.exit(main())
