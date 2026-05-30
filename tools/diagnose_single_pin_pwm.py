#!/usr/bin/env python3
"""Phase 13 deepest diagnose — test if a single Servo2040 pin actually
honors different PWM values.

Workflow per test step:
    1. RESET
    2. SET_TARGETS with the test PWM for the target pin, pulse_zero for all
       others (so the rest of the robot stays in a known mid pose)
    3. ENABLE_SERVO for the target pin only
    4. Hold 3 s with continuous SET_TARGETS
    5. User confirms what they see physically

Test matrix per pin: pulse_min, pulse_zero, pulse_max  → three different
mechanical positions according to the user's cal table. If the servo always
goes to the same place regardless of PWM, the issue is in the Pimoroni-PIO
→ servo wire path (or the servo itself doesn't accept these widths).

Usage:
    python3 tools/diagnose_single_pin_pwm.py [pin] [/dev/ttyACMx]

    pin defaults to 1 (= leg_1_femur). Other useful pins:
        1  leg_1_femur (right-side, dir=+1, pulse_min=815, pulse_zero=1460, pulse_max=2120)
        4  leg_2_femur (right-side, dir=+1, pulse_min=880, pulse_zero=1550, pulse_max=2190)
        10 leg_4_femur (left-side,  dir=-1, pulse_min=870, pulse_zero=1560, pulse_max=2190)

Requires:
    - firmware up
    - plugin/ROS2 NOT running
    - PSU on
    - hexapod aufgebockt
"""
from __future__ import annotations

import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from test_servo2040 import (
    Link, NUM_SERVOS, DEFAULT_TTY,
    send_reset, send_set_targets, send_enable, get_state,
)
from diagnose_phase13_fix import PIN_NAMES, SUSPENDED_PWMS

# Per-pin pulse table from user's servo_mapping.yaml cal.
PIN_CAL = {
    0:  (1145, 1460, 1700),   # leg_1_coxa
    1:  ( 815, 1460, 2120),   # leg_1_femur   (right, dir=+1)
    2:  ( 870, 1680, 2185),   # leg_1_tibia
    3:  (1375, 1575, 1750),   # leg_2_coxa
    4:  ( 880, 1550, 2190),   # leg_2_femur   (right, dir=+1)
    5:  ( 860, 1680, 2200),   # leg_2_tibia
    6:  (1200, 1410, 1745),   # leg_3_coxa
    7:  ( 800, 1445, 2100),   # leg_3_femur   (right, dir=+1)
    8:  ( 790, 1620, 2120),   # leg_3_tibia
    9:  (1200, 1520, 1700),   # leg_4_coxa
    10: ( 870, 1560, 2190),   # leg_4_femur   (left,  dir=-1)
    11: ( 815, 1320, 2140),   # leg_4_tibia
    12: (1350, 1550, 1750),   # leg_5_coxa
    13: ( 860, 1530, 2190),   # leg_5_femur   (left,  dir=-1)
    14: ( 885, 1390, 2210),   # leg_5_tibia
    15: (1290, 1530, 1870),   # leg_6_coxa
    16: ( 840, 1540, 2170),   # leg_6_femur   (left,  dir=-1)
    17: ( 850, 1340, 2170),   # leg_6_tibia
}


def hold_pose_for_seconds(link: Link, pulses, seconds: float):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        send_set_targets(link, pulses)
        link.drain(0.02)
        time.sleep(0.03)


def input_with_keepalive(link: Link, prompt: str, pulses) -> str:
    print(prompt, end="", flush=True)
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
        if ready:
            return sys.stdin.readline().strip()
        try:
            send_set_targets(link, pulses)
            link.drain(0.02)
        except Exception:
            pass


def test_pwm_value(link: Link, target_pin: int, pwm_value: int, label: str,
                   expected: str) -> str:
    """Send target_pin → pwm_value, all other pins → their pulse_zero,
    enable only target_pin, hold 3s, ask user. Returns user answer."""
    # Build pulse vector: all pulse_zero except target
    pulses = [PIN_CAL[p][1] for p in range(NUM_SERVOS)]
    pulses[target_pin] = pwm_value

    print(f"\n--- {label}: Pin {target_pin} ({PIN_NAMES[target_pin]}) = {pwm_value} µs ---")
    print(f"    Erwartete physische Pose: {expected}")

    # RESET to clean state (clears watchdog/trips)
    send_reset(link)
    time.sleep(0.05)
    link.drain(0.1)

    # Send pulses (pre-enable, syncs current=target via FW Fix 3.2)
    send_set_targets(link, pulses)
    time.sleep(0.05)
    link.drain(0.05)

    # Enable only this single pin
    send_enable(link, target_pin, True)
    time.sleep(0.05)
    link.drain(0.05)

    # Hold 3s with keepalive
    hold_pose_for_seconds(link, pulses, 3.0)

    # Prompt user
    print()
    print(f"    Wo ist Pin {target_pin} ({PIN_NAMES[target_pin]}) physisch?")
    print(f"      Erwartung: {expected}")
    print(f"      m  = Mitte (horizontal / neutral)")
    print(f"      u  = nach unten / richtung boden")
    print(f"      o  = nach oben")
    print(f"      <free text>")
    answer = input_with_keepalive(link, "    Beobachtung: ", pulses)

    # Disable for clean state
    send_enable(link, target_pin, False)
    time.sleep(0.05)
    link.drain(0.05)

    return answer


def main() -> int:
    pin = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    tty = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_TTY

    if pin not in PIN_CAL:
        print(f"Unknown pin {pin}. Valid: 0..17")
        return 1

    pulse_min, pulse_zero, pulse_max = PIN_CAL[pin]
    pin_name = PIN_NAMES[pin]

    print("=" * 70)
    print(f"Phase 13 Single-Pin-PWM-Test: Pin {pin} ({pin_name})")
    print("=" * 70)
    print()
    print(f"Test-Werte fuer Pin {pin}:")
    print(f"    pulse_min  = {pulse_min:4d} µs  (mech-extrem 1)")
    print(f"    pulse_zero = {pulse_zero:4d} µs  (Mitte)")
    print(f"    pulse_max  = {pulse_max:4d} µs  (mech-extrem 2)")
    print()
    print("Pro Test: nur Pin {pin} wird enabled, alle anderen Pins disabled.")
    print("Halt 3 s pro Position, du beobachtest visuell.")
    print()
    input("Bereit? (Enter zum Starten): ")

    print(f"\n[..] Opening {tty}")
    link = Link(tty)

    results = {}

    # Start at an extreme position (NOT center) so the user sees a clear
    # movement from rest. If the servo is passive at start (gravity), a
    # command to center is a small motion; a command to an extreme is a
    # large, unambiguous motion.

    # Test 1: pulse_min (extreme 1)
    results["pulse_min"] = test_pwm_value(
        link, pin, pulse_min, f"TEST 1: pulse_min={pulse_min}",
        "ein mech-Extrem (z.B. femur ganz oben am Koerper)")

    # Test 2: pulse_max (extreme 2 — opposite)
    results["pulse_max"] = test_pwm_value(
        link, pin, pulse_max, f"TEST 2: pulse_max={pulse_max}",
        "das andere mech-Extrem (z.B. femur ganz unten richtung boden)")

    # Test 3: pulse_zero (center — for comparison)
    results["pulse_zero"] = test_pwm_value(
        link, pin, pulse_zero, f"TEST 3: pulse_zero={pulse_zero}",
        "Mitte/neutral (z.B. femur horizontal)")

    # Test 4: back to pulse_min to confirm return
    results["pulse_min_again"] = test_pwm_value(
        link, pin, pulse_min, f"TEST 4: pulse_min={pulse_min} (Re-Check)",
        "wieder das erste mech-Extrem")

    # Cleanup
    print("\n--- Cleanup ---")
    send_reset(link)
    time.sleep(0.1)
    link.drain(0.1)

    # Summary
    print()
    print("=" * 70)
    print(f"Ergebnis fuer Pin {pin} ({pin_name})")
    print("=" * 70)
    print(f"    pulse_min ={pulse_min:4d}: {results['pulse_min']}")
    print(f"    pulse_max ={pulse_max:4d}: {results['pulse_max']}")
    print(f"    pulse_zero={pulse_zero:4d}: {results['pulse_zero']}")
    print(f"    pulse_min ={pulse_min:4d} (re-check): {results['pulse_min_again']}")
    print()
    print("Interpretation:")
    print("  - Wenn pulse_zero/min/max zu DREI VERSCHIEDENEN Positionen fuehren:")
    print("    → FW + PWM-Pfad arbeitet korrekt. Bug liegt im Plugin oder")
    print("      in einer Schicht oberhalb.")
    print("  - Wenn ALLE PWMs zur GLEICHEN Position (z.B. immer Mitte) fuehren:")
    print("    → Hardware-Issue (Servo, PWM-Pin, Versorgung) oder Servo akzeptiert")
    print("      diese Pulsbreiten nicht.")
    print("  - Wenn pulse_min/max zur Mitte, aber pulse_zero zur Mitte fuehrt:")
    print("    → Servo's eigene Limit-Range ist anders als die User-Cal-Werte.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
