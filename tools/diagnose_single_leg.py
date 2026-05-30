#!/usr/bin/env python3
"""Phase 13 single-leg suspended-pose test.

Tests if the suspended pose works for one leg in isolation. If yes,
the bug is "all 18 servos at once" → mechanical/current limit. If no,
it's something else.

Workflow:
    1. RESET
    2. SET_TARGETS suspended-PWMs for ALL 18 pins (so target_pulse_us
       is set everywhere, including the single leg we'll enable)
    3. Enable only the 3 pins of leg N (coxa/femur/tibia)
    4. Hold 5 s with continuous SET_TARGETS (user observes)
    5. Prompt user
    6. Disable, repeat for next leg

Usage:
    python3 tools/diagnose_single_leg.py [/dev/ttyACMx]
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
from diagnose_phase13_fix import SUSPENDED_PWMS, PIN_NAMES


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


def main() -> int:
    tty = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TTY

    print("=" * 70)
    print("Phase 13 Single-Leg Test: nur EIN Bein wird aktiviert")
    print("=" * 70)
    print()
    print("Per Bein: alle 3 Pins (coxa/femur/tibia) enabled mit suspended-PWMs.")
    print("Andere 15 Pins bleiben disabled (passiv, keine Servo-Last).")
    print()
    print("Wenn EIN Bein sauber in suspended (nach unten) faehrt:")
    print("  → Beweis dass FW+Servo OK funktioniert mit kleiner Last")
    print("  → Bug bei alle-18-gleichzeitig ist mechanisch/Strom-Limit")
    print()
    print("Wenn EIN Bein auch zu Mitte faehrt:")
    print("  → tieferes Issue, weiter graben.")
    print()
    input("Bereit? (Enter): ")

    print(f"\n[..] Opening {tty}")
    link = Link(tty)

    observations = {}
    for leg in range(1, 7):
        pins = [(leg - 1) * 3 + j for j in range(3)]
        print(f"\n=== Bein {leg} (Pins {pins}) ===")

        # RESET
        send_reset(link)
        time.sleep(0.05)
        link.drain(0.1)

        # SET_TARGETS suspended for all 18 (target_pulse_us korrekt eingestellt)
        send_set_targets(link, SUSPENDED_PWMS)
        time.sleep(0.05)
        link.drain(0.05)

        # Enable only the 3 pins of this leg
        for pin in pins:
            send_enable(link, pin, True)
            time.sleep(0.05)
        link.drain(0.05)

        print(f"    {pins[0]}={SUSPENDED_PWMS[pins[0]]}, "
              f"{pins[1]}={SUSPENDED_PWMS[pins[1]]}, "
              f"{pins[2]}={SUSPENDED_PWMS[pins[2]]} µs gesendet")
        print(f"    Erwartete Pose: Bein {leg} zeigt vertikal NACH UNTEN")
        print(f"    Halte 5 s ...")

        hold_pose_for_seconds(link, SUSPENDED_PWMS, 5.0)

        print()
        print(f"    Pose von Bein {leg}:")
        print(f"      d = nach unten (suspended, ✓)")
        print(f"      m = Mitte/horizontal (✗)")
        print(f"      o = nach oben (✗)")
        print(f"      <free text>")
        answer = input_with_keepalive(link, f"    Bein {leg}: ", SUSPENDED_PWMS)
        observations[leg] = answer

        # Disable
        for pin in pins:
            send_enable(link, pin, False)
            time.sleep(0.02)
        link.drain(0.05)

        time.sleep(0.5)  # let it relax

    # Cleanup
    print("\n--- Cleanup ---")
    send_reset(link)
    time.sleep(0.1)
    link.drain(0.1)

    # Summary
    print()
    print("=" * 70)
    print("Zusammenfassung — pro Bein, isoliert aktiviert mit suspended-PWMs")
    print("=" * 70)
    for leg in range(1, 7):
        ans = observations.get(leg, "<keine>")
        meaning = {
            "d": "↓ nach unten (suspended, ✓)",
            "m": "→ Mitte (✗)",
            "o": "↑ nach oben (✗)",
        }.get(ans.lower(), f"\"{ans}\"")
        print(f"    Bein {leg}: {meaning}")
    print()
    print("Interpretation:")
    print("  - Alle 6 Beine 'd' (nach unten):")
    print("    → FW+Servos OK mit kleiner Last. Bug bei 18-zugleich ist mechanisch")
    print("      (Drehmoment, Strom, Spannungseinbruch).")
    print("  - Manche/alle 'm' (Mitte) auch isoliert:")
    print("    → tieferes Issue, eventuell Servo-Spezifisch oder Plugin-Bug")
    print("      (das Plugin-write() schickt vielleicht andere PWMs).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
