#!/usr/bin/env python3
"""Phase 13 FW-fix VISUAL diagnose — Bein-für-Bein mit User-Bestätigung.

Workflow:
    1. RESET + SET_TARGETS suspended for all 18 pins
    2. For each leg n=1..6:
         a. Enable the 3 servos of leg n (coxa/femur/tibia)
         b. Hold 2 s with continuous SET_TARGETS (watchdog stays armed)
         c. Prompt user: "Was siehst du an Bein N? (Enter um weiterzumachen)"
            — while waiting for input, SET_TARGETS keeps being sent in the
            background so the watchdog (200 ms) does NOT trip and the servos
            stay at their commanded position. User can take their time.
    3. After all 6 legs: prompt for overall observation, then RESET.

Why this script: the firmware has NO position feedback. STATE.current_pulse_us
shows what the FW _emits_ as PWM, but not where the servo actually went.
This script ensures every leg gets activated in a controlled timing AND keeps
the servos powered while you visually inspect each one.

Usage:
    python3 tools/diagnose_phase13_visual.py [/dev/ttyACMx]

Requires:
    - firmware up (flash_and_verify.py done)
    - plugin/ROS2 NOT running
    - PSU on
    - hexapod aufgebockt
    - 30 cm radius clear
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


def hold_pose_for_seconds(link: Link, pulses: list[int], seconds: float, label: str = ""):
    """Send SET_TARGETS every 50 ms for `seconds`, keeping servos at `pulses`.

    Keeps the watchdog armed (frames every 50 ms is well under the 200 ms
    timeout) and re-commands the targets, so servos stay where they should.
    """
    if label:
        print(f"    {label}")
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        send_set_targets(link, pulses)
        link.drain(0.02)
        time.sleep(0.03)


def input_with_keepalive(link: Link, prompt: str, pulses: list[int]) -> str:
    """Read a line from stdin, but every 100 ms send SET_TARGETS to keep
    watchdog armed and servos at their commanded position.

    Returns the line (stripped). Empty line = user pressed Enter immediately.
    """
    print(prompt, end="", flush=True)
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
        if ready:
            line = sys.stdin.readline().strip()
            return line
        try:
            send_set_targets(link, pulses)
            link.drain(0.02)
        except Exception:
            pass


def main() -> int:
    tty = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TTY

    print("=" * 70)
    print("Phase 13 FW-fix VISUAL diagnose")
    print("=" * 70)
    print()
    print("Voraussetzungen:")
    print("  - Hexapod aufgebockt")
    print("  - PSU AN, ~7-8 V")
    print("  - ROS2-Plugin NICHT laufend (USB exklusiv für dieses Skript)")
    print("  - 30 cm Radius frei, Hand am PSU-Aus")
    print()
    input_with_keepalive_dummy_link = None  # Link nicht offen yet, kein keepalive
    line = input("Bereit? (Enter um zu starten, Strg+C zum Abbrechen): ")

    print(f"\n[..] Opening {tty}")
    link = Link(tty)

    # === Phase A: RESET + SET_TARGETS for all 18 pins ===
    print("\n=== Phase A: RESET + SET_TARGETS für alle 18 Pins ===")
    send_reset(link)
    time.sleep(0.1)
    link.drain(0.1)
    send_set_targets(link, SUSPENDED_PWMS)
    time.sleep(0.05)
    link.drain(0.1)
    print("    OK — FW hat target_pulse_us = suspended für alle 18 Pins,")
    print("         current_pulse_us = target (Fix 3.2), Servos sind disabled.")
    print("    Visuell: Beine sollten passiv hängen (kein PWM, kein Strom).")
    print()

    # === Phase B: Activate each leg one at a time ===
    print("=== Phase B: Bein-für-Bein-Aktivierung ===")
    print()
    print("Für jedes Bein aktiviert das Skript die 3 Servos (coxa, femur, tibia),")
    print("hält ihre Position über kontinuierliches SET_TARGETS (Watchdog bleibt")
    print("armed), und wartet auf deine Antwort. Du kannst dir Zeit lassen —")
    print("solange du nicht Strg+C drückst, halten die Servos ihre Pose.")
    print()

    observations = {}
    for leg in range(1, 7):
        pins = [(leg - 1) * 3 + j for j in range(3)]
        print(f"--- Bein {leg} ---")
        print(f"    Aktiviere Pins {pins} ({PIN_NAMES[pins[0]]}/{PIN_NAMES[pins[1]]}/{PIN_NAMES[pins[2]]})")
        print(f"    Erwartete PWMs: coxa={SUSPENDED_PWMS[pins[0]]}, "
              f"femur={SUSPENDED_PWMS[pins[1]]}, tibia={SUSPENDED_PWMS[pins[2]]}")
        print(f"    Erwartete Pose: Bein zeigt vertikal nach UNTEN")
        print(f"                    (Femur Richtung Boden, Tibia in Verlängerung)")

        # Activate the 3 pins of this leg
        for pin in pins:
            send_enable(link, pin, True)
            time.sleep(0.05)
        link.drain(0.1)

        # Hold pose for 2 s so user can see
        hold_pose_for_seconds(link, SUSPENDED_PWMS, 2.0,
                              label=f"    Halte Pose 2 s...")

        # Prompt user with keepalive — servos stay powered while user types
        print()
        print(f"    Bein {leg} ist aktiviert. Was siehst du?")
        print(f"      d = Bein zeigt nach UNTEN (suspended, korrekt)")
        print(f"      h = Bein zeigt horizontal (= weg vom hexapod)")
        print(f"      u = Bein zeigt nach OBEN (an den Körper)")
        print(f"      x = Bein zuckt/bewegt sich nicht / unklar")
        print(f"      <free text> = sonstige Beschreibung")
        answer = input_with_keepalive(link, f"    Bein {leg} Beobachtung: ", SUSPENDED_PWMS)
        observations[leg] = answer
        print()

    # === Phase C: Final GET_STATE ===
    print("=== Phase C: Final-State-Check ===")
    try:
        pulses, currents, voltage, flags = get_state(link)
        print(f"    voltage={voltage} mV   total_current={currents[0]} mA   flags=0x{flags:02X}")
        print(f"    Femur-Pulse: ", end="")
        for pin in (1, 4, 7, 10, 13, 16):
            print(f"pin{pin}={pulses[pin]}  ", end="")
        print()
    except Exception as e:
        print(f"    [FAIL] GET_STATE: {e}")

    # === Phase D: Cleanup ===
    print("\n=== Phase D: Cleanup RESET ===")
    send_reset(link)
    time.sleep(0.1)
    link.drain(0.1)
    print("    OK — alle Servos disabled.")

    # === Summary ===
    print()
    print("=" * 70)
    print("Beobachtungs-Zusammenfassung")
    print("=" * 70)
    for leg in range(1, 7):
        ans = observations.get(leg, "<keine>")
        # Map short codes to human descriptions
        meaning = {
            "d": "↓ nach unten (suspended, ✓)",
            "h": "→ horizontal (✗ Bug)",
            "u": "↑ nach oben (✗ Bug)",
            "x": "? unklar",
        }.get(ans.lower(), f"\"{ans}\"")
        print(f"    Bein {leg}: {meaning}")
    print()
    print("Schick mir diese Zusammenfassung in einer Nachricht.")
    print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
