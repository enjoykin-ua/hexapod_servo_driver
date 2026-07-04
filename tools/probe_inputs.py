#!/usr/bin/env python3
"""HW5 bench probe — poll GET_INPUTS and show the foot-contact bitmask live.

Sends GET_INPUTS (0x40) at ~10 Hz and decodes the INPUTS (0xC0) reply, printing
the 1-byte bitmask as per-leg contact flags plus USER_SW. Use this to verify the
firmware side of the HW-foot-contact feature on the bench:

  - HW5.8: wire leg 1 to SENSOR_1 (IN + GND, normally-open). Press the switch →
    L1 flips ●; release → ○. Only bit 0 should move.
  - HW5.9: wire all 6 → each Ln flips with its own switch.

Bitmask (PROTOCOL.md §3.2): bit 0..5 = leg 1..6 foot contact (SENSOR_1..6),
bit 6 = USER_SW (onboard button, active-low), bit 7 reserved.

Pressed (foot on ground / switch closed to GND) reads 1; open reads 0. Firmware
debounces (2 ticks / ~20 ms), so a stable press/release settles within a frame.

This is a read-only probe: it never enables servos or closes the relay, so it is
safe to run with the robot jacked up or powered down (bare USB is enough — the
firmware samples inputs every tick regardless of host or servo state).

Usage:
    python3 tools/probe_inputs.py [/dev/ttyACMx]   # Ctrl-C to stop
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from test_servo2040 import (
    Link, encode_frame, decode_frame, next_seq,
    CMD_GET_INPUTS, CMD_INPUTS_RESP,
)

POLL_HZ = 10.0


def render(mask: int) -> str:
    """One-line view: L1..L6 contact flags + SW, plus raw hex."""
    legs = " ".join(
        f"L{n}{'●' if (mask >> (n - 1)) & 1 else '○'}" for n in range(1, 7)
    )
    sw = "SW●" if (mask >> 6) & 1 else "SW○"
    return f"{legs}  {sw}   raw=0x{mask:02x} 0b{mask:08b}"


def poll_once(link: Link, timeout_s: float = 0.3):
    """Send one GET_INPUTS and return the reply mask, or None on timeout."""
    seq = next_seq()
    link.write(encode_frame(seq, CMD_GET_INPUTS, b""))
    deadline = time.monotonic() + timeout_s
    for frame in link.read_frames(deadline):
        decoded = decode_frame(frame)
        if not decoded:
            continue
        _rseq, cmd, payload = decoded
        if cmd == CMD_INPUTS_RESP and len(payload) == 1:
            return payload[0]
    return None


def main() -> None:
    tty = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    print(f"=== HW5 GET_INPUTS probe on {tty} (Ctrl-C to stop) ===")
    print("● = contact/pressed, ○ = open. Legs L1..L6 = SENSOR_1..6, SW = USER_SW.\n")

    link = Link(tty)
    link.drain(0.6)  # swallow the boot banner
    period = 1.0 / POLL_HZ
    last = None
    try:
        while True:
            mask = poll_once(link)
            if mask is None:
                print("  (no INPUTS reply — timeout)")
            else:
                line = render(mask)
                # Mark the lines where something changed, so a bench run has an
                # obvious visual event on every press/release.
                marker = " <-- change" if last is not None and mask != last else ""
                print(f"  {line}{marker}")
                last = mask
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n=== probe_inputs done ===")
    finally:
        link.close()


if __name__ == "__main__":
    main()
