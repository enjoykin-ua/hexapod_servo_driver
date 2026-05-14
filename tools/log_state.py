#!/usr/bin/env python3
"""Continuously poll GET_STATE and append to a CSV file.

Use case (Phase 10 calibration): record how the rail current and pulse-µs evolve
during a motion so you can plot/inspect response curves without instrumenting
the firmware itself.

Usage:
    python3 tools/log_state.py [/dev/ttyACMx] [--rate HZ] [--out PATH] [--duration SEC]

Defaults:
    tty       /dev/ttyACM0
    --rate    20   (poll every 50 ms; firmware sense rate is 20 Hz anyway)
    --out     state_log_<UTC-timestamp>.csv
    --duration  0  (= run until Ctrl-C)

Columns (one row per poll):
    t_s         seconds since script start (monotonic)
    voltage_mv  rail voltage from STATE
    current_ma  rail current (smoothed) from STATE
    flags       status_flags byte (hex)
    p0..p17     current_pulse_us for servos 0..17

The script does NOT issue any SET_TARGETS / ENABLE_SERVO — drive the board from
another terminal (e.g. test_servo2040.py) while this logger runs.

Run AFTER tools/flash_and_verify.py; firmware must already be up. Exits 0 on
clean Ctrl-C, non-zero on link error.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import os
import sys
import time

# Reuse plumbing from test_servo2040 so we don't duplicate frame code.
_tools_dir = os.path.dirname(os.path.abspath(__file__))
if _tools_dir not in sys.path:
    sys.path.insert(0, _tools_dir)

from test_servo2040 import (  # noqa: E402
    DEFAULT_TTY, NUM_SERVOS,
    Link, get_state,
)


def _default_csv_path() -> str:
    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"state_log_{ts}.csv"


def main():
    parser = argparse.ArgumentParser(description="Poll Servo2040 GET_STATE → CSV.")
    parser.add_argument("tty", nargs="?", default=DEFAULT_TTY,
                        help=f"serial device (default {DEFAULT_TTY})")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="poll rate in Hz (default 20 = firmware sense rate)")
    parser.add_argument("--out", default=None,
                        help=f"CSV output path (default state_log_<UTC>.csv in CWD)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="run for this many seconds, then exit (0 = until Ctrl-C)")
    args = parser.parse_args()

    csv_path = args.out or _default_csv_path()
    period_s = 1.0 / args.rate

    try:
        link = Link(args.tty)
    except OSError as e:
        print(f"[FAIL] cannot open {args.tty}: {e}", file=sys.stderr)
        sys.exit(1)

    header = ["t_s", "voltage_mv", "current_ma", "flags"] + \
             [f"p{i}" for i in range(NUM_SERVOS)]

    print(f"logging to {csv_path}  at {args.rate:g} Hz "
          f"({'until Ctrl-C' if args.duration <= 0 else f'for {args.duration:g} s'})")

    rows = 0
    t0 = time.monotonic()
    deadline = (t0 + args.duration) if args.duration > 0 else float("inf")
    next_poll = t0

    try:
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now < next_poll:
                    time.sleep(next_poll - now)
                t = time.monotonic() - t0
                try:
                    pulses, _currents_unused, voltage_mv, flags = get_state(link, timeout_s=0.3)
                    rail_current_ma = _currents_unused[0]  # slot 0 = total rail (firmware convention)
                except TimeoutError:
                    print(f"[warn] t={t:.2f}s: GET_STATE timeout, skipping row")
                    next_poll += period_s
                    continue
                writer.writerow([f"{t:.4f}", voltage_mv, rail_current_ma,
                                 f"0x{flags:02X}", *pulses])
                rows += 1
                # Live status line — overwrite same line so the CSV stays clean.
                sys.stdout.write(
                    f"\r  t={t:7.2f}s  V={voltage_mv:5d}mV  I={rail_current_ma:5d}mA  "
                    f"flags=0x{flags:02X}  rows={rows}  ")
                sys.stdout.flush()
                next_poll += period_s
    except KeyboardInterrupt:
        print()  # newline after live status line
        print("interrupted by user")
    finally:
        link.close()

    elapsed = time.monotonic() - t0
    print(f"\nwrote {rows} rows in {elapsed:.1f}s → {csv_path}")


if __name__ == "__main__":
    main()
