#!/usr/bin/env python3
"""Diagnostic probe — shows raw RX bytes from the Servo2040 firmware.

Sends RESET, then SET_TARGETS([100]*18), and dumps every byte received
between/after, with 0x00 frame separators highlighted. Use this when
test_servo2040.py fails to figure out *what* the firmware is actually
sending (or not).

Usage:
    python3 tools/probe.py [/dev/ttyACMx]
"""
from __future__ import annotations

import os
import select
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from test_servo2040 import (
    Link, encode_frame, decode_frame,
    CMD_RESET, CMD_SET_TARGETS, CMD_GET_STATE,
    NUM_SERVOS,
)


def hex_dump(label: str, data: bytes) -> None:
    if not data:
        print(f"  {label}: (no bytes)")
        return
    chunks = []
    cur = []
    for b in data:
        cur.append(f"{b:02x}")
        if b == 0x00:
            chunks.append(" ".join(cur))
            cur = []
    if cur:
        chunks.append(" ".join(cur) + "  (no trailing 0x00)")
    print(f"  {label} ({len(data)} bytes):")
    for c in chunks:
        print(f"    | {c}")


def collect_rx(link: Link, duration_s: float) -> bytes:
    """Read raw bytes for `duration_s`, drain Link._rx, return collected."""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        link._drain_into_buffer(min(0.05, deadline - time.monotonic()))
    out = bytes(link._rx)
    link._rx.clear()
    return out


def split_frames(buf: bytes) -> list[bytes]:
    """Split on 0x00 (the COBS frame separator). Empty slices dropped."""
    if not buf:
        return []
    parts: list[bytes] = []
    cur = bytearray()
    for b in buf:
        if b == 0x00:
            if cur:
                parts.append(bytes(cur))
                cur.clear()
        else:
            cur.append(b)
    if cur:
        parts.append(bytes(cur))  # tail without terminator
    return parts


def main() -> None:
    tty = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    print(f"=== Servo2040 probe on {tty} ===\n")

    link = Link(tty)
    try:
        # Phase 1: passive listen for boot output
        print("[1] Passive listen 0.6s after open (banner expected)")
        rx = collect_rx(link, 0.6)
        hex_dump("RX", rx)

        # Phase 2: send RESET, listen
        print("\n[2] Send RESET, listen 0.4s")
        link.write(encode_frame(0x55, CMD_RESET, b""))
        rx = collect_rx(link, 0.4)
        hex_dump("RX", rx)
        for i, f in enumerate(split_frames(rx)):
            decoded = decode_frame(f)
            if decoded:
                seq, cmd, payload = decoded
                print(f"    [{i}] decoded: seq=0x{seq:02x} cmd=0x{cmd:02x} "
                      f"payload={bytes(payload).hex() or '<empty>'}")
            else:
                print(f"    [{i}] decode failed (not a valid frame)")

        # Phase 3: send GET_STATE — quick sanity that round-trip works
        print("\n[3] Send GET_STATE, listen 0.4s")
        link.write(encode_frame(0x56, CMD_GET_STATE, b""))
        rx = collect_rx(link, 0.4)
        hex_dump("RX", rx)
        for i, f in enumerate(split_frames(rx)):
            decoded = decode_frame(f)
            if decoded:
                seq, cmd, payload = decoded
                print(f"    [{i}] decoded: seq=0x{seq:02x} cmd=0x{cmd:02x} "
                      f"len={len(payload)} (payload omitted)")
            else:
                print(f"    [{i}] decode failed")

        # Phase 4: send SET_TARGETS with pulse=100 (should trigger ERROR_REPORT)
        print("\n[4] Send SET_TARGETS([100]*18) — expect ERROR_REPORT/0x10, listen 0.6s")
        payload = struct.pack(f"<{NUM_SERVOS}h", *([100] * NUM_SERVOS))
        link.write(encode_frame(0x57, CMD_SET_TARGETS, payload))
        rx = collect_rx(link, 0.6)
        hex_dump("RX", rx)
        for i, f in enumerate(split_frames(rx)):
            decoded = decode_frame(f)
            if decoded:
                seq, cmd, payload = decoded
                print(f"    [{i}] decoded: seq=0x{seq:02x} cmd=0x{cmd:02x} "
                      f"payload={bytes(payload).hex() or '<empty>'}")
            else:
                print(f"    [{i}] decode failed")

        # Phase 5: send SET_TARGETS with valid pulse=1500 — expect ACK
        print("\n[5] Send SET_TARGETS([1500]*18) — expect ACK, listen 0.4s")
        payload = struct.pack(f"<{NUM_SERVOS}h", *([1500] * NUM_SERVOS))
        link.write(encode_frame(0x58, CMD_SET_TARGETS, payload))
        rx = collect_rx(link, 0.4)
        hex_dump("RX", rx)
        for i, f in enumerate(split_frames(rx)):
            decoded = decode_frame(f)
            if decoded:
                seq, cmd, payload = decoded
                print(f"    [{i}] decoded: seq=0x{seq:02x} cmd=0x{cmd:02x} "
                      f"payload={bytes(payload).hex() or '<empty>'}")
            else:
                print(f"    [{i}] decode failed")

    finally:
        link.close()

    print("\n=== probe done ===")


if __name__ == "__main__":
    main()
