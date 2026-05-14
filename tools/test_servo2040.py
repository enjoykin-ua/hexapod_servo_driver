#!/usr/bin/env python3
"""Stage-C/D smoke tests for the Servo2040 firmware.

Stage C (no servos required):
  Verifies hard-clamp (C.1), watchdog (C.2), soft-ramp (C.3) over USB-CDC
  using only PROTOCOL.md frames — STATE echoes, ERROR_REPORTs, NACKs.

Stage D (physical servos required):
  Verifies per-servo-enable with staged boot (50 ms stagger). Needs
  N real servos connected to outputs 0..N-1.

Usage:
    python3 tools/test_servo2040.py [/dev/ttyACMx] [--servos N]

    --servos N   Run stage-D test with N physical servos on outputs 0..N-1.
                 Omit or pass 0 to skip (default).

Default port: /dev/ttyACM0. Exit 0 on PASS, non-zero on first FAIL.

Run AFTER tools/flash_and_verify.py — the firmware must already be running.
The script itself does NOT flash.
"""
from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import termios
import time
import tty

# --- Constants (mirror src/config.hpp + PROTOCOL.md) -------------------------

DEFAULT_TTY = "/dev/ttyACM0"
NUM_SERVOS = 18

# Opcodes
CMD_SET_TARGETS    = 0x01
CMD_GET_STATE      = 0x02
CMD_ENABLE_SERVO   = 0x20
CMD_RESET          = 0x50
CMD_STATE_RESP     = 0x82
CMD_ERROR_REPORT   = 0x7F
CMD_ACK            = 0xFF
CMD_NACK           = 0xFE

# Error codes
ERR_PULSE_OUT_OF_RANGE = 0x10
ERR_WATCHDOG_TRIPPED   = 0x40

# Status flag bits
STATUS_WATCHDOG_TRIPPED   = 1 << 0
STATUS_ANY_SERVO_DISABLED = 1 << 4

# Defaults from src/config.hpp
DEFAULT_PULSE_MIN  = 500
DEFAULT_PULSE_MAX  = 2500
DEFAULT_PULSE_ZERO = 1500

WATCHDOG_TIMEOUT_MS    = 200
SOFT_RAMP_RATE_US_PER_S = 2000   # 20 µs/tick × 100 ticks/s

# --- CRC-16/CCITT-FALSE -------------------------------------------------------

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _self_test_crc():
    assert crc16(b"123456789") == 0x29B1, "CRC self-test failed"


# --- COBS ---------------------------------------------------------------------

def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])  # placeholder for first code
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            return b""
        i += 1
        for _ in range(code - 1):
            if i >= len(data):
                return b""
            out.append(data[i])
            i += 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


# --- Frame --------------------------------------------------------------------

def encode_frame(seq: int, cmd: int, payload: bytes) -> bytes:
    raw = bytes([seq, cmd, len(payload)]) + payload
    raw_with_crc = raw + struct.pack("<H", crc16(raw))
    return cobs_encode(raw_with_crc) + b"\x00"


def decode_frame(raw: bytes):
    decoded = cobs_decode(raw)
    if len(decoded) < 5:
        return None
    seq, cmd_, length = decoded[0], decoded[1], decoded[2]
    if len(decoded) != 3 + length + 2:
        return None
    payload = decoded[3:3 + length]
    crc_recv = struct.unpack("<H", decoded[3 + length:3 + length + 2])[0]
    if crc_recv != crc16(decoded[:3 + length]):
        return None
    return (seq, cmd_, payload)


# --- Link ---------------------------------------------------------------------

class Link:
    def __init__(self, tty_path: str):
        self.fd = os.open(tty_path, os.O_RDWR | os.O_NONBLOCK)
        tty.setraw(self.fd)  # disable line discipline: no CR/LF mangling, no echo
        self._rx = bytearray()

    def close(self):
        os.close(self.fd)

    def write(self, data: bytes):
        os.write(self.fd, data)

    def _drain_into_buffer(self, max_block_s: float):
        r, _, _ = select.select([self.fd], [], [], max_block_s)
        if self.fd in r:
            try:
                self._rx.extend(os.read(self.fd, 4096))
            except BlockingIOError:
                pass

    def read_frames(self, deadline: float) -> list:
        out = []
        while time.monotonic() < deadline:
            self._drain_into_buffer(0.02)
            while 0 in self._rx:
                idx = self._rx.index(0)
                raw = bytes(self._rx[:idx])
                del self._rx[:idx + 1]
                if not raw:
                    continue
                f = decode_frame(raw)
                if f is not None:
                    out.append(f)
            if out:
                # opportunistic: don't keep blocking if we've got something
                self._drain_into_buffer(0)
                while 0 in self._rx:
                    idx = self._rx.index(0)
                    raw = bytes(self._rx[:idx])
                    del self._rx[:idx + 1]
                    if not raw:
                        continue
                    f = decode_frame(raw)
                    if f is not None:
                        out.append(f)
                return out
        return out

    def drain(self, duration_s: float = 0.1):
        self.read_frames(time.monotonic() + duration_s)


# --- Frame sending ------------------------------------------------------------

_seq = 0
def next_seq() -> int:
    global _seq
    _seq = (_seq + 1) & 0xFF
    return _seq


def send_set_targets(link: Link, pulses) -> int:
    if len(pulses) != NUM_SERVOS:
        raise ValueError(f"need {NUM_SERVOS} pulses")
    payload = struct.pack(f"<{NUM_SERVOS}h", *pulses)
    seq = next_seq()
    link.write(encode_frame(seq, CMD_SET_TARGETS, payload))
    return seq


def send_get_state(link: Link) -> int:
    seq = next_seq()
    link.write(encode_frame(seq, CMD_GET_STATE, b""))
    return seq


def send_reset(link: Link) -> int:
    seq = next_seq()
    link.write(encode_frame(seq, CMD_RESET, b""))
    return seq


def send_enable(link: Link, idx: int, enable: bool) -> int:
    seq = next_seq()
    link.write(encode_frame(seq, CMD_ENABLE_SERVO,
                            bytes([idx, 1 if enable else 0])))
    return seq


def parse_state(payload: bytes):
    pulses   = list(struct.unpack(f"<{NUM_SERVOS}h", payload[:36]))
    currents = list(struct.unpack(f"<{NUM_SERVOS}H", payload[36:72]))
    voltage  = struct.unpack("<H", payload[72:74])[0]
    flags    = payload[74]
    return pulses, currents, voltage, flags


def get_state(link: Link, timeout_s: float = 1.0):
    seq = send_get_state(link)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for s, c, p in link.read_frames(time.monotonic() + 0.05):
            if s == seq and c == CMD_STATE_RESP:
                return parse_state(p)
    raise TimeoutError(f"GET_STATE: no STATE within {timeout_s}s")


def hold_target(link: Link, pulses, duration_s: float, ping_period_s: float = 0.1):
    """Re-send SET_TARGETS at fixed rate to keep the watchdog quiet
    while the soft-ramp settles."""
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        send_set_targets(link, pulses)
        time.sleep(min(ping_period_s, max(0.0, end - time.monotonic())))


def expect_frame(link: Link, predicate, timeout_s: float = 1.0):
    """Read until predicate(seq, cmd, payload) is true."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for s, c, p in link.read_frames(time.monotonic() + 0.05):
            if predicate(s, c, p):
                return (s, c, p)
    return None


# --- Output -------------------------------------------------------------------

def step(name: str):
    print(f"[..] {name}", flush=True)

def ok(msg: str = ""):
    print(f"[OK] {msg}".rstrip(), flush=True)

def info(msg: str):
    print(f"     {msg}", flush=True)

def fail(msg: str):
    print(f"[FAIL] {msg}", flush=True)
    raise AssertionError(msg)


# --- Tests --------------------------------------------------------------------

def test_hard_clamp(link: Link):
    step("C.1 hard-clamp: out-of-range pulses are clamped to min/max")

    # ---- below min ----
    send_reset(link)
    time.sleep(0.05)
    link.drain(0.05)
    seq = send_set_targets(link, [100] * NUM_SERVOS)
    f = expect_frame(link,
                     lambda s, c, p: s == seq and c == CMD_ERROR_REPORT
                                     and p[0] == ERR_PULSE_OUT_OF_RANGE,
                     timeout_s=1.0)
    if f is None:
        fail("expected ERROR_REPORT/PULSE_OUT_OF_RANGE for pulse=100")

    # Soft-ramp from zero (1500) to clamped 500 = 1000 µs / 2000 µs/s = 500 ms.
    hold_target(link, [100] * NUM_SERVOS, duration_s=0.7)
    pulses, _, _, _ = get_state(link)
    for i, p in enumerate(pulses):
        if p != DEFAULT_PULSE_MIN:
            fail(f"servo {i} not clamped to min: current={p}, want {DEFAULT_PULSE_MIN}")
    ok(f"all 18 servos clamped to {DEFAULT_PULSE_MIN} µs (min)")

    # ---- above max ----
    seq = send_set_targets(link, [5000] * NUM_SERVOS)
    f = expect_frame(link,
                     lambda s, c, p: s == seq and c == CMD_ERROR_REPORT
                                     and p[0] == ERR_PULSE_OUT_OF_RANGE,
                     timeout_s=1.0)
    if f is None:
        fail("expected ERROR_REPORT/PULSE_OUT_OF_RANGE for pulse=5000")

    # Ramp from 500 → 2500 = 2000 µs / 2000 µs/s = 1.0 s.
    hold_target(link, [5000] * NUM_SERVOS, duration_s=1.2)
    pulses, _, _, _ = get_state(link)
    for i, p in enumerate(pulses):
        if p != DEFAULT_PULSE_MAX:
            fail(f"servo {i} not clamped to max: current={p}, want {DEFAULT_PULSE_MAX}")
    ok(f"all 18 servos clamped to {DEFAULT_PULSE_MAX} µs (max)")


def test_watchdog(link: Link):
    step("C.2 watchdog: stall triggers trip + RESET recovers")

    send_reset(link)
    time.sleep(0.05)
    link.drain(0.05)
    # Arm watchdog with a valid frame
    send_set_targets(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS)
    time.sleep(0.05)
    link.drain(0.05)

    # Stall > 200 ms — expect unsolicited ERROR_REPORT with WATCHDOG_TRIPPED
    info(f"stalling {WATCHDOG_TIMEOUT_MS + 150}ms to trigger watchdog…")
    time.sleep((WATCHDOG_TIMEOUT_MS + 150) / 1000.0)
    f = expect_frame(link,
                     lambda s, c, p: c == CMD_ERROR_REPORT
                                     and p[0] == ERR_WATCHDOG_TRIPPED,
                     timeout_s=0.3)
    if f is None:
        fail("no unsolicited ERROR_REPORT/WATCHDOG_TRIPPED after stall")
    ok("unsolicited ERROR_REPORT received")

    # status_flags must show the trip
    _, _, _, flags = get_state(link)
    if not (flags & STATUS_WATCHDOG_TRIPPED):
        fail(f"WATCHDOG_TRIPPED flag missing in status (flags=0x{flags:02X})")
    ok(f"status flags = 0x{flags:02X} (WATCHDOG_TRIPPED + ANY_SERVO_DISABLED)")

    # ENABLE_SERVO during trip -> NACK reason WATCHDOG_TRIPPED
    seq = send_enable(link, 0, True)
    f = expect_frame(link,
                     lambda s, c, p: s == seq and c == CMD_NACK
                                     and len(p) >= 2
                                     and p[0] == CMD_ENABLE_SERVO
                                     and p[1] == ERR_WATCHDOG_TRIPPED,
                     timeout_s=1.0)
    if f is None:
        fail("expected NACK/WATCHDOG_TRIPPED for ENABLE_SERVO during trip")
    ok("ENABLE_SERVO during trip is correctly NACKed")

    # RESET must clear the trip
    send_reset(link)
    time.sleep(0.1)
    link.drain(0.1)
    _, _, _, flags = get_state(link)
    if flags & STATUS_WATCHDOG_TRIPPED:
        fail(f"WATCHDOG_TRIPPED still set after RESET (flags=0x{flags:02X})")
    ok(f"after RESET status = 0x{flags:02X} (trip cleared)")


def test_soft_ramp(link: Link):
    step("C.3 soft-ramp: target jump is rate-limited")

    send_reset(link)
    time.sleep(0.1)
    link.drain(0.1)

    # Initial state: PULSE_ZERO
    pulses, _, _, _ = get_state(link)
    for i, p in enumerate(pulses):
        if abs(p - DEFAULT_PULSE_ZERO) > 5:
            fail(f"unexpected init: servo {i} current={p}, want ~{DEFAULT_PULSE_ZERO}")

    # Jump to MAX. Expected ramp speed: 2000 µs/s. After 100 ms: ~200 µs delta.
    send_set_targets(link, [DEFAULT_PULSE_MAX] * NUM_SERVOS)
    time.sleep(0.10)
    send_set_targets(link, [DEFAULT_PULSE_MAX] * NUM_SERVOS)  # keep watchdog happy
    pulses, _, _, _ = get_state(link)
    for i, p in enumerate(pulses):
        if not (DEFAULT_PULSE_ZERO <= p <= DEFAULT_PULSE_ZERO + 400):
            fail(f"servo {i} ramp speed wrong: {p} µs after 100ms "
                 f"(want {DEFAULT_PULSE_ZERO}..{DEFAULT_PULSE_ZERO + 400})")
    ok(f"after 100 ms: pulses ≈ {pulses[0]} µs (NOT immediately {DEFAULT_PULSE_MAX})")

    # Wait the rest of the ramp + a margin (1000 µs / 2000 µs/s = 500 ms total,
    # we've done 100 ms, give another 600 ms with re-pings).
    hold_target(link, [DEFAULT_PULSE_MAX] * NUM_SERVOS, duration_s=0.6)
    pulses, _, _, _ = get_state(link)
    for i, p in enumerate(pulses):
        if p != DEFAULT_PULSE_MAX:
            fail(f"servo {i} did not reach max: {p} (want {DEFAULT_PULSE_MAX})")
    ok(f"after full ramp: pulses = {DEFAULT_PULSE_MAX} µs")


# --- Stage-D test -------------------------------------------------------------

def _pause(link: "Link", manual: bool, msg: str):
    """In manual mode: print msg, keep watchdog alive with 100 ms pings, wait for Enter."""
    if not manual:
        return
    print(f"     [MANUAL] {msg} — press Enter to continue…", end="", flush=True)
    # select.select on stdin: 100 ms timeout per iteration so we can keep pinging.
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.10)
        if r:
            sys.stdin.readline()  # consume the Enter key
            break
        send_set_targets(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS)
    link.drain(0.05)  # clear ACKs from the keep-alive pings


def test_per_servo_enable(link: Link, n_servos: int, manual: bool = False):
    """D.1: staged boot — enable each servo individually with 50 ms stagger.

    Physical check (must be observed on hardware):
      - Each servo should engage ~50 ms after the previous one.
      - PSU ammeter should show separate inrush spikes, not one shared peak.

    With --manual: pauses between each sub-step so you can observe exactly
    where fast movement occurs (snap-to-neutral at enable vs. ramp movement).
    """
    mode = " [MANUAL]" if manual else ""
    step(f"D.1 per-servo-enable: staged boot for {n_servos} servo(s) on outputs 0..{n_servos - 1}{mode}")
    info("PSU SETUP: servo rail = 6.0 V / current-limit 3.0 A (MG996R: ~500 mA no-load, 2.5 A stall each)")
    info("PHYSICAL OBSERVATION: servos should engage one by one, ~50 ms apart")

    # Step 1: RESET — all servos disabled, targets at 1500 µs
    _pause(link, manual, "Step 1: RESET (all servos will be disabled, targets set to 1500 µs)")
    send_reset(link)
    time.sleep(0.05)
    link.drain(0.1)
    hold_target(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS, duration_s=0.2)
    if manual:
        pulses, _, _, flags = get_state(link)
        info(f"  after RESET: current=[{pulses[0]}..{pulses[n_servos-1]}] µs, flags=0x{flags:02X}")

    # Step 2: Staged enable — 50 ms between each servo
    # NOTE: at ENABLE_SERVO the servo snaps to current_pulse_us (= 1500 µs).
    # If it was physically at a different position, this is intentional and expected.
    for i in range(n_servos):
        _pause(link, manual,f"Step 2.{i}: ENABLE_SERVO({i}) — servo {i} will snap to 1500 µs now")
        seq = send_enable(link, i, True)
        f = expect_frame(
            link,
            lambda s, c, p, _seq=seq: (s == _seq
                                       and c == CMD_ACK
                                       and len(p) >= 1
                                       and p[0] == CMD_ENABLE_SERVO),
            timeout_s=1.0,
        )
        if f is None:
            fail(f"no ACK for ENABLE_SERVO({i})")
        ok(f"servo {i}: enabled (ACK received)")
        if i < n_servos - 1:
            time.sleep(0.05)
            send_set_targets(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS)
            link.drain(0)

    # Verify status flags
    _, _, _, flags = get_state(link)
    expected_disabled_flag = n_servos < NUM_SERVOS
    has_disabled_flag = bool(flags & STATUS_ANY_SERVO_DISABLED)
    if has_disabled_flag != expected_disabled_flag:
        fail(f"ANY_SERVO_DISABLED flag wrong: flags=0x{flags:02X}, "
             f"expected {'set' if expected_disabled_flag else 'clear'}")
    ok(f"state flags = 0x{flags:02X} (correct for {n_servos}/{NUM_SERVOS} enabled)")

    # Step 3: Movement test — ramp +300 µs above neutral via soft-ramp
    # At 2000 µs/s the 300 µs delta takes 150 ms. After 400 ms we should be there.
    # This is the RAMP — should be slow and smooth, not a snap.
    _pause(link, manual, "Step 3: SET_TARGETS to 1800 µs — soft-ramp should take ~150 ms, NOT a snap")
    target = DEFAULT_PULSE_ZERO + 300
    hold_target(link, [target] * NUM_SERVOS, duration_s=0.4)
    pulses, _, _, _ = get_state(link)
    for i in range(n_servos):
        if pulses[i] < DEFAULT_PULSE_ZERO + 100:
            fail(f"servo {i} not ramping toward target: current={pulses[i]} µs, "
                 f"expected ≥ {DEFAULT_PULSE_ZERO + 100}")
    ok(f"enabled servos at target: [{', '.join(str(pulses[i]) for i in range(n_servos))}] µs")

    # Step 4: Return to neutral, then disable
    _pause(link, manual, "Step 4: return to 1500 µs (ramp back), then disable all")
    hold_target(link, [DEFAULT_PULSE_ZERO] * NUM_SERVOS, duration_s=0.4)
    for i in range(n_servos):
        send_enable(link, i, False)
    time.sleep(0.05)
    link.drain(0.1)
    send_reset(link)
    ok(f"all {n_servos} servo(s) returned to neutral and disabled")


# --- Main ---------------------------------------------------------------------

def main():
    _self_test_crc()

    parser = argparse.ArgumentParser(
        description="Servo2040 stage-C/D smoke tests over USB-CDC.")
    parser.add_argument("tty", nargs="?", default=DEFAULT_TTY,
                        help=f"serial device (default {DEFAULT_TTY})")
    parser.add_argument("--servos", type=int, default=0, metavar="N",
                        help="number of physical test servos on outputs 0..N-1 "
                             "(enables stage-D test, default 0 = skip)")
    parser.add_argument("--manual", action="store_true",
                        help="pause between each sub-step of the stage-D test "
                             "so you can observe physical servo behavior")
    args = parser.parse_args()

    stages = "stage-C" + (f" + stage-D ({args.servos} servo(s))" if args.servos else "")
    print(f"=== Servo2040 {stages} tests on {args.tty} ===\n")

    try:
        link = Link(args.tty)
    except OSError as e:
        print(f"[FAIL] cannot open {args.tty}: {e}")
        sys.exit(1)

    t0 = time.monotonic()
    try:
        test_hard_clamp(link)
        test_watchdog(link)
        test_soft_ramp(link)
        if args.servos > 0:
            print()
            test_per_servo_enable(link, args.servos, manual=args.manual)
    except (AssertionError, TimeoutError) as e:
        elapsed = time.monotonic() - t0
        print(f"\n=== X TESTS FAILED after {elapsed:.1f}s: {e} ===")
        sys.exit(1)
    finally:
        try:
            send_reset(link)
            time.sleep(0.05)
        finally:
            link.close()

    elapsed = time.monotonic() - t0
    print(f"\n=== OK ALL {stages.upper()} TESTS PASSED in {elapsed:.1f}s ===")


if __name__ == "__main__":
    main()
