#!/usr/bin/env python3
"""Flash + verify the Servo2040 firmware.

Usage:
    python3 tools/flash_and_verify.py [path/to/firmware.uf2]

Default firmware path: build/Hexapod_servo_driver.uf2 (relative to repo root).
Exit code 0 on full success, non-zero on first failure.

Notes:
    - Uses `picotool` from PATH or ~/.local/bin/picotool as fallback.
    - Calls `sudo picotool ...` because no udev rules are installed.
      Sudo token is usually cached; one password prompt per ~15 min session.
    - Stdlib-only (no pyserial). Reads /dev/ttyACM0 directly via os.open.
"""

from __future__ import annotations

import argparse
import glob
import os
import select
import shutil
import subprocess
import sys
import time
from pathlib import Path

# --- Tweakable constants -----------------------------------------------------

DEFAULT_UF2 = "build/Hexapod_servo_driver.uf2"
EXPECTED_BOOT_MSG = b"Servo2040 USB-UART Communication Started"

BOOTSEL_WAIT_S = 2.0          # after force-reboot to BOOTSEL
BOOT_RE_ENUM_WAIT_S = 3.0     # after flash + execute, before scanning ttyACM
TTYACM_READ_TIMEOUT_S = 5.0   # how long to wait for boot message
RP2040_USB_VID = "2e8a"

# --- Helpers -----------------------------------------------------------------


class StepFailed(Exception):
    pass


def step(name: str) -> None:
    print(f"[..] {name}", flush=True)


def ok(msg: str = "") -> None:
    print(f"[OK] {msg}".rstrip(), flush=True)


def fail(msg: str) -> None:
    print(f"[FAIL] {msg}", flush=True)
    raise StepFailed(msg)


def info(msg: str) -> None:
    print(f"     {msg}", flush=True)


def picotool_path() -> str:
    p = shutil.which("picotool")
    if p:
        return p
    fallback = os.path.expanduser("~/.local/bin/picotool")
    if os.path.isfile(fallback) and os.access(fallback, os.X_OK):
        return fallback
    fail("picotool not found in PATH or ~/.local/bin/")
    return ""  # unreachable, satisfies type checker


def run(cmd: list[str], sudo: bool = False) -> subprocess.CompletedProcess:
    if sudo:
        cmd = ["sudo"] + cmd
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def list_ttyacm() -> list[str]:
    return sorted(glob.glob("/dev/ttyACM*"))


def lsusb_has_rp2040() -> bool:
    res = run(["lsusb"])
    return RP2040_USB_VID in res.stdout.lower()


def read_until(tty: str, deadline: float) -> bytes:
    """Read raw bytes from tty until deadline or boot-message appears."""
    fd = os.open(tty, os.O_RDONLY | os.O_NONBLOCK)
    buf = b""
    try:
        while time.monotonic() < deadline:
            r, _, _ = select.select([fd], [], [], 0.2)
            if fd in r:
                chunk = os.read(fd, 4096)
                if chunk:
                    buf += chunk
                    if EXPECTED_BOOT_MSG in buf:
                        return buf
    finally:
        os.close(fd)
    return buf


# --- Steps -------------------------------------------------------------------


def check_uf2(uf2_path: Path) -> None:
    step(f"Checking firmware file: {uf2_path}")
    if not uf2_path.is_file():
        fail(f"firmware file not found: {uf2_path}")
    size = uf2_path.stat().st_size
    if size < 1024:
        fail(f"firmware file suspiciously small: {size} bytes")
    ok(f"{size} bytes")


def check_dialout() -> None:
    step("Checking dialout group membership")
    res = run(["id"])
    if "dialout" not in res.stdout:
        fail(
            "user not in 'dialout' group — fix: "
            "sudo usermod -aG dialout $USER  (then re-login)"
        )
    ok()


def check_picotool() -> str:
    step("Checking picotool")
    p = picotool_path()
    res = run([p, "version"])
    if res.returncode != 0:
        fail(f"picotool failed: {res.stderr.strip() or res.stdout.strip()}")
    first_line = res.stdout.strip().splitlines()[0]
    ok(first_line)
    return p


def detect_board_state() -> str:
    step("Detecting board state")
    if not lsusb_has_rp2040():
        fail(
            f"no RP2040 device found on USB (vendor {RP2040_USB_VID}). "
            "Plug in the Servo2040 board."
        )
    ttyacm = list_ttyacm()
    if ttyacm:
        info(f"found {ttyacm[0]} -> board is in run-mode")
        return "run"
    info("no ttyACM* present -> board is in BOOTSEL-mode")
    return "bootsel"


def force_bootsel(picotool: str) -> None:
    step("Forcing board into BOOTSEL mode")
    res = run([picotool, "reboot", "-f", "-u"], sudo=True)
    if res.returncode != 0:
        fail(f"picotool reboot -f -u failed: {res.stderr.strip() or res.stdout.strip()}")
    info(f"waiting {BOOTSEL_WAIT_S}s for re-enumeration")
    time.sleep(BOOTSEL_WAIT_S)
    if not lsusb_has_rp2040():
        fail("RP2040 not visible after reboot to BOOTSEL")
    ok()


def flash(picotool: str, uf2_path: Path) -> None:
    step(f"Flashing {uf2_path.name}")
    res = run([picotool, "load", "-x", str(uf2_path)], sudo=True)
    if res.returncode != 0:
        fail(f"picotool load failed:\n{res.stderr or res.stdout}")
    last_line = ""
    if res.stdout.strip():
        last_line = res.stdout.strip().splitlines()[-1]
    ok(last_line)


def wait_for_ttyacm() -> str:
    step(f"Waiting {BOOT_RE_ENUM_WAIT_S}s for USB-CDC re-enumeration")
    time.sleep(BOOT_RE_ENUM_WAIT_S)
    ttyacm = list_ttyacm()
    if not ttyacm:
        time.sleep(1.0)
        ttyacm = list_ttyacm()
    if not ttyacm:
        fail("no /dev/ttyACM* appeared after flash")
    ok(f"found {ttyacm[0]}")
    return ttyacm[0]


def verify_boot_message(tty: str, picotool: str) -> None:
    step(f"Reading boot message from {tty} (timeout {TTYACM_READ_TIMEOUT_S}s)")
    info(f"expecting line containing: {EXPECTED_BOOT_MSG.decode()}")
    try:
        deadline = time.monotonic() + TTYACM_READ_TIMEOUT_S
        buf = read_until(tty, deadline)
    except PermissionError:
        fail(
            f"permission denied opening {tty} — "
            "is dialout-group membership active in current shell?"
        )
    except OSError as e:
        fail(f"cannot open {tty}: {e}")

    if EXPECTED_BOOT_MSG in buf:
        ok("boot message received")
        return

    if buf:
        info(f"received {len(buf)} bytes but no match. Sample: {buf[:120]!r}")
    info("no boot message — trying one reboot to re-trigger")

    # Force-reboot via USB into application (-f -a). The board is in
    # run-mode now, so a plain `picotool reboot` would fail with
    # "No accessible RP-series devices in BOOTSEL mode".
    res = run([picotool, "reboot", "-f", "-a"], sudo=True)
    if res.returncode != 0:
        fail(f"picotool reboot retry failed: {res.stderr.strip() or res.stdout.strip()}")
    time.sleep(BOOT_RE_ENUM_WAIT_S)
    ttyacm = list_ttyacm()
    if not ttyacm:
        fail("no ttyACM* after reboot retry")

    deadline = time.monotonic() + TTYACM_READ_TIMEOUT_S
    buf = read_until(ttyacm[0], deadline)
    if EXPECTED_BOOT_MSG in buf:
        ok("boot message received on retry")
        return
    fail(f"no boot message within {TTYACM_READ_TIMEOUT_S}s (after retry)")


# --- Main --------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Flash the Servo2040 firmware and verify it boots."
    )
    parser.add_argument(
        "uf2",
        nargs="?",
        default=DEFAULT_UF2,
        help=f"path to .uf2 file (default: {DEFAULT_UF2})",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    uf2_path = Path(args.uf2)
    if not uf2_path.is_absolute():
        uf2_path = repo_root / uf2_path

    print("=== Servo2040 flash-and-verify ===\n")
    t0 = time.monotonic()

    try:
        check_uf2(uf2_path)
        check_dialout()
        picotool = check_picotool()
        state = detect_board_state()
        if state == "run":
            force_bootsel(picotool)
        flash(picotool, uf2_path)
        tty = wait_for_ttyacm()
        verify_boot_message(tty, picotool)
    except StepFailed as e:
        elapsed = time.monotonic() - t0
        print(f"\n=== X FLASH+VERIFY FAILED after {elapsed:.1f}s ===")
        print(f"=== reason: {e} ===")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n=== aborted by user ===")
        sys.exit(130)

    elapsed = time.monotonic() - t0
    print(f"\n=== OK FLASH+VERIFY successful in {elapsed:.1f}s ===")
    print("=== Firmware running on Servo2040, ready for next step ===")


if __name__ == "__main__":
    main()
