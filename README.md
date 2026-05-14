# Hexapod Servo Driver

Firmware für das **Pimoroni Servo2040**-Board (RP2040, 18 PWM-Kanäle).
Steuert die 18 Servos eines 6-beinigen Hexapods und kommuniziert mit dem
Host über USB-CDC.

> **Status:** Phase-7-Rework im Gange. Letzter funktionierender Stand vor
> dem Rework: Tag `legacy-pushups` (Commit `2301eb2`, „pushups running").
>
> **Vor dem Code-Lesen:** `CLAUDE.md` (Arbeitsanweisung) und für Protokoll-
> Details `PROTOCOL.md` (entsteht in Phase-7-Stufe B).

---

## Voraussetzungen

### Toolchain (Ubuntu 24.04)

```bash
sudo apt install -y \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    libusb-1.0-0-dev
```

`picotool` (zum Flashen) ist in Ubuntu 24.04 **nicht** im apt-Repo
verfügbar — wird aus Source gebaut, siehe Abschnitt „picotool bauen" unten.

### SDKs

Beide Repos liegen als Geschwister-Ordner neben diesem Repo:

```
/home/enjoykin/
├── pico-sdk/             # https://github.com/raspberrypi/pico-sdk
├── pimoroni-pico/        # https://github.com/pimoroni/pimoroni-pico
└── hexapod_servo_driver/ # dieses Repo
```

```bash
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
git clone --recurse-submodules https://github.com/pimoroni/pimoroni-pico.git ~/pimoroni-pico
```

### Umgebungsvariable

```bash
# in ~/.bashrc:
export PICO_SDK_PATH="$HOME/pico-sdk"
```

Pimoroni-Pico wird automatisch als Geschwister-Ordner neben `pico-sdk`
gefunden (siehe `pimoroni_pico_import.cmake`).

---

## Build

```bash
cd hexapod_servo_driver
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

Ergebnis: `build/Hexapod_servo_driver.uf2` (≈ 131 KB).

Bei CMake-Konfig-Änderungen oder unklarer Lage: `rm -rf build` und neu.

---

## Flash

### Variante A — picotool (kein USB-Replug nötig)

```bash
sudo picotool load build/Hexapod_servo_driver.uf2
sudo picotool reboot
```

Setzt `picotool` im `PATH` voraus (siehe „picotool bauen" unten).

### Variante B — BOOTSEL-Drag-and-Drop

1. USB-Kabel vom Servo2040 trennen
2. BOOTSEL-Knopf gedrückt halten
3. USB-Kabel einstecken (BOOTSEL noch gedrückt)
4. Board erscheint als USB-Stick `RPI-RP2`
5. `build/Hexapod_servo_driver.uf2` auf das Volume kopieren
6. Board bootet automatisch neu, `RPI-RP2` verschwindet

---

## picotool bauen (einmalig)

```bash
git clone https://github.com/raspberrypi/picotool.git ~/picotool
cd ~/picotool && mkdir -p build && cd build
cmake ..                       # nutzt PICO_SDK_PATH aus ENV
make -j$(nproc)
mkdir -p ~/.local/bin
ln -sf ~/picotool/build/picotool ~/.local/bin/picotool
```

`~/.local/bin` muss im `PATH` sein (auf Ubuntu via `~/.profile` automatisch,
sonst manuell in `~/.bashrc` ergänzen).

---

## Test

Standalone-Host-Test-Skript (entsteht in Phase-7-Stufe G):

```bash
python3 tools/test_servo2040.py
```

Verifiziert ohne ROS2 alle Sicherheits-Ebenen (Hard-Clamp, Watchdog,
Soft-Ramp, Strom-Limits) und kann 2–3 angeschlossene Test-Servos definiert
bewegen.

---

## Architektur

USB-CDC-Loop mit Command-Pattern. Pro Frame:

```
[opcode:1] [args:N]  →  CommandHandler  →  Command::runCommand()
                                       →   Command::getResponse()
```

Sicherheits-Schicht zwischen Sollwert und PWM-Hardware:

```
Host-Sollwert
  → Frame-Decode (CRC-Check)
  → Hard-Clamp (pulse_min/max pro Servo)
  → Soft-Ramp (max ΔPulse/Tick)
  → Watchdog-Gate (disable_all bei Timeout)
  → Strom-/Spannungs-Trip
  → ServoCluster::pulse() (PWM-Output)
```

Details: `CLAUDE.md` Abschnitt 4 und 7, sowie
`~/hexapod_ws/docs_raspi/phase_7_servo2040_fw.md`.

---

## Lizenz

Siehe `LICENSE`.
