# hexapod_servo_driver — Arbeitsanweisung für Claude

> **Lies diese Datei und `README.md` zu Beginn jeder Session in diesem Repo.**
> Dieses Repo ist eigenständig und enthält **kein ROS2**.

---

## 1. Zweck dieses Repos

Firmware für das **Pimoroni Servo2040**-Board (RP2040, 18 PWM-Kanäle).
Das Board steuert die 18 Servos des Hexapods und kommuniziert mit dem Host
(Desktop oder Raspberry Pi 4) über **USB-CDC**.

Die Firmware kennt **keine Kinematik**, kein ROS2, keine Joint-Namen.
Sie ist ein dummes Bindeglied zwischen einem Host und 18 PWM-Kanälen, mit
einer Sicherheits-Schicht dazwischen (Hard-Clamp, Watchdog, Soft-Ramp,
Strom-Limits, Low-Voltage-Cutoff).

---

## 2. Beziehung zum Hexapod-Workspace

Dieses Repo ist Teil der **Phase 7** des Hexapod-Projekts.

- Workspace: `~/hexapod_ws/`
- Phase-7-Plan: `~/hexapod_ws/docs_raspi/phase_7_servo2040_fw.md`
- Phase-7-Progress: `~/hexapod_ws/docs_raspi/phase_7_progress.md`
- Übergeordnete CLAUDE.md: `~/hexapod_ws/CLAUDE.md` (User-Profil, Tech-Stack, Verbote)

Die ROS2-Anbindung dieser Firmware erfolgt in **Phase 9** im Paket
`hexapod_hardware` (im hexapod_ws, **nicht** hier). Hier endet die
Verantwortung an der USB-CDC-Schnittstelle.

---

## 3. Tech-Stack

| Komponente | Version / Pfad |
|---|---|
| Toolchain | `gcc-arm-none-eabi 13.2.1` (apt-Paket) |
| Pico SDK | `/home/enjoykin/pico-sdk` (master, mit Submodules) |
| Pimoroni Pico | `/home/enjoykin/pimoroni-pico` (main) |
| Build | CMake ≥ 3.22, Make |
| Flash | `picotool v2.2.0-a4` (aus Source gebaut, `~/.local/bin/picotool`) |
| Sprache | C++17 |
| Standard-I/O | USB-CDC (`pico_enable_stdio_usb 1`, `pico_enable_stdio_uart 0`) |

`PICO_SDK_PATH=/home/enjoykin/pico-sdk` ist in `~/.bashrc` gesetzt.
Pimoroni-Pico wird über die Geschwister-Order-Heuristik in
`pimoroni_pico_import.cmake` automatisch gefunden (Zeile 32–33 dort).

---

## 4. Soll-Verzeichnisstruktur (nach Phase-7-Rework)

```
hexapod_servo_driver/
├── CLAUDE.md                  # diese Datei
├── README.md                  # Quickstart Build/Flash
├── PROTOCOL.md                # Wire-Protokoll (entsteht in Stufe B)
├── LICENSE
├── CMakeLists.txt
├── pico_sdk_import.cmake
├── pimoroni_pico_import.cmake
├── .gitignore
├── src/
│   ├── main.cpp               # Loop, USB-CDC-Init, Tick-Generator
│   ├── command.hpp            # Command-Interface (abstract)
│   ├── CommandHandler.hpp     # opcode → unique_ptr<Command>
│   ├── config.hpp             # Konstanten (Pins, Opcodes, Timeouts)
│   ├── includes.hpp
│   ├── commands/              # Pro Command eine Header-Datei
│   │   └── CMD_*.hpp
│   ├── safety/                # NEU in Phase 7 — Sicherheits-Ebenen
│   │   ├── hard_clamp.hpp     # Ebene 1 (Stufe C.1)
│   │   ├── watchdog.hpp       # Ebene 2 (Stufe C.2)
│   │   ├── soft_ramp.hpp      # Ebene 5 (Stufe C.3)
│   │   ├── current_limit.hpp  # Ebenen 3+4 (Stufe E.1+E.2)
│   │   └── undervoltage.hpp   # Low-Voltage-Cutoff (Stufe E.3)
│   ├── proto/                 # Frame-Encoding (Stufe B)
│   │   ├── frame.hpp
│   │   └── crc.hpp
│   └── utils/
│       ├── analog_reader.hpp
│       └── conversion.hpp
├── tools/                     # Host-seitige Test-Skripte (Python)
│   └── test_servo2040.py      # Standalone-Test-Suite (Stufe G)
└── contrib/
    └── servo_mapping.yaml     # Skelett (Stufe F, später nach hexapod_hardware)
```

---

## 5. Arbeitsweise

- **Phasenweise.** Aktuelle Phase + Sub-Stufe stehen in
  `~/hexapod_ws/PHASE.md` und `~/hexapod_ws/docs_raspi/phase_7_progress.md`.
  Wenn der User in diesem Repo arbeitet, ist immer Phase 7 aktiv.
- **Pro Schritt:** erst Konzept besprechen → dann Implementierung → dann Test.
- **Tests grün vor Commit.**
- **Commits referenzieren Phase + Teilziel:** z. B.
  `phase7-c1: hard-clamp per servo`, `phase7-d: per-servo enable test`.
- **Verworfener Code wird nicht stehen gelassen.** Wenn wir Teile der alten
  Implementation ersetzen, raus damit — Rollback geht über Tag
  `legacy-pushups` (Commit `2301eb2`, vor dem Phase-7-Rework).
- **Bei Unsicherheit zur Pimoroni-API:** Quelle nachschlagen unter
  `/home/enjoykin/pimoroni-pico/` (lokaler Klon), nicht raten.

---

## 6. Tag-Strategie

- `legacy-pushups` — letzter funktionierender Stand vor Phase-7-Rework
  (Commit `2301eb2`, "pushups running"). Reference-Point, nicht löschen.
- `phase-7-stage-<X>-done` — pro abgeschlossener Stufe (B, C, D, E, F, G).
- `phase-7-done` — am Ende der Phase, parallel zum gleichnamigen Tag im
  hexapod_ws.

---

## 7. Strikte Grenzen

### Hier **nicht** zu finden / nicht zu schreiben:

- **Keine Kinematik** — gehört nach `hexapod_kinematics` im hexapod_ws.
- **Kein ROS2** — Anbindung kommt in Phase 9 in `hexapod_hardware`.
- **Keine Joint-Namen** — auf der Wire sind nur Servo-Indizes 0..17 und
  Pulse-Werte. Das Mapping Joint↔Index lebt im `hexapod_hardware`-Paket.
- **Keine Bein-Reihenfolge / Gait-Logik** — bleibt in `hexapod_gait`.
- **Keine URDF-Werte** — Pulse-Kalibrierung kommt vom Host, nicht aus URDF.

### Hier **strikt** zu beachten (Sicherheits-Architektur):

- Hard-Clamp pro Servo **muss** vor jedem PWM-Write greifen.
- Watchdog **muss** alle Servos bei USB-Disconnect/Frame-Timeout stromlos
  schalten.
- Soft-Ramp **muss** alle Sollwert-Sprünge zwischen Frames begrenzen.
- Strom-Limit pro Servo und Total **müssen** unabhängig prüfen.
- Low-Voltage-Cutoff **muss** vorhanden sein (auch ohne Akku bereits aktiv,
  Schwellen kalibrierbar).

Diese Ebenen werden in Phase-7-Stufen C–E implementiert. Reihenfolge ist
nicht zufällig — C (Watchdog/Clamp/Ramp) fängt Software-Bugs am Host
hardwareseitig ab und braucht keine echten Hexapod-Servos zur Verifikation.

---

## 8. Konventionen

### Naming

- C++ Klassen `PascalCase`, Methoden `camelCase`, Konstanten
  `SCREAMING_SNAKE`.
- Datei-Naming für Commands: `CMD_<DIR>_<Action>.hpp`, z. B.
  `CMD_SET_Pulse_To_Servo.hpp`.
- Opcode-Defines in `config.hpp`, Präfix `CMD_`.

### Wire-Format (Stand vor Stufe B)

- Pulse-Werte: aktuell `float` µs, Plan-Empfehlung `int16` µs.
  Final-Entscheidung in Stufe B, dokumentiert in `PROTOCOL.md` und in
  `phase_7_progress.md` „Design-Entscheidungen".
- Frame-Format: aktuell Sentinel `0x55…0xAA` **ohne CRC**, Plan-Empfehlung
  COBS + CRC16. Final-Entscheidung in Stufe B.
- Update-Rate Host → Firmware: 50 Hz (gait_node-Rate), 100 Hz max.

### Sprache

- Code-Kommentare und Doxygen: Englisch.
- Diese Doku, `README.md`, `PROTOCOL.md`: Deutsch (Konsistenz mit
  hexapod_ws).
- Commit-Messages: Englisch (GitHub-Repo-Konvention).

---

## 9. Build / Flash

Kurzfassung, Details in `README.md`:

```bash
mkdir build && cd build
cmake ..                                  # PICO_SDK_PATH aus ENV
make -j$(nproc)                           # erzeugt Hexapod_servo_driver.uf2
sudo picotool load Hexapod_servo_driver.uf2
sudo picotool reboot
```

Alternative ohne picotool: Board mit gedrücktem BOOTSEL-Knopf einstecken,
`.uf2` per Drag-and-Drop auf das `RPI-RP2`-Volume kopieren.

---

## 10. Was bei Fehler zuerst geprüft wird

1. Board als `/dev/ttyACM*` sichtbar? (`dmesg | tail`, `lsusb`)
2. User in `dialout`-Gruppe? (`id`)
3. `PICO_SDK_PATH` in der aktuellen Shell exportiert? (`echo $PICO_SDK_PATH`)
4. Pimoroni-Pico-Klon auf aktuellem `main`? (`cd ~/pimoroni-pico && git log -1`)
5. Build-Ordner sauber? (`rm -rf build && mkdir build && cd build && cmake ..`)

**Nicht** als Erstes: System-Update, Toolchain-Reinstall, Treiber-Neubau.
Diagnose vor Eingriff — Goldene Regel aus hexapod_ws CLAUDE.md §5.
