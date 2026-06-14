# PROTOCOL — hexapod_servo_driver Wire-Protocol

**Stand:** 2026-05-14, Phase 7 Stufe B fixiert.

Verbindlich für die Firmware (RP2040) und für jeden Host, der mit ihr
spricht — Stufe G (`tools/test_servo2040.py`) und Phase 9
(`hexapod_hardware`-Paket im hexapod_ws).

---

## 1. Transport

- Physisch: USB-CDC (`/dev/ttyACM*`), Standard CDC-ACM
- Baudrate: irrelevant (USB-CDC ignoriert sie), Host kann beliebig öffnen
- Update-Rate Host → Firmware: **50 Hz Default**, bis 100 Hz erlaubt
- Mindest-Pause zwischen Frames: keine, Empfänger ist Byte-getrieben
- Frame-Trenner: Byte `0x00` (siehe COBS unten)

---

## 2. Frame-Encoding (COBS + CRC16)

### 2.1 Was ist COBS

**Consistent Overhead Byte Stuffing** — ein Verfahren, um Frames in einem
Byte-Stream byte-eindeutig zu trennen. COBS ersetzt jede `0x00` im Payload
durch einen Zähler, sodass `0x00` ausschließlich als **Frame-Trenner**
vorkommen kann. Overhead: maximal 1 Byte zusätzlich pro 254 Payload-Byte.

**Wozu**: Empfänger kann jederzeit auf Frame-Anfang resync'en, indem er bis
zum nächsten `0x00` liest und mit dem nächsten Frame neu beginnt. Robuster
als Sentinel-Markierung mit Escape-Sequenz, weil der Trenner ein einziges
Byte ist und nirgends im Payload auftauchen kann.

Referenz: Cheshire & Baker, "Consistent Overhead Byte Stuffing", 1999.

### 2.2 Was ist CRC-16/CCITT-FALSE

**CRC** = Cyclic Redundancy Check, eine Prüfsumme zur Erkennung von
Bit-Fehlern. CRC-16/CCITT-FALSE ist eine Standard-Variante mit:

| Parameter | Wert |
|---|---|
| Polynom | `0x1021` |
| Init | `0xFFFF` |
| Reflect Input | nein |
| Reflect Output | nein |
| XorOut | `0x0000` |
| Selbsttest (CRC über `"123456789"`) | `0x29B1` |

**Wozu**: USB-CDC hat zwar Transport-CRC, aber sobald die Firmware in
Phase 11 auf den Pi portiert wird oder wir später UART statt USB nutzen,
kann ein einzelnes Bit-Flip einen Servo auf eine völlig falsche Position
schicken. CRC-16 erkennt alle 1- und 2-Bit-Fehler, alle Burst-Fehler ≤
16 Bit, und 99,997 % aller längeren Burst-Fehler.

**Implementation**: Lookup-Table 256 × 2 Byte = 512 Byte Flash. Pro Byte:
ein XOR + ein Shift + ein Tabellen-Lookup.

Referenz-Implementation in C kommt mit der Firmware in `src/proto/crc.hpp`
(Stufe B-Implementierung).

### 2.3 Frame-Layout (vor COBS-Encoding)

```
+-------+-------+-------+-------------------+----------+
| SEQ   | CMD   | LEN   | PAYLOAD (LEN B)   | CRC16    |
+-------+-------+-------+-------------------+----------+
  1 B    1 B     1 B     0..253 B             2 B (LE)
```

| Feld | Größe | Beschreibung |
|---|---|---|
| `SEQ` | 1 Byte | Sequenznummer 0..255, Host inkrementiert pro Frame, Firmware echoiert in Antwort |
| `CMD` | 1 Byte | Opcode (siehe Kommando-Tabelle Abschnitt 3) |
| `LEN` | 1 Byte | Anzahl Payload-Bytes (0..253) |
| `PAYLOAD` | LEN Byte | Kommando-spezifisch |
| `CRC16` | 2 Byte LE | CRC-16/CCITT-FALSE über `SEQ ‖ CMD ‖ LEN ‖ PAYLOAD` |

Maximale Frame-Größe vor COBS: 5 + 253 = **258 Byte**.
Nach COBS: maximal 260 Byte + 1 Trenner = **261 Byte/Frame**.

### 2.4 Wire-Form

```
[ COBS(SEQ ‖ CMD ‖ LEN ‖ PAYLOAD ‖ CRC16) ] [ 0x00 ]
```

Empfänger liest Bytes bis zum nächsten `0x00`, dekodiert COBS, prüft CRC,
dann verarbeitet oder verwirft.

---

## 3. Kommando-Tabelle

Opcodes sind in Themen-Bereiche gruppiert:

| Bereich | Zweck |
|---|---|
| 0x01–0x0F | Servo-Steuerung + State-Roundtrip |
| 0x10–0x1F | Servo-Konfiguration |
| 0x20–0x2F | Servo Enable/Disable |
| 0x30–0x3F | LEDs (6 × WS2812 onboard) |
| 0x40–0x4F | Inputs (Schalter, Sensoren) |
| 0x50–0x5F | System (Reset …) |
| 0x80–0xBF | Antwort-Opcodes (Anfrage-Code OR 0x80) |
| 0x7F / 0xFE / 0xFF | ERROR_REPORT / NACK / ACK |

| Code | Name              | Richtung      | Payload                                                                 |
|------|-------------------|---------------|-------------------------------------------------------------------------|
| 0x01 | `SET_TARGETS`     | Host → FW     | 18 × `int16` LE Pulse-µs (= 36 B)                                       |
| 0x02 | `GET_STATE`       | Host → FW     | — (LEN=0)                                                               |
| 0x82 | `STATE`           | FW → Host     | siehe §3.1 (= 75 B)                                                     |
| 0x10 | `SET_CALIBRATION` | Host → FW     | `servo_idx` (uint8), `pulse_min` (int16 LE), `pulse_max` (int16 LE), `pulse_zero` (int16 LE) (= 7 B) |
| 0x20 | `ENABLE_SERVO`    | Host → FW     | `servo_idx` (uint8), `enable` (uint8: 0 oder 1) (= 2 B)                 |
| 0x30 | `SET_LED`         | Host → FW     | `led_idx` (uint8: 0..5), `R` (uint8), `G` (uint8), `B` (uint8) (= 4 B)  |
| 0x31 | `SET_LEDS_ALL`    | Host → FW     | 6 × (R, G, B) — alle LEDs in einem Frame (= 18 B)                       |
| 0x40 | `GET_INPUTS`      | Host → FW     | — (LEN=0)                                                               |
| 0xC0 | `INPUTS`          | FW → Host     | 1 B Bit-Maske, siehe §3.2                                               |
| 0x50 | `RESET`           | Host → FW     | — (LEN=0)                                                               |
| 0x7F | `ERROR_REPORT`    | FW → Host     | `error_code` (uint8), `servo_idx` (uint8), `aux` (int16 LE) (= 4 B)     |
| 0xFF | `ACK`             | FW → Host     | `original_cmd` (uint8) (= 1 B)                                          |
| 0xFE | `NACK`            | FW → Host     | `original_cmd` (uint8), `reason` (uint8) (= 2 B)                        |

**Antwort-Konvention**: Antwort-Opcodes setzen Bit 7 vom Anfrage-Code.
Beispiele: `STATE` (0x82) auf `GET_STATE` (0x02), `INPUTS` (0xC0) auf
`GET_INPUTS` (0x40). Ausnahmen: `ACK`/`NACK`/`ERROR_REPORT` haben feste
Codes oben in der Tabelle.

**Endianness**: alle Multi-Byte-Felder Little-Endian (passt zum nativen
Format auf RP2040 und x86_64).

**Bereich-Reservierungen**:
- 0x40 ist `GET_INPUTS` (digital — Pimoroni `AnalogMux::read() → bool` mit
  `configure_pulls()` für Pull-Up am Schalter-Pin).
- 0x41 ist reserviert für künftiges `GET_SENSORS_ANALOG` (ADC-Raw je
  Mux-Adresse), falls später Strom-/Sensor-Bauteile statt Schalter dranhängen.
- 0x32 ist reserviert für `SET_LEDS_RANGE` (Subset von LEDs, falls nötig).

### 3.1 STATE-Payload (Antwort auf GET_STATE)

```
+-----------------+---------------------+--------------+----------------+
| 18 × int16 LE   | 18 × uint16 LE      | uint16 LE    | uint8          |
| last_pulse_us   | last_current_mA     | voltage_mV   | status_flags   |
+-----------------+---------------------+--------------+----------------+
   36 B             36 B                  2 B            1 B
```

= **75 Byte Payload**.

`status_flags` (Bit-Maske):

| Bit | Name | Bedeutung |
|---|---|---|
| 0 | `WATCHDOG_TRIPPED` | Frame-Timeout, alle Servos disabled |
| 1 | `UNDERVOLTAGE_TRIPPED` | Servo-Rail < CRIT, alle Servos disabled |
| 2 | `TOTAL_OVERCURRENT_TRIPPED` | Σ Strom > Limit, alle Servos disabled |
| 3 | `ANY_SERVO_OVERCURRENT_TRIPPED` | mindestens ein Servo wegen Stall disabled |
| 4 | `ANY_SERVO_DISABLED` | mindestens ein Servo aktuell disabled |
| 5 | `UNDERVOLTAGE_WARNING` | Servo-Rail < WARN (warn-only, kein Disable, auto-clearing) |
| 6 | `RELAY_ON` | Relay-Power-Gate geschlossen (Servo-Rail bestromt) |
| 7 | `SHUTDOWN_REQUEST` | Shutdown-Schalter (A1/GP27) ≥ `SHUTDOWN_HOLD_MS` (3 s) offen gehalten; Host-seitiger kontrollierter Shutdown (Block F) |

### 3.2 INPUTS-Payload (Antwort auf GET_INPUTS)

1 Byte Bit-Maske. Bit gesetzt = aktiv (Schalter geschlossen / gedrückt
gegen Pull-Up).

| Bit | Quelle | Pin / Mux |
|---|---|---|
| 0 | `SENSOR_1` | Mux-Adresse `0b000`, geteilt über GPIO 29 |
| 1 | `SENSOR_2` | Mux-Adresse `0b001` |
| 2 | `SENSOR_3` | Mux-Adresse `0b010` |
| 3 | `SENSOR_4` | Mux-Adresse `0b011` |
| 4 | `SENSOR_5` | Mux-Adresse `0b100` |
| 5 | `SENSOR_6` | Mux-Adresse `0b101` |
| 6 | `USER_SW` | GPIO 23 (Onboard Boot/User-Switch) |
| 7 | reserviert | (0) |

**Hinweis**: Mux-Adressen `0b110` und `0b111` sind für Voltage-/Current-Sense
reserviert (siehe §3.1 STATE) und tauchen hier **nicht** auf.

**Pull-Konfiguration**: Firmware konfiguriert beim Boot pro Sensor-Pin
intern Pull-Up. Schalter sind also gegen GND zu schalten (aktiv = LOW
am Pin → Bit gesetzt nach Negation in der Firmware).

### 3.3 SET_LED / SET_LEDS_ALL — LED-Range

Onboard sind **6 WS2812 LEDs** (`servo2040.hpp`: `NUM_LEDS = 6`,
`LED_DATA = GPIO 18`). `led_idx` läuft 0..5. Werte ≥ 6 → `NACK` mit
`reason = ERR_PAYLOAD_LEN`.

RGB-Reihenfolge: R, G, B (jeweils 0..255). Helligkeit wird **nicht** vom
Protokoll begrenzt — bei voller Helligkeit aller 6 LEDs auf Weiß zieht
der LED-Bus ~360 mA aus dem 5 V-Rail (60 mA/LED). Auf USB-Strom achten
wenn das Board nur über USB versorgt wird.

### 3.4 Error-Code-Tabelle (für ERROR_REPORT 0x7F)

| Code | Name                      | aux-Feld-Bedeutung           |
|------|---------------------------|------------------------------|
| 0x01 | `ERR_FRAME_CRC`           | (servo_idx=0, aux=0)         |
| 0x02 | `ERR_FRAME_MALFORMED`     | erwartete LEN                |
| 0x03 | `ERR_UNKNOWN_OPCODE`      | (servo_idx=0)                |
| 0x04 | `ERR_PAYLOAD_LEN`         | erwartete LEN                |
| 0x10 | `ERR_PULSE_OUT_OF_RANGE`  | clamped value (int16)        |
| 0x20 | `ERR_SERVO_OVERCURRENT`   | gemessener Strom in mA       |
| 0x21 | `ERR_TOTAL_OVERCURRENT`   | gemessener Total-Strom in mA |
| 0x30 | `ERR_UNDERVOLTAGE`        | gemessene Spannung in mV     |
| 0x40 | `ERR_WATCHDOG_TRIPPED`    | (servo_idx=0, aux=0)         |

Erweiterungen werden hier nachgetragen.

---

## 4. Beispiel-Frames (Hex-Dump)

> Die CRC-Werte unten sind Platzhalter `CRC_L CRC_H` — werden in der
> Firmware-Implementation und im Test-Skript automatisch berechnet.

### 4.1 SET_TARGETS, alle 18 Servos auf 1500 µs

`int16 LE`(1500) = `DC 05`. SEQ=0, CMD=0x01, LEN=36.

Vor COBS (43 Byte):
```
00 01 24 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC
05 DC 05 DC 05 DC 05 CRCL CRCH
```

### 4.2 GET_STATE

SEQ=1, CMD=0x02, LEN=0. Vor COBS (5 Byte):
```
01 02 00 CRCL CRCH
```

### 4.3 ENABLE_SERVO (Servo 5 enable)

SEQ=2, CMD=0x20, LEN=2, PAYLOAD=`05 01`. Vor COBS (7 Byte):
```
02 20 02 05 01 CRCL CRCH
```

### 4.4 RESET

SEQ=3, CMD=0x50, LEN=0. Vor COBS (5 Byte):
```
03 50 00 CRCL CRCH
```

### 4.5 SET_LED (LED 0 auf Rot)

SEQ=4, CMD=0x30, LEN=4, PAYLOAD=`00 FF 00 00` (idx=0, R=255, G=0, B=0).
Vor COBS (9 Byte):
```
04 30 04 00 FF 00 00 CRCL CRCH
```

### 4.6 SET_LEDS_ALL (alle 6 LEDs auf Grün)

SEQ=5, CMD=0x31, LEN=18, PAYLOAD = 6 × `00 FF 00`. Vor COBS (23 Byte):
```
05 31 12 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF
00 00 FF 00 CRCL CRCH
```

### 4.7 GET_INPUTS

SEQ=6, CMD=0x40, LEN=0. Vor COBS (5 Byte):
```
06 40 00 CRCL CRCH
```

Antwort z.B. (Schalter 1 + 3 + USER_SW gedrückt → Bitmaske `0b01000101 = 0x45`):
SEQ=6 (echo), CMD=0xC0, LEN=1, PAYLOAD=`45`. Vor COBS (6 Byte):
```
06 C0 01 45 CRCL CRCH
```

---

## 5. Boot-Sequenz und Initial-State

Beim Power-On / nach Reset:

1. Firmware initialisiert USB-CDC und ServoCluster
2. **Alle Servos sind `disabled`** (`is_enabled[i] == false` für alle i)
3. Firmware sendet Boot-Banner per `printf` über USB-CDC (aktuell:
   `Servo2040 USB-UART Communication Started…`, später Versionsstring)
4. Firmware wartet auf Host-Frames

**Host-Pflicht**: explizit `ENABLE_SERVO` (0x20) für jeden Servo, der
bewegt werden soll. Empfehlung: **gestaffelt mit 50 ms Pause zwischen
Servos** (siehe Phase-7-Plan Stufe D), um die Inrush-Strom-Peaks der
18 Servos zeitlich zu trennen — sonst Spitzenlast über PSU-/Akku-Limit.

---

## 6. Watchdog

Wenn die Firmware **200 ms** lang **kein gültiges Frame** (CRC ok,
Opcode bekannt) empfängt:

1. Alle Servos werden `disabled`
2. `status_flags.WATCHDOG_TRIPPED` wird gesetzt
3. `ERROR_REPORT` mit `ERR_WATCHDOG_TRIPPED` (0x40) wird gesendet (SEQ=0, unsolicited)

**Recovery**: Host sendet `RESET` (0x50) **und danach** `ENABLE_SERVO`
für jeden gewünschten Servo. Ohne `RESET` bleibt der `WATCHDOG_TRIPPED`-
Flag gesetzt und neue `ENABLE_SERVO`-Frames werden mit `NACK`
beantwortet.

---

## 7. Implementierungs-Hinweise

### Sender (Host)

1. Frame-Buffer aufbauen: `SEQ ‖ CMD ‖ LEN ‖ PAYLOAD`
2. CRC-16/CCITT-FALSE über diesen Buffer berechnen, anhängen (LE)
3. COBS-Encode den gesamten Buffer (inkl. CRC)
4. COBS-Output + ein `0x00`-Trenner-Byte ausgeben

**Wichtig**: CRC **vor** COBS-Encoding berechnen, sonst sieht der
Empfänger die COBS-modifizierten Bytes und CRC schlägt fehl.

### Empfänger (Firmware)

1. Bytes lesen bis `0x00` empfangen wird
2. COBS-Decode des Buffers (ohne den `0x00`-Trenner)
3. Aus dem decoded Buffer die letzten 2 Byte als CRC abzwacken
4. CRC über die restlichen Bytes prüfen, bei Mismatch → `ERR_FRAME_CRC`
5. SEQ/CMD/LEN extrahieren, LEN gegen Buffer-Länge prüfen → `ERR_PAYLOAD_LEN`
6. Opcode dispatchen
7. Bei Decode-Fehler oder Frame-Drop: bis zum nächsten `0x00` lesen,
   dann neu beginnen (Re-Sync ist garantiert durch COBS)

### Pulse-Wertebereich

`int16` deckt ±32 768 µs ab. Sinnvoller Wertebereich für Servos:
500–2500 µs. Werte außerhalb der pro-Servo-Kalibrierung (`pulse_min` /
`pulse_max`) werden in der Firmware **hard-clamped** (Phase-7-Stufe C.1)
und ein `ERR_PULSE_OUT_OF_RANGE` wird einmalig gesendet.

---

## 8. Versionierung

Dieses Dokument ist **Version 1.0**, fixiert am 2026-05-14.

Änderungen am Wire-Protokoll erfordern eine neue Version + Eintrag in
`phase_7_progress.md` Design-Entscheidungen + Tag im fw-Repo
(`protocol-vX.Y`). Inkompatible Änderungen markieren wir an Major-Version
(2.0), kompatible Erweiterungen (z. B. neue Opcodes) an Minor-Version
(1.1).
