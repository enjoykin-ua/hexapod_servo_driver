// Phase 13 — minimaler Pimoroni-only Test (kein USB-Protokoll, kein Plugin).
//
// Sicherer Single-Pin-Test: NUR Pin 1 (leg_1_femur) wird aktiviert,
// mit moderaten PWM-Werten innerhalb der mech-Range. Alle anderen
// 17 Pins bleiben disabled (passiv).
//
// Kein enable_all() — das wuerde laut Pimoroni-Doku "all servos to
// middle" machen, und das ist genau das was wir vermeiden wollen
// zu testen. Stattdessen direkt pulse() — wenn das funktioniert,
// hat unser hexapod_servo_driver FW-Code einen Bug.
//
// PWM-Werte fuer Pin 1 (leg_1_femur, dir=+1):
//   pulse_zero = 1460 µs (User-Cal: horizontal/Mitte)
//   "leicht oben"  = 1200 µs (~halbweg zwischen pulse_zero und pulse_min=815)
//   "leicht unten" = 1800 µs (~halbweg zwischen pulse_zero und pulse_max=2120)
//
// Beide Werte sind WELL INNERHALB der mech-Range — kein Servo-Schaden,
// keine Mechanik-Kollision. User-Cal sagt:
//   1200 µs → Bein leicht hoch
//   1800 µs → Bein leicht runter
//
// Flash:  sudo picotool load test_pimoroni_direct.uf2 && sudo picotool reboot
//
// Erwartung mit aufgebocktem Hexapod, PSU on:
//   1. Servos starten passiv (Bein 1 haengt nach unten durch Schwerkraft).
//   2. Test-Sequenz: pulse(1200) → 3s → pulse(1800) → 3s → pulse(1460) → 3s
//      → disable. ALLE ANDEREN 17 Pins bleiben durchgehend passiv.
//   3. Erwartet: Bein 1's Femur bewegt sich sichtbar zwischen "leicht oben"
//      und "leicht unten" und "horizontal".
//
// Wenn das funktioniert  → Pimoroni's pulse() direkt OHNE enable_all()
//                          arbeitet korrekt. Unser FW-Code hat den Bug.
// Wenn Bein bleibt in Mitte → Pimoroni's pulse() selbst geht intern zur
//                             Mitte. Then Pimoroni-Edit gerechtfertigt.

#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace servo;

const uint NUM_SERVOS  = 18;
const uint START_PIN   = servo2040::SERVO_1;
const uint TEST_PIN    = 1;   // Pin 1 = leg_1_femur (laut servo_mapping.yaml)

// Sichere PWM-Werte INNERHALB unserer mech-Range fuer Pin 1.
// pulse_min=815, pulse_zero=1460, pulse_max=2120 laut User-Cal.
constexpr float PULSE_LIGHT_UP   = 1200.0f;  // ~halbweg pulse_zero ↔ pulse_min
constexpr float PULSE_LIGHT_DOWN = 1800.0f;  // ~halbweg pulse_zero ↔ pulse_max
constexpr float PULSE_CENTER     = 1460.0f;  // user-cal pulse_zero

int main() {
    stdio_init_all();

    // Wait briefly for USB-CDC enumeration (optional, just for clean reboot)
    for (int i = 0; i < 30; i++) {
        if (stdio_usb_connected()) break;
        sleep_ms(50);
    }

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    // === Setup: ServoCluster init ===
    ServoCluster servos(pio0, 0, START_PIN, NUM_SERVOS);
    servos.init();

    // === Pimoroni-Sample-Pattern: enable() ZUERST (= geht zur Mitte) ===
    // Pimoroni-Doku sagt: enable() puts servo at MIDDLE. Das ist by-design.
    // Danach: pulse() bringt Servo zu spezifischer Position.
    //
    // Wir testen jetzt OB dieses Pattern funktioniert: enable() →
    // 2s halten in Mitte (laut Pimoroni) → pulse() zu leicht hoch.
    // Wenn das klappt, muessen wir unsere FW so anpassen dass sie
    // dieses Pattern verwendet.
    //
    // Sicher: nur Pin 1 enabled, andere 17 bleiben passiv.
    servos.enable(TEST_PIN, true);   // sollte zu MID gehen (Pimoroni-by-design)
    sleep_ms(2000);                  // halten in Mitte ~2s, visuell beobachten

    // === Test-Sequenz NACH enable ===
    // Step 1: Bein leicht hoch  — sollte JETZT funktionieren
    servos.pulse(TEST_PIN, PULSE_LIGHT_UP, true);
    sleep_ms(3000);

    // Step 2: Bein leicht runter
    servos.pulse(TEST_PIN, PULSE_LIGHT_DOWN, true);
    sleep_ms(3000);

    // Step 3: zur horizontalen Mitte
    servos.pulse(TEST_PIN, PULSE_CENTER, true);
    sleep_ms(3000);

    // Step 4: nochmal Bein leicht hoch (Re-Check)
    servos.pulse(TEST_PIN, PULSE_LIGHT_UP, true);
    sleep_ms(3000);

    // === Disable ===
    servos.disable(TEST_PIN, true);

    // LED blinkt = Test durch
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(500);
    }
}
