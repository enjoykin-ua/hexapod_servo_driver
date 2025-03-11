#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();  // USB-Serielle Schnittstelle aktivieren

    // Warten, bis USB verbunden ist
    sleep_ms(2000);
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("Servo2040 USB-UART Test gestartet...\n");

    while (true) {
        int ch = getchar_timeout_us(1000000); // Warte 1 Sekunde auf Eingabe
        if (ch != PICO_ERROR_TIMEOUT) {
            printf("Empfangen: %c\n", ch);
        }
    }
}
