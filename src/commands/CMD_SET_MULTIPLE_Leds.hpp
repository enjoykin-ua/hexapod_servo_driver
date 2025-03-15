#ifndef CMD_SET_MULTIPLE_LEDS_HPP
#define CMD_SET_MULTIPLE_LEDS_HPP

#include "command.hpp"
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace plasma;
using namespace servo;

/**
 * @class CMD_SET_MULTIPLE_Leds
 * @brief Command to set multiple LEDs on the WS2812 LED bar.
 *
 * This command allows setting the RGB values of multiple LEDs simultaneously
 * by passing a structured argument list.
 */
class CMD_SET_MULTIPLE_Leds : public Command {
private:
    WS2812& led_bar; ///< Reference to the WS2812 LED bar

public:
    /**
     * @brief Constructs a CMD_SET_MULTIPLE_Leds command with a WS2812 LED bar reference.
     * @param led_bar Reference to a WS2812 instance for LED control.
     */
    CMD_SET_MULTIPLE_Leds(WS2812& led_bar) :
        led_bar(led_bar) {}

    /**
     * @brief Executes the command to set multiple LED colors.
     * @param args A vector where:
     * - args[0] specifies the number of 4-byte groups.
     * - Each group consists of [LED index, R, G, B] (4 bytes per LED).
     *
     * If the data is invalid or LEDs are out of range, they are ignored.
     * example from uart usb:
        echo -ne "\x55\x05\x06\x00\xFF\xFF\xFF\x01\xFF\x00\x00\x02\x00\xFF\x00\x03\x00\x00\xFF\x04\xFF\xFF\x00\x05\xEE\x82\xEE\xAA" > /dev/ttyACM0
        \x55 → Start byte
        \x05 → OpCode for CMD_SET_MULTIPLE_Leds
        \x06 → Number of LEDs being set (6)
        \x00\xFF\xFF\xFF → LED 0 = White (255,255,255)
        \x01\xFF\x00\x00 → LED 1 = Red (255,0,0)
        \x02\x00\xFF\x00 → LED 2 = Green (0,255,0)
        \x03\x00\x00\xFF → LED 3 = Blue (0,0,255)
        \x04\xFF\xFF\x00 → LED 4 = Yellow (255,255,0)
        \x05\xEE\x82\xEE → LED 5 = Violet (238,130,238)
        \xAA → End byte

        example 2 from uart usb:
        echo -ne "\x55\x05\x06\x00\xFF\xFF\xFF\x01\x00\x00\x00\x02\x00\x00\x00\x03\xFF\xFF\xFF\x04\x00\x00\x00\x05\x00\x00\x00\xAA" > /dev/ttyACM0
        \x55 → Start byte
        \x05 → OpCode for CMD_SET_MULTIPLE_Leds
        \x06 → Number of LEDs being set (6 in total)
        \x00\xFF\xFF\xFF → LED 0 = White (255,255,255)
        \x01\x00\x00\x00 → LED 1 = OFF (0,0,0)
        \x02\x00\x00\x00 → LED 2 = OFF (0,0,0)
        \x03\xFF\xFF\xFF → LED 3 = White (255,255,255)
        \x04\x00\x00\x00 → LED 4 = OFF (0,0,0)
        \x05\x00\x00\x00 → LED 5 = OFF (0,0,0)
        \xAA → End byte
     */
    void runCommand(const std::vector<uint8_t>& args) override {
        if (args.empty()) {
            return;
        }

        // The first byte indicates how many LEDs are being set (group count)
        uint8_t group_count = args[0];

        // Expecting (1 + 4 * group_count) bytes in total
        if (args.size() != (1 + 4 * group_count)) {
            return;
        }

        // Loop through each LED group and update the colors
        for (uint8_t i = 0; i < group_count; ++i) {
            uint8_t base_index = 1 + i * 4;
            uint8_t pin = args[base_index];

            // Ensure the pin index is within a valid range
            if (pin >= servo2040::NUM_LEDS) {
                continue; // Skip this LED if the index is out of range
            }

            // Set the LED color for the given index
            led_bar.set_rgb(pin, args[base_index + 1], args[base_index + 2], args[base_index + 3]);
        }
    }

    /**
     * @brief Returns a success response after setting multiple LEDs.
     * @return A vector containing {0x00} to indicate success.
     */
    std::vector<uint8_t> getResponse() override {
        return {0x00};
    }
};

#endif // CMD_SET_MULTIPLE_LEDS_HPP
