#ifndef CMD_SET_LED_HPP
#define CMD_SET_LED_HPP

#include "command.hpp"
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace plasma;
using namespace servo;

/**
 * @class CMD_SET_Led
 * @brief Command to control an LED on the WS2812 LED bar.
 *
 * This command sets the RGB color of an LED at a specific position
 * on the LED bar using the provided arguments.
 */
class CMD_SET_Led : public Command {
private:
    WS2812& led_bar; ///< Reference to the WS2812 LED bar

public:
    /**
     * @brief Constructs a CMD_SET_Led command with a WS2812 LED bar reference.
     * @param led_bar Reference to a WS2812 instance for LED control.
     */
    CMD_SET_Led(WS2812& led_bar) :
        led_bar(led_bar) {}

    /**
     * @brief Executes the command to set an LED color.
     * @param args A vector of 4 bytes: [LED index, R, G, B].
     *
     * - args[0]: LED index (0-5).
     * - args[1]: Red intensity (0-255).
     * - args[2]: Green intensity (0-255).
     * - args[3]: Blue intensity (0-255).
     * example for uart usb: echo -ne "\x55\x04\x00\xFF\xFF\x00\xAA" > /dev/ttyACM0
     * If the argument size is incorrect or the LED index is out of range, the command does nothing.
     */
    void runCommand(const std::vector<uint8_t>& args) override {
        // Ensure exactly 4 arguments (LED index, R, G, B)
        if (args.size() != 4) {
            return;
        }

        uint8_t pin = args[0];
        if (pin > 5) { // Validate LED index (assuming 6 LEDs: 0-5)
            return;
        }

        // Set LED color using WS2812 API
        led_bar.set_rgb(pin, args[1], args[2], args[3]);
    }

    /**
     * @brief Returns a success response after setting the LED.
     * @return A vector containing {0x00} to indicate success.
     */
    std::vector<uint8_t> getResponse() override {
        return {0x00};
    }
};

#endif // CMD_SET_LED_HPP
