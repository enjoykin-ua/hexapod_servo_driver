#ifndef CMD_SET_PULSE_TO_SERVO_HPP
#define CMD_SET_PULSE_TO_SERVO_HPP

#include "command.hpp"
#include "utils/conversion.hpp"

using namespace servo;

/**
 * @class CMD_SET_Pulse_To_Servo
 * @brief Command to set a pulse width to a specific servo.
 *
 * This command allows setting the pulse width of a servo motor
 * by specifying the servo pin and the desired pulse width in microseconds.
 */
class CMD_SET_Pulse_To_Servo : public Command {
private:
    ServoCluster& servos; ///< Reference to the ServoCluster instance

public:
    /**
     * @brief Constructs a CMD_SET_Pulse_To_Servo command with a ServoCluster reference.
     * @param servos Reference to a ServoCluster instance controlling multiple servos.
     */
    CMD_SET_Pulse_To_Servo(ServoCluster& servos) :
        servos(servos) {}

    /**
     * @brief Executes the command to set a servo pulse width.
     * @param args A vector of 5 bytes:
     * - args[0]: Servo pin index.
     * - args[1-4]: 4-byte float representing the pulse width in microseconds.
     *
     * If the argument size is incorrect, the command does nothing.
       example uart usb:
       set servo 0 to 1500us:   echo -ne "\x55\x06\x00\x00\x00\xB4\x44\xAA" > /dev/ttyACM0
        \x55	0x55	    Start-Bit
        \x06	0x06	    Opcode: CMD_SET_SERVO_PULSE
        \x00	0x00	    Servo Pin 0
        \x00\x00\xB4\x44	1500.0 (float)	Pulsweite in µs (1500 = Neutralstellung)
        \xAA	0xAA	    End-Bit
        set servo 0 to 900us:   echo -ne "\x55\x06\x00\x00\x00\x70\x43\xAA" > /dev/ttyACM0

        set servo 1 to 1500us:   echo -ne "\x55\x06\x01\x00\x00\xB4\x44\xAA" > /dev/ttyACM0
        \x55	0x55	    Start-Bit
        \x06	0x06	    Opcode: CMD_SET_SERVO_PULSE
        \x01	0x01	    Servo Pin 1
        \x00\x00\xB4\x44	1500.0 (float)	Pulsweite in µs (1500 = Neutralstellung)
        \xAA	0xAA	    End-Bit

        set servo 1 to 900us:    echo -ne "\x55\x06\x01\x00\x00\x70\x43\xAA" > /dev/ttyACM0

        just for manual tests, to ensure 0,1 and 17 and 18 dont manipulate eachother like they do in micropython
            1000us: echo -ne "\x55\x06\x01\x00\x00\x7A\x44\xAA" > /dev/ttyACM0
            1800us: echo -ne "\x55\x06\x01\x00\x00\xE1\x44\xAA" > /dev/ttyACM0
        
        servo 17
            1000us: echo -ne "\x55\x06\x10\x00\x00\x7A\x44\xAA" > /dev/ttyACM0
            1800us: echo -ne "\x55\x06\x10\x00\x00\xE1\x44\xAA" > /dev/ttyACM0

        servo 18
            1000us: echo -ne "\x55\x06\x11\x00\x00\x7A\x44\xAA" > /dev/ttyACM0
            1800us: echo -ne "\x55\x06\x11\x00\x00\xE1\x44\xAA" > /dev/ttyACM0

     */
    void runCommand(const std::vector<uint8_t>& args) override {
        // Ensure exactly 5 arguments (1 for pin + 4 for float pulse width)
        if (args.size() != 5) {
            return;
        }

        uint8_t pin = args[0]; // Servo pin index
        float pulse_width = vec2float(args, 1); // Convert 4 bytes to float

        // Set the pulse width for the given servo pin
        servos.pulse(pin, pulse_width);
    }

    /**
     * @brief Returns a success response after setting the servo pulse.
     * @return A vector containing {0x00} to indicate success.
     */
    std::vector<uint8_t> getResponse() override {
        return {0x00};
    }
};

#endif // CMD_SET_PULSE_TO_SERVO_HPP
