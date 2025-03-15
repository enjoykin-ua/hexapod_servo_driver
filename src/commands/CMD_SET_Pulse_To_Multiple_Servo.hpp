#ifndef CMD_SET_PULSE_TO_MULTIPLE_SERVO_HPP
#define CMD_SET_PULSE_TO_MULTIPLE_SERVO_HPP

#include "command.hpp"
#include "utils/conversion.hpp"

using namespace servo;

/**
 * @class CMD_SET_Pulse_To_Multiple_Servo
 * @brief Command to set pulse widths for multiple servos.
 *
 * This command allows setting the pulse width of multiple servos simultaneously
 * by specifying servo pins and the desired pulse widths in microseconds.
 */
class CMD_SET_Pulse_To_Multiple_Servo : public Command {
private:
    ServoCluster& servos; ///< Reference to the ServoCluster instance

public:
    /**
     * @brief Constructs a CMD_SET_Pulse_To_Multiple_Servo command with a ServoCluster reference.
     * @param servos Reference to a ServoCluster instance controlling multiple servos.
     */
    CMD_SET_Pulse_To_Multiple_Servo(ServoCluster& servos) :
        servos(servos) {}

    /**
     * @brief Executes the command to set pulse widths for multiple servos.
     * @param args A vector where:
     * - args[0]: Number of servos to update (group count).
     * - Each group consists of [Servo pin (1 byte) + Pulse width (4-byte float)].
     *
     * If the argument size is incorrect or servos are out of range, they are ignored.
      
        echo -ne "\x55\x07\x07\x01\x01\x00\x00\x7A\x44\xAA" > /dev/ttyACM0
        \x55                # COMMAND_START
        \x07                # LENGTH (7 Bytes folgen nach diesem Byte)
        \x07                # CMD_SET_SERVO_PULSES (0x07)
        \x01                # Anzahl der Servos (1 Servo)
        \x01                # Servo-ID (Servo 1)
        \x00\x00\x7A\x44    # Pulse (1000 µs als 4-Byte Float)
        \xAA                # COMMAND_END

        echo -ne "\x55\x07\x07\x01\x01\xCD\xCC\xE4\x44\xAA" > /dev/ttyACM0
        \x55                # COMMAND_START
        \x07                # LENGTH (7 Bytes folgen nach diesem Byte)
        \x07                # CMD_SET_SERVO_PULSES (0x07)
        \x01                # Anzahl der Servos (1 Servo)
        \x01                # Servo-ID (Servo 1)
        \xCD\xCC\xE4\x44    # Pulse (1800 µs als 4-Byte Float)
        \xAA                # COMMAND_END


     *
    void runCommand_original(const std::vector<uint8_t>& args) override {
        if (args.empty()) {
            return;
        }

        // The first byte indicates how many servos are being set (group count)
        uint8_t group_count = args[0];

        // Expecting (1 + 5 * group_count) bytes in total
        if (args.size() != (1 + 5 * group_count)) {
            return;
        }

        // Loop through each servo group and update the pulse widths
        for (uint8_t i = 0; i < group_count; ++i) {
            uint8_t base_index = 1 + i * 5;
            uint8_t pin = args[base_index];

            // Ensure the pin index is within a valid range
            if (pin >= servo2040::NUM_SERVOS) {
                continue; // Skip this servo if the index is out of range
            }

            float pulse_width = vec2float(args, base_index + 1); // Convert bytes to float

            // Set the pulse width for the given servo pin
            servos.pulse(pin, pulse_width);
        }
    }
    
    1000
    echo -ne "\x55\x07\x01\x00\x00\x00\x7A\x44\xAA" > /dev/ttyACM0

    1400:
    echo -ne "\x55\x07\x01\x00\x00\x00\xAF\x44\xAA" > /dev/ttyACM0
    
    800:
    echo -ne "\x55\x07\x01\x00\x00\x00\x48\x44\xAA" > /dev/ttyACM0

    1500:
    echo -ne "\x55\x07\x01\x00\x00\x80\xBB\x44\xAA" > /dev/ttyACM0

    1800:
    echo -ne "\x55\x07\x01\x00\x00\x00\xE1\x44\xAA" > /dev/ttyACM0

    2000:
    echo -ne "\x55\x07\x01\x00\x00\x00\xFA\x44\xAA" > /dev/ttyACM0

    servo 0 = 800, servo 1 = 1500
    echo -ne "\x55\x07\x02\x00\x00\x00\x48\x44\x01\x00\x00\xBB\x44\xAA" > /dev/ttyACM0
    umgekehrt
    echo -ne "\x55\x07\x02\x00\x00\x00\xBB\x44\x01\x00\x00\x48\x44\xAA" > /dev/ttyACM0
        
    servo 17 and 18:
    echo -ne "\x55\x07\x02\x10\x00\x00\x48\x44\x11\x00\x00\xBB\x44\xAA" > /dev/ttyACM0
    echo -ne "\x55\x07\x02\x11\x00\x00\x48\x44\x10\x00\x00\xBB\x44\xAA" > /dev/ttyACM0

    servo 0 and 17
    echo -ne "\x55\x07\x02\x00\x00\x00\x48\x44\x10\x00\x00\xD4\x44\xAA" > /dev/ttyACM0
    echo -ne "\x55\x07\x02\x00\x00\x00\xD4\x44\x10\x00\x00\x48\x44\xAA" > /dev/ttyACM0
    \x55	COMMAND_START
    \x07	opCode (CMD_SET_SERVO_PULSES)
    \x02	group_count = 2 (zwei Servos setzen)
    \x00	Servo-Pin 0 (0x00)
    \x00\x00\xD4\x44	1700.0 als Float (Little-Endian)
    \x10	Servo-Pin 17 (0x10)
    \x00\x00\x48\x44	800.0 als Float (Little-Endian)
    \xAA	COMMAND_END



*/

    void runCommand(const std::vector<uint8_t>& args) override {
        printf("DEBUG: runCommand wurde aufgerufen!\n");

        if (args.empty()) {
            printf("DEBUG: Keine Argumente empfangen!\n");
            return;
        }

        uint8_t group_count = args[0];
        printf("DEBUG: Empfangenes group_count: %d\n", group_count);

        if (args.size() != (1 + 5 * group_count)) {
            printf("DEBUG: Falsche Anzahl an Bytes! Erwartet: %d, Erhalten: %d\n", (1 + 5 * group_count), (int)args.size());
            return;
        }

        for (uint8_t i = 0; i < group_count; ++i) {
            uint8_t base_index = 1 + i * 5;
            uint8_t pin = args[base_index];

            printf("DEBUG: Servo %d\n", pin);

            if (pin >= servo2040::NUM_SERVOS) {
                printf("ERROR: Ungültiges Servo-Pin: %d\n", pin);
                continue;
            }

            printf("DEBUG: Raw Bytes: %02X %02X %02X %02X\n",
                args[base_index + 1], args[base_index + 2], args[base_index + 3], args[base_index + 4]);
         

            float pulse_width = vec2float(args, base_index + 1);
            printf("DEBUG: Servo %d -> Pulse Width: %.2f\n", pin, pulse_width);

            servos.pulse(pin, pulse_width);
        }
    }

    /**
     * @brief Returns a success response after setting multiple servo pulses.
     * @return A vector containing {0x00} to indicate success.
     */
    std::vector<uint8_t> getResponse() override {
        return {0x00};
    }
};

#endif // CMD_SET_PULSE_TO_MULTIPLE_SERVO_HPP
