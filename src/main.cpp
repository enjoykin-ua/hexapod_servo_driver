#include "includes.hpp"

// Define the servo pins range
const uint START_PIN    = servo2040::SERVO_1;
const uint END_PIN      = servo2040::SERVO_18;
const uint NUM_SERVOS   = (END_PIN - START_PIN) + 1;

// Function to initialize hardware components
void setupHardware(ServoCluster& servos, WS2812& leds, AnalogReader& reader) {
    leds.start();
    servos.init();
}

// Function to register commands
void registerCommands(CommandHandler& cmd_handler, AnalogReader& reader, WS2812& leds, ServoCluster& servos) {
    cmd_handler.addCommand(CMD_GET_VOLTAGE,         std::make_unique<CMD_GET_Voltage>(reader));
    cmd_handler.addCommand(CMD_GET_CURRENT,         std::make_unique<CMD_GET_Current>(reader));
    cmd_handler.addCommand(CMD_READ_SWITCH,         std::make_unique<CMD_GET_Switch_state>(reader));
    cmd_handler.addCommand(CMD_SET_LED,             std::make_unique<CMD_SET_Led>(leds));
    cmd_handler.addCommand(CMD_SET_LEDS,            std::make_unique<CMD_SET_MULTIPLE_Leds>(leds));
    cmd_handler.addCommand(CMD_SET_SERVO_PULSE,     std::make_unique<CMD_SET_Pulse_To_Servo>(servos));
    cmd_handler.addCommand(CMD_SET_SERVO_PULSES,    std::make_unique<CMD_SET_Pulse_To_Multiple_Servo>(servos));
}

int main() 
{
    stdio_init_all(); // Initialize USB serial interface

    // Wait until USB is connected
    sleep_ms(2000);
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("Servo2040 USB-UART Communication Started...\n");

    // Initialize hardware
    ServoCluster servos(pio0, 0, START_PIN, NUM_SERVOS);
    WS2812 leds(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);
    AnalogReader reader;

    setupHardware(servos, leds, reader);

    // Initialize command handler and register commands
    CommandHandler cmd_handler;
    registerCommands(cmd_handler, reader, leds, servos);

    std::vector<uint8_t> response;
    std::vector<uint8_t> buffer;

    while (true) {
        
        // Read incoming data from USB-UART
        int ch = getchar_timeout_us(1000000);
        if (ch != PICO_ERROR_TIMEOUT) {

            switch (ch) {

                case COMMAND_START:
                    // Start of a new command, clear buffers
                    buffer.clear();
                    response.clear();
                    break;

                case COMMAND_END:
                    // Process the received command
                    if (!buffer.empty()) {
                        uint8_t opCode = buffer[0];
                        std::vector<uint8_t> args(buffer.begin() + 1, buffer.end());
                        
                        // Execute the command and generate a response
                        response = cmd_handler.runCommand(opCode, args);

                        // Send the response back via USB-UART
                        putchar(COMMAND_START);
                        for (uint8_t byte : response) {
                            putchar(byte);
                        }
                        putchar(COMMAND_END);
                    }
                    break;

                default:
                    // Store received byte in buffer
                    buffer.push_back(ch);
                    break;
            }
        }
    }

    return 0;
}
