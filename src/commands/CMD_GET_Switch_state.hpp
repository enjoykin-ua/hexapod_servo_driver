#ifndef CMD_GET_SWITCH_STATE_HPP
#define CMD_GET_SWITCH_STATE_HPP

#include "command.hpp"
#include "utils/conversion.hpp"
#include "utils/analog_reader.hpp"

using namespace servo;

/**
 * @class CMD_GET_Switch_state
 * @brief Command to read the state of a switch using an AnalogReader.
 *
 * This command retrieves the switch state from an analog sensor
 * and converts it into a byte vector response.
 */
class CMD_GET_Switch_state : public Command {
private:
    float value;          ///< Stores the last read switch state
    uint8_t pin;         ///< Pin number associated with the switch
    AnalogReader& reader; ///< Reference to the AnalogReader instance

public:
    /**
     * @brief Constructs a CMD_GET_Switch_state command with an AnalogReader reference.
     * @param reader Reference to an AnalogReader instance for switch state readings.
     */
    CMD_GET_Switch_state(AnalogReader& reader) :
        value(0.0f),
        pin(0),
        reader(reader) {}

    /**
     * @brief Executes the command by reading the switch state from the AnalogReader.
     * @param args A vector containing a single byte that represents the pin number.
     */
    void runCommand(const std::vector<uint8_t>& args) override {
        
        // Error handling: Expecting exactly one argument (pin number)
        if (args.size() != 1) {
            value = 0.0f;  // Default invalid state
            return;
        }

        // Valid pin range: 0 to 5; out-of-range values return 0.0f
        value = reader.readSensor(args[0]);
    }

    /**
     * @brief Returns the last read switch state as a byte vector.
     * @return A vector of uint8_t representing the switch state.
     */
    std::vector<uint8_t> getResponse() override {
        return float2vec(value); // Converts the switch state to a byte vector
    }
};

#endif // CMD_GET_SWITCH_STATE_HPP
