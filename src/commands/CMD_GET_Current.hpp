#ifndef CMD_GET_CURRENT_HPP
#define CMD_GET_CURRENT_HPP

#include "command.hpp"
#include "utils/conversion.hpp"
#include "utils/analog_reader.hpp"

using namespace servo;

/**
 * @class CMD_GET_Current
 * @brief Command to read the current value using an AnalogReader.
 *
 * This command retrieves the current measurement from an analog sensor
 * and converts it into a byte vector response.
 */
class CMD_GET_Current : public Command {
private:
    AnalogReader& reader;  ///< Reference to the AnalogReader instance
    float current;         ///< Stores the last measured current

public:
    /**
     * @brief Constructs a CMD_GET_Current command with an AnalogReader reference.
     * @param reader Reference to an AnalogReader instance for current measurements.
     */
    CMD_GET_Current(AnalogReader& reader):
        reader(reader),
        current(0.0f) {}

    /**
     * @brief Executes the command by reading the current from the AnalogReader.
     * @param args Unused parameter (required by the base class interface).
     */
    void runCommand(const std::vector<uint8_t>& args) override {
        current = reader.readCurrent();  // Reads current from the sensor
    }

    /**
     * @brief Returns the last read current value as a byte vector.
     * @return A vector of uint8_t representing the current value.
     */
    std::vector<uint8_t> getResponse() override {
        return float2vec(current); // Converts the float current to a byte vector
    }
};

#endif // CMD_GET_CURRENT_HPP
