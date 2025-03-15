#ifndef CMD_GET_VOLTAGE_HPP
#define CMD_GET_VOLTAGE_HPP

#include "command.hpp"
#include "utils/conversion.hpp"
#include "utils/analog_reader.hpp"

using namespace servo;

/**
 * @class CMD_GET_Voltage
 * @brief Command to read the current voltage value using an AnalogReader.
 *
 * This command reads the voltage from an analog sensor and converts it
 * into a byte vector response.
 */
class CMD_GET_Voltage : public Command {
private:
    AnalogReader& reader;  ///< Reference to the AnalogReader instance
    float voltage;         ///< Stores the last read voltage value

public:
    /**
     * @brief Constructs a CMD_GET_Voltage command with a reference to an AnalogReader.
     * @param reader Reference to an AnalogReader instance for voltage measurements.
     */
    CMD_GET_Voltage(AnalogReader& reader):
        reader(reader),
        voltage(0.0f) {}

    /**
     * @brief Executes the command by reading the voltage from the AnalogReader.
     * @param args Unused parameter (required by the base class interface).
     */
    void runCommand(const std::vector<uint8_t>& args) override {
        voltage = reader.readVoltage();  // Reads the voltage
    }

    /**
     * @brief Returns the last read voltage as a byte vector.
     * @return A vector of uint8_t representing the voltage value.
     */
    std::vector<uint8_t> getResponse() override {
        return float2vec(voltage); // Converts the float voltage to a byte vector
    }
};

#endif // CMD_GET_VOLTAGE_HPP
