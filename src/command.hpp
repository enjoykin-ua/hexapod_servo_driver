#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <vector>

/**
 * @class Command
 * @brief Abstract base class for all command implementations.
 *
 * This class defines a common interface for commands.
 */
class Command {
public:

    /**
     * @brief Default constructor
     */
    Command() = default;

    /**
     * @brief Virtual destructor for cleanup 
     */
    virtual ~Command() = default;

    /**
     * @brief Executes the command
     *
     * This is a pure virtual function, meaning all derived classes
     * must implement their own execution logic.
     *
     * @param args A vector of uint8_t values containing command parameters.
     */
    virtual void runCommand(const std::vector<uint8_t>& args) = 0;

    /**
     * @brief Retrieves the response from the last executed command
     *
     * This is a pure virtual function, meaning derived classes
     * must define how the response is generated and returned.
     *
     * @return A vector of uint8_t values representing the command response.
     */
    virtual std::vector<uint8_t> getResponse() = 0;
};

#endif // COMMAND_HPP
