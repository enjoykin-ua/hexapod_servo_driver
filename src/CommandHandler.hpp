#ifndef COMMAND_HANDLER_HPP
#define COMMAND_HANDLER_HPP

#include "command.hpp"
#include <map>
#include <memory>
#include <vector>

/**
 * @class CommandHandler
 * @brief Manages and executes registered commands based on an opcode.
 *
 * This class allows dynamic registration of command objects, 
 * storing them in a command table and executing them via `runCommand()`.
 */
class CommandHandler {
private:
    /**
     * @brief A mapping of opcodes to their corresponding command objects.
     * Each command is stored as a unique pointer to manage its lifecycle.
     */
    std::map<uint8_t, std::unique_ptr<Command>> commandTable;

public:
    /**
     * @brief Registers a command with a specified opcode.
     *
     * @param opCode The opcode that identifies the command.
     * @param command A unique pointer to the command object.
     */
    void addCommand(uint8_t opCode, std::unique_ptr<Command> command) {
        commandTable[opCode] = std::move(command);
    }

    /**
     * @brief Executes a command associated with the given opcode.
     *
     * If the command exists, it executes with the provided arguments
     * and returns the response. If the opcode is unknown, an error response is returned.
     *
     * @param opCode The opcode of the command to execute.
     * @param args A vector of arguments passed to the command.
     * @return A vector containing the command's response.
     */
    std::vector<uint8_t> runCommand(uint8_t opCode, const std::vector<uint8_t>& args) {
        auto it = commandTable.find(opCode);
        if (it != commandTable.end()) {
            it->second->runCommand(args); 
            return it->second->getResponse(); 
        } else {
            return {0xFF, 0x01}; 
        }
    }
};

#endif // COMMAND_HANDLER_HPP
